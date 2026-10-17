"""Arm callback/FIFO adapter over CANTransport's explicit lifecycle.

Motion targets enqueue without waiting; lifecycle/configuration writes observe
physical send results. TransmitArbiter owns the bounded FIFO; CANTransport owns
physical I/O and exact protocol-address receive routes.
No pending motion is replayed and no connection is automatically recovered.
"""

import logging
import queue
import threading
import time
from collections.abc import Callable

import can
from linkerbot_transport import (
    CANRouteKey,
    DeviceStateError,
    QueueFullError,
    SendOutcome,
    TransmitArbiter,
    TransmitHealth,
    TransportError,
    TransportHealth,
)
from linkerbot_transport.can import CanInterface, CANTransport
from linkerbot_transport.errors import TimeoutError as TransportTimeoutError
from linkerbot_transport.errors import ValidationError as TransportValidationError

from linkerbot_arm.exceptions import CANError, StateError, TimeoutError, ValidationError

from .pacing import wait_for_send_deadline
from .reception import reception_time
from .targets import TargetQueueMixin

logger = logging.getLogger(__name__)


def _lite_route(message: can.Message) -> CANRouteKey:
    # RS00 responses carry motor ID in bits 8..15; comm type, status and
    # destination vary. Decode them in the motor using the unchanged message.
    identifier = message.arbitration_id
    if message.is_extended_id:
        identifier &= 0xFF00
    return CANRouteKey(identifier, message.is_extended_id)


def _routes(model: str, side: str) -> tuple[CANRouteKey, ...]:
    ids = range(61, 68) if side == "left" else range(51, 58)
    if model == "a7_lite":
        return tuple(CANRouteKey(mid << 8, True) for mid in ids)
    keys = [CANRouteKey(mid, False) for mid in ids]
    if model == "p7":
        keys.extend(CANRouteKey(0x600 + mid, False) for mid in list(ids)[5:])
    return tuple(keys)


def _mapped_error(error: BaseException) -> BaseException:
    if isinstance(error, QueueFullError):
        return queue.Full()
    if isinstance(error, TransportValidationError):
        return ValidationError(*error.args)
    if isinstance(error, TransportTimeoutError):
        return TimeoutError(*error.args)
    if isinstance(error, DeviceStateError):
        return StateError(*error.args)
    if isinstance(error, TransportError):
        return CANError(*error.args)
    return error


def _raise_mapped(error: BaseException):
    mapped = _mapped_error(error)
    if mapped is error:
        raise error
    raise mapped from error


class ManagedCANDispatcher(TargetQueueMixin):
    """Internal, exclusively owned connection for one arm.

    Recovery means closing this arm and constructing a fresh one, which reruns
    motor discovery and initial-state reads. Exposing transport.recover() on a
    live arm would leave its motor caches and enable state stale.
    """

    SEND_QUEUE_SIZE = 2000
    SEND_INTERVAL_S = 0.0003
    MAX_CONSECUTIVE_ERRORS = 10
    JOIN_TIMEOUT_S = 2.0

    def __init__(
        self,
        name: str,
        backend: str,
        model: str,
        side: str,
        on_bus_error: Callable[[Exception], None] | None = None,
    ) -> None:
        self._init_target_queue()
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._callbacks = []
        self._on_bus_error = on_bus_error
        self._error = None
        self._stopped = threading.Event()
        self._available = threading.Event()
        self._recv_thread = None
        self._route = None
        self._send_errors = 0
        self._transport = None
        self._tx = TransmitArbiter(
            self._dispatch,
            fifo_capacity=self.SEND_QUEUE_SIZE,
            join_timeout_s=self.JOIN_TIMEOUT_S,
        )
        try:
            options = {"route_key_of": _lite_route} if model == "a7_lite" else {}
            self._transport = CANTransport(name, backend, **options)
            self._transport.open()
            self._route = self._transport.subscribe_routes(
                _routes(model, side),
                queue_capacity=256,
                on_available=self._available.set,
            )
            self._tx.open(generation=self._transport.get_health().generation)
            self._recv_thread = threading.Thread(
                target=self._receive,
                name="LinkerbotArmCAN.rx",
                daemon=True,
            )
            self._recv_thread.start()
        except BaseException as error:
            try:
                self.stop()
            except BaseException:
                logger.exception("CANTransport initialization cleanup failed")
            _raise_mapped(error)

    @property
    def can_interface(self) -> CanInterface:
        return self._transport.can_interface

    def subscribe(self, callback: Callable[[can.Message], None]) -> None:
        with self._lock:
            self._require_running()
            if callback not in self._callbacks:
                self._callbacks.append(callback)

    def unsubscribe(self, callback: Callable[[can.Message], None]) -> None:
        with self._lock:
            if callback in self._callbacks:
                self._callbacks.remove(callback)

    def send(self, message: can.Message) -> None:
        with self._lock:
            self._require_running()
            try:
                # One FIFO endpoint preserves ordering across all seven joints,
                # including reads, mode writes, calibration and stop commands.
                self._tx.submit_fifo(message, endpoint_id="arm")
            except Exception as error:
                _raise_mapped(error)
        return None

    def _submit_target_frames(self, frames, **options):
        with self._lock:
            self._require_running()
            return self._tx.submit_fifo_batch(frames, endpoint_id="arm", **options)

    def _require_running(self) -> None:
        if self._error is not None:
            raise CANError(str(self._error)) from self._error
        if self._stopped.is_set():
            raise RuntimeError("CAN dispatcher is stopped")

    def _dispatch(self, message):
        deadline = time.perf_counter() + self.SEND_INTERVAL_S
        with self._lock:
            self._require_running()
        receipt = self._transport.send(message)
        try:
            result = receipt.result(timeout=0)
            if result.outcome is not SendOutcome.DISPATCHED:
                raise CANError(f"CAN send ended as {result.outcome.value}")
            self._send_errors = 0
        except Exception as error:
            self._send_errors += 1
            logger.warning("CAN send failed: %s", error)
            if self._send_errors >= self.MAX_CONSECUTIVE_ERRORS:
                self._fault(error)
            raise
        # Preserve the original successful-send pacing without blocking callers.
        wait_for_send_deadline(deadline, self._stopped)
        return receipt

    def _fault(self, error: Exception) -> None:
        with self._lock:
            if self._stopped.is_set() or self._error is not None:
                return
            self._error = _mapped_error(error)
        self._fail_checked_sends(self._error)
        self._available.set()

    def _receive(self) -> None:
        try:
            while not self._stopped.is_set():
                self._available.wait()
                self._available.clear()
                if self._stopped.is_set() or self._error is not None:
                    break
                while not self._stopped.is_set() and self._error is None:
                    received = self._route.try_receive()
                    if received is None:
                        break
                    with self._lock:
                        callbacks = tuple(self._callbacks)
                    with reception_time(received.received_at_s):
                        for callback in callbacks:
                            if self._stopped.is_set() or self._error is not None:
                                break
                            try:
                                callback(received.message)
                            except Exception:
                                logger.exception("Error in arm CAN receive callback")
                if self._route.get_health().overflows:
                    raise CANError("arm receive queue overflowed; feedback was lost")
        except Exception as error:
            self._fault(error)
        finally:
            if self._error is not None:
                if self._on_bus_error is not None:
                    try:
                        self._on_bus_error(self._error)
                    except Exception:
                        logger.exception("Error in arm bus-fault callback")
                try:
                    self.stop()
                except Exception:
                    logger.exception("CANTransport fault cleanup failed; retry close")

    def stop(self) -> None:
        self._fail_checked_sends(CANError("CAN connection is closing"))
        periodic = getattr(self, "_arm_periodic_scheduler", None)
        if periodic is not None:
            periodic.close()
        errors = []
        with self._lifecycle_lock:
            with self._lock:
                self._stopped.set()
                self._callbacks.clear()
            self._available.set()
            try:
                self._tx.close()
            except Exception:
                # Retry after physical close has interrupted any stuck send.
                pass
            if self._transport is not None:
                try:
                    self._transport.close()
                except Exception as error:
                    errors.append(error)
            # A backend ignoring its send timeout may need physical close first.
            try:
                self._tx.close()
            except Exception as error:
                errors.append(error)
            thread = self._recv_thread
        # Never hold the operation lock while joining a callback that can close.
        if thread is not None and thread.ident is not None:
            if thread is not threading.current_thread():
                thread.join(self.JOIN_TIMEOUT_S)
                if thread.is_alive():
                    errors.append(TimeoutError("arm CAN receive worker did not stop"))
        if errors:
            _raise_mapped(errors[-1])

    def get_health(self) -> tuple[TransportHealth, TransmitHealth]:
        """Connection health and the actual asynchronous transmit queue."""
        return self._transport.get_health(), self._tx.get_health()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
