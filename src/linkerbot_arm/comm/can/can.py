"""Compatibility boundary over the low-level transport dispatcher.

Transport owns CAN I/O and the FIFO arbiter. This adapter keeps the arm SDK's
legacy calls and exception categories with one active transmit worker.
"""

import logging
import queue
import threading
import time
from collections.abc import Callable

import can
from linkerbot_transport import SendReceipt, TransmitArbiter
from linkerbot_transport.can.can import (
    DEFAULT_MAX_CONSECUTIVE_ERRORS as DEFAULT_MAX_CONSECUTIVE_ERRORS,
)
from linkerbot_transport.can.can import (
    ERROR_BACKOFF_BASE_S as ERROR_BACKOFF_BASE_S,
)
from linkerbot_transport.can.can import (
    ERROR_BACKOFF_MAX_S as ERROR_BACKOFF_MAX_S,
)
from linkerbot_transport.can.can import (
    THREAD_JOIN_TIMEOUT_S as THREAD_JOIN_TIMEOUT_S,
)
from linkerbot_transport.can.can import (
    CANMessageDispatcher as _TransportDispatcher,
)
from linkerbot_transport.can.can import (
    _error_backoff_s as _error_backoff_s,
)
from linkerbot_transport.errors import (
    QueueFullError,
    TransportError,
)
from linkerbot_transport.errors import (
    ValidationError as _TransportValidationError,
)

from linkerbot_arm._transport.pacing import wait_for_send_deadline
from linkerbot_arm._transport.reception import reception_time
from linkerbot_arm._transport.targets import TargetQueueMixin
from linkerbot_arm.exceptions import CANError, ValidationError

from .types import CanInterface as CanInterface

logger = logging.getLogger(__name__)


class CANMessageDispatcher(TargetQueueMixin, _TransportDispatcher):
    """Use transport workers while retaining the arm dispatcher call contract."""

    def __init__(
        self,
        interface_name: str,
        interface_type: str = "socketcan",
        on_bus_error: Callable[[Exception], None] | None = None,
        max_consecutive_errors: int = DEFAULT_MAX_CONSECUTIVE_ERRORS,
    ) -> None:
        self._init_target_queue()
        self._arm_callbacks = []
        self._arm_callbacks_lock = threading.Lock()
        self._tx_stopped = threading.Event()
        self._tx_errors = 0
        self._tx = TransmitArbiter(
            self._dispatch_frame, fifo_capacity=self.SEND_QUEUE_SIZE
        )
        try:
            super().__init__(
                interface_name,
                interface_type,
                on_bus_error=on_bus_error,
                max_consecutive_errors=max_consecutive_errors,
                start_send_worker=False,
            )
            # One transport callback captures time before any arm subscriber
            # can delay delivery to the remaining motor decoders.
            super().subscribe(self._receive_message)
            with self._error_lock:
                self._raise_if_cannot_send_locked()
                self._tx.open(generation=1)
        except BaseException as error:
            # If arbiter startup fails after opening CAN, still release the bus.
            if hasattr(self, "_bus"):
                try:
                    self.stop()
                except BaseException:
                    logger.exception("Arm dispatcher initialization cleanup failed")
            if isinstance(error, _TransportValidationError):
                raise ValidationError(*error.args) from error
            raise

    def subscribe(self, callback: Callable[[can.Message], None]) -> None:
        with self._arm_callbacks_lock:
            if callback not in self._arm_callbacks:
                self._arm_callbacks.append(callback)

    def unsubscribe(self, callback: Callable[[can.Message], None]) -> None:
        with self._arm_callbacks_lock:
            if callback in self._arm_callbacks:
                self._arm_callbacks.remove(callback)

    def _receive_message(self, message: can.Message) -> None:
        received_at_s = time.monotonic()
        with self._arm_callbacks_lock:
            callbacks = tuple(self._arm_callbacks)
        with reception_time(received_at_s):
            for callback in callbacks:
                if not self._running:
                    break
                try:
                    callback(message)
                except Exception:
                    logger.exception("Error in arm CAN receive callback")

    def send(self, msg: can.Message) -> None:
        """Enqueue without waiting; preserve None, CANError and queue.Full."""
        try:
            with self._error_lock:
                self._raise_if_cannot_send_locked()
                self._tx.submit_fifo(msg, endpoint_id="arm")
        except QueueFullError as error:
            raise queue.Full() from error
        except TransportError as error:
            raise CANError(*error.args) from error

    def _submit_target_frames(self, frames, **options):
        with self._error_lock:
            try:
                self._raise_if_cannot_send_locked()
            except TransportError as error:
                raise CANError(*error.args) from error
            return self._tx.submit_fifo_batch(frames, endpoint_id="arm", **options)

    def _handle_bus_error(self, error: Exception) -> None:
        # Wake checked-send waiters before callbacks acquire an arm's locks.
        # Otherwise a lifecycle caller can wait on the TX thread while its
        # fault callback is waiting for that same caller's motion lock.
        self._fail_checked_sends(error)
        try:
            self._tx.abort(error)
        except Exception:
            logger.exception("Failed to drain arm transmitter after bus error")
        super()._handle_bus_error(error)
        periodic = getattr(self, "_arm_periodic_scheduler", None)
        if periodic is not None:
            periodic.close()

    def _dispatch_frame(self, message):
        deadline = time.perf_counter() + self.SEND_INTERVAL_S
        try:
            super().send_now(message)
            self._tx_errors = 0
        except Exception as error:
            self._tx_errors += 1
            logger.warning("CAN send failed: %s", error)
            if self._tx_errors >= self._max_consecutive_errors:
                self._handle_bus_error(error)
            raise
        wait_for_send_deadline(deadline, self._tx_stopped)
        return SendReceipt.completed()

    def stop(self) -> None:
        self._fail_checked_sends(CANError("CAN connection is closing"))
        with self._arm_callbacks_lock:
            self._arm_callbacks.clear()
        periodic = getattr(self, "_arm_periodic_scheduler", None)
        if periodic is not None:
            periodic.close()
        self._tx_stopped.set()
        with self._error_lock:
            self._running = False
        try:
            self._tx.close()
        except Exception:
            # A physical close may be needed to interrupt a blocked send.
            pass
        try:
            super().stop()
        finally:
            self._tx.close()
