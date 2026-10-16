"""Timer-based motion state tracker with reset support."""

import math
import threading
from collections.abc import Callable
from contextlib import contextmanager

from linkerbot_arm.exceptions import StateError


class MotionTimer:
    """Track motion state using an estimated duration timer.

    Supports resetting the timer when a new motion starts before the
    previous one finishes.

    Usage::

        timer = MotionTimer()
        timer.start(2.0)        # expect motion to take 2s
        timer.is_moving()       # True
        timer.start(3.0)        # new move resets the timer to 3s
        timer.wait_done()       # block until motion finishes
        timer.is_moving()       # False
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._event = threading.Event()
        self._cancel_event = threading.Event()
        self._external_cancel: threading.Event | None = None
        self._timer: threading.Timer | None = None
        self._generation = 0
        self._stream_active = False
        self._worker_done = threading.Event()
        self._worker_done.set()
        self._worker_ident = None
        self._worker_error = None
        self._closing = False
        self._event.set()  # not moving initially

    def _begin_worker(self):
        self._worker_ident = threading.get_ident()
        self._worker_error = None
        self._worker_done.clear()

    def _is_worker_thread(self):
        return self._worker_ident == threading.get_ident()

    def _wait_worker_done(self, *, report_error=False):
        if self._is_worker_thread():
            raise StateError("Cannot wait for motion cleanup from its own worker")
        # Older Windows locks do not interrupt an unbounded wait for Python
        # signal handlers. Return to Python periodically without releasing the
        # worker's resources or treating a timeout as completion.
        while not self._worker_done.wait(timeout=0.05):
            pass
        if report_error and self._worker_error is not None:
            raise self._worker_error

    def start(self, duration: float) -> None:
        """Start or reset the motion timer.

        If already moving, cancels the previous timer and starts a new one.
        """
        _validate_timeout(duration, name="duration")
        with self._lock:
            self._require_admission()
            if self._stream_active:
                raise StateError(
                    "A host trajectory is still planning, streaming or cleaning up"
                )
            if self._external_cancel is not None and self._external_cancel.is_set():
                self.cancel()
                return
            self._cancel_event.clear()
            if self._timer is not None:
                self._timer.cancel()
            self._generation += 1
            generation = self._generation
            self._event.clear()
            self._timer = threading.Timer(duration, self._on_done, args=(generation,))
            self._timer.daemon = True
            self._timer.start()

    def _on_done(self, generation: int) -> None:
        with self._lock:
            if generation != self._generation:
                return
            self._timer = None
            self._event.set()

    def is_moving(self) -> bool:
        """Return whether motion is in progress."""
        return self._stream_active or not self._event.is_set()

    def wait_done(self, timeout: float | None = None) -> bool:
        """Block until motion completes.

        Returns True if motion finished, False if timed out.
        """
        if timeout is not None:
            _validate_timeout(timeout, name="timeout")
        external = self._external_cancel
        if external is None:
            return self._event.wait(timeout)
        # Cancellable home must also wake if the stop request arrives before
        # the stop worker gets to the SDK. This does not send a hardware stop.
        import time

        deadline = None if timeout is None else time.monotonic() + timeout
        while not self._event.is_set():
            if external.is_set():
                self.cancel()
                return True
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                return False
            self._event.wait(0.01 if remaining is None else min(0.01, remaining))
        return True

    def cancel(self) -> None:
        """Cancel the timer and mark motion as done immediately."""
        with self._lock:
            self._cancel_event.set()
            self._generation += 1
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            self._event.set()

    def _wait_cancelled(self, timeout: float) -> bool:
        """Interrupt a host-streaming delay without confusing timer expiry."""
        return self._cancel_event.wait(timeout) or (
            self._external_cancel is not None and self._external_cancel.is_set()
        )

    def _run_unless_cancelled(self, action: Callable[[], None]) -> bool:
        """Serialize target admission with cancellation, never physical I/O."""
        with self._lock:
            if self._cancel_event.is_set() or (
                self._external_cancel is not None and self._external_cancel.is_set()
            ):
                return False
            action()
            return True

    def _start_with_target(
        self, duration: Callable[[], float], action: Callable[[], None]
    ) -> None:
        with self._lock:
            self._require_admission()
            if self.is_moving():
                raise StateError("Cannot start new motion while arm is moving")
            if self._external_cancel is not None and self._external_cancel.is_set():
                self.cancel()
                return
            try:
                # The caller validates the target first. Reject unusable motion
                # parameters before admission, then time from accepted submission.
                seconds = duration()
                _validate_timeout(seconds, name="duration")
                if self._external_cancel is not None and self._external_cancel.is_set():
                    self.cancel()
                    return
                action()
                self.start(seconds)
            except BaseException:
                self.cancel()
                raise

    def _plan_and_start(
        self, plan: Callable, duration: Callable, action: Callable, *, guard=None
    ) -> None:
        """Own point planning through target admission, without locking out stop.

        Cancellation during IK must not be cleared by starting a later timer.
        Handoff to the timer is atomic with target admission; failed/cancelled
        planning releases busy state only after the planner has unwound.
        """
        with self._lock:
            self._require_admission()
            if guard is not None:
                guard()
            if self.is_moving():
                raise StateError("Cannot start new motion while arm is moving")
            self._stream_active = True
            self._begin_worker()
            self._generation += 1
            self._cancel_event.clear()
            self._event.clear()
        started = False
        try:
            if self._wait_cancelled(0):
                return
            target = plan()
            with self._lock:
                if self._wait_cancelled(0):
                    return
                seconds = duration(target)
                _validate_timeout(seconds, name="duration")
                if self._wait_cancelled(0):
                    return
                action(target)
                if self._wait_cancelled(0):
                    return
                self._stream_active = False
                self.start(seconds)
                started = True
        except BaseException as error:
            self._worker_error = error
            raise
        finally:
            with self._lock:
                self._stream_active = False
                if not started:
                    self._event.set()
                self._worker_ident = None
                self._worker_done.set()

    @contextmanager
    def _stream_scope(self, *, guard=None):
        """Reserve motion ownership without starting an estimated countdown.

        Planning/cleanup stay busy even after cancellation, so another motion
        cannot replace their temporary parameters before the owner unwinds.
        The stop caller never waits for planning to finish to set cancellation.
        """
        with self._lock:
            self._require_admission()
            if guard is not None:
                guard()
            if self.is_moving():
                raise StateError("Cannot start new motion while arm is moving")
            self._stream_active = True
            self._begin_worker()
            self._generation += 1
            self._cancel_event.clear()
            self._event.clear()
            active = self._external_cancel is None or not self._external_cancel.is_set()
            if not active:
                self._cancel_event.set()
        try:
            yield active
        except BaseException as error:
            self._worker_error = error
            raise
        finally:
            with self._lock:
                self._stream_active = False
                self._event.set()
                self._worker_ident = None
                self._worker_done.set()

    @contextmanager
    def _cancellation_scope(self, cancel_event: threading.Event):
        with self._lock:
            if self._external_cancel is not None:
                raise RuntimeError("A cancellable home is already running")
            self._external_cancel = cancel_event
        try:
            yield
        finally:
            with self._lock:
                self._external_cancel = None

    def _require_admission(self):
        if self._closing and not self._is_worker_thread():
            raise StateError(
                "Arm connection is closing or closed; new motion is unavailable"
            )


class CancellableHomeMixin:
    def home_cancellable(self, cancel_event: threading.Event) -> None:
        """Run home with cooperative cancellation for lifecycle integrations.

        Cancellation ends waiting and prevents new host-streamed targets. The
        caller must also call emergency_stop() or disable() to stop the motors;
        already accepted CAN frames keep their existing FIFO order.
        """
        with self._motion_timer._cancellation_scope(cancel_event):
            self.home(blocking=True)


def _validate_timeout(value: float, *, name: str) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ValueError(f"{name} must be a finite, non-negative number")
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be a finite, non-negative number")
