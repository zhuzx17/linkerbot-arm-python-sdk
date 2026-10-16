"""Thread-safe data relay: cache + blocking wait + event sink."""

import math
import threading
import time
from collections.abc import Callable
from typing import Generic, TypeVar, cast

from linkerbot_arm.exceptions import TimeoutError, ValidationError

T = TypeVar("T")

_SENTINEL = object()


class DataRelay(Generic[T]):
    """Thread-safe data relay: cache + blocking wait + event sink.

    Combines three responsibilities shared by all sensor managers:
    1. Cache the latest data value (snapshot)
    2. Blocking wait with timeout for the next value
    3. Event sink callback for streaming
    """

    def __init__(self) -> None:
        self._latest: T | None = None
        self._waiters: list[tuple[threading.Event, dict[str, object]]] = []
        self._lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._sink: Callable[[T], None] | None = None
        self._notifications: dict[threading.Event, Callable[[], None]] = {}

    def snapshot(self) -> T | None:
        """Return the most recently pushed value, or None."""
        with self._lock:
            return self._latest

    def wait(self, timeout_s: float) -> T:
        """Block until the next push() call, or raise TimeoutError."""
        self._validate_timeout(timeout_s)
        return self._wait_for(self._register_waiter(), timeout_s)

    def request(self, send: Callable[[], object], timeout_s: float) -> T:
        """Register a waiter, send one request, then wait for its response.

        Requests on one relay are serialized; lock and reply waits share one
        monotonic budget. ``send`` must return promptly. Registering before it
        also captures synchronous replies, including a zero-timeout request.
        """
        if not callable(send):
            raise ValidationError("send must be callable")
        self._validate_timeout(timeout_s)
        deadline = time.monotonic() + timeout_s
        if not self._request_lock.acquire(
            timeout=min(timeout_s, threading.TIMEOUT_MAX)
        ):
            raise TimeoutError(
                f"Request ownership not available within {timeout_s * 1000:.0f}ms"
            )
        try:
            if timeout_s > 0 and time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Request ownership not available within {timeout_s * 1000:.0f}ms"
                )
            waiter = self._register_waiter()
            send_completed = False
            try:
                send()
                send_completed = True
            finally:
                if not send_completed:
                    self._discard_waiter(waiter)
            return self._wait_for(waiter, max(0.0, deadline - time.monotonic()))
        finally:
            self._request_lock.release()

    def try_request(self, send: Callable[[], object], notify: Callable[[], None]):
        """Register before send, without blocking a periodic scheduler.

        Return None when a synchronous request already owns this relay. The
        returned request must be consumed or cancelled to release ownership.
        """
        if not self._request_lock.acquire(blocking=False):
            return None
        waiter = self._register_notified_waiter(notify)
        try:
            send()
        except BaseException:
            self._discard_waiter(waiter)
            self._request_lock.release()
            raise
        return _PendingRequest(self, waiter)

    @staticmethod
    def _validate_timeout(timeout_s: float) -> None:
        if (
            not isinstance(timeout_s, (int, float))
            or isinstance(timeout_s, bool)
            or not math.isfinite(timeout_s)
            or timeout_s < 0
        ):
            raise ValidationError("timeout_s must be a finite, non-negative number")

    def _register_waiter(self) -> tuple[threading.Event, dict[str, object]]:
        return self._register_notified_waiter(None)

    def _register_notified_waiter(
        self, notify
    ) -> tuple[threading.Event, dict[str, object]]:
        event = threading.Event()
        result_holder: dict[str, object] = {"data": _SENTINEL}
        waiter = (event, result_holder)

        with self._lock:
            self._waiters.append(waiter)
            if notify is not None:
                self._notifications[event] = notify
        return waiter

    def _wait_for(
        self,
        waiter: tuple[threading.Event, dict[str, object]],
        timeout_s: float,
    ) -> T:
        event, result_holder = waiter
        if event.wait(min(timeout_s, threading.TIMEOUT_MAX)):
            if result_holder["data"] is _SENTINEL:
                raise TimeoutError(f"No data received within {timeout_s * 1000:.0f}ms")
            return cast(T, result_holder["data"])
        self._discard_waiter(waiter)
        raise TimeoutError(f"No data received within {timeout_s * 1000:.0f}ms")

    def _discard_waiter(
        self, waiter: tuple[threading.Event, dict[str, object]]
    ) -> None:
        with self._lock:
            if waiter in self._waiters:
                self._waiters.remove(waiter)
            self._notifications.pop(waiter[0], None)

    def push(self, data: T) -> None:
        """Update cache, wake all waiters, and notify sink."""
        with self._lock:
            self._latest = data
            for event, result_holder in self._waiters:
                result_holder["data"] = data
                event.set()
            self._waiters.clear()
            notifications = tuple(self._notifications.values())
            self._notifications.clear()

        for notify in notifications:
            notify()

        if self._sink is not None:
            self._sink(data)

    def set_sink(self, sink: Callable[[T], None] | None) -> None:
        """Set the event sink callback."""
        self._sink = sink


class _PendingRequest:
    def __init__(self, relay, waiter):
        self._relay, self._waiter = relay, waiter
        self._released = False

    @property
    def done(self):
        return self._waiter[0].is_set()

    def cancel(self):
        if not self._released:
            self._relay._discard_waiter(self._waiter)
            self._released = True
            self._relay._request_lock.release()

    def result(self):
        try:
            return self._relay._wait_for(self._waiter, 0)
        finally:
            self.cancel()
