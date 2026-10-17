"""Short send intervals must survive coarse clocks and Windows lock waits."""

import ctypes
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import can
import pytest
from linkerbot_transport import SendReceipt

from linkerbot_arm._transport import can as managed
from linkerbot_arm._transport import pacing
from linkerbot_arm.comm.can import can as legacy


@pytest.mark.parametrize("kind", ("managed", "legacy"))
@pytest.mark.parametrize("send_duration_s", (0.0, 0.0001, 0.001))
def test_coarse_monotonic_tick_does_not_shorten_send_interval(
    monkeypatch, kind, send_duration_s
):
    elapsed = [0.0156]
    clock = SimpleNamespace(
        # Cross a 1/64-second tick during an otherwise short physical send.
        monotonic=lambda: int(elapsed[0] * 64) / 64,
        perf_counter=lambda: elapsed[0],
    )
    waits = []

    def send(*args):
        elapsed[0] += send_duration_s
        return SendReceipt.completed()

    def wait(seconds):
        waits.append(seconds)
        elapsed[0] += seconds

    interval = 0.0003
    monkeypatch.setattr(pacing, "time", clock)
    monkeypatch.setattr(pacing, "sys", SimpleNamespace(platform="linux"))
    stopped = SimpleNamespace(wait=wait, is_set=lambda: False)
    if kind == "managed":
        monkeypatch.setattr(managed, "time", clock)
        dispatcher = SimpleNamespace(
            SEND_INTERVAL_S=interval,
            _lock=threading.RLock(),
            _require_running=Mock(),
            _transport=SimpleNamespace(send=send),
            _stopped=stopped,
        )

        def dispatch(message):
            return managed.ManagedCANDispatcher._dispatch(dispatcher, message)
    else:
        monkeypatch.setattr(legacy, "time", clock)
        monkeypatch.setattr(legacy._TransportDispatcher, "send_now", send)
        dispatcher = legacy.CANMessageDispatcher.__new__(legacy.CANMessageDispatcher)
        dispatcher._tx_stopped = stopped
        interval = dispatcher.SEND_INTERVAL_S
        dispatch = dispatcher._dispatch_frame

    started = elapsed[0]
    dispatch(can.Message(arbitration_id=61))
    assert elapsed[0] - started >= interval - 1e-12
    expected = [interval - send_duration_s] if send_duration_s < interval else []
    assert waits == pytest.approx(expected)


@pytest.mark.parametrize(
    ("platform", "version", "expected"),
    (
        ("linux", (3, 10), "event"),
        ("darwin", (3, 13), "event"),
        ("win32", (3, 10), "native"),
        ("win32", (3, 11), "sleep"),
        ("win32", (3, 13), "sleep"),
    ),
)
def test_send_wait_uses_high_resolution_windows_wait(
    monkeypatch, platform, version, expected
):
    elapsed = [1.0]
    calls = []

    def coarse_event_wait(seconds):
        calls.append(("event", seconds))
        elapsed[0] += max(seconds, 1 / 64)

    def sleep(seconds):
        calls.append(("sleep", seconds))
        elapsed[0] += seconds

    def native_wait(deadline):
        calls.append(("native", deadline - elapsed[0]))
        elapsed[0] = deadline

    monkeypatch.setattr(
        pacing, "time", SimpleNamespace(perf_counter=lambda: elapsed[0], sleep=sleep)
    )
    monkeypatch.setattr(
        pacing, "sys", SimpleNamespace(platform=platform, version_info=version)
    )
    monkeypatch.setattr(pacing, "_wait_windows_timer", native_wait)
    stopped = SimpleNamespace(is_set=lambda: False, wait=coarse_event_wait)
    pacing.wait_for_send_deadline(1.0003, stopped)

    assert len(calls) == 1
    assert calls[0][0] == expected
    assert calls[0][1] == pytest.approx(0.0003)
    if platform == "win32":
        # Falling back to Event.wait would turn a 300 us period into 15.6 ms.
        assert elapsed[0] - 1.0 == pytest.approx(0.0003)


@pytest.mark.parametrize(
    ("remaining", "stopped_already"),
    ((-0.001, False), (0.0, False), (-0.001, True), (0.0, True), (0.0003, True)),
)
def test_expired_or_stopped_wait_does_not_block(
    monkeypatch, remaining, stopped_already
):
    sleep = Mock()
    native_wait = Mock()
    event_wait = Mock()
    monkeypatch.setattr(
        pacing, "time", SimpleNamespace(perf_counter=lambda: 1.0, sleep=sleep)
    )
    monkeypatch.setattr(pacing, "_wait_windows_timer", native_wait)
    stopped = SimpleNamespace(is_set=lambda: stopped_already, wait=event_wait)
    pacing.wait_for_send_deadline(1.0 + remaining, stopped)

    sleep.assert_not_called()
    native_wait.assert_not_called()
    event_wait.assert_not_called()


@pytest.fixture
def windows_timer_api(monkeypatch):
    """Exercise native error/ownership branches on either host OS."""
    kernel32 = SimpleNamespace(
        CreateWaitableTimerExW=Mock(return_value=123),
        SetWaitableTimer=Mock(return_value=True),
        WaitForSingleObject=Mock(return_value=0),
        CloseHandle=Mock(return_value=True),
    )
    windows_ctypes = SimpleNamespace(
        WinDLL=Mock(return_value=kernel32),
        c_void_p=ctypes.c_void_p,
        c_longlong=ctypes.c_longlong,
        POINTER=ctypes.POINTER,
        byref=ctypes.byref,
        get_last_error=Mock(return_value=5),
        WinError=Mock(side_effect=lambda code: OSError(code, "native timer failure")),
    )
    monkeypatch.setattr(pacing, "ctypes", windows_ctypes)
    monkeypatch.setattr(pacing, "time", SimpleNamespace(perf_counter=lambda: 0.0))
    return kernel32, windows_ctypes


def test_windows_timer_uses_high_resolution_relative_due_time(windows_timer_api):
    kernel32, windows_ctypes = windows_timer_api
    pacing._wait_windows_timer(0.00030001)

    windows_ctypes.WinDLL.assert_called_once_with("kernel32", use_last_error=True)
    kernel32.CreateWaitableTimerExW.assert_called_once_with(None, None, 0x2, 0x100002)
    assert kernel32.CreateWaitableTimerExW.restype is ctypes.c_void_p
    arguments = kernel32.SetWaitableTimer.call_args.args
    assert arguments[0] == 123
    assert arguments[1]._obj.value == -3001
    assert arguments[2:] == (0, None, None, False)
    kernel32.WaitForSingleObject.assert_called_once_with(123, 0xFFFFFFFF)
    kernel32.CloseHandle.assert_called_once_with(123)


def test_windows_timer_counts_creation_time_and_closes_expired_timer(
    monkeypatch, windows_timer_api
):
    kernel32, _ = windows_timer_api
    monkeypatch.setattr(pacing, "time", SimpleNamespace(perf_counter=lambda: 0.001))
    pacing._wait_windows_timer(0.0003)

    kernel32.SetWaitableTimer.assert_not_called()
    kernel32.WaitForSingleObject.assert_not_called()
    kernel32.CloseHandle.assert_called_once_with(123)


def test_windows_timer_creation_failure_is_not_silently_ignored(windows_timer_api):
    kernel32, _ = windows_timer_api
    kernel32.CreateWaitableTimerExW.return_value = None
    with pytest.raises(OSError, match="native timer failure"):
        pacing._wait_windows_timer(0.0003)

    kernel32.SetWaitableTimer.assert_not_called()
    kernel32.CloseHandle.assert_not_called()


@pytest.mark.parametrize(
    ("operation", "result", "expected"),
    (
        ("SetWaitableTimer", False, "native timer failure"),
        ("WaitForSingleObject", 0xFFFFFFFF, "native timer failure"),
        ("WaitForSingleObject", 0x102, "Unexpected waitable timer result: 0x102"),
        ("CloseHandle", False, "native timer failure"),
    ),
)
def test_windows_timer_closes_handle_after_native_error(
    windows_timer_api, operation, result, expected
):
    kernel32, _ = windows_timer_api
    getattr(kernel32, operation).return_value = result
    with pytest.raises(OSError, match=expected):
        pacing._wait_windows_timer(0.0003)

    kernel32.CloseHandle.assert_called_once_with(123)
    if operation == "SetWaitableTimer":
        kernel32.WaitForSingleObject.assert_not_called()


def test_windows_timer_interrupt_releases_handle(windows_timer_api):
    kernel32, _ = windows_timer_api
    kernel32.WaitForSingleObject.side_effect = KeyboardInterrupt
    with pytest.raises(KeyboardInterrupt):
        pacing._wait_windows_timer(0.0003)

    kernel32.CloseHandle.assert_called_once_with(123)


def test_windows_timer_cleanup_error_preserves_primary_error(windows_timer_api, caplog):
    kernel32, windows_ctypes = windows_timer_api
    kernel32.SetWaitableTimer.return_value = False
    kernel32.CloseHandle.return_value = False
    primary = OSError(5, "set timer failed")
    cleanup = OSError(6, "close timer failed")
    errors = iter((primary, cleanup))
    windows_ctypes.WinError.side_effect = lambda code: next(errors)
    with pytest.raises(OSError, match="set timer failed") as raised:
        pacing._wait_windows_timer(0.0003)

    assert raised.value is primary
    kernel32.CloseHandle.assert_called_once_with(123)
    assert "Failed to close send-pacing timer" in caplog.text
    assert "close timer failed" in caplog.text
