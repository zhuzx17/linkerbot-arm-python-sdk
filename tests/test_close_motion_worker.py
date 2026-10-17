"""The shared closer waits for worker restoration, including after cancel."""

import json
import subprocess
import sys
import textwrap
import threading
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import pytest

from linkerbot_arm._transport import close_arm
from linkerbot_arm.exceptions import StateError
from linkerbot_arm.motion_timer import MotionTimer


def fake_arm():
    timer = MotionTimer()
    profiles = [0.5, 1.0]
    stopped = []
    arm = SimpleNamespace(
        _motion_timer=timer,
        _closed=False,
        _resources_closed=False,
        _can_dispatcher=SimpleNamespace(stop=lambda: stopped.append(list(profiles))),
        wait_motion_done=timer.wait_done,
        stop_polling=lambda: None,
    )
    return arm, profiles, stopped


@pytest.mark.parametrize("cancel", [False, True])
def test_shared_close_waits_for_worker_cleanup(cancel):
    arm, profiles, stopped = fake_arm()
    timer = arm._motion_timer
    entered, resume, closing = threading.Event(), threading.Event(), threading.Event()
    wait = timer._wait_worker_done

    def observed(**kwargs):
        closing.set()
        return wait(**kwargs)

    timer._wait_worker_done = observed

    def work():
        with timer._stream_scope():
            profiles[:] = [50, 50]
            entered.set()
            try:
                assert resume.wait(2)
            finally:
                profiles[:] = [0.5, 1.0]

    with ThreadPoolExecutor(max_workers=3) as pool:
        worker = pool.submit(work)
        try:
            assert entered.wait(1)
            if cancel:
                timer.cancel()
            closer = pool.submit(close_arm, arm)
            assert closing.wait(1)
            second = pool.submit(close_arm, arm)
            assert not closer.done() and not stopped
            with pytest.raises(StateError, match="closing"):
                timer._start_with_target(lambda: 0, lambda: pytest.fail("new target"))
        finally:
            resume.set()
        worker.result(2)
        closer.result(2)
        second.result(2)
    assert stopped == [[0.5, 1.0]]
    assert arm._closed and arm._resources_closed


def test_shared_close_preserves_worker_cleanup_error():
    arm, _, stopped = fake_arm()
    timer = arm._motion_timer
    entered, resume, closing = threading.Event(), threading.Event(), threading.Event()
    original = RuntimeError("restore failed")
    wait = timer._wait_worker_done

    def observed(**kwargs):
        closing.set()
        return wait(**kwargs)

    timer._wait_worker_done = observed

    def work():
        with timer._stream_scope():
            entered.set()
            assert resume.wait(2)
            raise original

    with ThreadPoolExecutor(max_workers=2) as pool:
        worker = pool.submit(work)
        try:
            assert entered.wait(1)
            closer = pool.submit(close_arm, arm)
            assert closing.wait(1)
        finally:
            resume.set()
        assert worker.exception(2) is original
        assert closer.exception(2) is original
    assert len(stopped) == 1 and arm._resources_closed


def test_shared_close_from_own_worker_is_rejected_without_closing():
    arm, _, stopped = fake_arm()
    with arm._motion_timer._stream_scope():
        with pytest.raises(StateError, match="active motion worker"):
            close_arm(arm)
        assert not arm._closed and not stopped
    close_arm(arm)
    assert arm._resources_closed


def test_polling_failure_still_closes_connection():
    arm, _, stopped = fake_arm()
    original = RuntimeError("polling cleanup failed")

    def fail():
        raise original

    arm.stop_polling = fail
    with pytest.raises(RuntimeError) as caught:
        close_arm(arm)
    assert caught.value is original
    assert len(stopped) == 1 and arm._resources_closed


@pytest.mark.parametrize("kind", [KeyboardInterrupt, RuntimeError])
def test_repeated_interrupts_preserve_first_exception_until_cleanup(kind, monkeypatch):
    from unittest.mock import Mock

    arm, _, stopped = fake_arm()
    original = kind("first failure")
    wait = Mock(side_effect=[original, KeyboardInterrupt(), SystemExit(), None])
    monkeypatch.setattr(arm._motion_timer, "_wait_worker_done", wait)
    with pytest.raises(kind) as caught:
        close_arm(arm)
    assert caught.value is original
    assert wait.call_count == 4
    assert len(stopped) == 1 and arm._resources_closed


@pytest.mark.parametrize("variant", ["shared", "h75"])
@pytest.mark.parametrize("interrupt_source", ["native", "pending"])
def test_two_real_sigints_defer_bus_close_until_worker_restoration(
    variant, interrupt_source
):
    # Exercise native control events and pending Python interrupts in a child;
    # mocked exceptions alone do not verify the blocking Event.wait path.
    program = textwrap.dedent("""
        import _thread
        import json
        import os
        import signal
        import sys
        import threading
        from types import SimpleNamespace
        from linkerbot_arm import H75
        from linkerbot_arm._transport import close_arm
        from linkerbot_arm.motion_timer import MotionTimer

        signal.signal(signal.SIGINT, signal.default_int_handler)
        if sys.platform == 'win32' and sys.argv[2] == 'native':
            import ctypes
            # Runners can pass down an ignored Ctrl+C attribute. Installing a
            # Python handler does not clear that Win32 console attribute.
            kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
            kernel32.SetConsoleCtrlHandler.argtypes = [ctypes.c_void_p, ctypes.c_int]
            kernel32.SetConsoleCtrlHandler.restype = ctypes.c_int
            if not kernel32.SetConsoleCtrlHandler(None, False):
                raise ctypes.WinError(ctypes.get_last_error())
        timer = MotionTimer()
        entered, release = threading.Event(), threading.Event()
        first, second, continuation = (
            threading.Event(), threading.Event(), threading.Event()
        )
        events = []
        count = [0]
        original_wait = timer._wait_worker_done
        original_error = None

        def send_interrupt():
            if sys.argv[2] == 'pending':
                _thread.interrupt_main()
            elif sys.platform == 'win32':
                # This child has its own console, so Ctrl+C cannot reach pytest.
                os.kill(0, signal.CTRL_C_EVENT)
            else:
                os.kill(os.getpid(), signal.SIGINT)

        def stop_bus():
            events.append(['bus_closed', timer._worker_done.is_set()])
            continuation.set()

        arm = SimpleNamespace(
            _motion_timer=timer, _closed=False, _resources_closed=False,
            _close_lock=threading.Lock(), wait_motion_done=timer.wait_done,
            stop_polling=lambda: None,
            _can_dispatcher=SimpleNamespace(stop=stop_bus),
        )

        def observed_wait(**kwargs):
            count[0] += 1
            if count[0] == 1:
                first.set()
            elif count[0] == 2:
                second.set()
            else:
                continuation.set()
            return original_wait(**kwargs)

        timer._wait_worker_done = observed_wait

        def worker():
            with timer._stream_scope():
                entered.set()
                assert release.wait(5)
                events.append(['restore', arm._closed])

        def interrupt():
            assert first.wait(2)
            send_interrupt()
            assert second.wait(2)
            send_interrupt()
            assert continuation.wait(2)
            release.set()

        motion = threading.Thread(target=worker)
        signaller = threading.Thread(target=interrupt)
        motion.start()
        assert entered.wait(1)
        signaller.start()
        try:
            if sys.argv[1] == 'h75':
                H75.close(arm)
            else:
                close_arm(arm)
        except KeyboardInterrupt:
            original_error = 'KeyboardInterrupt'
        finally:
            release.set()
            motion.join(3)
            signaller.join(3)
        assert not motion.is_alive() and not signaller.is_alive()
        print(json.dumps([events, original_error, arm._resources_closed]))
    """)
    options = {}
    if sys.platform == "win32":
        # A separate console isolates real control events from the runner and
        # supplies a console even when pytest itself has none attached.
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = subprocess.SW_HIDE
        options = {
            "creationflags": subprocess.CREATE_NEW_CONSOLE,
            "startupinfo": startup,
        }
    result = subprocess.run(
        [sys.executable, "-c", program, variant, interrupt_source],
        capture_output=True,
        text=True,
        timeout=10,
        **options,
    )
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout) == [
        [["restore", False], ["bus_closed", True]],
        "KeyboardInterrupt",
        True,
    ]
