"""Pad a successful CAN send without Windows' coarse Event timeout."""

import ctypes
import logging
import math
import sys
import threading
import time
from ctypes import wintypes

logger = logging.getLogger(__name__)


def _wait_windows_timer(deadline: float) -> None:
    """Use a local high-resolution timer on CPython 3.10 / Windows 10 1803+."""
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateWaitableTimerExW.argtypes = (
        ctypes.c_void_p,
        wintypes.LPCWSTR,
        wintypes.DWORD,
        wintypes.DWORD,
    )
    kernel32.CreateWaitableTimerExW.restype = wintypes.HANDLE
    kernel32.SetWaitableTimer.argtypes = (
        wintypes.HANDLE,
        ctypes.POINTER(ctypes.c_longlong),
        wintypes.LONG,
        ctypes.c_void_p,
        ctypes.c_void_p,
        wintypes.BOOL,
    )
    kernel32.SetWaitableTimer.restype = wintypes.BOOL
    kernel32.WaitForSingleObject.argtypes = (wintypes.HANDLE, wintypes.DWORD)
    kernel32.WaitForSingleObject.restype = wintypes.DWORD
    kernel32.CloseHandle.argtypes = (wintypes.HANDLE,)
    kernel32.CloseHandle.restype = wintypes.BOOL
    # High-resolution, auto-reset timer; only modify and synchronization access.
    timer = kernel32.CreateWaitableTimerExW(None, None, 0x2, 0x100002)
    if not timer:
        raise ctypes.WinError(ctypes.get_last_error())
    primary_error = None
    try:
        # Creating the timer also consumes part of the successful-send period.
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        # A negative due time is relative, in 100 ns units. Round up to avoid
        # shortening the requested send interval.
        due_time = ctypes.c_longlong(-math.ceil(remaining * 10_000_000))
        if not kernel32.SetWaitableTimer(
            timer, ctypes.byref(due_time), 0, None, None, False
        ):
            raise ctypes.WinError(ctypes.get_last_error())
        result = kernel32.WaitForSingleObject(timer, 0xFFFFFFFF)
        if result == 0xFFFFFFFF:
            raise ctypes.WinError(ctypes.get_last_error())
        if result != 0:
            raise OSError(f"Unexpected waitable timer result: {result:#x}")
    except BaseException as error:
        primary_error = error
        raise
    finally:
        if not kernel32.CloseHandle(timer):
            error = ctypes.WinError(ctypes.get_last_error())
            if primary_error is None:
                raise error
            logger.error("Failed to close send-pacing timer", exc_info=error)


def wait_for_send_deadline(deadline: float, stopped: threading.Event) -> None:
    """Wait only for the remaining portion of the existing 300 us send period.

    Windows' condition/lock timeout can round this short wait to a scheduler
    tick. CPython 3.11+ sleep uses a high-resolution waitable timer; 3.10 needs
    an explicit one. Requested waits cover at most the remaining send period;
    OS scheduling can resume later. No global timer changes or CPU busy waits.
    """
    if stopped.is_set():
        return
    remaining = deadline - time.perf_counter()
    if remaining <= 0:
        return
    if sys.platform != "win32":
        stopped.wait(remaining)
    elif sys.version_info >= (3, 11):
        time.sleep(remaining)
    else:
        _wait_windows_timer(deadline)
