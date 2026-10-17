import threading
from collections.abc import Callable
from types import SimpleNamespace
from typing import Any

import pytest
from linkerbot_transport.canfd import ctypes_backend as transport_backend

from linkerbot_arm.comm.canfd import CANFDConfigOptions, CANFDMessage
from linkerbot_arm.comm.canfd import ctypes_backend as backend
from linkerbot_arm.exceptions import CANError, ValidationError


class FakeFunction:
    def __init__(
        self,
        result: int = 0,
        hook: Callable[..., int] | None = None,
    ) -> None:
        self.result = result
        self.hook = hook
        self.calls: list[tuple[Any, ...]] = []
        self.restype: Any = None
        self.argtypes: list[Any] | None = None

    def __call__(self, *args: Any) -> int:
        self.calls.append(args)
        if self.hook is not None:
            return self.hook(*args)
        return self.result


class FakeLibrary:
    def __init__(self) -> None:
        self.LibCANbus_Init = FakeFunction(0)
        self.LibCANbus_Exit = FakeFunction(0)
        self.CAN_ScanDevice = FakeFunction(1)
        self.CAN_OpenDevice = FakeFunction(0)
        self.CAN_CloseDevice = FakeFunction(0)
        self.CAN_ReadDevInfo = FakeFunction(0)
        self.CANFD_Init = FakeFunction(0)
        self.CANFD_Transmit = FakeFunction(1)
        self.CANFD_Receive = FakeFunction(0)


def make_interface(
    monkeypatch: pytest.MonkeyPatch, lib: FakeLibrary, *, system: str = "Linux"
) -> backend.CANFDInterface:
    # Select the fake library's ABI explicitly, independently of the test host.
    monkeypatch.setattr(
        transport_backend, "platform", SimpleNamespace(system=lambda: system)
    )
    monkeypatch.setattr(transport_backend, "_load_library", lambda library_path: lib)
    return backend.CANFDInterface()


@pytest.mark.parametrize("system", ("Linux", "Windows"))
def test_concurrent_close_waits_for_io_device_close_and_lifecycle_release(
    monkeypatch: pytest.MonkeyPatch, system: str
) -> None:
    lib = FakeLibrary()
    send_entered = threading.Event()
    receive_entered = threading.Event()
    vendor_close_entered = threading.Event()
    lifecycle_exit_entered = threading.Event()
    allow_send = threading.Event()
    allow_receive = threading.Event()
    allow_vendor_close = threading.Event()
    allow_lifecycle_exit = threading.Event()

    def blocking_send(*args: Any) -> int:
        _ = args
        send_entered.set()
        if not allow_send.wait(timeout=2.0):
            raise TimeoutError("test did not release CANFD_Transmit")
        return 1

    def blocking_receive(*args: Any) -> int:
        _ = args
        receive_entered.set()
        if not allow_receive.wait(timeout=2.0):
            raise TimeoutError("test did not release CANFD_Receive")
        return 0

    def blocking_vendor_close(*args: Any) -> int:
        _ = args
        vendor_close_entered.set()
        if not allow_vendor_close.wait(timeout=2.0):
            raise TimeoutError("test did not release CAN_CloseDevice")
        return 0

    def blocking_lifecycle_exit(*args: Any) -> int:
        _ = args
        lifecycle_exit_entered.set()
        if not allow_lifecycle_exit.wait(timeout=2.0):
            raise TimeoutError("test did not release LibCANbus_Exit")
        return 0

    lib.CANFD_Transmit.hook = blocking_send
    lib.CANFD_Receive.hook = blocking_receive
    lib.CAN_CloseDevice.hook = blocking_vendor_close
    lib.LibCANbus_Exit.hook = blocking_lifecycle_exit
    interface = make_interface(monkeypatch, lib, system=system)

    errors: list[Exception] = []
    send_done = threading.Event()
    receive_done = threading.Event()
    first_close_done = threading.Event()
    second_close_done = threading.Event()

    def run(operation: Callable[[], object], done: threading.Event) -> None:
        try:
            operation()
        except Exception as error:
            errors.append(error)
        finally:
            done.set()

    message = CANFDMessage(arbitration_id=1, data=b"x")
    send_thread = threading.Thread(
        target=run,
        args=(lambda: interface.send(message), send_done),
        daemon=True,
    )
    receive_thread = threading.Thread(
        target=run,
        args=(interface.receive, receive_done),
        daemon=True,
    )
    send_thread.start()
    receive_thread.start()
    assert send_entered.wait(timeout=1.0)
    assert receive_entered.wait(timeout=1.0)

    first_close_thread = threading.Thread(
        target=run,
        args=(interface.close, first_close_done),
        daemon=True,
    )
    first_close_thread.start()
    assert interface._closed_event.wait(timeout=1.0)

    second_close_thread = threading.Thread(
        target=run,
        args=(interface.close, second_close_done),
        daemon=True,
    )
    second_close_thread.start()
    assert not second_close_done.wait(timeout=0.05)

    # The owner drains transmit before receive, and neither close caller may
    # return while either in-flight vendor call still owns its direction lock.
    allow_send.set()
    assert send_done.wait(timeout=1.0)
    assert not vendor_close_entered.wait(timeout=0.05)
    assert not second_close_done.is_set()

    allow_receive.set()
    assert receive_done.wait(timeout=1.0)
    assert vendor_close_entered.wait(timeout=1.0)
    assert not first_close_done.is_set()
    assert not second_close_done.is_set()

    # Linux also owns a process-level lifecycle; Windows owns only the channel.
    allow_vendor_close.set()
    if system == "Linux":
        assert lifecycle_exit_entered.wait(timeout=1.0)
        assert not first_close_done.is_set()
        assert not second_close_done.is_set()

    allow_lifecycle_exit.set()
    assert first_close_done.wait(timeout=1.0)
    assert second_close_done.wait(timeout=1.0)
    for thread in (
        send_thread,
        receive_thread,
        first_close_thread,
        second_close_thread,
    ):
        thread.join(timeout=1.0)
        assert not thread.is_alive()

    assert errors == []
    channel_args = (0,) if system == "Windows" else (0, 0)
    lifecycle_calls = int(system == "Linux")
    assert lib.CAN_CloseDevice.calls == [channel_args]
    assert len(lib.LibCANbus_Init.calls) == lifecycle_calls
    assert len(lib.LibCANbus_Exit.calls) == lifecycle_calls

    reopened = backend.CANFDInterface()
    assert len(lib.LibCANbus_Init.calls) == 2 * lifecycle_calls
    reopened.close()
    assert len(lib.CAN_CloseDevice.calls) == 2
    assert len(lib.LibCANbus_Exit.calls) == 2 * lifecycle_calls


@pytest.mark.parametrize("system", ("Linux", "Windows"))
def test_concurrent_close_waiter_is_notified_when_vendor_close_raises(
    monkeypatch: pytest.MonkeyPatch, system: str
) -> None:
    lib = FakeLibrary()
    vendor_close_entered = threading.Event()
    allow_vendor_close = threading.Event()

    def failing_vendor_close(*args: Any) -> int:
        _ = args
        vendor_close_entered.set()
        if not allow_vendor_close.wait(timeout=2.0):
            raise TimeoutError("test did not release CAN_CloseDevice")
        raise OSError("vendor close crashed")

    lib.CAN_CloseDevice.hook = failing_vendor_close
    interface = make_interface(monkeypatch, lib, system=system)
    first_errors: list[Exception] = []
    second_errors: list[Exception] = []
    first_done = threading.Event()
    second_done = threading.Event()

    def call_close(errors: list[Exception], done: threading.Event) -> None:
        try:
            interface.close()
        except Exception as error:
            errors.append(error)
        finally:
            done.set()

    first_thread = threading.Thread(
        target=call_close,
        args=(first_errors, first_done),
        daemon=True,
    )
    first_thread.start()
    assert vendor_close_entered.wait(timeout=1.0)

    second_thread = threading.Thread(
        target=call_close,
        args=(second_errors, second_done),
        daemon=True,
    )
    second_thread.start()
    assert not second_done.wait(timeout=0.05)

    allow_vendor_close.set()
    assert first_done.wait(timeout=1.0)
    assert second_done.wait(timeout=1.0)
    first_thread.join(timeout=1.0)
    second_thread.join(timeout=1.0)

    assert len(first_errors) == 1
    assert isinstance(first_errors[0], CANError)
    assert "vendor close crashed" in str(first_errors[0])
    assert len(second_errors) == 1
    assert isinstance(second_errors[0], CANError)
    channel_args = (0,) if system == "Windows" else (0, 0)
    assert lib.CAN_CloseDevice.calls == [channel_args, channel_args]
    assert len(lib.LibCANbus_Exit.calls) == 0
    assert interface._close_complete_event.is_set()
    with pytest.raises(CANError):
        interface.send(CANFDMessage(1, b"x"))
    lib.CAN_CloseDevice.hook = None
    interface.close()
    interface.close()
    assert len(lib.CAN_CloseDevice.calls) == 3
    assert len(lib.LibCANbus_Exit.calls) == int(system == "Linux")


@pytest.mark.parametrize("system", ("Linux", "Windows"))
@pytest.mark.parametrize(
    "operation", ["send", "send_batch", "receive", "read_dev_info"]
)
def test_vendor_failures_keep_arm_error_type(monkeypatch, operation, system):
    lib = FakeLibrary()
    interface = make_interface(monkeypatch, lib, system=system)
    lib.CANFD_Transmit.result = -1
    lib.CANFD_Receive.result = -1
    lib.CAN_ReadDevInfo.result = -1
    message = CANFDMessage(1, b"x")
    args = {"send": (message,), "send_batch": ([message, message],)}
    try:
        with pytest.raises(CANError) as caught:
            getattr(interface, operation)(*args.get(operation, ()))
        assert type(caught.value) is CANError
    finally:
        interface.close()


@pytest.mark.parametrize("system", ("Linux", "Windows"))
@pytest.mark.parametrize("operation", ["send", "send_batch", "receive"])
def test_vendor_input_validation_keeps_arm_error_type(monkeypatch, operation, system):
    interface = make_interface(monkeypatch, FakeLibrary(), system=system)
    message = CANFDMessage(1, b"x")
    args = {"send": (message,), "send_batch": ([message, message],)}
    try:
        with pytest.raises(ValidationError) as caught:
            getattr(interface, operation)(*args.get(operation, ()), timeout_ms=-1)
        assert type(caught.value) is ValidationError
    finally:
        interface.close()


@pytest.mark.parametrize("system", ("Linux", "Windows"))
def test_shared_vendor_lifecycle_keeps_transport_close_policy(monkeypatch, system):
    lib = FakeLibrary()
    lib.CAN_ScanDevice.result = 2
    interface = make_interface(monkeypatch, lib, system=system)
    other = transport_backend.CANFDInterface(
        **({"device_index": 1} if system == "Windows" else {"channel_index": 1})
    )
    try:
        assert len(lib.LibCANbus_Init.calls) == int(system == "Linux")
        lib.CAN_CloseDevice.result = -1
        with pytest.raises(CANError):
            interface.close()
        with pytest.raises(CANError):
            interface.close()
        assert len(lib.CAN_CloseDevice.calls) == 2
        assert len(lib.LibCANbus_Exit.calls) == 0
        from linkerbot_transport.errors import TransportError

        with pytest.raises(TransportError):
            other.close()
        assert len(lib.LibCANbus_Exit.calls) == 0
        lib.CAN_CloseDevice.result = 0
        other.close()
        assert len(lib.CAN_CloseDevice.calls) == 4
        assert len(lib.LibCANbus_Exit.calls) == 0
        interface.close()
        interface.close()
        assert len(lib.CAN_CloseDevice.calls) == 5
        assert len(lib.LibCANbus_Exit.calls) == int(system == "Linux")
    finally:
        lib.CAN_CloseDevice.result = 0
        interface.close()
        other.close()


@pytest.mark.parametrize("system", ("Linux", "Windows"))
def test_vendor_messages_and_config_keep_arm_identity(monkeypatch, system):
    import pickle

    lib = FakeLibrary()
    interface = make_interface(monkeypatch, lib, system=system)
    message = CANFDMessage(0x123, b"payload", timestamp=7)
    try:
        frame = interface._message_to_frame(message)
        restored = interface._frame_to_message(frame)
        assert type(restored) is CANFDMessage
        assert type(interface.config) is CANFDConfigOptions
        assert pickle.loads(pickle.dumps(restored)) == restored
        assert backend.CanFDMsg is transport_backend.CanFDMsg
        assert backend.CanFDConfig is transport_backend.CanFDConfig
        assert backend.DevInfo is transport_backend.DevInfo
    finally:
        interface.close()


def test_socketcan_models_errors_and_retryable_shutdown(monkeypatch, caplog):
    from unittest.mock import Mock

    import can
    from linkerbot_transport.canfd import socketcan_backend as transport_socket

    from linkerbot_arm.comm.canfd import SocketCANFDBackend

    bus = Mock()
    monkeypatch.setattr(
        transport_socket,
        "_read_link_state",
        lambda channel: transport_socket._LinkState(
            up=True, fd=True, bitrate=1000000, data_bitrate=5000000
        ),
    )
    monkeypatch.setattr(can, "Bus", lambda **kw: bus)
    interface = SocketCANFDBackend("vcan-test")
    bus.recv.side_effect = [
        can.Message(arbitration_id=1, data=b"payload", is_extended_id=True, is_fd=True),
        None,
    ]
    messages = interface.receive()
    assert messages == [CANFDMessage(1, b"payload", frame_type=4, timestamp=0)]
    assert type(messages[0]) is CANFDMessage
    bus.send.side_effect = can.CanError("send failed")
    with pytest.raises(CANError):
        interface.send(messages[0])
    with pytest.raises(ValidationError):
        interface.send(messages[0], timeout_ms=-1)
    bus.shutdown.side_effect = OSError("shutdown failed")
    interface.close()
    interface.close()
    assert bus.shutdown.call_count == 2
    assert "close can be retried" in caplog.text
    with pytest.raises(CANError):
        interface.receive()
    with pytest.raises(CANError):
        interface.send(messages[0])
    from linkerbot_transport.canfd.python_can_backend import PythonCANFDBackend
    from linkerbot_transport.errors import TransportError

    replacement = None
    try:
        with pytest.raises(TransportError, match="different settings"):
            PythonCANFDBackend("vcan-test", "socketcan", bitrate=500000)
        bus.shutdown.side_effect = None
        interface.close()
        interface.close()
        assert bus.shutdown.call_count == 3
        replacement = PythonCANFDBackend("vcan-test", "socketcan", bitrate=500000)
    finally:
        interface.close()
        if replacement is not None:
            replacement.close()


def test_vendor_exit_failure_retries_without_reclosing_device(monkeypatch):
    lib = FakeLibrary()
    interface = make_interface(monkeypatch, lib)
    try:
        lib.LibCANbus_Exit.result = -1
        with pytest.raises(CANError, match="LibCANbus_Exit"):
            interface.close()
        assert len(lib.CAN_CloseDevice.calls) == 1
        assert len(lib.LibCANbus_Exit.calls) == 1
        lib.LibCANbus_Exit.result = 0
        interface.close()
        interface.close()
        assert len(lib.CAN_CloseDevice.calls) == 1
        assert len(lib.LibCANbus_Exit.calls) == 2
    finally:
        lib.LibCANbus_Exit.result = 0
        interface.close()


def test_socketcan_deferred_receive_error_preserves_arm_frames(monkeypatch):
    from unittest.mock import Mock

    import can
    from linkerbot_transport.canfd import socketcan_backend as transport_socket

    from linkerbot_arm.comm.canfd import SocketCANFDBackend

    bus = Mock()
    monkeypatch.setattr(can, "Bus", lambda **kw: bus)
    monkeypatch.setattr(
        transport_socket,
        "_read_link_state",
        lambda channel: transport_socket._LinkState(
            up=True, fd=True, bitrate=1000000, data_bitrate=5000000
        ),
    )
    interface = SocketCANFDBackend("deferred-frame-test")
    try:
        bus.recv.side_effect = [
            can.Message(
                arbitration_id=1,
                data=b"valid",
                is_extended_id=True,
                is_fd=True,
                timestamp=0.125,
            ),
            can.CanError("receive failed"),
            None,
        ]
        frames = interface.receive()
        assert frames == [CANFDMessage(1, b"valid", frame_type=4, timestamp=125000)]
        assert type(frames[0]) is CANFDMessage
        with pytest.raises(CANError, match="receive failed"):
            interface.receive()
        assert bus.recv.call_count == 2
        assert interface.receive() == []
    finally:
        interface.close()
