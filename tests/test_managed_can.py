"""Exercise CANTransport through real transport lifecycle and queues."""

import queue
import threading
import time
import uuid

import can
import pytest
from linkerbot_transport import TransportState
from linkerbot_transport.can import CANTransport
from linkerbot_transport.testing import FakeDispatcher

from linkerbot_arm._transport import can as managed
from linkerbot_arm.exceptions import CANError, TimeoutError


def wait_until(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "condition did not become true"
        time.sleep(0.001)


@pytest.fixture
def connection(monkeypatch):
    backends = []
    adapters = []

    def transport(name, backend, **kwargs):
        def create(on_error):
            dispatcher = FakeDispatcher(on_error)
            backends.append(dispatcher)
            return dispatcher

        return CANTransport(name, backend, dispatcher_factory=create, **kwargs)

    monkeypatch.setattr(managed, "CANTransport", transport)

    def create(model="a7", side="left", on_error=None):
        adapter = managed.ManagedCANDispatcher(
            "virtual", "virtual", model, side, on_error
        )
        adapters.append(adapter)
        return adapter, backends[-1]

    yield create
    for adapter in adapters:
        adapter.stop()
        assert not adapter._recv_thread.is_alive()
        assert not adapter._tx.get_health().running


@pytest.mark.parametrize("model", ["a7", "a7_lite", "p7"])
@pytest.mark.parametrize("side,mid", [("left", 61), ("right", 51)])
def test_routes_preserve_original_frame_identity(connection, model, side, mid):
    adapter, backend = connection(model, side)
    received = []
    adapter.subscribe(received.append)
    adapter.subscribe(received.append)  # duplicate/fresh bound-method identity
    extended = model == "a7_lite"
    identifier = (0x02FF0000 | mid << 8 | 0xFD) if extended else mid
    message = can.Message(
        arbitration_id=identifier, is_extended_id=extended, data=b"12345678"
    )
    backend.emit(message)
    wait_until(lambda: len(received) == 1)
    assert received == [message]
    assert received[0] is message
    assert adapter.can_interface.interface_name == "virtual"
    if model == "p7":
        flash = can.Message(arbitration_id=0x600 + mid + 6, is_extended_id=False)
        backend.emit(flash)
        wait_until(lambda: len(received) == 2)
        assert received[-1] is flash
    adapter.unsubscribe(received.append)
    backend.emit(message)
    wait_until(lambda: adapter._route.get_health().queue_depth == 0)
    assert len(received) == (2 if model == "p7" else 1)


def test_same_number_with_wrong_frame_format_and_other_motor_do_not_route(connection):
    adapter, backend = connection()
    received = []
    adapter.subscribe(received.append)
    backend.emit(can.Message(arbitration_id=61, is_extended_id=True))
    backend.emit(can.Message(arbitration_id=51, is_extended_id=False))
    assert adapter.get_health()[0].route_misses == 2
    assert received == []


def test_async_fifo_is_bounded_and_close_does_not_replay_pending_targets(
    connection, monkeypatch
):
    monkeypatch.setattr(managed.ManagedCANDispatcher, "SEND_QUEUE_SIZE", 2)
    adapter, backend = connection()
    entered, release = threading.Event(), threading.Event()
    original = backend.send_now

    def blocked(message):
        entered.set()
        assert release.wait(2)
        original(message)

    backend.send_now = blocked
    messages = [
        can.Message(arbitration_id=61, data=[n], is_extended_id=False) for n in range(4)
    ]
    try:
        assert adapter.send(messages[0]) is None
        assert entered.wait(1)
        assert adapter.send(messages[1]) is None
        assert adapter.send(messages[2]) is None
        with pytest.raises(queue.Full) as error:
            adapter.send(messages[3])
        assert error.value.args == ()
        closer = threading.Thread(target=adapter.stop)
        closer.start()
        assert adapter._stopped.wait(1)
        release.set()
        closer.join(2)
        assert not closer.is_alive()
        assert backend.sent == [messages[0]]
        with pytest.raises(RuntimeError, match="stopped"):
            adapter.send(messages[3])
        assert adapter.get_health()[0].state is TransportState.CLOSED
    finally:
        release.set()


def test_fifo_order_and_legacy_successful_send_spacing(connection):
    adapter, backend = connection()
    times = []
    original = backend.send_now

    def record(message):
        times.append(time.perf_counter())
        original(message)

    backend.send_now = record
    messages = [can.Message(arbitration_id=61, data=[n]) for n in range(12)]
    for message in messages:
        assert adapter.send(message) is None
    wait_until(lambda: len(backend.sent) == len(messages))
    assert backend.sent == messages
    intervals = [b - a for a, b in zip(times, times[1:])]
    minimum = adapter.SEND_INTERVAL_S * 0.8
    assert all(interval >= minimum for interval in intervals), {
        "intervals_s": intervals,
        "minimum_s": minimum,
    }


def test_rx_disconnect_reports_once_cleans_connection_and_rejects_more_commands(
    connection,
):
    faults = []
    adapter, backend = connection(on_error=faults.append)
    backend.disconnect(OSError("disconnected"))
    wait_until(lambda: not adapter._recv_thread.is_alive())
    assert len(faults) == 1
    assert isinstance(faults[0], CANError)
    assert adapter.get_health()[0].state is TransportState.CLOSED
    assert not adapter.get_health()[1].running
    with pytest.raises(CANError, match="disconnected"):
        adapter.send(can.Message(arbitration_id=61))
    adapter.stop()
    assert len(faults) == 1


def test_background_send_failures_are_observed_and_fault_after_original_threshold(
    connection,
):
    faults = []
    adapter, backend = connection(on_error=faults.append)

    def fail(message):
        raise OSError("write failed")

    backend.send_now = fail
    for _ in range(adapter.MAX_CONSECUTIVE_ERRORS):
        adapter.send(can.Message(arbitration_id=61))
    wait_until(lambda: not adapter._recv_thread.is_alive())
    assert len(faults) == 1
    assert isinstance(faults[0], CANError)
    assert adapter.get_health()[1].failed == adapter.MAX_CONSECUTIVE_ERRORS


def test_receive_overflow_becomes_visible_fault(connection):
    faults = []
    adapter, backend = connection(on_error=faults.append)
    entered, release = threading.Event(), threading.Event()

    def slow_callback(message):
        entered.set()
        assert release.wait(2)

    adapter.subscribe(slow_callback)
    message = can.Message(arbitration_id=61, is_extended_id=False)
    try:
        backend.emit(message)
        assert entered.wait(1)
        for _ in range(257):
            backend.emit(message)
        assert adapter.get_health()[0].rx_overflows == 1
        release.set()
        wait_until(lambda: not adapter._recv_thread.is_alive())
        assert "overflow" in str(faults[0])
    finally:
        release.set()


def test_close_failure_retains_transport_for_explicit_retry(connection):
    adapter, backend = connection()
    backend.stop_error = OSError("shutdown failed")
    with pytest.raises(CANError, match="shutdown failed"):
        adapter.stop()
    assert adapter.get_health()[0].state is TransportState.FAULTED
    adapter.stop()
    assert adapter.get_health()[0].state is TransportState.CLOSED


def test_concurrent_and_callback_close_finish_without_deadlock(connection):
    adapter, backend = connection()
    entered, release = threading.Event(), threading.Event()

    def callback(message):
        entered.set()
        assert release.wait(2)
        adapter.stop()

    adapter.subscribe(callback)
    backend.emit(can.Message(arbitration_id=61, is_extended_id=False))
    assert entered.wait(1)
    closers = [threading.Thread(target=adapter.stop) for _ in range(2)]
    for closer in closers:
        closer.start()
    release.set()
    for closer in closers:
        closer.join(2)
        assert not closer.is_alive()
    assert not adapter._recv_thread.is_alive()


def test_virtual_can_backend_sends_and_receives_real_transport_frames():
    name = "arm-managed-" + uuid.uuid4().hex
    adapter = managed.ManagedCANDispatcher(name, "virtual", "a7_lite", "left")
    peer = can.Bus(channel=name, interface="virtual")
    seen = threading.Event()
    received = []

    def receive(message):
        received.append(message)
        seen.set()

    adapter.subscribe(receive)
    try:
        command = can.Message(
            arbitration_id=0x0100013D, is_extended_id=True, data=b"12345678"
        )
        adapter.send(command)
        actual = peer.recv(1)
        assert actual.arbitration_id == command.arbitration_id
        assert actual.data == command.data
        feedback = can.Message(
            arbitration_id=0x027F3D00, is_extended_id=True, data=b"87654321"
        )
        peer.send(feedback)
        assert seen.wait(1)
        assert received[0].arbitration_id == feedback.arbitration_id
        assert received[0].data == feedback.data
    finally:
        adapter.stop()
        peer.shutdown()


def test_mapping_keeps_timeout_category():
    from linkerbot_transport.errors import TimeoutError as TransportTimeout

    assert isinstance(managed._mapped_error(TransportTimeout("timeout")), TimeoutError)
