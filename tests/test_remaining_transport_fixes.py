"""Bounded read ownership, valid A7 replies and serialized relay creation."""

import struct
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import can
import pytest

from linkerbot_arm.arm.a7 import motor as motor_module
from linkerbot_arm.arm.a7.motor import A7Motor, InternalSensorType, SensorType
from linkerbot_arm.exceptions import TimeoutError
from linkerbot_arm.relay import DataRelay
from tests.test_transport_frames import FakeDispatcher


def position_reply(value=1.25):
    return can.Message(
        arbitration_id=61,
        is_extended_id=False,
        data=bytes([SensorType.POSITION]) + struct.pack("<f", value),
    )


@pytest.mark.parametrize(
    "invalid",
    [
        "id",
        "extended",
        "remote",
        "error",
        "fd",
        "echo",
        "short",
        "long",
        "dlc",
        "nan",
        "inf",
    ],
)
def test_a7_rejects_bad_reply_and_accepts_next_valid_one(invalid):
    dispatcher = FakeDispatcher()
    motor = A7Motor(61, dispatcher)
    message = position_reply()
    if invalid == "id":
        message.arbitration_id += 1
    elif invalid == "extended":
        message.is_extended_id = True
    elif invalid in ("remote", "error"):
        setattr(message, f"is_{invalid}_frame", True)
    elif invalid == "fd":
        message.is_fd = True
    elif invalid == "echo":
        message.data = message.data[:1]
        message.dlc = 1
    elif invalid == "short":
        message.data = message.data[:4]
        message.dlc = 4
    elif invalid == "long":
        message.data += b"\x00" * 4
        message.dlc = 9
    elif invalid == "dlc":
        message.dlc = 1
    else:
        message = position_reply(float(invalid))
    dispatcher.send = lambda request: dispatcher.feedback(message)
    with pytest.raises(TimeoutError):
        motor._read_register_float(SensorType.POSITION, timeout_s=0)
    valid = position_reply()
    valid.data += b"\x00" * 3
    valid.dlc = 8
    dispatcher.send = lambda request: dispatcher.feedback(valid)
    assert motor._read_register_float(SensorType.POSITION, timeout_s=0) == 1.25


def test_a7_handshake_request_echo_does_not_prove_alive():
    dispatcher = FakeDispatcher()
    motor = A7Motor(61, dispatcher)
    dispatcher.send = lambda request: dispatcher.feedback(request)
    assert not motor.check_alive(timeout_s=0)
    dispatcher.send = lambda request: dispatcher.feedback(
        can.Message(
            arbitration_id=61,
            is_extended_id=False,
            data=[InternalSensorType.HANDSHAKE, 1],
        )
    )
    assert motor.check_alive(timeout_s=0)


def test_a7_first_concurrent_reads_share_one_relay(monkeypatch):
    motor = A7Motor(61, FakeDispatcher())
    original = DataRelay
    count = []
    barrier = threading.Barrier(8)

    def slow_constructor():
        count.append(True)
        time.sleep(0.01)
        return original()

    monkeypatch.setattr(motor_module, "DataRelay", slow_constructor)

    def get():
        barrier.wait(timeout=2)
        return motor._get_relay(SensorType.POSITION)

    with ThreadPoolExecutor(max_workers=8) as pool:
        relays = list(pool.map(lambda _: get(), range(8)))
    assert len(count) == 1
    assert all(relay is relays[0] for relay in relays)


def test_relay_timeout_covers_a_pending_periodic_request():
    relay = DataRelay()
    pending = relay.try_request(lambda: None, lambda: None)
    done = threading.Event()
    errors = []
    sent = []

    def read():
        try:
            relay.request(lambda: sent.append(True), 0.02)
        except Exception as error:
            errors.append(error)
        finally:
            done.set()

    thread = threading.Thread(target=read, daemon=True)
    thread.start()
    try:
        assert done.wait(0.4), "the ownership lock ignored the read deadline"
        assert len(errors) == 1 and isinstance(errors[0], TimeoutError)
        assert not sent
    finally:
        pending.cancel()
        thread.join(1)
    assert not thread.is_alive()
    assert relay.request(lambda: relay.push(42), 0) == 42


def test_zero_timeout_busy_relay_does_not_send():
    relay = DataRelay()
    pending = relay.try_request(lambda: None, lambda: None)
    try:
        with pytest.raises(TimeoutError):
            relay.request(lambda: pytest.fail("timed-out request sent"), 0)
    finally:
        pending.cancel()


def test_lock_and_send_consume_the_reply_budget(monkeypatch):
    relay = DataRelay()
    clock = [0.0]
    released = []
    budgets = []

    class TimedLock:
        def acquire(self, timeout):
            clock[0] += 0.75
            return True

        def release(self):
            released.append(True)

    relay._request_lock = TimedLock()
    monkeypatch.setattr("linkerbot_arm.relay.time.monotonic", lambda: clock[0])

    def send():
        clock[0] += 0.1
        relay.push(42)

    original_wait = relay._wait_for

    def wait(waiter, budget):
        budgets.append(budget)
        return original_wait(waiter, budget)

    monkeypatch.setattr(relay, "_wait_for", wait)
    assert relay.request(send, 1) == 42
    assert budgets == pytest.approx([0.15])
    assert released == [True] and not relay._waiters
