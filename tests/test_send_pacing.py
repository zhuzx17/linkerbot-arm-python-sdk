"""Short send intervals must survive a coarse host monotonic clock tick."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import can
import pytest
from linkerbot_transport import SendReceipt

from linkerbot_arm._transport import can as managed
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
    if kind == "managed":
        monkeypatch.setattr(managed, "time", clock)
        dispatcher = SimpleNamespace(
            SEND_INTERVAL_S=interval,
            _lock=threading.RLock(),
            _require_running=Mock(),
            _transport=SimpleNamespace(send=send),
            _stopped=SimpleNamespace(wait=wait),
        )

        def dispatch(message):
            return managed.ManagedCANDispatcher._dispatch(dispatcher, message)
    else:
        monkeypatch.setattr(legacy, "time", clock)
        monkeypatch.setattr(legacy._TransportDispatcher, "send_now", send)
        dispatcher = legacy.CANMessageDispatcher.__new__(legacy.CANMessageDispatcher)
        dispatcher._tx_stopped = SimpleNamespace(wait=wait)
        interval = dispatcher.SEND_INTERVAL_S
        dispatch = dispatcher._dispatch_frame

    started = elapsed[0]
    dispatch(can.Message(arbitration_id=61))
    assert elapsed[0] - started >= interval - 1e-12
    assert waits == pytest.approx([max(0.0, interval - send_duration_s)])
