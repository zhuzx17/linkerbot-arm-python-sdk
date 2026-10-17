"""Cancellation, persistent software stops and bounded P7 targets, without CAN."""

import importlib
import math
import random
import struct
import threading
import time
from queue import Full
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from linkerbot_arm import A7, P7, A7lite, Pose, StateError, ValidationError
from linkerbot_arm.arm.a7.motor import A7Motor
from linkerbot_arm.arm.a7_lite.motor import A7liteMotor, CommType
from linkerbot_arm.arm.common import ControlMode
from linkerbot_arm.arm.common.model import (
    AccelerationState,
    AngleState,
    TemperatureState,
    TorqueState,
    VelocityState,
)
from linkerbot_arm.arm.p7.motor_v2 import MotorV2
from linkerbot_arm.motion_timer import MotionTimer


class Dispatcher:
    def __init__(self):
        self.sent = []
        self.reject = False

    def subscribe(self, callback):
        pass

    def send(self, message):
        if self.reject:
            raise Full("injected admission failure")
        self.sent.append(message)


@pytest.fixture
def make_arm(kinetix_module):
    ArmKinetix = kinetix_module.ArmKinetix
    arms = []

    def make(cls, position=0.0, dispatcher=None):
        arm = cls.__new__(cls)
        arm._motion_timer = MotionTimer()
        arm._closed = False
        arm._control_modes = [ControlMode.PP] * 7 if cls is A7lite else None
        dispatcher = Dispatcher() if dispatcher is None else dispatcher
        arm._can_dispatcher = dispatcher
        classes = [A7liteMotor if cls is A7lite else A7Motor] * 7
        if cls is P7:
            classes[5:] = [MotorV2] * 2
        arm._motors = [c(61 + i, dispatcher) for i, c in enumerate(classes)]
        for motor in arm._motors:
            now = time.time()
            motor._angle = AngleState(angle=position, timestamp=now)
            motor._control_angle = AngleState(angle=position, timestamp=now)
            motor._velocity = VelocityState(velocity=0, timestamp=now)
            motor._control_velocity = VelocityState(velocity=0.5, timestamp=now)
            motor._control_acceleration = AccelerationState(
                acceleration=1, timestamp=now
            )
            motor._torque = TorqueState(torque=0, timestamp=now)
            motor._temperature = TemperatureState(temperature=25, timestamp=now)
            motor._limit_spd = 0.5
            if isinstance(motor, A7Motor):
                motor._read_register_float = lambda *args: position
        arm._kx = SimpleNamespace(
            get_joint_limits=lambda: [(-3, 3)] * 7,
            _trap_params=ArmKinetix._trap_params,
            _trap_s=ArmKinetix._trap_s,
        )
        arms.append(arm)
        return arm, dispatcher

    yield make
    for arm in arms:
        arm._motion_timer.cancel()


@pytest.mark.parametrize("cls", [A7, A7lite, P7])
@pytest.mark.parametrize("stop", ["disable", "emergency_stop"])
def test_stop_during_ik_prevents_later_targets(make_arm, cls, stop):
    arm, _ = make_arm(cls)
    entered, release = threading.Event(), threading.Event()
    errors = []

    def ik(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return [0.2] * 7

    arm.inverse_kinematics = ik
    arm._set_angles = Mock()

    def move():
        try:
            arm.move_p(Pose.from_list([0] * 6), blocking=False)
        except BaseException as error:
            errors.append(error)

    worker = threading.Thread(target=move)
    worker.start()
    try:
        assert entered.wait(1)
        assert arm.is_moving()
        with pytest.raises(StateError, match="moving"):
            arm.move_j([0] * 7)
        getattr(arm, stop)()
        assert arm.is_moving()  # Ownership remains until IK unwinds.
    finally:
        release.set()
        worker.join(2)
    assert not worker.is_alive() and not errors
    arm._set_angles.assert_not_called()
    assert not arm.is_moving()


@pytest.mark.parametrize("goal", [0.1, 1.0])
@pytest.mark.parametrize("method", ["move_j", "move_p"])
def test_p7_generated_motion_has_velocity_and_acceleration_ramps(
    make_arm, monkeypatch, goal, method
):
    arm, dispatcher = make_arm(P7)
    captured = []
    module = importlib.import_module(P7.__module__)

    def stream(owner, points, **kwargs):
        for point in points:
            owner._set_angles(point, check_limits=False)
            captured.append(point)

    monkeypatch.setattr(module, "stream_targets", stream)
    target = [0] * 7
    target[5] = goal
    arm.inverse_kinematics = lambda *args, **kwargs: target
    getattr(arm, method)(target if method == "move_j" else Pose.from_list([0] * 6))
    assert len(captured) > 1
    assert captured[-1] == pytest.approx(target)
    assert 0 < captured[0][5] < goal
    points = [0.0] + [p[5] for p in captured] + [goal]
    velocities = [(b - a) / 0.01 for a, b in zip(points, points[1:])]
    accelerations = [(b - a) / 0.01 for a, b in zip([0.0] + velocities, velocities)]
    assert max(map(abs, velocities)) <= 0.5 + 1e-8
    assert max(map(abs, accelerations)) <= 1 + 1e-8
    assert velocities[0] < max(velocities)
    assert velocities[-2] < max(velocities)
    assert len(
        [frame for frame in dispatcher.sent if frame.arbitration_id == 66]
    ) == len(captured)


@pytest.mark.parametrize("method", ["move_j", "move_p"])
@pytest.mark.parametrize("blocking", [True, False])
@pytest.mark.parametrize("stalls", [[], [0.2], [0.015, 0.08, 0.2], [1.0, 0.2]])
def test_p7_point_execution_keeps_real_time_derivatives_bounded(
    make_arm, monkeypatch, method, blocking, stalls
):
    from linkerbot_arm.arm.common import _stream

    arm, _ = make_arm(P7)
    clock = [0.0]
    speeds = [0.3, 0.4, 0.5, 0.5, 0.5, 0.5, 0.35]
    ramps = [1.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.5]
    arm.set_velocities(speeds)
    arm.set_accelerations(ramps)
    target = [0.25, -0.1, 0.0, 0.0, 0.0, 1.0, -0.75]
    samples = [(0.0, [0.0] * 7)]
    injected = set()
    original_wait = arm._motion_timer._wait_cancelled
    original_set = arm._set_angles

    def wait(seconds):
        clock[0] += seconds
        return original_wait(0)

    def send(angles, **kwargs):
        index = len(samples)
        for number, stall in enumerate(stalls):
            if index == 60 + 40 * number and number not in injected:
                injected.add(number)
                clock[0] += stall
        # Delay before actual admission, after the executor's ordinary wait.
        # Rejected timing attempts must not advance this accepted sample list.
        original_set(angles, **kwargs)
        samples.append((clock[0], list(angles)))

    monkeypatch.setattr(_stream.time, "perf_counter", lambda: clock[0])
    monkeypatch.setattr(arm._motion_timer, "_wait_cancelled", wait)
    arm._set_angles = send
    arm.inverse_kinematics = lambda *args, **kwargs: target
    getattr(arm, method)(
        target if method == "move_j" else Pose.from_list([0.0] * 6), blocking=blocking
    )

    velocity = [0.0] * 7
    for (previous_time, previous), (now, positions) in zip(samples, samples[1:]):
        elapsed = now - previous_time
        next_velocity = [(b - a) / elapsed for a, b in zip(previous, positions)]
        for value, old, speed, ramp in zip(next_velocity, velocity, speeds, ramps):
            assert abs(value) <= speed + 1e-8
            assert abs(value - old) / elapsed <= ramp + 1e-8
        velocity = next_velocity
    assert samples[-1][1] == pytest.approx(target)
    assert len(injected) == len(stalls)
    # Delay changes timing only: every planned spatial waypoint is preserved.
    duration = arm._move_duration([0.0] * 7, target, speeds, ramps)
    assert len(samples) - 1 == math.ceil(duration / _stream.WAYPOINT_INTERVAL)
    if not stalls:
        assert clock[0] == pytest.approx(
            (len(samples) - 1 + int(blocking)) * _stream.WAYPOINT_INTERVAL
        )
    assert arm.get_control_velocities() == speeds
    assert arm.get_control_acceleration() == ramps
    assert arm._point_target_profile is None and arm._stream_target_state is None
    assert not arm.is_moving()


def test_p7_point_timing_wait_is_cancellable_and_restores_profiles(
    make_arm, monkeypatch
):
    from linkerbot_arm.arm.common import _stream

    arm, _ = make_arm(P7)
    clock = [0.0]
    samples = []
    original_set = arm._set_angles
    original_wait = arm._motion_timer._wait_cancelled
    delayed = False

    def send(angles, **kwargs):
        nonlocal delayed
        if len(samples) == 99 and not delayed:
            delayed = True
            clock[0] += 0.2
        original_set(angles, **kwargs)
        samples.append(list(angles))

    def wait(seconds):
        if delayed and seconds > _stream.WAYPOINT_INTERVAL:
            arm.disable()
        clock[0] += seconds
        return original_wait(0)

    monkeypatch.setattr(_stream.time, "perf_counter", lambda: clock[0])
    monkeypatch.setattr(arm._motion_timer, "_wait_cancelled", wait)
    arm._set_angles = send
    arm.move_j([0.0] * 5 + [1.0, 0.0])
    assert delayed and len(samples) == 99
    assert arm.get_control_velocities() == [0.5] * 7
    assert arm.get_control_acceleration() == [1.0] * 7
    assert arm._point_target_profile is None and arm._stream_target_state is None
    assert not arm.is_moving()


def test_point_timing_interval_satisfies_each_joint_after_mixed_delays():
    from linkerbot_arm.arm.common._motion import _point_target_interval

    rng = random.Random(20261013)
    for _ in range(200):
        positions = [rng.uniform(-2, 2) for _ in range(7)]
        target = [p + rng.uniform(-0.1, 0.1) for p in positions]
        speeds = [rng.uniform(0.05, 5) for _ in range(7)]
        ramps = [rng.uniform(1, 50) for _ in range(7)]
        velocities = [rng.uniform(-speed, speed) for speed in speeds]
        elapsed = rng.uniform(0.001, 1)
        interval = _point_target_interval(
            positions, target, velocities, speeds, ramps, elapsed
        )
        assert math.isfinite(interval) and interval >= elapsed
        for start, goal, previous, speed, ramp in zip(
            positions, target, velocities, speeds, ramps
        ):
            velocity = (goal - start) / interval
            assert abs(velocity) <= speed + 1e-10
            assert abs(velocity - previous) / interval <= ramp + 1e-8


def test_rejected_p7_point_does_not_advance_timing_or_skip_profile_restore(
    make_arm, monkeypatch
):
    from linkerbot_arm.arm.common import _stream

    arm, dispatcher = make_arm(P7)
    clock = [0.0]
    original_wait = arm._motion_timer._wait_cancelled
    original_set = arm._set_angles
    count = [0]

    def wait(seconds):
        clock[0] += seconds
        return original_wait(0)

    def send(angles, **kwargs):
        count[0] += 1
        if count[0] == 60:
            previous = arm._stream_target_state
            dispatcher.reject = True
            try:
                original_set(angles, **kwargs)
            except Full:
                assert arm._stream_target_state is previous
                raise
            finally:
                dispatcher.reject = False
        else:
            original_set(angles, **kwargs)

    monkeypatch.setattr(_stream.time, "perf_counter", lambda: clock[0])
    monkeypatch.setattr(arm._motion_timer, "_wait_cancelled", wait)
    arm._set_angles = send
    with pytest.raises(Full):
        arm.move_j([0.0] * 5 + [1.0, 0.0])
    assert count[0] == 60
    assert arm.get_control_velocities() == [0.5] * 7
    assert arm.get_control_acceleration() == [1.0] * 7
    assert arm._point_target_profile is None and arm._stream_target_state is None
    assert not arm.is_moving()


def test_external_p7_targets_limit_steps_reversal_and_stalls(make_arm, monkeypatch):
    arm, _ = make_arm(P7)
    clock = [0.0]
    module = importlib.import_module(P7.__module__)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    assert arm._set_angles([1] * 7) is None
    assert arm.get_control_angles() == [0.0] * 7
    previous, velocity = 0.0, 0.0
    for step in range(1, 100):
        clock[0] += 0.01
        arm._set_angles([1 if step < 50 else -1] * 7)
        position = arm.get_control_angles()[5]
        next_velocity = (position - previous) / 0.01
        assert abs(next_velocity) <= 0.5 + 1e-9
        assert abs(next_velocity - velocity) <= 0.01 + 1e-9
        previous, velocity = position, next_velocity
    clock[0] += 10
    arm._set_angles([2] * 7)
    after_stall = arm.get_control_angles()[5]
    slow_velocity = (after_stall - previous) / 10
    assert abs(slow_velocity) <= 0.5 + 1e-9
    assert abs(slow_velocity - velocity) <= 10 + 1e-9
    clock[0] += 0.01
    arm._set_angles([2] * 7)
    next_velocity = (arm.get_control_angles()[5] - after_stall) / 0.01
    assert abs(next_velocity - slow_velocity) <= 0.01 + 1e-9


def test_p7_rejected_batch_keeps_limiter_seed_and_lifecycle_reseeds(
    make_arm, monkeypatch
):
    arm, dispatcher = make_arm(P7, position=0.2)
    clock = [0.0]
    module = importlib.import_module(P7.__module__)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    dispatcher.reject = True
    with pytest.raises(Full):
        arm._set_angles([1] * 7)
    assert getattr(arm, "_stream_target_state", None) is None
    dispatcher.reject = False
    arm._set_angles([1] * 7)
    state = arm._stream_target_state
    clock[0] += 0.01
    dispatcher.reject = True
    with pytest.raises(Full):
        arm._set_angles([1] * 7)
    assert arm._stream_target_state is state
    dispatcher.reject = False
    arm.disable()
    for motor in arm._motors:
        motor._angle = AngleState(angle=0.3, timestamp=time.time())
    arm._set_angles([1] * 7)
    assert arm.get_control_angles() == [0.3] * 7


@pytest.mark.parametrize("mode", [ControlMode.PP, ControlMode.CSP, ControlMode.MIT])
def test_lite_stop_latches_all_external_target_entries(make_arm, mode):
    arm, dispatcher = make_arm(A7lite, position=0.2)
    arm._control_modes = [mode] * 7
    arm._kx.get_joint_limits = lambda: [(-0.1, 0.1)] * 7
    arm.emergency_stop()  # A measured hold outside soft limits is still admitted.
    count = len(dispatcher.sent)
    for operation in (
        lambda: arm._set_angles([0] * 7),
        lambda: arm.set_mits([0] * 7, kps=[20] * 7, kds=[1] * 7),
        lambda: arm.set_csp_angles([0] * 7),
        lambda: arm.stream_joint_targets([0] * 7),
        lambda: arm.move_j([0] * 7),
        lambda: arm.move_p(Pose.from_list([0] * 6)),
    ):
        with pytest.raises(StateError):
            operation()
    assert len(dispatcher.sent) == count
    assert arm._stopped
    if mode == ControlMode.PP:
        assert arm.get_control_velocities() == [0] * 7
    if mode == ControlMode.CSP:
        assert arm.get_limit_spds() == [0] * 7


def test_lite_zero_mit_gains_use_hold_minimums(make_arm):
    arm, dispatcher = make_arm(A7lite)
    arm._control_modes = [ControlMode.MIT] * 7
    for motor in arm._motors:
        motor.set_mit(1, 0, 0, 0, 1)
    dispatcher.sent.clear()
    arm.emergency_stop()
    assert all(m.mit_kp == 20 and m.mit_kd == 1 for m in arm._motors)
    assert all((m.arbitration_id >> 24) == CommType.Type1 for m in dispatcher.sent)


def test_lite_enable_restores_only_after_current_hold(make_arm, monkeypatch):
    arm, dispatcher = make_arm(A7lite, position=0.2)
    monkeypatch.setattr("linkerbot_arm.arm.a7_lite.a7_lite.time.sleep", lambda _: None)
    arm.emergency_stop()
    arm.set_velocities([0.4] * 7)  # Stage configuration; must not undo the stop.
    assert arm.get_control_velocities() == [0] * 7
    dispatcher.sent.clear()
    arm.enable()
    assert not arm._stopped
    assert arm.get_control_angles() == [0.2] * 7
    assert arm.get_control_velocities() == [0.4] * 7
    arm._set_angles([0] * 7)


@pytest.mark.parametrize("cancelled", [False, True])
def test_lite_home_recovery_keeps_targets_blocked_until_success(
    make_arm, monkeypatch, cancelled
):
    arm, _ = make_arm(A7lite, position=0.2)
    monkeypatch.setattr("linkerbot_arm.arm.a7_lite.a7_lite.time.sleep", lambda _: None)
    arm.emergency_stop()
    arm.home(blocking=False)
    assert arm.is_moving() and arm._stopped
    with pytest.raises(StateError, match="stopped"):
        arm._set_angles([0.1] * 7)
    if cancelled:
        arm.disable()
        with pytest.raises(StateError, match="stopped"):
            arm._set_angles([0.1] * 7)
        arm.enable()
    else:
        arm._motion_timer._on_done(arm._motion_timer._generation)
    arm._set_angles([0.1] * 7)
    assert not arm._stopped


def test_lite_failed_resume_relatches_and_does_not_admit_external_target(
    make_arm, monkeypatch
):
    arm, _ = make_arm(A7lite, position=0.2)
    monkeypatch.setattr("linkerbot_arm.arm.a7_lite.a7_lite.time.sleep", lambda _: None)
    arm.emergency_stop()
    arm._motors[3].enable = Mock(side_effect=RuntimeError("enable failed"))
    with pytest.raises(RuntimeError, match="enable failed"):
        arm.enable()
    assert arm._stopped and arm.get_control_velocities() == [0] * 7
    with pytest.raises(StateError, match="stopped"):
        arm._set_angles([1] * 7)


def test_lite_cancellable_recovery_reinstates_zero_cap_before_return(
    make_arm, monkeypatch
):
    arm, _ = make_arm(A7lite, position=0.2)
    monkeypatch.setattr("linkerbot_arm.arm.a7_lite.a7_lite.time.sleep", lambda _: None)
    arm.emergency_stop()
    cancel = threading.Event()
    original = arm.get_control_velocities

    def cancel_during_admission():
        cancel.set()
        return original()

    arm.get_control_velocities = cancel_during_admission
    arm.home_cancellable(cancel)
    assert arm._stopped and not arm._recovering_home
    assert original() == [0.0] * 7
    with pytest.raises(StateError, match="stopped"):
        arm._set_angles([0] * 7)


@pytest.mark.parametrize("limit_count", [0, 6, 8])
@pytest.mark.parametrize(
    "cls,command",
    [(A7, "register"), (A7lite, "register"), (A7lite, "mit"), (A7lite, "mixed")],
)
def test_raw_target_admission_rejects_wrong_limit_count(
    make_arm, cls, command, limit_count
):
    arm, dispatcher = make_arm(cls)
    arm._kx.get_joint_limits = lambda: [(-1, 1)] * limit_count
    if command == "mit":
        arm._control_modes = [ControlMode.MIT] * 7
    elif command == "mixed":
        arm._control_modes = [ControlMode.MIT] + [ControlMode.PP] * 6

    def send():
        if command == "mit":
            arm.set_mits([0] * 7, kps=[20] * 7, kds=[1] * 7)
        elif command == "mixed":
            arm.stream_joint_targets([0] * 7, kps=[20] * 7, kds=[1] * 7)
        else:
            arm._set_angles([0] * 7)

    with pytest.raises(StateError, match="limit count"):
        send()
    assert dispatcher.sent == []


@pytest.mark.parametrize("gap", [0.101, 0.2, 10.0])
@pytest.mark.parametrize("direction", [1, -1])
def test_p7_stream_derivatives_remain_bounded_across_variable_periods(
    make_arm, monkeypatch, gap, direction
):
    arm, _ = make_arm(P7)
    arm._kx.get_joint_limits = lambda: [(-10, 10)] * 7
    arm.set_velocities([2.0] * 7)
    clock = [0.0]
    module = importlib.import_module(P7.__module__)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    arm._set_angles([5] * 7)
    previous, velocity = 0.0, 0.0

    def step(elapsed, goal):
        nonlocal previous, velocity
        previous_time = clock[0]
        clock[0] += elapsed
        arm._set_angles([goal] * 7)
        position = arm.get_control_angles()[5]
        actual_elapsed = arm._stream_target_state[2] - previous_time
        next_velocity = (position - previous) / actual_elapsed
        assert abs(next_velocity) <= 2.0 + 1e-9
        assert abs(next_velocity - velocity) / actual_elapsed <= 1.0 + 1e-9
        previous, velocity = position, next_velocity

    for _ in range(200):
        step(0.01, 5)
    assert velocity == pytest.approx(2.0)
    for index, period in enumerate((gap, 0.01, 0.101, 0.2, 0.01, 10, 0.03)):
        step(period, 5 * direction * (-1 if index % 2 else 1))


def test_lite_enable_after_completed_nonblocking_home_refreshes_recovery(
    make_arm, monkeypatch
):
    arm, _ = make_arm(A7lite, position=0.2)
    monkeypatch.setattr("linkerbot_arm.arm.a7_lite.a7_lite.time.sleep", lambda _: None)
    arm.emergency_stop()
    arm.home(blocking=False)
    assert arm._recovering_home
    arm._motion_timer._on_done(arm._motion_timer._generation)
    arm.enable()
    assert not arm._stopped and not arm._recovering_home
    arm._set_angles([0.1] * 7)


@pytest.mark.parametrize("failure", ["cap", "hold_send", "unknown", "nonfinite"])
@pytest.mark.parametrize("fallback_failure", [False, True])
def test_lite_failed_stop_requests_disable_and_continues_other_joints(
    make_arm, failure, fallback_failure, caplog
):
    arm, dispatcher = make_arm(A7lite, position=0.2)
    motor = arm._motors[0]
    motor.set_angle(1.0)
    dispatcher.sent.clear()
    original = RuntimeError("primary stop failure")
    if failure == "nonfinite":
        motor._angle = AngleState(angle=float("nan"), timestamp=time.time())
    elif failure == "unknown":
        del motor._angle
    else:
        send = dispatcher.send
        failed = False
        register = 0x7024 if failure == "cap" else 0x7016

        def fail_once(frame):
            nonlocal failed
            if (
                not failed
                and frame.arbitration_id & 0xFF == 61
                and frame.arbitration_id >> 24 == CommType.Type18
                and struct.unpack_from("<H", frame.data)[0] == register
            ):
                failed = True
                raise original
            send(frame)

        dispatcher.send = fail_once
    disable = motor.disable
    motor.disable = Mock(
        side_effect=RuntimeError("fallback failure") if fallback_failure else disable
    )
    error_type = {
        "cap": RuntimeError,
        "hold_send": RuntimeError,
        "unknown": StateError,
        "nonfinite": ValidationError,
    }[failure]
    with pytest.raises(error_type) as raised:
        arm.emergency_stop()
    if failure in ("cap", "hold_send"):
        assert raised.value is original
    motor.disable.assert_called_once()
    assert arm._stopped
    if fallback_failure:
        assert "Stop disable fallback failed" in caplog.text
    else:
        assert any(
            (frame.arbitration_id & 0xFF) == 61
            and (frame.arbitration_id >> 24) == CommType.Type4
            for frame in dispatcher.sent
        )
    # All six other joints still receive both zero cap and measured hold.
    for mid in range(62, 68):
        assert (
            len(
                [
                    frame
                    for frame in dispatcher.sent
                    if frame.arbitration_id & 0xFF == mid
                ]
            )
            == 2
        )


def test_a7_closing_rejects_external_batch_but_allows_owned_worker(
    make_arm, monkeypatch
):
    from .test_motion_batches import QueuedDispatcher

    dispatcher = QueuedDispatcher(capacity=14)
    arm, _ = make_arm(A7, dispatcher=dispatcher)
    ready, release, closing = threading.Event(), threading.Event(), threading.Event()
    failures = []
    original_wait = arm._motion_timer._wait_worker_done

    def waiting(**kwargs):
        closing.set()
        return original_wait(**kwargs)

    def stop_dispatcher():
        dispatcher._tx.drain()
        dispatcher._tx.close()

    monkeypatch.setattr(arm._motion_timer, "_wait_worker_done", waiting)
    dispatcher.stop = stop_dispatcher

    def work():
        try:
            with arm._motion_timer._stream_scope():
                ready.set()
                assert release.wait(2)
                arm._set_angles([0.1] * 7)
        except BaseException as error:
            failures.append(error)

    def close():
        try:
            arm.close()
        except BaseException as error:
            failures.append(error)

    worker = threading.Thread(target=work)
    closer = threading.Thread(target=close)
    worker.start()
    try:
        assert ready.wait(1)
        closer.start()
        assert closing.wait(1)
        assert arm._closing and closer.is_alive()
        with pytest.raises(StateError, match="closing"):
            arm._set_angles([0.2] * 7)
        assert dispatcher._tx.get_health().pending_fifo == 0
    finally:
        release.set()
        worker.join(2)
        if closer.ident is not None:
            closer.join(2)
        dispatcher._tx.close()
    assert not worker.is_alive() and not closer.is_alive() and not failures
    assert len(dispatcher.sent) == 7
    assert arm.get_control_angles() == [0.1] * 7
    with pytest.raises(StateError, match="closed"):
        arm._set_angles([0.2] * 7)
