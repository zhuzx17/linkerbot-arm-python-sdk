"""Three-model host timing with a virtual clock and injected failures, no CAN."""

import threading
from queue import Full
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from linkerbot_arm import A7, P7, A7lite, Pose, StateError, ValidationError
from linkerbot_arm.arm.common import _stream
from linkerbot_arm.motion_timer import MotionTimer


@pytest.fixture(params=[A7, A7lite, P7])
def rig(request, monkeypatch, kinetix_module):
    ArmKinetix = kinetix_module.ArmKinetix
    arm = request.param.__new__(request.param)
    arm._motors = [Mock() for _ in range(7)]
    arm._control_modes = None
    arm._motion_timer = MotionTimer()
    clock = [0.0]
    events = []
    zero = Pose.from_list([0.0] * 6)
    target = Pose.from_list([0.001, 0, 0, 0, 0, 0])

    def plan(*args, **kwargs):
        events.append(("plan", clock[0]))
        assert arm.is_moving()
        assert arm._motion_timer._timer is None
        clock[0] += 0.08  # Previously consumed the early waypoint deadlines.
        for value in (0.01, 0.02, 0.03):
            yield SimpleNamespace(angles=[value] * 7)
        events.append(("planned", clock[0]))

    def configure(kind, values):
        events.append((kind, clock[0], values[:]))

    arm._kx = SimpleNamespace(
        get_joint_limits=lambda: [(-3, 3)] * 7,
        forward_kinematics=lambda angles: zero,
        plan_move_l=plan,
        _trap_params=ArmKinetix._trap_params,
        _trap_s=ArmKinetix._trap_s,
    )
    arm.get_angles = lambda: [0.0] * 7
    arm.get_control_velocities = lambda: [0.5] * 7
    arm.get_control_acceleration = lambda: [1.0] * 7
    arm.set_velocities = Mock(side_effect=lambda values: configure("velocity", values))
    arm.set_accelerations = Mock(
        side_effect=lambda values: configure("acceleration", values)
    )

    def send(angles, **kwargs):
        assert arm.is_moving()
        assert arm._motion_timer._timer is None
        events.append(("target", clock[0], angles[:]))

    arm._set_angles = Mock(side_effect=send)
    cancelled = arm._motion_timer._wait_cancelled

    def wait(duration):
        if cancelled(0):
            return True
        clock[0] += duration
        return cancelled(0)

    monkeypatch.setattr(_stream.time, "perf_counter", lambda: clock[0])
    monkeypatch.setattr(arm._motion_timer, "_wait_cancelled", wait)
    yield SimpleNamespace(
        arm=arm, clock=clock, events=events, target=target, wait=wait, send=send
    )
    arm._motion_timer.cancel()


def test_planning_finishes_before_limits_and_trajectory_clock(rig):
    rig.arm.move_l(rig.target)
    assert [e[0] for e in rig.events] == [
        "plan",
        "planned",
        "velocity",
        "acceleration",
        "target",
        "target",
        "target",
        "velocity",
        "acceleration",
    ]
    submissions = [e[1] for e in rig.events if e[0] == "target"]
    assert submissions == pytest.approx([0.09, 0.10, 0.11])
    assert rig.clock[0] == pytest.approx(0.12)
    assert not rig.arm.is_moving()
    assert rig.arm._motion_timer.wait_done(0)


@pytest.mark.parametrize("stall", [0.015, 0.08])
def test_slow_submission_stretches_time_without_burst_or_dropped_points(rig, stall):
    def send(angles, **kwargs):
        rig.send(angles, **kwargs)
        if angles[0] == 0.01:
            rig.clock[0] += stall

    rig.arm._set_angles.side_effect = send
    rig.arm.move_l(rig.target)
    targets = [e for e in rig.events if e[0] == "target"]
    assert [e[2][0] for e in targets] == [0.01, 0.02, 0.03]
    assert targets[1][1] - targets[0][1] == pytest.approx(stall + 0.01)
    assert targets[2][1] - targets[1][1] == pytest.approx(0.01)


def test_scheduler_oversleep_does_not_trigger_catchup(rig, monkeypatch):
    delayed = False

    def wait(duration):
        nonlocal delayed
        if duration and not delayed:
            rig.clock[0] += 0.08
            delayed = True
        return rig.wait(duration)

    monkeypatch.setattr(rig.arm._motion_timer, "_wait_cancelled", wait)
    rig.arm.move_l(rig.target)
    submissions = [e[1] for e in rig.events if e[0] == "target"]
    assert submissions == pytest.approx([0.17, 0.18, 0.19])


@pytest.mark.parametrize("fault", ["planner", "late_nan", "late_count", "late_limit"])
def test_entire_plan_validated_before_configuration_or_targets(rig, fault):
    def plan(*args, **kwargs):
        yield SimpleNamespace(angles=[0.1] * 7)
        if fault == "planner":
            raise RuntimeError("IK failed on later waypoint")
        values = {
            "late_nan": [float("nan")] * 7,
            "late_count": [0.1] * 6,
            "late_limit": [4.0] * 7,
        }
        yield SimpleNamespace(angles=values[fault])

    rig.arm._kx.plan_move_l = plan
    with pytest.raises((RuntimeError, ValidationError, ValueError)):
        rig.arm.move_l(rig.target)
    rig.arm._set_angles.assert_not_called()
    rig.arm.set_velocities.assert_not_called()
    rig.arm.set_accelerations.assert_not_called()
    assert not rig.arm.is_moving()


@pytest.mark.parametrize("stage", ["planning", "target"])
def test_cancellation_stops_admission_and_keeps_cleanup_owned(rig, stage):
    if stage == "planning":
        plan = rig.arm._kx.plan_move_l

        def cancelled_plan(*args, **kwargs):
            rig.arm.disable()
            yield from plan(*args, **kwargs)

        rig.arm._kx.plan_move_l = cancelled_plan
    else:

        def send(angles, **kwargs):
            rig.send(angles, **kwargs)
            rig.arm.disable()
            assert rig.arm.is_moving()  # owns parameters until the finally exits
            with pytest.raises(StateError, match="moving"):
                rig.arm.move_j([0.0] * 7)

        rig.arm._set_angles.side_effect = send
    rig.arm.move_l(rig.target)
    assert rig.arm._set_angles.call_count == (0 if stage == "planning" else 1)
    if stage == "planning":
        rig.arm.set_velocities.assert_not_called()
    else:
        rig.arm.set_velocities.assert_called_with([0.5] * 7)
        rig.arm.set_accelerations.assert_called_with([1.0] * 7)
    assert not rig.arm.is_moving()


@pytest.mark.parametrize("stage", ["velocity", "acceleration", "target"])
@pytest.mark.parametrize("kind", [RuntimeError, KeyboardInterrupt])
def test_failed_setup_or_send_restores_changed_limits_and_finishes(rig, stage, kind):
    original = kind("injected failure")
    operation = {
        "velocity": rig.arm.set_velocities,
        "acceleration": rig.arm.set_accelerations,
        "target": rig.arm._set_angles,
    }[stage]
    operation.side_effect = [original, None]
    with pytest.raises(kind) as exc:
        rig.arm.move_l(rig.target)
    assert exc.value is original
    rig.arm.set_velocities.assert_called_with([0.5] * 7)
    if stage != "velocity":
        rig.arm.set_accelerations.assert_called_with([1.0] * 7)
    if stage != "target":
        rig.arm._set_angles.assert_not_called()
    assert not rig.arm.is_moving()


@pytest.mark.parametrize("send_fails", [True, False])
def test_restore_errors_are_visible_without_hiding_original_failure(
    rig, send_fails, caplog
):
    original = Full("target queue rejected")
    cleanup = RuntimeError("velocity restore failed")
    rig.arm.set_velocities.side_effect = [None, cleanup]
    rig.arm.set_accelerations.side_effect = [
        None,
        RuntimeError("acceleration restore failed"),
    ]
    if send_fails:
        rig.arm._set_angles.side_effect = original
    with pytest.raises(Full if send_fails else RuntimeError) as exc:
        rig.arm.move_l(rig.target)
    assert exc.value is (original if send_fails else cleanup)
    assert "velocity restore failed" in caplog.text
    assert "acceleration restore failed" in caplog.text
    assert rig.arm.set_accelerations.call_count == 2
    assert not rig.arm.is_moving()


@pytest.mark.parametrize("rig", [P7], indirect=True)
@pytest.mark.parametrize("blocking", [True, False])
def test_p7_move_j_uses_same_no_catchup_stream_and_actual_completion(rig, blocking):
    rig.arm._move_duration = lambda *args: 0.025  # ceil to three 10 ms steps

    def send(angles, **kwargs):
        rig.send(angles, **kwargs)
        rig.clock[0] += 0.04

    rig.arm._set_angles.side_effect = send
    rig.arm.move_j([0.3] * 7, blocking=blocking)
    targets = [e for e in rig.events if e[0] == "target"]
    assert [e[1] for e in targets] == pytest.approx([0.01, 0.06, 0.11])
    # Mocked total time tests scheduling only; the positions now follow the
    # actual acceleration/cruise/deceleration progress instead of linear steps.
    assert [e[2][0] for e in targets] == pytest.approx([0.0968, 0.2758, 0.3])
    assert rig.clock[0] == pytest.approx(0.16 if blocking else 0.15)
    assert not rig.arm.is_moving()


def test_stop_during_blocked_planning_cancels_without_waiting_for_planner(rig):
    entered, release = threading.Event(), threading.Event()
    errors = []

    def plan(*args, **kwargs):
        entered.set()
        assert release.wait(1)
        yield SimpleNamespace(angles=[0.1] * 7)

    def move():
        try:
            rig.arm.move_l(rig.target)
        except BaseException as error:
            errors.append(error)

    rig.arm._kx.plan_move_l = plan
    worker = threading.Thread(target=move)
    worker.start()
    try:
        assert entered.wait(1)
        rig.arm.disable()
        assert not release.is_set()
        assert rig.arm.is_moving()  # reserved until planner unwinds
    finally:
        release.set()
        worker.join(1)
    assert not worker.is_alive() and not errors
    rig.arm._set_angles.assert_not_called()
    rig.arm.set_velocities.assert_not_called()
    assert not rig.arm.is_moving()


@pytest.mark.parametrize("rig", [A7lite], indirect=True)
def test_cleanup_cannot_override_lite_emergency_zero_speed_hold(rig, monkeypatch):
    first_target = threading.Event()
    hold_started, release_hold = threading.Event(), threading.Event()
    errors = []

    def send(angles, **kwargs):
        rig.send(angles, **kwargs)
        first_target.set()

    def wait(duration):
        if duration and first_target.is_set():
            assert hold_started.wait(1)
            return True
        return rig.wait(duration)

    for i, motor in enumerate(rig.arm._motors):
        motor._id = 61 + i
        motor.angle = SimpleNamespace(angle=0.0)
        motor.control_velocity = SimpleNamespace(velocity=0.5)

        def configure(value, motor=motor, i=i):
            motor.control_velocity.velocity = value
            if value == 0 and i == 0:
                hold_started.set()
                assert release_hold.wait(1)

        motor.set_velocity.side_effect = configure

    # Exercise the production setter, so finally restoration is staged while
    # stopped rather than accidentally overwriting the drive's persistent zero.
    rig.arm.set_velocities = Mock(wraps=A7lite.set_velocities.__get__(rig.arm, A7lite))

    def run(operation):
        try:
            operation()
        except BaseException as error:
            errors.append(error)

    rig.arm._set_angles.side_effect = send
    monkeypatch.setattr(rig.arm._motion_timer, "_wait_cancelled", wait)
    moving = threading.Thread(target=run, args=(lambda: rig.arm.move_l(rig.target),))
    stopping = threading.Thread(target=run, args=(rig.arm.emergency_stop,))
    moving.start()
    try:
        assert first_target.wait(1)
        stopping.start()
        assert hold_started.wait(1)
        # Only the initial MAX setup is admitted while the stop holds speed zero.
        assert rig.arm.set_velocities.call_count == 1
        assert rig.arm.set_accelerations.call_count == 1
        assert rig.arm._set_angles.call_count == 1
    finally:
        release_hold.set()
        moving.join(1)
        if stopping.ident is not None:
            stopping.join(1)
    assert not errors and not moving.is_alive() and not stopping.is_alive()
    rig.arm.set_velocities.assert_called_with([0.5] * 7)
    rig.arm.set_accelerations.assert_called_with([1.0] * 7)
    assert [m.control_velocity.velocity for m in rig.arm._motors] == [0.0] * 7
    assert list(rig.arm._stopped_profiles["pp"].values()) == [0.5] * 7
    assert not rig.arm.is_moving()


@pytest.mark.parametrize("rig", [A7, A7lite, P7], indirect=True)
def test_observer_waits_until_stream_and_cleanup_complete(rig):
    ready, finish = threading.Event(), threading.Event()
    errors = []

    def restore(values):
        if values == [1.0] * 7:
            ready.set()
            assert finish.wait(1)

    def move():
        try:
            rig.arm.move_l(rig.target)
        except BaseException as error:
            errors.append(error)

    rig.arm.set_accelerations.side_effect = restore
    worker = threading.Thread(target=move)
    worker.start()
    try:
        assert ready.wait(1)
        assert rig.arm.is_moving()
        assert not rig.arm._motion_timer.wait_done(0)
    finally:
        finish.set()
        worker.join(1)
    assert not errors and not worker.is_alive()
    assert rig.arm._motion_timer.wait_done(0)
