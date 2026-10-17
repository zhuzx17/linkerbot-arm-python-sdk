import functools
import math
import time
import warnings
from typing import Literal

import numpy as np
from scipy.spatial.transform import Rotation as R

from linkerbot_arm.feedback import ArmFeedback, read_feedback
from linkerbot_arm._transport import arm_dispatcher, close_arm, exit_arm
from linkerbot_arm._transport.targets import ArmMotionQueueMixin, submit_targets
from linkerbot_arm.arm.a7.motor import A7Motor, SensorType
from linkerbot_arm.arm.p7.consts import (
    DEFAULT_POLL_INTERVALS,
    MAX_ACCELERATION,
    MAX_VELOCITY,
    MIN_ACCELERATION,
    MIN_VELOCITY,
    MOVE_L_DEFAULT_ACCELERATION,
    MOVE_L_DEFAULT_ANGULAR_ACCELERATION,
    MOVE_L_DEFAULT_MAX_ANGULAR_VELOCITY,
    MOVE_L_DEFAULT_MAX_VELOCITY,
    MOVE_L_MAX_ACCELERATION,
    MOVE_L_MAX_ANGULAR_ACCELERATION,
    MOVE_L_MAX_MAX_ANGULAR_VELOCITY,
    MOVE_L_MAX_MAX_VELOCITY,
    NUM_JOINTS,
)
from linkerbot_arm.arm.p7.motor_v2 import MotorV2
from linkerbot_arm.arm.common import ControlMode, Pose, State
from linkerbot_arm.arm.common._diagnostics import describe_motor
from linkerbot_arm.arm.common._discovery import check_motors
from linkerbot_arm.arm.common._lifecycle import disable_motors
from linkerbot_arm.arm.common._motion import (
    _point_target_interval,
    joint_duration,
    trapezoidal_duration,
    validate_target,
)
from linkerbot_arm.arm.common._stream import (
    _TargetDelay,
    WAYPOINT_INTERVAL,
    plan_linear,
    stream_targets,
)
from linkerbot_arm.comm import CanInterface, CANMessageDispatcher
from linkerbot_arm.exceptions import StateError, ValidationError
from linkerbot_arm.motion_timer import CancellableHomeMixin, MotionTimer

# Joint-space waypoint streaming interval (matches A7's move_l interval).
# 10 ms = 100 Hz; chosen to fit within current CAN dispatcher send-interval
# headroom alongside the V1 motors' polling traffic. The shared trajectory
# executor stretches this period when submission or scheduling is delayed.


def _guard_not_moving(method):
    @functools.wraps(method)
    def wrapper(self: "P7", *args, **kwargs):
        if self.is_moving():
            raise StateError("Cannot start new motion while arm is moving.")
        return method(self, *args, **kwargs)

    return wrapper


def _guard_configuration(method):
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._motion_timer._lock:
            self._require_target_connection()
            owned = self._motion_timer._is_worker_thread()
            if self.is_moving() and not owned:
                raise StateError("Cannot change P7 profiles during an SDK motion")
            result = method(self, *args, **kwargs)
            if not owned:
                self._reset_stream_target()
            return result

    return wrapper


class P7(ArmMotionQueueMixin, CancellableHomeMixin):
    """7-DoF arm with two motor variants on a single CAN bus.

    Joints 0-4 reuse :class:`linkerbot_arm.arm.a7.motor.A7Motor` (CMD-byte
    protocol, drive-side trapezoidal trajectory generation).

    Joints 5-6 use :class:`linkerbot_arm.arm.p7.motor_v2.MotorV2`
    (packed-frame protocol, Motorevo P-T-M / 力位混合 mode with drive-side
    position PD + torque feedforward — no internal trajectory generator).
    Init idempotently writes Flash Control Mode ``0x02`` on those joints.

    To unify behaviour across the mixed motor types, the motion methods
    (:meth:`move_j` / :meth:`move_l`) raise per-joint velocity / acceleration
    limits to their maxima during streaming so the V1 motors' drive-side
    trapezoidal becomes a thin pass-through, then host-stream interpolated
    waypoints that both motor types track in lock-step.
    """

    def __init__(
        self,
        side: Literal["left", "right"],
        interface_name: str,
        interface_type: str = "socketcan",
        tcp_offset: list[float] = [0.0, 0.0, 0.0],
        world_frame: Literal["urdf", "maestro"] = "urdf",
    ) -> None:
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        if world_frame not in ("urdf", "maestro"):
            raise ValueError(
                f"world_frame must be 'urdf' or 'maestro', got {world_frame!r}"
            )
        with arm_dispatcher(
            self, "p7", side, interface_name, interface_type, CANMessageDispatcher
        ) as self._can_dispatcher:
            # Per-side single bus. IDs 51-57 (right) / 61-67 (left). Joints 0-4
            # use the V1 motor protocol (A7Motor); joints 5-6 use the V2 motor
            # protocol (MotorV2). The list order matches the URDF joint order
            # R1..R7 / L1..L7.
            motor_ids = list(range(61, 68) if side == "left" else range(51, 58))
            self._motors: list[A7Motor | MotorV2] = [
                *(A7Motor(mid, self._can_dispatcher) for mid in motor_ids[:5]),
                *(MotorV2(mid, self._can_dispatcher) for mid in motor_ids[5:]),
            ]
            from linkerbot_arm.arm.kinetix import ArmKinetix

            self._kx: ArmKinetix = ArmKinetix.from_builtin(
                "p7", side, tcp_offset=tcp_offset, world_frame=world_frame
            )
            self._control_mode: ControlMode | None = None
            self._motion_timer = MotionTimer()
            self._stream_target_state = None
            self._closed: bool = False
            self._polling: bool = False
            self._check_motors()
            self._ensure_v2_ptm_mode()
            for motor in self._motors:
                motor.read_initial_state()
            for motor in self._motors:
                if not motor.has_initial_data():
                    raise StateError(
                        f"{describe_motor(motor._id)} did not report initial data."
                    )
            self.start_polling()

    def _check_motors(self) -> None:
        check_motors(self._motors)

    def _ensure_v2_ptm_mode(self) -> None:
        """Ensure P-T-M on V2 joints before initial feedback and heartbeats."""
        for motor in self._motors:
            if isinstance(motor, MotorV2):
                motor.ensure_ptm_mode()

    def _on_bus_error(self, error: Exception) -> None:
        self._bus_error = error
        self._closed = True

    @property
    def can_interface(self) -> CanInterface:
        """CAN interface this arm is bound to."""
        return self._can_dispatcher.can_interface

    def start_polling(self, intervals: dict[SensorType, float] | None = None) -> None:
        if self._polling:
            self.stop_polling()
        poll_intervals = intervals if intervals is not None else DEFAULT_POLL_INTERVALS
        for motor in self._motors:
            motor.start_polling(poll_intervals)
        self._polling = True

    def stop_polling(self) -> None:
        for motor in self._motors:
            motor.stop_polling()
        self._polling = False

    def is_moving(self) -> bool:
        return self._motion_timer.is_moving()

    def wait_motion_done(self) -> None:
        self._motion_timer.wait_done()

    def set_control_mode(self, mode: ControlMode) -> None:
        for motor in self._motors:
            motor.set_control_mode(mode)
        self._control_mode = mode

    @_guard_configuration
    def enable(self) -> None:
        self._reset_stream_target()
        self.reset_error()
        control_mode = (
            self._control_mode if self._control_mode is not None else ControlMode.PP
        )
        self.set_control_mode(control_mode)
        for motor in self._motors:
            motor.enable()

        time.sleep(0.1)

    def disable(self) -> None:
        with self._motion_timer._lock:
            self._motion_timer.cancel()
            self._reset_stream_target()
            disable_motors(self._motors)

    def reset_error(self) -> None:
        for motor in self._motors:
            motor.reset_error()

    def emergency_stop(self) -> None:
        with self._motion_timer._lock:
            self._motion_timer.cancel()
            self._reset_stream_target()
            # Use cached ``motor.angle`` (refreshed by each motor's own polling
            # or feedback path) rather than per-motor _read_register_float, which
            # only A7Motor supports. Setting the current angle as the target
            # tells each motor to "hold here".
            for motor in self._motors:
                motor.set_angle(motor.angle.angle)

    def _set_angles(self, angles: list[float], *, check_limits: bool = True) -> None:
        """Admit one FIFO batch; external targets follow configured joint limits.

        The first external cycle seeds from measured positions without moving.
        Later cycles bound target velocity and acceleration using actual elapsed
        monotonic time and admit only one batch per call. This bounds the
        commanded trajectory, not measured motor velocity or physical arrival.
        Owned point trajectories preserve their planned waypoints and defer any
        admission that would exceed the captured profile at the actual interval.
        MoveL preserves its Cartesian path without this joint-space timing check.
        Rejected batches never advance the accepted state.
        """
        with self._motion_timer._lock:
            self._require_target_connection()
            validate_target(self, angles, check_limits=check_limits)
            owned = self._motion_timer._is_worker_thread()
            if self.is_moving() and not owned:
                raise StateError("Cannot stream external targets during an SDK motion")
            profile = getattr(self, "_point_target_profile", None) if owned else None
            if profile is not None:
                previous = self._stream_target_state
                now = time.perf_counter()
                elapsed = max(0.0, now - previous[2])
                interval = _point_target_interval(
                    previous[0], angles, previous[1], *profile, elapsed
                )
                if elapsed == 0 or interval > elapsed:
                    raise _TargetDelay(max(interval - elapsed, math.ulp(now)))
                state = (
                    list(angles),
                    [
                        (goal - start) / elapsed
                        for start, goal in zip(previous[0], angles)
                    ],
                    now,
                )
            else:
                state = None if owned else self._limit_stream_target(angles)
            accepted = angles if owned else state[0]
            validate_target(self, accepted, check_limits=check_limits)
            submit_targets(
                motor._prepare_angle(angle)
                for motor, angle in zip(self._motors, accepted)
            )
            self._stream_target_state = state

    def _require_target_connection(self) -> None:
        if getattr(self, "_closed", False) or (
            getattr(self, "_closing", False)
            and not self._motion_timer._is_worker_thread()
        ):
            raise StateError("P7 connection is closed or closing")

    def _reset_stream_target(self) -> None:
        self._stream_target_state = None

    def _limit_stream_target(self, target):
        now = time.monotonic()
        previous = getattr(self, "_stream_target_state", None)
        positions = self.get_angles() if previous is None else previous[0]
        speeds = self.get_control_velocities()
        accelerations = self.get_control_acceleration()
        # Validate feedback/counts and reject unusable limits before admission.
        self._move_duration(positions, target, speeds, accelerations)
        if previous is None:
            return (list(positions), [0.0] * NUM_JOINTS, now)
        elapsed = max(0.0, now - previous[2])
        if elapsed == 0:
            return (list(positions), list(previous[1]), previous[2])
        dt = elapsed
        limited, velocities = [], []
        for position, velocity, goal, speed, acceleration in zip(
            positions, previous[1], target, speeds, accelerations
        ):
            error = goal - position
            if error == 0 and velocity == 0:
                limited.append(position)
                velocities.append(0.0)
                continue
            if not math.isfinite(acceleration) or acceleration <= 0:
                raise StateError("Streaming motion requires positive acceleration")
            # Reserve a deceleration step near the target instead of endlessly
            # oscillating across it. A target reversal still brakes gradually.
            braking_speed = (
                math.sqrt((acceleration * dt) ** 2 + 2 * acceleration * abs(error))
                - acceleration * dt
            )
            desired = math.copysign(min(speed, braking_speed, abs(error) / dt), error)
            change = acceleration * dt
            next_velocity = max(velocity - change, min(velocity + change, desired))
            delta = next_velocity * dt
            next_position = position + delta
            limited.append(next_position)
            # Derive both this velocity and the next cycle's acceleration from
            # the same accepted position/time samples, including long pauses.
            velocities.append((next_position - position) / elapsed)
        return (limited, velocities, now)

    @_guard_configuration
    def set_velocities(self, velocities: list[float]) -> None:
        if len(velocities) != len(self._motors):
            raise ValueError(
                f"Velocities count must be {NUM_JOINTS}, got {len(velocities)}"
            )
        for i, v in enumerate(velocities):
            if not (MIN_VELOCITY <= v <= MAX_VELOCITY):
                raise ValidationError(
                    f"Joint {i} velocity {v} out of range [{MIN_VELOCITY}, {MAX_VELOCITY}]"
                )
        for motor, velocity in zip(self._motors, velocities):
            motor.set_velocity(velocity)

    @_guard_configuration
    def set_accelerations(self, accelerations: list[float]) -> None:
        if len(accelerations) != len(self._motors):
            raise ValueError(
                f"Accelerations count must be {NUM_JOINTS}, got {len(accelerations)}"
            )
        for i, a in enumerate(accelerations):
            if not (MIN_ACCELERATION <= a <= MAX_ACCELERATION):
                raise ValidationError(
                    f"Joint {i} acceleration {a} out of range [{MIN_ACCELERATION}, {MAX_ACCELERATION}]"
                )
        for motor, acceleration in zip(self._motors, accelerations):
            motor.set_acceleration(acceleration)
            motor.set_deceleration(acceleration)

    def get_feedback(self) -> ArmFeedback:
        """Read cached measured joints with monotonic reception times, without I/O."""
        return read_feedback(self)

    def get_state(self) -> State:
        return State(
            pose=self.get_pose(),
            joint_angles=[motor.angle for motor in self._motors],
            joint_control_angles=[motor.control_angle for motor in self._motors],
            joint_velocities=[motor.velocity for motor in self._motors],
            joint_control_velocities=[motor.control_velocity for motor in self._motors],
            joint_control_acceleration=[
                motor.control_acceleration for motor in self._motors
            ],
            joint_torques=[motor.torque for motor in self._motors],
            joint_temperatures=[motor.temperature for motor in self._motors],
        )

    def get_angles(self) -> list[float]:
        return [motor.angle.angle for motor in self._motors]

    def get_control_angles(self) -> list[float]:
        return [motor.control_angle.angle for motor in self._motors]

    def get_velocities(self) -> list[float]:
        return [motor.velocity.velocity for motor in self._motors]

    def get_control_velocities(self) -> list[float]:
        return [motor.control_velocity.velocity for motor in self._motors]

    def get_control_acceleration(self) -> list[float]:
        return [motor.control_acceleration.acceleration for motor in self._motors]

    def get_torques(self) -> list[float]:
        return [motor.torque.torque for motor in self._motors]

    def get_temperatures(self) -> list[float]:
        return [motor.temperature.temperature for motor in self._motors]

    def get_pose(self) -> Pose:
        return self._kx.forward_kinematics(self.get_angles())

    def home(self, *, blocking: bool = True) -> None:
        self.move_j([0.0] * NUM_JOINTS, blocking=blocking)

    @_guard_not_moving
    def move_j(
        self,
        target_joints: list[float],
        *,
        blocking: bool = True,
    ) -> None:
        # V2 joints require host interpolation; retain synchronous streaming
        # even when blocking=False, which only omits the final settling wait.
        with self._motion_timer._stream_scope() as active:
            if not active:
                return
            self._execute_joint_target(target_joints, blocking=blocking)

    def _execute_joint_target(self, target_joints, *, blocking=True):
        """Execute in an already owned stream scope (also used after IK)."""
        validate_target(self, target_joints)
        try:
            start = self.get_angles()
            speeds = self.get_control_velocities()
            accelerations = self.get_control_acceleration()
            duration = self._move_duration(start, target_joints, speeds, accelerations)
        except AttributeError as error:
            raise StateError(
                "Motion feedback or control parameters are unavailable"
            ) from error
        if self._motion_timer._wait_cancelled(0):
            return
        # Each joint's original trapezoid is slowed to the longest duration;
        # this lowers both velocity and acceleration while synchronizing arrival.
        profiles = [
            self._kx._trap_params(abs(goal - position), speed, acceleration)
            for position, goal, speed, acceleration in zip(
                start, target_joints, speeds, accelerations
            )
        ]
        steps = max(1, math.ceil(duration / WAYPOINT_INTERVAL))

        def waypoints():
            for step in range(1, steps + 1):
                t = min(step * WAYPOINT_INTERVAL, duration)
                yield [
                    goal
                    if duration == 0
                    else position
                    + (goal - position)
                    * self._kx._trap_s(
                        t * profile[3] / duration,
                        profile[0],
                        profile[1],
                        profile[2],
                        acceleration,
                        abs(goal - position),
                    )
                    for position, goal, acceleration, profile in zip(
                        start, target_joints, accelerations, profiles
                    )
                ]

        try:
            stream_targets(
                self,
                waypoints(),
                max_velocity=MAX_VELOCITY,
                max_acceleration=MAX_ACCELERATION,
                settle=blocking,
                joint_profile=(start, speeds, accelerations),
            )
        finally:
            self._reset_stream_target()

    @_guard_not_moving
    def move_p(
        self,
        target_pose: Pose,
        *,
        current_angles: list[float] | None = None,
        blocking: bool = True,
    ) -> None:
        with self._motion_timer._stream_scope() as active:
            if not active:
                return
            angles = self.get_angles() if current_angles is None else current_angles
            target = self.inverse_kinematics(target_pose, current_angles=angles)
            if not self._motion_timer._wait_cancelled(0):
                self._execute_joint_target(target, blocking=blocking)

    @_guard_not_moving
    def move_l(
        self,
        target_pose: Pose,
        *,
        max_velocity: float = MOVE_L_DEFAULT_MAX_VELOCITY,
        max_angular_velocity: float = MOVE_L_DEFAULT_MAX_ANGULAR_VELOCITY,
        acceleration: float = MOVE_L_DEFAULT_ACCELERATION,
        angular_acceleration: float = MOVE_L_DEFAULT_ANGULAR_ACCELERATION,
        current_pose: Pose | None = None,
        current_angles: list[float] | None = None,
    ) -> None:
        if not (0 < max_velocity <= MOVE_L_MAX_MAX_VELOCITY):
            raise ValidationError(
                f"max_velocity {max_velocity} out of range (0, {MOVE_L_MAX_MAX_VELOCITY}]"
            )
        if not (0 < max_angular_velocity <= MOVE_L_MAX_MAX_ANGULAR_VELOCITY):
            raise ValidationError(
                f"max_angular_velocity {max_angular_velocity} out of range (0, {MOVE_L_MAX_MAX_ANGULAR_VELOCITY}]"
            )
        if not (0 < acceleration <= MOVE_L_MAX_ACCELERATION):
            raise ValidationError(
                f"acceleration {acceleration} out of range (0, {MOVE_L_MAX_ACCELERATION}]"
            )
        if not (0 < angular_acceleration <= MOVE_L_MAX_ANGULAR_ACCELERATION):
            raise ValidationError(
                f"angular_acceleration {angular_acceleration} out of range (0, {MOVE_L_MAX_ANGULAR_ACCELERATION}]"
            )

        with self._motion_timer._stream_scope() as active:
            if not active:
                return
            if current_angles is not None and current_pose is not None:
                fk_pose = self._kx.forward_kinematics(current_angles)
                if not self._poses_close(fk_pose, current_pose):
                    warnings.warn(
                        "current_pose does not match FK(current_angles). "
                        "Using FK result instead.",
                        stacklevel=2,
                    )
                    current_pose = fk_pose
            elif current_angles is not None:
                current_pose = self._kx.forward_kinematics(current_angles)
            elif current_pose is not None:
                raise ValueError(
                    "current_angles must be provided when current_pose is specified."
                )
            else:
                current_angles = self.get_angles()
                current_pose = self._kx.forward_kinematics(current_angles)

            waypoints = plan_linear(
                self,
                current_pose,
                current_angles,
                target_pose,
                max_velocity=max_velocity,
                acceleration=acceleration,
                max_angular_velocity=max_angular_velocity,
                angular_acceleration=angular_acceleration,
            )
            if waypoints:
                stream_targets(
                    self,
                    waypoints,
                    max_velocity=MAX_VELOCITY,
                    max_acceleration=MAX_ACCELERATION,
                )

    def set_position_kps(self, kps: list[float]) -> None:
        if len(kps) != len(self._motors):
            raise ValueError(f"KPS count must be {NUM_JOINTS}, got {len(kps)}")
        for motor, kp in zip(self._motors, kps):
            motor.set_position_kp(kp)

    def set_velocity_kps(self, kps: list[float]) -> None:
        if len(kps) != len(self._motors):
            raise ValueError(f"KPS count must be {NUM_JOINTS}, got {len(kps)}")
        for motor, kp in zip(self._motors, kps):
            motor.set_velocity_kp(kp)

    def set_velocity_kis(self, kis: list[float]) -> None:
        if len(kis) != len(self._motors):
            raise ValueError(f"KIS count must be {NUM_JOINTS}, got {len(kis)}")
        for motor, ki in zip(self._motors, kis):
            motor.set_velocity_ki(ki)

    def calibrate_zero(self) -> None:
        self.reset_error()
        for motor in self._motors:
            motor.calibrate_zero()

    def forward_kinematics(self, angles: list[float]) -> Pose:
        return self._kx.forward_kinematics(angles)

    def inverse_kinematics(
        self,
        pose: Pose,
        *,
        current_angles: list[float] | None = None,
    ) -> list[float]:
        current_angles = (
            current_angles if current_angles is not None else self.get_angles()
        )
        return self._kx.inverse_kinematics(pose, current_angles=current_angles)

    def set_joint_limits(self, limits: list[tuple[float, float]]) -> None:
        self._kx.set_joint_limits(limits)

    def get_joint_limits(self) -> list[tuple[float, float]]:
        return self._kx.get_joint_limits()

    def close(self) -> None:
        """Release CAN resources; failures are raised and close may be retried."""
        close_arm(self)

    def __enter__(self) -> "P7":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return exit_arm(self, exc_val)

    @staticmethod
    def _poses_close(
        a: Pose, b: Pose, pos_tol: float = 1e-3, rot_tol: float = 1e-2
    ) -> bool:
        pos_diff = np.linalg.norm(np.array([a.x, a.y, a.z]) - np.array([b.x, b.y, b.z]))
        rot_a = R.from_euler("zyx", [a.rz, a.ry, a.rx])
        rot_b = R.from_euler("zyx", [b.rz, b.ry, b.rx])
        rot_diff = (rot_a.inv() * rot_b).magnitude()
        return bool(pos_diff < pos_tol and rot_diff < rot_tol)

    @staticmethod
    def _trapezoidal_duration(
        distance: float, speed: float, acceleration: float
    ) -> float:
        return trapezoidal_duration(distance, speed, acceleration)

    def _move_duration(
        self,
        current_angles: list[float],
        target_angles: list[float],
        control_speeds: list[float],
        control_accelerations: list[float],
    ) -> float:
        return joint_duration(
            current_angles, target_angles, control_speeds, control_accelerations
        )

    def save_params(self) -> None:
        for motor in self._motors:
            motor._save_params()
