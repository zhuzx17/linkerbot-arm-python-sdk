"""Ordered host trajectories with no catch-up bursts after planning or stalls."""

import logging
import math
import time

from linkerbot_arm.arm.common._motion import validate_target
from linkerbot_arm.exceptions import StateError

logger = logging.getLogger(__name__)
WAYPOINT_INTERVAL = 0.01


class _TargetDelay(Exception):
    """Retry the same owned waypoint after a cancellable profile delay."""

    def __init__(self, seconds):
        self.seconds = seconds


def plan_linear(arm, current_pose, current_angles, target_pose, **kwargs):
    """Finish and validate the whole plan before modifying drive parameters."""
    waypoints = []
    for waypoint in arm._kx.plan_move_l(
        current_pose,
        current_angles,
        target_pose,
        waypoint_interval=WAYPOINT_INTERVAL,
        **kwargs,
    ):
        if arm._motion_timer._wait_cancelled(0):
            return []
        angles = list(waypoint.angles)
        validate_target(arm, angles)
        waypoints.append(angles)
    return waypoints


def _profile_bounds(maximum, count):
    values = [maximum] * count if isinstance(maximum, (int, float)) else list(maximum)
    if len(values) != count or any(not math.isfinite(v) or v <= 0 for v in values):
        raise StateError("Invalid per-joint streaming profile bounds")
    return values


def _read_limits(
    arm,
    max_velocity,
    max_acceleration,
    *,
    min_acceleration=1.0,
    independent_deceleration=False,
):
    try:
        velocities = arm.get_control_velocities()
        accelerations = arm.get_control_acceleration()
        decelerations = (
            arm.get_control_decelerations() if independent_deceleration else None
        )
    except AttributeError as error:
        raise StateError(
            "Control limits unavailable; read or set them before motion"
        ) from error
    fields = [
        ("velocity", velocities, 0, max_velocity),
        ("acceleration", accelerations, min_acceleration, max_acceleration),
    ]
    if decelerations is not None:
        fields.append(
            ("deceleration", decelerations, min_acceleration, max_acceleration)
        )
    for name, values, minimum, maxima in fields:
        if len(values) != len(arm._motors) or any(
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not minimum <= value <= maximum
            for value, maximum in zip(values, maxima)
        ):
            raise StateError(
                f"Cannot restore invalid control {name}; set valid limits before motion"
            )
    return (
        list(velocities),
        list(accelerations),
        None if decelerations is None else list(decelerations),
    )


def stream_targets(
    arm,
    waypoints,
    *,
    max_velocity,
    max_acceleration,
    settle=True,
    min_acceleration=1.0,
    independent_deceleration=False,
    joint_profile=None,
):
    """Execute in an owned MotionTimer._stream_scope; preserve every waypoint.

    Wait one interval after each submission completes, including the initial
    configuration. Slow submission/scheduling stretches time instead of sending
    overdue points back-to-back. SDK admission is not physical completion.
    A joint_profile also checks real admission intervals and can defer the same
    point without changing its position or waiting while holding the motion lock.
    """
    timer = arm._motion_timer
    if timer._wait_cancelled(0):
        return
    max_velocities = _profile_bounds(max_velocity, len(arm._motors))
    max_accelerations = _profile_bounds(max_acceleration, len(arm._motors))
    velocities, accelerations, decelerations = _read_limits(
        arm,
        max_velocities,
        max_accelerations,
        min_acceleration=min_acceleration,
        independent_deceleration=independent_deceleration,
    )
    changed_velocity = changed_acceleration = False
    original_error = None
    try:
        with timer._lock:
            if timer._wait_cancelled(0):
                return
            # Set the flag first: a per-joint configuration can fail partway.
            changed_velocity = True
            arm.set_velocities(max_velocities)
            if timer._wait_cancelled(0):
                return
            changed_acceleration = True
            arm.set_accelerations(max_accelerations)
            if joint_profile is not None:
                start, speeds, ramps = joint_profile
                arm._point_target_profile = (speeds, ramps)
                arm._stream_target_state = (
                    list(start),
                    [0.0] * len(start),
                    time.perf_counter(),
                )
        for angles in waypoints:
            deadline = time.perf_counter() + WAYPOINT_INTERVAL
            while True:
                if timer._wait_cancelled(max(0.0, deadline - time.perf_counter())):
                    return
                if time.perf_counter() >= deadline:
                    break
            while True:
                try:
                    if not timer._run_unless_cancelled(
                        lambda: arm._set_angles(angles, check_limits=False)
                    ):
                        return
                    break
                except _TargetDelay as deferred:
                    # Admission releases the motion lock before waiting, so
                    # stop/disable can still cancel a delayed point immediately.
                    if timer._wait_cancelled(deferred.seconds):
                        return
        if settle:
            timer._wait_cancelled(WAYPOINT_INTERVAL)
    except BaseException as error:
        original_error = error
        raise
    finally:
        # Stop/disable use the same lock. In particular, an A7lite stop's
        # temporary zero-speed hold must not be overwritten midway by cleanup.
        errors = []
        with timer._lock:
            if joint_profile is not None:
                arm._point_target_profile = None
            restores = [
                (changed_velocity, arm.set_velocities, velocities),
                (changed_acceleration, arm.set_accelerations, accelerations),
            ]
            if decelerations is not None:
                restores.append(
                    (changed_acceleration, arm.set_decelerations, decelerations)
                )
            for changed, restore, values in restores:
                if changed:
                    try:
                        restore(values)
                    except BaseException as error:
                        errors.append(error)
                        logger.error("Failed to restore motion parameters: %s", error)
        if errors and original_error is None:
            raise errors[0]
