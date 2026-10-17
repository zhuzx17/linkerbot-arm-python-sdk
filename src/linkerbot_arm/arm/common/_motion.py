"""Shared validation and timing for arm motion commands."""

import math
import struct

from linkerbot_arm.arm.common import ControlMode
from linkerbot_arm.exceptions import StateError, ValidationError


def validate_target(arm, angles, *, check_limits=True):
    """Validate without submitting or modifying accepted-target caches."""
    modes = getattr(arm, "_control_modes", None)
    if modes is not None and ControlMode.MIT in modes:
        raise StateError(
            "Register angle command is unavailable while any joint is in "
            "MIT mode; use set_mits() or stream_joint_targets()"
        )
    if len(angles) != len(arm._motors):
        raise ValueError(f"Angles count must be {len(arm._motors)}, got {len(angles)}")
    for i, angle in enumerate(angles):
        try:
            if not math.isfinite(angle):
                raise ValueError("non-finite angle")
            struct.pack("<f", angle)
        except (TypeError, ValueError, OverflowError, struct.error) as error:
            raise ValidationError(
                f"Joint {i} angle must be a finite float32 value"
            ) from error
    if check_limits:
        limits = arm._kx.get_joint_limits()
        if len(limits) != len(angles):
            raise StateError("Joint limit count must match the joint count")
        for i, (angle, (lo, hi)) in enumerate(zip(angles, limits)):
            if not lo <= angle <= hi:
                raise ValidationError(
                    f"Joint {i} angle {angle:.4f} rad out of range [{lo:.4f}, {hi:.4f}]"
                )


def trapezoidal_duration(distance, speed, acceleration):
    if not math.isfinite(distance) or distance < 0:
        raise StateError("Motion distance must be finite and non-negative")
    if distance < 1e-12:
        return 0.0
    if any(
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(value)
        or value <= 0
        for value in (speed, acceleration)
    ):
        raise StateError(
            "Nonzero motion requires finite positive control velocity and acceleration; "
            "set_velocities() and set_accelerations() before moving"
        )
    # Compare times to avoid squaring a very large velocity limit.
    t_ramp = math.sqrt(distance) / math.sqrt(acceleration)
    if speed / acceleration >= t_ramp:
        duration = 2 * t_ramp
    else:
        duration = distance / speed + speed / acceleration
    if not math.isfinite(duration):
        raise StateError("Motion duration is not finite; check control parameters")
    return duration


def joint_duration(current, target, speeds, accelerations):
    if not (len(current) == len(target) == len(speeds) == len(accelerations)):
        raise StateError("Motion state and parameter counts must match the joint count")
    durations = []
    for i, (cur, tgt, speed, acceleration) in enumerate(
        zip(current, target, speeds, accelerations), 1
    ):
        if not math.isfinite(cur) or not math.isfinite(tgt):
            raise StateError(f"Joint {i} motion position must be finite")
        try:
            durations.append(trapezoidal_duration(abs(tgt - cur), speed, acceleration))
        except StateError as error:
            raise StateError(f"Joint {i}: {error}") from error
    return max(durations, default=0.0)


def point_duration(arm, target):
    try:
        return arm._move_duration(
            arm.get_angles(),
            target,
            arm.get_control_velocities(),
            arm.get_control_acceleration(),
        )
    except AttributeError as error:
        raise StateError(
            "Motion feedback or control parameters are unavailable; "
            "read initial state and set_velocities()/set_accelerations() before moving"
        ) from error


def _point_target_interval(
    previous, target, velocities, speeds, accelerations, elapsed
):
    """Find a shared interval that keeps an unchanged joint waypoint admissible.

    For a displacement d and previous signed velocity v, the next sample has
    velocity d / dt. Both |d / dt| <= speed and |d / dt - v| <= a * dt must hold.
    Deceleration can exclude an interval between two roots; after a scheduling
    stall, wait past that interval rather than dropping or changing the point.
    """
    if elapsed > 0 and all(
        abs((goal - start) / elapsed) <= speed + 1e-12
        and abs((goal - start) / elapsed - velocity) <= (acceleration + 1e-9) * elapsed
        for start, goal, velocity, speed, acceleration in zip(
            previous, target, velocities, speeds, accelerations
        )
    ):
        # Roundoff at a deceleration root must not turn a valid nominal cycle
        # into a long wait across the quadratic's excluded interval.
        return elapsed
    interval = elapsed
    gaps = []
    for start, goal, velocity, speed, acceleration in zip(
        previous, target, velocities, speeds, accelerations
    ):
        delta = goal - start
        distance = abs(delta)
        if distance == 0:
            interval = max(interval, abs(velocity) / acceleration)
            continue
        if speed <= 0:
            raise StateError("Nonzero motion requires positive control velocity")
        velocity = math.copysign(1.0, delta) * velocity
        root = math.sqrt(velocity * velocity + 4 * acceleration * distance)
        ramp = (
            2 * distance / (root + velocity)
            if velocity >= 0
            else (root - velocity) / (2 * acceleration)
        )
        interval = max(interval, distance / speed, ramp)
        discriminant = velocity * velocity - 4 * acceleration * distance
        if velocity > 0 and discriminant > 0:
            root = math.sqrt(discriminant)
            gaps.append(
                (
                    2 * distance / (velocity + root),
                    (velocity + root) / (2 * acceleration),
                    distance,
                    velocity,
                    acceleration,
                )
            )
    # Increasing the shared interval can enter another joint's excluded range.
    # Each extension exits at least one range: at most joint_count extensions.
    while True:
        extended = max(
            (
                upper
                for lower, upper, distance, velocity, acceleration in gaps
                if lower < interval < upper
                and abs(distance / interval - velocity)
                > (acceleration + 1e-9) * interval
            ),
            default=interval,
        )
        if extended == interval:
            return interval
        interval = extended
