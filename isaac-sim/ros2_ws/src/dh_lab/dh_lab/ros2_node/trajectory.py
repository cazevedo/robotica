"""Velocity-limited joint-space interpolation.

This asset's articulation only actually responds to *position* commands
-- velocity and effort commands are wired on ``joint_command`` but
produce no real motion, because every joint's drive already has
significant position-holding stiffness/damping baked in (see CLAUDE.md).
So "send a velocity-limited trajectory, not an instantaneous jump" means
*this* module generates a sequence of intermediate position setpoints
and the node publishes them one at a time at a fixed control rate --
it is not something the bridge does for us.

Pure Python, no rclpy: importable and testable standalone.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TrajectoryStep:
    positions: dict  # joint_name -> radians
    time_from_start: float  # seconds


def interpolate_trajectory(
    start: dict,
    target: dict,
    max_speed_rad_s: float,
    speed_scale: float,
    control_rate_hz: float = 50.0,
) -> list:
    """Linear, constant-velocity-per-joint interpolation from ``start``
    to ``target`` (both ``{joint_name: radians}``, same keys), slow
    enough that no joint exceeds ``max_speed_rad_s * speed_scale``.

    Returns a list of :class:`TrajectoryStep`, evenly spaced at
    ``control_rate_hz`` starting one step after ``start``, always ending
    with the exact ``target`` as the last step (never overshoots or
    undershoots due to step rounding).
    """
    if start.keys() != target.keys():
        raise ValueError("start and target must give a value for exactly the same joints")
    if not (0.0 < speed_scale <= 1.0):
        raise ValueError(f"speed_scale must be in (0, 1], got {speed_scale}")
    allowed_speed = max_speed_rad_s * speed_scale
    if allowed_speed <= 0.0:
        raise ValueError(f"max_speed_rad_s * speed_scale must be positive, got {allowed_speed}")

    max_delta = max((abs(target[name] - start[name]) for name in start), default=0.0)
    duration = max_delta / allowed_speed

    if duration <= 0.0:
        return [TrajectoryStep(positions=dict(target), time_from_start=0.0)]

    dt = 1.0 / control_rate_hz
    steps = []
    t = dt
    while t < duration:
        alpha = t / duration
        steps.append(TrajectoryStep(
            positions={name: start[name] + alpha * (target[name] - start[name]) for name in start},
            time_from_start=t,
        ))
        t += dt
    steps.append(TrajectoryStep(positions=dict(target), time_from_start=duration))
    return steps


@dataclass(frozen=True)
class ClampResult:
    clamped: dict  # joint_name -> radians, within [lower, upper]
    out_of_range: dict  # joint_name -> (requested, lower, upper), only entries that were clamped


def clamp_configuration(q_by_name: dict, joint_limits: dict) -> ClampResult:
    """Clamp every joint in ``q_by_name`` to ``joint_limits[name] =
    (lower, upper)`` (radians). This is the last-resort safety net before
    anything is sent to the robot -- it always runs, regardless of
    whether the GUI already validated the entry; the GUI additionally
    *rejects* out-of-range entry with a visible message rather than
    silently clamping (spec section 9), which is a separate, earlier
    check this function does not perform or assume.
    """
    clamped = {}
    out_of_range = {}
    for name, value in q_by_name.items():
        lower, upper = joint_limits[name]
        clamped_value = min(max(value, lower), upper)
        clamped[name] = clamped_value
        if clamped_value != value:
            out_of_range[name] = (value, lower, upper)
    return ClampResult(clamped=clamped, out_of_range=out_of_range)
