"""Time parameterisation: a joint path in, a timed trajectory out.

The cycle-time number this repository reports is the sum of the durations this
module produces, so how it computes them is the whole of that claim.

THE ALGORITHM

The path is a polyline in joint space. Parameterise it by a scalar ``s`` that
runs from 0 to 1 along the path, and the problem becomes finding a speed
profile ``s_dot(s)`` subject to:

* **Velocity.** On segment *i* the joints move along a fixed direction
  ``u_i = dq/ds``, so ``s_dot <= min_j (v_j / |u_ij|)``.
* **Acceleration.** Along a straight segment ``q_ddot = u_i * s_ddot``, giving
  ``|s_ddot| <= min_j (a_j / |u_ij|)``.
* **Corners.** At a waypoint the direction changes discontinuously. Passing
  through at speed ``s_dot`` demands a joint velocity step of
  ``s_dot * |u_out - u_in|``, which no finite acceleration delivers. Real
  controllers blend the corner over a short time; this bounds the speed so the
  step fits inside ``CORNER_BLEND_TIME`` at the acceleration limit.

A forward and a backward pass then propagate those bounds along the path -- the
standard time-optimal parameterisation of a fixed path -- and each segment is
integrated exactly as a constant-acceleration trapezoid.

WHAT THE FIRST VERSION DID INSTEAD, AND WHY IT IS WORTH RECORDING

It assigned each segment its velocity-limited duration and then iteratively
stretched any segment whose implied acceleration was too high. That diverges on
a finely sampled path: stretching one segment lowers its speed, which *creates*
a velocity discontinuity with its neighbour, which demands stretching that one,
and so on. A 100 mm Cartesian approach sampled every 5 mm came out at nine
seconds, and the cycle time it produced -- 34 s for a pick and place a UR5e
does in under ten -- was wrong in a direction that happens to look
conservative, which is the easiest kind of wrong number to publish.

Everything is scaled by ``velocity_scale`` and ``acceleration_scale``, which
default to 0.5 and 0.35 -- not 1.0. A UR5e running a pick and place at its
datasheet limits is not something anyone commissions in a cell with a person
near it, and quoting a cycle time obtained at 100% of the limits would be
quoting a number nobody would run.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from cell_motion.core.kinematics import ACCELERATION_LIMITS, VELOCITY_LIMITS

DEFAULT_VELOCITY_SCALE = 0.5
DEFAULT_ACCELERATION_SCALE = 0.35

# How long a controller is allowed to take to swing the joint velocity vector
# through a corner, in seconds. A typical industrial blend parameter.
CORNER_BLEND_TIME = 0.06


@dataclass
class Trajectory:
    """A timed joint trajectory."""

    times: np.ndarray  # (N,) seconds, starting at 0.
    positions: np.ndarray  # (N, 6) radians.
    label: str = ""

    @property
    def duration(self) -> float:
        return float(self.times[-1]) if len(self.times) else 0.0

    def velocities(self) -> np.ndarray:
        """Finite-difference joint velocities, one per waypoint."""
        if len(self.times) < 2:
            return np.zeros_like(self.positions)
        deltas = np.diff(self.positions, axis=0)
        durations = np.diff(self.times)[:, None]
        segment = deltas / np.maximum(durations, 1e-9)
        return np.vstack([segment[:1], (segment[:-1] + segment[1:]) / 2.0, segment[-1:]])

    def peak_velocity_fraction(self, scale: float = DEFAULT_VELOCITY_SCALE) -> float:
        """Highest joint velocity as a fraction of the scaled limit.

        At or just under 1.0 on a long move. Materially above it means the
        parameterisation is not respecting its own limits, which
        ``tests/test_trajectory.py`` asserts it does.
        """
        if len(self.times) < 2:
            return 0.0
        return float(np.max(np.abs(self.velocities()) / (VELOCITY_LIMITS * scale)))

    def peak_acceleration_fraction(self, scale: float = DEFAULT_ACCELERATION_SCALE) -> float:
        """Highest joint acceleration as a fraction of the scaled limit.

        Measured between waypoint velocities, so it includes the corner
        blending the profile allows for, and can sit slightly above 1.0 at a
        sharp corner of an unsmoothed path.
        """
        if len(self.times) < 3:
            return 0.0
        velocities = self.velocities()
        gaps = np.maximum(np.diff(self.times)[:, None], 1e-9)
        accelerations = np.diff(velocities, axis=0) / gaps
        return float(np.max(np.abs(accelerations) / (ACCELERATION_LIMITS * scale)))

    def sample(self, time: float) -> np.ndarray:
        """Linear interpolation of the path at a time."""
        joints = range(self.positions.shape[1])
        return np.array([np.interp(time, self.times, self.positions[:, joint]) for joint in joints])


def time_parameterise(
    path: Sequence[np.ndarray],
    velocity_scale: float = DEFAULT_VELOCITY_SCALE,
    acceleration_scale: float = DEFAULT_ACCELERATION_SCALE,
    label: str = "",
) -> Trajectory:
    """Assign times to a joint path under velocity and acceleration limits."""
    positions = np.asarray([np.asarray(point, dtype=float) for point in path])
    if len(positions) == 0:
        return Trajectory(np.zeros(0), np.zeros((0, 6)), label)
    if len(positions) == 1:
        return Trajectory(np.zeros(1), positions, label)

    velocity_limit = VELOCITY_LIMITS * velocity_scale
    acceleration_limit = ACCELERATION_LIMITS * acceleration_scale

    deltas = np.diff(positions, axis=0)
    # Drop zero-length segments: they contribute nothing but a division by zero
    # and a direction vector of NaNs. RRT shortcutting produces them whenever
    # two waypoints coincide.
    keep = np.max(np.abs(deltas), axis=1) > 1e-9
    if not np.any(keep):
        return Trajectory(np.zeros(1), positions[:1], label)
    positions = np.vstack([positions[:1], positions[1:][keep]])
    deltas = np.diff(positions, axis=0)

    # Path parameter: each segment gets a share of s proportional to the time it
    # would take on its own at the velocity limit. That makes the profile below
    # act on a parameter whose natural speed is uniform, which keeps the
    # corner-speed bounds comparable between a long shoulder sweep and a short
    # wrist twist.
    segment_times = np.max(np.abs(deltas) / velocity_limit, axis=1)
    total = float(np.sum(segment_times))
    ds = segment_times / total
    directions = deltas / ds[:, None]  # dq/ds on each segment.

    # Per-segment bounds on s_dot and s_ddot.
    speed_limit = np.min(velocity_limit / np.maximum(np.abs(directions), 1e-12), axis=1)
    accel_limit = np.min(acceleration_limit / np.maximum(np.abs(directions), 1e-12), axis=1)

    count = len(positions)
    waypoint_speed = np.empty(count)
    waypoint_speed[0] = 0.0
    waypoint_speed[-1] = 0.0
    for index in range(1, count - 1):
        change = np.abs(directions[index] - directions[index - 1])
        corner = np.min(acceleration_limit * CORNER_BLEND_TIME / np.maximum(change, 1e-12))
        waypoint_speed[index] = min(corner, speed_limit[index - 1], speed_limit[index])

    # Forward pass: cannot arrive faster than accelerating from the last one.
    for index in range(1, count):
        reachable = np.sqrt(waypoint_speed[index - 1] ** 2 + 2.0 * accel_limit[index - 1] * ds[index - 1])
        waypoint_speed[index] = min(waypoint_speed[index], reachable)
    # Backward pass: cannot leave faster than can be braked to the next one.
    for index in range(count - 2, -1, -1):
        reachable = np.sqrt(waypoint_speed[index + 1] ** 2 + 2.0 * accel_limit[index] * ds[index])
        waypoint_speed[index] = min(waypoint_speed[index], reachable)

    times = np.zeros(count)
    for index in range(count - 1):
        times[index + 1] = times[index] + _segment_duration(
            waypoint_speed[index], waypoint_speed[index + 1], ds[index], speed_limit[index], accel_limit[index]
        )
    return Trajectory(times=times, positions=positions, label=label)


def _segment_duration(entry: float, exit: float, length: float, speed_limit: float, acceleration: float) -> float:
    """Exact duration of a constant-acceleration segment, trapezoid or triangle."""
    if length <= 0.0:
        return 0.0
    if acceleration <= 1e-12:
        return length / max(min(entry, exit), 1e-9)

    # Peak speed if the segment accelerates then immediately decelerates.
    peak_squared = (entry**2 + exit**2) / 2.0 + acceleration * length
    peak = float(np.sqrt(max(peak_squared, 0.0)))
    if peak <= speed_limit:
        return float((peak - entry) / acceleration + (peak - exit) / acceleration)

    # Trapezoid: accelerate to the limit, cruise, decelerate.
    ramp_up = (speed_limit**2 - entry**2) / (2.0 * acceleration)
    ramp_down = (speed_limit**2 - exit**2) / (2.0 * acceleration)
    cruise = max(length - ramp_up - ramp_down, 0.0)
    return float(
        (speed_limit - entry) / acceleration + (speed_limit - exit) / acceleration + cruise / speed_limit
    )


def resample(trajectory: Trajectory, period: float = 0.008) -> Trajectory:
    """Resample onto a fixed control period, 125 Hz by default.

    The UR control interface takes servoj commands at 125 Hz, so a trajectory
    that is going to be streamed to a real controller has to exist at that rate
    somewhere. Nothing offline needs this; the ROS node does, and it is here so
    that the node and the benchmark share one implementation.
    """
    if trajectory.duration <= 0.0:
        return trajectory
    times = np.arange(0.0, trajectory.duration + period, period)
    positions = np.stack([trajectory.sample(time) for time in times])
    return Trajectory(times=times, positions=positions, label=trajectory.label)


@dataclass
class CycleSegment:
    """One timed piece of the cycle, for the cycle-time breakdown."""

    name: str
    duration: float
    kind: str  # "motion", "gripper", "sensor", "fieldbus", "compute"

    def as_dict(self) -> dict:
        return {"name": self.name, "duration": round(self.duration, 4), "kind": self.kind}


def total_duration(segments: Sequence[CycleSegment]) -> float:
    return float(sum(segment.duration for segment in segments))


def duration_by_kind(segments: Sequence[CycleSegment]) -> dict:
    """Cycle time split by what it was spent on.

    The split is the useful part. A cycle time that is 70% motion is improved
    by tuning the trajectory; one that is 40% waiting for the camera is not,
    and that distinction is invisible in a single total.
    """
    totals: dict = {}
    for segment in segments:
        totals[segment.kind] = totals.get(segment.kind, 0.0) + segment.duration
    return {key: round(value, 4) for key, value in sorted(totals.items())}


def concatenate(trajectories: Sequence[Trajectory], label: str = "") -> Trajectory:
    """Join trajectories end to end, each starting where the last finished."""
    kept: List[Trajectory] = [item for item in trajectories if len(item.times) > 1]
    if not kept:
        return Trajectory(np.zeros(0), np.zeros((0, 6)), label)
    times = [kept[0].times]
    positions = [kept[0].positions]
    offset = kept[0].duration
    for item in kept[1:]:
        times.append(item.times[1:] + offset)
        positions.append(item.positions[1:])
        offset += item.duration
    return Trajectory(np.concatenate(times), np.vstack(positions), label)
