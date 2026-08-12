"""Did the grasp actually hold? The ground-truth judge.

The grasp planner works from an *estimate* of the part. This module works from
what the part actually is, and decides whether the grasp the planner chose
would have picked it up. It is the only place in the pick pipeline allowed to
read the true part pose, and keeping that boundary sharp is what makes the
reported success rate mean anything.

THE CRITERIA, AND WHY EACH ONE IS THERE

A parallel-jaw grasp on a rigid box holds when all of these are true. Each maps
to a failure class the benchmark reports separately.

``missed``
    The closing axis through the tool centre point does not intersect the part
    at the commanded height. Caused by centroid error, and by a part that
    moved between the scan and the grasp -- which does not happen here, there
    being no conveyor motion during the pick.

``too_wide``
    The part's extent along the closing axis exceeds the jaw stroke. The
    planner rejects these using its *estimated* extent; this catches the ones
    where the estimate was several millimetres light.

``shallow``
    The finger pads do not overlap the part's side faces by enough to grip:
    either the tips stopped above the part, or the part is so thin that the
    engaged depth is under ``MIN_ENGAGEMENT``.

``non_antipodal``
    The closing direction lies outside the friction cone about the contact
    face normal, so the jaws would rotate the part rather than hold it. This is
    the classic antipodal condition and it is where a yaw estimate error
    actually costs you: the cone half-angle is ``atan(mu)``, which for the
    slipperiest part in the catalogue is 26.6 degrees.

``torque_slip``
    The grasp is far enough from the centre of mass that the part's weight
    exceeds what the contact patch can resist in torsion. Modelled with the
    soft-finger limit-surface approximation, ``M_max = mu * F * r``, with ``r``
    the effective patch radius.

``finger_collision``
    An open finger intersects the part on the way down. The planner checks this
    against the estimated box; the true box is a few millimetres elsewhere.

A grasp that passes all six is a hold. Nothing here models the part deforming,
the pads deforming, or dynamic effects during the lift -- see
``docs/limitations.md``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from cell_common.cell_model import GripperSpec

GRAVITY = 9.81

# Minimum vertical overlap between pad and part face, in metres. Below this the
# pads are gripping the top corner radius of a real part rather than its side.
MIN_ENGAGEMENT = 0.003

# Effective torsional patch radius as a fraction of the pad width. A uniform
# pressure distribution over a rectangular pad of width w gives an effective
# radius of about w/4 for the limit-surface approximation.
PATCH_RADIUS_FRACTION = 0.25


@dataclass
class GraspOutcome:
    """Whether the grasp held, and everything that went into deciding."""

    held: bool
    failure: str  # "" when held.
    width: float  # True part extent across the jaws, metres.
    engagement: float  # Vertical pad-to-face overlap, metres.
    contact_angle: float  # Angle between closing direction and face normal, radians.
    friction_angle: float  # atan(mu) for this part.
    offset: float  # Grasp line to centre of mass, along the jaw axis, metres.
    torque: float  # Required torsional moment, N m.
    torque_capacity: float  # What the contact can resist, N m.

    def as_dict(self) -> Dict:
        return {
            "held": self.held,
            "failure": self.failure,
            "width_mm": round(self.width * 1000.0, 2),
            "engagement_mm": round(self.engagement * 1000.0, 2),
            "contact_angle_deg": round(float(np.degrees(self.contact_angle)), 2),
            "friction_angle_deg": round(float(np.degrees(self.friction_angle)), 2),
            "offset_mm": round(self.offset * 1000.0, 2),
            "torque_nm": round(self.torque, 4),
            "torque_capacity_nm": round(self.torque_capacity, 4),
        }


def _rectangle_extent(size_x: float, size_y: float, yaw: float, direction: float) -> float:
    """Extent of a yaw-rotated rectangle along a world direction angle.

    Exact for a rectangle: the support function of a box, which for a
    2D rectangle is the sum of the absolute projections of its half-extents.
    """
    delta = direction - yaw
    return abs(size_x * np.cos(delta)) + abs(size_y * np.sin(delta))


def evaluate_grasp(
    grasp_position: np.ndarray,
    grasp_yaw: float,
    opening: float,
    part_pose: Dict,
    gripper: GripperSpec,
) -> GraspOutcome:
    """Decide whether a commanded grasp holds the part it was aimed at.

    ``part_pose`` is a sample from :func:`cell_common.cell_model.sample_part_pose`
    -- the ground truth. ``grasp_position`` is where the tool centre point
    actually ended up, which is not necessarily where the planner asked for it
    if the trajectory was clamped.
    """
    grasp_position = np.asarray(grasp_position, dtype=float)
    size = np.asarray(part_pose["size"], dtype=float)
    centre = np.asarray(part_pose["xyz"], dtype=float)
    yaw = float(part_pose["yaw"])
    friction = float(part_pose["friction"])
    mass = float(part_pose["mass"])

    friction_angle = float(np.arctan(friction))

    # Work in the part's own horizontal frame.
    jaw_axis = np.array([-np.sin(grasp_yaw), np.cos(grasp_yaw)])
    to_grasp = grasp_position[:2] - centre[:2]

    # Angle between the closing direction and the nearest face normal. The
    # normals are the part's own x and y axes, so this is the distance to the
    # nearest multiple of 90 degrees.
    delta = abs(((grasp_yaw - yaw + np.pi / 4) % (np.pi / 2)) - np.pi / 4)

    # Width the jaws have to span: the part's extent along the closing
    # direction, through the grasp point.
    width = _rectangle_extent(size[0], size[1], yaw, grasp_yaw)

    # Vertical engagement between the pads and the part's sides.
    top = centre[2] + size[2] / 2.0
    bottom = centre[2] - size[2] / 2.0
    tip = grasp_position[2] - 0.0  # The TCP sits at the pad centre height.
    pad_top = tip + gripper.finger_length / 2.0
    pad_bottom = tip - gripper.finger_length / 2.0
    engagement = float(max(0.0, min(pad_top, top) - max(pad_bottom, bottom)))

    # Distance from the grasp line to the centre of mass, along the jaw axis.
    offset = float(abs(to_grasp @ jaw_axis))
    patch_radius = gripper.finger_width * PATCH_RADIUS_FRACTION
    torque_capacity = float(friction * gripper.grip_force * patch_radius)
    torque = float(mass * GRAVITY * offset)

    def outcome(held: bool, failure: str) -> GraspOutcome:
        return GraspOutcome(
            held=held,
            failure=failure,
            width=width,
            engagement=engagement,
            contact_angle=float(delta),
            friction_angle=friction_angle,
            offset=offset,
            torque=torque,
            torque_capacity=torque_capacity,
        )

    # 1. Does the closing axis pass through the part at all? The grasp line runs
    #    along the closing direction through the tool centre point, so it
    #    crosses the part when its offset along the perpendicular jaw axis is
    #    inside the part's extent along that same axis.
    perpendicular_extent = _rectangle_extent(size[0], size[1], yaw, grasp_yaw + np.pi / 2)
    if offset > perpendicular_extent / 2.0 - MIN_ENGAGEMENT:
        return outcome(False, "missed")

    # 2. Stroke.
    if width > gripper.max_opening:
        return outcome(False, "too_wide")

    # 3. Engagement.
    if engagement < MIN_ENGAGEMENT:
        return outcome(False, "shallow")

    # 4. Antipodal condition.
    if delta > friction_angle:
        return outcome(False, "non_antipodal")

    # 5. Torsional slip about the grasp axis.
    if torque > torque_capacity:
        return outcome(False, "torque_slip")

    # 6. Did an open finger clip the part on the way down? The jaws descend at
    #    `opening`; each pad's inner face is opening/2 from the axis, and the
    #    part's half-width along the closing direction is width/2.
    if opening < width - 1e-9:
        return outcome(False, "finger_collision")

    return outcome(True, "")


def summarise(outcomes: List[GraspOutcome]) -> Dict:
    """Success rate and failure histogram over a set of attempts."""
    total = len(outcomes)
    held = sum(1 for outcome in outcomes if outcome.held)
    failures: Dict[str, int] = {}
    for outcome in outcomes:
        if not outcome.held:
            failures[outcome.failure] = failures.get(outcome.failure, 0) + 1
    return {
        "attempts": total,
        "held": held,
        "success_rate": round(held / total, 4) if total else 0.0,
        "failures": dict(sorted(failures.items(), key=lambda item: -item[1])),
    }


def wilson_interval(successes: int, trials: int, z: float = 1.96) -> Optional[Dict[str, float]]:
    """95% Wilson score interval for a success rate.

    Reported alongside every rate in this repository. Two hundred trials at a
    rate near 0.9 gives a half-width of about four percentage points, and a
    result quoted without that is a result quoted more precisely than it was
    measured.
    """
    if trials <= 0:
        return None
    proportion = successes / trials
    denominator = 1.0 + z**2 / trials
    centre = (proportion + z**2 / (2 * trials)) / denominator
    margin = z * np.sqrt(proportion * (1 - proportion) / trials + z**2 / (4 * trials**2)) / denominator
    return {
        "point": round(proportion, 4),
        "low": round(float(max(0.0, centre - margin)), 4),
        "high": round(float(min(1.0, centre + margin)), 4),
    }
