"""Vision-guided grasp planning: candidates, scores, and the reason for each.

The planner takes a :class:`~cell_grasping.core.perception.PartEstimate` --
which is all it is allowed to know, because that is all a real cell knows --
and produces a ranked list of top-down parallel-jaw grasps.

CANDIDATE GENERATION

A box lying on a face has two sensible closing directions: across its short
horizontal axis and across its long one. For each, the planner sweeps a few
offsets along the perpendicular axis and a couple of insertion depths, because
the best grasp is not always through the centroid -- a part close to the belt's
guide rail can only be taken near one end.

For a *square* footprint the PCA yaw is meaningless (see
:meth:`PartEstimate.yaw_is_meaningful`), so the planner additionally sweeps
yaw in 15-degree steps over a quadrant. That looks like brute force and it is:
there is no information in the estimate to do better, and the alternative --
trusting a yaw derived from which corner happened to drop out -- grasps square
parts across the diagonal about a third of the time.

SCORING

Six terms, each in [0, 1], combined with the weights in ``SCORE_WEIGHTS``. Any
term can also *veto*: a candidate wider than the stroke is not a low-scoring
grasp, it is not a grasp. Vetoes are recorded by name so the benchmark can say
why a part had no candidates rather than just that it had none.

    width       margin between the jaw stroke and the width to be spanned
    contact     alignment of the closing direction with the face normal it
                will land on, relative to the friction cone
    balance     how far the grasp line sits from the estimated centre of mass
    clearance   distance from the fingers to everything that is not the part
    posture     manipulability of the arm configuration that reaches it
    approach    clearance along the vertical descent into the grasp

The weights are not tuned against the benchmark's success rate. They were set
once, from the reasoning in ``docs/grasp-planning.md``, and left alone --
tuning them on the same 200 poses the result is reported over would make the
number a measure of overfitting.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from cell_common.cell_model import GripperSpec
from cell_common.transforms import from_rpy
from cell_grasping.core.perception import PartEstimate
from cell_motion.core.arm import ArmModel, approach_pose
from cell_motion.core.collision import Box

SCORE_WEIGHTS = {
    "width": 0.22,
    "contact": 0.24,
    "balance": 0.16,
    "clearance": 0.16,
    "posture": 0.10,
    "approach": 0.12,
}

# Clearance the jaws are *asked* to leave on each side of the part before
# closing, in metres. Below about 4 mm the finger tips catch the part on the way
# down whenever the width estimate is a couple of millimetres light, which it
# routinely is.
JAW_CLEARANCE = 0.006

# Clearance the jaws must leave for the grasp to be attempted at all. This is
# not the same number, and conflating the two costs the widest part in the
# catalogue every one of its grasps: an 85 mm stroke on a 78 mm plate leaves
# 3.5 mm a side, which is a tight but perfectly ordinary industrial grasp, and
# demanding the preferred 6 mm rejects it before it is scored.
MIN_JAW_CLEARANCE = 0.002

# Vertical distance the pre-grasp pose sits above the grasp, in metres.
APPROACH_HEIGHT = 0.10


@dataclass
class GraspCandidate:
    """One scored top-down grasp."""

    position: np.ndarray  # World position of the TCP at the grasp.
    yaw: float  # Heading of the closing direction.
    width: float  # Estimated part width across the jaws.
    opening: float  # Commanded jaw opening during the approach.
    score: float
    terms: Dict[str, float] = field(default_factory=dict)
    joints: Optional[np.ndarray] = None
    pregrasp_joints: Optional[np.ndarray] = None
    clearance: float = 0.0

    @property
    def pose(self) -> np.ndarray:
        return approach_pose(self.position, self.yaw)

    @property
    def pregrasp_position(self) -> np.ndarray:
        return self.position + np.array([0.0, 0.0, APPROACH_HEIGHT])

    @property
    def pregrasp_pose(self) -> np.ndarray:
        return approach_pose(self.pregrasp_position, self.yaw)

    def closing_direction(self) -> np.ndarray:
        return np.array([np.cos(self.yaw), np.sin(self.yaw), 0.0])


@dataclass
class GraspPlan:
    """The ranked candidates, and why the rejected ones were rejected."""

    candidates: List[GraspCandidate]
    rejections: Dict[str, int]
    considered: int

    @property
    def best(self) -> Optional[GraspCandidate]:
        return self.candidates[0] if self.candidates else None

    def top(self, count: int) -> List[GraspCandidate]:
        return self.candidates[:count]


def _estimated_box(estimate: PartEstimate) -> Box:
    """The part as the planner believes it to be, for collision purposes."""
    size = np.array([estimate.extents[0], estimate.extents[1], estimate.extents[2]])
    pose = from_rpy(0.0, 0.0, estimate.yaw, xyz=estimate.centre)
    return Box(name="part", size=size, pose=pose, kind="part")


def _candidate_yaws(estimate: PartEstimate) -> List[Tuple[float, float, float]]:
    """(closing yaw, width across the jaws, extent along the jaw axis) triples."""
    long_extent, short_extent = float(estimate.extents[0]), float(estimate.extents[1])
    options = [
        (estimate.yaw + np.pi / 2, short_extent, long_extent),  # Close across the short axis.
        (estimate.yaw, long_extent, short_extent),  # Close across the long axis.
    ]
    if not estimate.yaw_is_meaningful:
        # No usable principal axis. Sweep a quadrant; the footprint's own
        # symmetry makes anything beyond 90 degrees a duplicate.
        diagonal = float(np.hypot(long_extent, short_extent))
        for step in range(1, 6):
            angle = estimate.yaw + step * np.pi / 12.0
            # Width across a rotated rectangle, exactly.
            width = abs(long_extent * np.sin(angle - estimate.yaw)) + abs(
                short_extent * np.cos(angle - estimate.yaw)
            )
            options.append((angle + np.pi / 2, width, diagonal))
    return options


def generate_candidates(
    estimate: PartEstimate,
    arm: ArmModel,
    gripper: Optional[GripperSpec] = None,
    reference_joints: Optional[Sequence[float]] = None,
    offsets: Sequence[float] = (0.0, -0.25, 0.25),
    depth_fractions: Sequence[float] = (0.5, 0.75),
) -> GraspPlan:
    """Score every top-down grasp the estimate admits, best first.

    ``offsets`` are fractions of the extent along the jaw axis, so 0.25 means a
    quarter of the way from the centre towards one end. ``depth_fractions`` are
    fractions of the part height, measured down from its top face: 0.5 puts the
    finger tips level with the middle of the part.
    """
    gripper = gripper if gripper is not None else arm.gripper
    reference = np.asarray(reference_joints if reference_joints is not None else arm.cell.home_joints, dtype=float)

    part = _estimated_box(estimate)
    arm.set_part(part)
    rejections: Dict[str, int] = {}
    candidates: List[GraspCandidate] = []
    considered = 0

    def reject(reason: str) -> None:
        rejections[reason] = rejections.get(reason, 0) + 1

    try:
        for yaw, width, axis_extent in _candidate_yaws(estimate):
            usable = gripper.max_opening - 2.0 * MIN_JAW_CLEARANCE
            if width > usable:
                reject("wider_than_stroke")
                continue

            for offset_fraction in offsets:
                for depth_fraction in depth_fractions:
                    considered += 1
                    perpendicular = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
                    top = estimate.support_height + float(estimate.extents[2])
                    height = top - depth_fraction * float(estimate.extents[2])
                    position = (
                        np.array([estimate.centre[0], estimate.centre[1], height])
                        + perpendicular * offset_fraction * axis_extent
                    )

                    terms = {}
                    terms["width"] = float(np.clip((usable - width) / (0.5 * usable), 0.0, 1.0))
                    terms["contact"] = _contact_term(estimate, yaw)
                    terms["balance"] = float(np.clip(1.0 - abs(offset_fraction) * 2.0, 0.0, 1.0))

                    opening = min(width + 2.0 * JAW_CLEARANCE, gripper.max_opening)
                    solution = arm.ik_nearest(approach_pose(position, yaw), reference)
                    if solution is None:
                        reject("unreachable")
                        continue

                    # The part is excluded here: the fingers are *meant* to be
                    # around it. Everything else -- belt, rail, pillar, nest --
                    # is not.
                    clearance, culprit = arm.clearance(solution.joints, opening, ignore=("part",))
                    if clearance < 0.002:
                        reject(f"collision_{culprit}" if culprit else "collision")
                        continue

                    pregrasp = arm.ik_nearest(approach_pose(position + [0, 0, APPROACH_HEIGHT], yaw), reference)
                    if pregrasp is None:
                        reject("pregrasp_unreachable")
                        continue
                    approach_clearance, _ = arm.clearance(pregrasp.joints, opening, ignore=("part",))
                    if approach_clearance < 0.002:
                        reject("pregrasp_collision")
                        continue

                    terms["clearance"] = float(np.clip(clearance / 0.02, 0.0, 1.0))
                    terms["posture"] = float(np.clip(arm.manipulability(solution.joints) / 0.08, 0.0, 1.0))
                    terms["approach"] = float(np.clip(approach_clearance / 0.05, 0.0, 1.0))

                    score = sum(SCORE_WEIGHTS[name] * value for name, value in terms.items())
                    candidates.append(
                        GraspCandidate(
                            position=position,
                            yaw=float(yaw),
                            width=float(width),
                            opening=float(opening),
                            score=float(score),
                            terms=terms,
                            joints=solution.joints,
                            pregrasp_joints=pregrasp.joints,
                            clearance=float(clearance),
                        )
                    )
    finally:
        arm.set_part(None)

    candidates.sort(key=lambda candidate: candidate.score, reverse=True)
    return GraspPlan(candidates=candidates, rejections=rejections, considered=considered)


def _contact_term(estimate: PartEstimate, yaw: float) -> float:
    """How square-on the jaws land, scaled against the friction cone.

    A parallel-jaw grasp holds when both contact normals lie inside the
    friction cone about the closing direction -- the antipodal condition. For a
    box that means the closing direction must be within ``atan(mu)`` of a face
    normal. This term is 1.0 for a perpendicular landing and falls to 0.0 at a
    45-degree corner contact, with the crossover placed at the friction angle
    for a typical dry steel-on-rubber pad, 30 degrees.

    The estimate carries no friction information -- the planner does not know
    which part it is looking at -- so the cone is fixed here and the *evaluator*
    uses the part's true coefficient. A grasp the planner considers marginal on
    a slippery part is one it will lose.
    """
    delta = abs(((yaw - estimate.yaw + np.pi / 4) % (np.pi / 2)) - np.pi / 4)
    return float(np.clip(1.0 - delta / np.radians(30.0), 0.0, 1.0))
