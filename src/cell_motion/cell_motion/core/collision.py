"""Collision checking between the arm and the cell.

The world is a set of oriented boxes: the pedestal, the conveyor, the two
fixtures, the tote, the camera-mount pillar and the part itself. The arm is a
chain of capsules around the DH link axes, plus two finger capsules and a
palm box for the gripper. Everything the planner asks about reduces to
capsule-versus-box and capsule-versus-capsule distance.

WHY CAPSULES AND BOXES AND NOT MESHES

MoveIt2 checks the real meshes through FCL, and the launch files in
``cell_motion`` set that up. This module is the offline half: it is what the
grasp benchmark and the planner run against when there is no ROS, and it has to
give the same answers about *whether a configuration is safe*, not the same
answers to the millimetre. Capsules around the link axes over-approximate a
UR5e everywhere -- the arm is thinner than its capsules at every point -- so a
configuration this module calls clear is clear for the mesh too. The converse
is not true, and that asymmetry is the right way round: the cost of a
conservative checker is a few rejected poses, and the cost of an optimistic one
is a benchmark that reports grasps the real cell would have crashed.

The capsule-box test discretises the capsule axis into points spaced at most
``_MAX_SAMPLE_SPACING`` apart and inflates the radius by half that spacing. The
inflated point set contains the capsule, so the test stays conservative, and it
vectorises into a single numpy expression over every point and every box at
once. A planner query runs a few thousand of these, and that is the reason the
exact segment-to-box distance is not used here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from cell_motion.core.kinematics import LINK_RADII, link_transforms

# Capsule axes are sampled at most this far apart, in metres, and the radius is
# inflated by half of it.
_MAX_SAMPLE_SPACING = 0.02


@dataclass(frozen=True)
class Box:
    """An oriented box obstacle.

    ``size`` is the full extent along each local axis, ``pose`` a 4x4 whose
    translation is the box centre.
    """

    name: str
    size: np.ndarray
    pose: np.ndarray
    kind: str = "static"

    @property
    def centre(self) -> np.ndarray:
        return np.asarray(self.pose)[:3, 3]

    @property
    def rotation(self) -> np.ndarray:
        return np.asarray(self.pose)[:3, :3]

    @property
    def half_extents(self) -> np.ndarray:
        return np.asarray(self.size, dtype=float) / 2.0

    def contains(self, points: np.ndarray) -> np.ndarray:
        """Boolean mask of which of an (N, 3) point array lie inside."""
        local = (np.atleast_2d(points) - self.centre) @ self.rotation
        return np.all(np.abs(local) <= self.half_extents + 1e-12, axis=1)

    def distance_to_points(self, points: np.ndarray) -> np.ndarray:
        """Exterior distance from each of an (N, 3) point array to the box.

        Points inside the box return 0.0. That is the right answer for a
        collision test and the wrong one for a penetration-depth query, which
        nothing here needs.
        """
        local = (np.atleast_2d(points) - self.centre) @ self.rotation
        outside = np.maximum(np.abs(local) - self.half_extents, 0.0)
        return np.linalg.norm(outside, axis=1)

    def corners(self) -> np.ndarray:
        """The eight corners in world coordinates."""
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
        return (signs * self.half_extents) @ self.rotation.T + self.centre


@dataclass
class Capsule:
    """A line segment with a radius."""

    start: np.ndarray
    end: np.ndarray
    radius: float
    name: str = ""

    def sample_points(self) -> Tuple[np.ndarray, np.ndarray]:
        """Sample points along the axis, and a radius for each.

        The interior samples carry the radius inflated by half the sample
        spacing, which is what makes the union of spheres contain the capsule.
        The two *endpoint* samples carry the true radius, and that distinction
        is not a micro-optimisation.

        Inflating the endpoints too adds half a sample spacing of phantom
        material beyond each cap. On a 28 mm fingertip capsule that is 7 mm of
        imaginary finger sticking out below the tip, which is more than the
        clearance a grasp on an 8 mm-tall part has to the belt: every thin part
        in the catalogue was reported as unreachable because of geometry that
        does not exist. The union is still a cover -- a point on the capsule
        between two samples is within ``sqrt(r^2 + s^2/4) <= r + s/2`` of the
        interior one -- so the check stays conservative where it matters.
        """
        start = np.asarray(self.start, dtype=float)
        end = np.asarray(self.end, dtype=float)
        length = float(np.linalg.norm(end - start))
        count = max(2, int(np.ceil(length / _MAX_SAMPLE_SPACING)) + 1)
        fractions = np.linspace(0.0, 1.0, count)[:, None]
        points = start + fractions * (end - start)
        spacing = length / (count - 1) if count > 1 else 0.0
        radii = np.full(count, self.radius + spacing / 2.0)
        radii[0] = radii[-1] = self.radius
        return points, radii


def segment_distance(p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray) -> float:
    """Shortest distance between two line segments.

    The standard clamped-parameter solution (Ericson, *Real-Time Collision
    Detection*, section 5.1.9), including the parallel case, which the naive
    formula divides by zero on and which two adjacent arm links hit exactly
    whenever a joint is at zero.
    """
    d1 = np.asarray(q1, dtype=float) - np.asarray(p1, dtype=float)
    d2 = np.asarray(q2, dtype=float) - np.asarray(p2, dtype=float)
    r = np.asarray(p1, dtype=float) - np.asarray(p2, dtype=float)
    a = float(d1 @ d1)
    e = float(d2 @ d2)
    f = float(d2 @ r)

    if a <= 1e-12 and e <= 1e-12:
        return float(np.linalg.norm(r))
    if a <= 1e-12:
        s, t = 0.0, float(np.clip(f / e, 0.0, 1.0))
    else:
        c = float(d1 @ r)
        if e <= 1e-12:
            t, s = 0.0, float(np.clip(-c / a, 0.0, 1.0))
        else:
            b = float(d1 @ d2)
            denominator = a * e - b * b
            s = float(np.clip((b * f - c * e) / denominator, 0.0, 1.0)) if denominator > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, float(np.clip(-c / a, 0.0, 1.0))
            elif t > 1.0:
                t, s = 1.0, float(np.clip((b - c) / a, 0.0, 1.0))

    closest1 = np.asarray(p1, dtype=float) + s * d1
    closest2 = np.asarray(p2, dtype=float) + t * d2
    return float(np.linalg.norm(closest1 - closest2))


@dataclass
class CollisionScene:
    """The set of obstacles a configuration is checked against.

    Obstacles are named, and named obstacles can be disabled for a query. That
    is not a convenience: once the gripper has closed on the part, the part is
    attached to the arm and checking the fingers against it would report a
    collision on every subsequent waypoint of the cycle. MoveIt2 models this
    with an attached collision object and an allowed-collision matrix; this is
    the same idea with a smaller vocabulary.
    """

    boxes: List[Box] = field(default_factory=list)

    def add(self, box: Box) -> None:
        self.boxes.append(box)

    def replace(self, box: Box) -> None:
        """Add ``box``, replacing any existing obstacle with the same name."""
        self.boxes = [existing for existing in self.boxes if existing.name != box.name]
        self.boxes.append(box)

    def remove(self, name: str) -> None:
        self.boxes = [existing for existing in self.boxes if existing.name != name]

    def get(self, name: str) -> Optional[Box]:
        for box in self.boxes:
            if box.name == name:
                return box
        return None

    def active(self, ignore: Sequence[str] = ()) -> List[Box]:
        ignored = set(ignore)
        return [box for box in self.boxes if box.name not in ignored]

    def clearance(self, capsules: Sequence[Capsule], ignore: Sequence[str] = ()) -> Tuple[float, str]:
        """Smallest signed clearance over all capsule-obstacle pairs.

        Negative means penetration. The second element names the obstacle
        responsible, which is what turns a failed benchmark trial from "planning
        failed" into "the approach hit the camera pillar".
        """
        boxes = self.active(ignore)
        if not boxes or not capsules:
            return float("inf"), ""

        worst = float("inf")
        culprit = ""
        for capsule in capsules:
            points, radii = capsule.sample_points()
            for box in boxes:
                gap = float(np.min(box.distance_to_points(points) - radii))
                if gap < worst:
                    worst, culprit = gap, box.name
        return worst, culprit

    def in_collision(self, capsules: Sequence[Capsule], ignore: Sequence[str] = (), margin: float = 0.0) -> bool:
        return self.clearance(capsules, ignore)[0] < margin


def arm_capsules(joints: Sequence[float]) -> List[Capsule]:
    """Capsules for the six arm links at a joint configuration.

    One capsule per link, spanning consecutive DH frame origins. The first
    capsule starts at the base rather than at frame 0 so the shoulder housing
    is covered; without it the pedestal sits inside a gap in the model and the
    arm can fold into its own mount undetected.
    """
    transforms = link_transforms(joints)
    origins = [transform[:3, 3] for transform in transforms]
    capsules = []
    for index in range(6):
        capsules.append(
            Capsule(
                start=origins[index],
                end=origins[index + 1],
                radius=float(LINK_RADII[index]),
                name=f"link_{index + 1}",
            )
        )
    return capsules


def self_collision(joints: Sequence[float], margin: float = 0.01) -> bool:
    """True if two non-adjacent arm links are closer than ``margin``.

    Adjacent links are excluded because they share an origin by construction
    and would always report contact. Links separated by one joint are also
    excluded: with a 90-degree twist between them their capsules genuinely do
    overlap near the shoulder and elbow, and no reachable configuration of a UR
    arm brings those two into a real collision.
    """
    capsules = arm_capsules(joints)
    for i in range(len(capsules)):
        for j in range(i + 3, len(capsules)):
            distance = segment_distance(capsules[i].start, capsules[i].end, capsules[j].start, capsules[j].end)
            if distance < capsules[i].radius + capsules[j].radius + margin:
                return True
    return False


def scene_from_dict(specification: Dict) -> CollisionScene:
    """Build a scene from the parsed ``simulation/cell.yaml``.

    Both this and the URDF are generated from that one file, so the collision
    scene the planner reasons about and the cell the simulator draws cannot
    drift apart. They had drifted in an early revision -- the conveyor was
    120 mm taller in the URDF than in the checker -- and every approach
    trajectory was cheerfully planned straight through the belt.
    """
    from cell_common.transforms import from_rpy

    scene = CollisionScene()
    for entry in specification["obstacles"]:
        pose = from_rpy(*entry.get("rpy", (0.0, 0.0, 0.0)), xyz=entry["xyz"])
        scene.add(
            Box(
                name=entry["name"],
                size=np.asarray(entry["size"], dtype=float),
                pose=pose,
                kind=entry.get("kind", "static"),
            )
        )
    return scene
