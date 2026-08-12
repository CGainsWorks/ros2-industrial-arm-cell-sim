"""Forward and inverse kinematics for a UR5e-class 6R arm.

The Denavit-Hartenberg table below is the public UR5e one from Universal
Robots' own kinematics page. It is a published specification of an
off-the-shelf product, not employer or client material, and it is the only
robot-specific data in this repository.

WHY CLOSED-FORM IK

The arm is a spherical-wrist 6R, so its inverse kinematics has a closed form
with at most eight solutions. A numerical solver would work, but it makes two
things impossible that this project needs:

* Enumerating *all* branches. The grasp planner scores a candidate partly on
  whether a collision-free arm configuration exists to reach it. A numerical
  solver returns whichever branch its seed fell into, so a pose that is only
  reachable elbow-up looks unreachable when the seed was elbow-down. Over the
  200-pose benchmark that difference is worth several percentage points of
  grasp success, all of them fictitious.
* A reachability answer that does not depend on a seed. "Unreachable" from a
  closed-form solver means unreachable.

The eight branches are the product of three binary choices: shoulder
left/right, wrist flip, and elbow up/down.

ACCURACY

The closed form is exact in real arithmetic, and in floating point it lands
around 1e-12 m. :func:`inverse_kinematics` still verifies every branch it
returns by running it back through :func:`forward_kinematics` and dropping any
whose residual exceeds a tolerance. That is not defensive padding: near a
shoulder or wrist singularity the ``acos`` arguments graze +-1, and a branch
that is genuinely degenerate can otherwise come back as a plausible-looking
configuration that does not reach the requested pose.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

# Standard (distal) DH parameters, UR5e. Metres and radians.
D = np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996])
A = np.array([0.0, -0.4250, -0.3922, 0.0, 0.0, 0.0])
ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])

JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# UR5e datasheet limits. The joint range is +-2 pi on every axis; the velocity
# limit is 180 deg/s. The acceleration limit is not a datasheet figure -- the
# controller's is far higher -- but 8 rad/s^2 is what the trajectory generator
# is allowed to ask for, and it is the number the cycle-time result depends on,
# so it lives here rather than being buried in the planner.
JOINT_LIMITS = np.array([[-2 * np.pi, 2 * np.pi]] * 6)
VELOCITY_LIMITS = np.full(6, np.pi)
ACCELERATION_LIMITS = np.full(6, 8.0)

# Capsule radii around the DH link axes, in metres, used by the collision
# model. Deliberately generous: the real arm is not a cylinder, and a collision
# checker that under-approximates the robot is worse than useless.
LINK_RADII = np.array([0.075, 0.065, 0.058, 0.050, 0.048, 0.045])


def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """One standard-DH link transform."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def link_transforms(joints: Sequence[float]) -> List[np.ndarray]:
    """Cumulative base-to-link transforms, base frame first.

    Returns seven 4x4 matrices: the identity for the base, then the frame at
    the far end of each of the six links.
    """
    transforms = [np.eye(4)]
    current = np.eye(4)
    for index, theta in enumerate(joints):
        current = current @ dh_transform(theta, D[index], A[index], ALPHA[index])
        transforms.append(current)
    return transforms


def forward_kinematics(joints: Sequence[float]) -> np.ndarray:
    """Flange pose in the arm base frame."""
    return link_transforms(joints)[-1]


def jacobian(joints: Sequence[float]) -> np.ndarray:
    """6x6 geometric Jacobian in the base frame, linear rows first.

    Built from the link frames rather than by finite differences, so it is
    exact and costs one FK call.
    """
    transforms = link_transforms(joints)
    end = transforms[-1][:3, 3]
    columns = []
    for index in range(6):
        origin = transforms[index][:3, 3]
        axis = transforms[index][:3, 2]
        columns.append(np.concatenate([np.cross(axis, end - origin), axis]))
    return np.column_stack(columns)


def manipulability(joints: Sequence[float]) -> float:
    """Yoshikawa manipulability index, sqrt(det(J J^T)).

    Used by the grasp scorer as a proxy for "how far is this configuration from
    a singularity". Zero at a singularity.
    """
    matrix = jacobian(joints)
    value = float(np.linalg.det(matrix @ matrix.T))
    return float(np.sqrt(max(value, 0.0)))


def within_limits(joints: Sequence[float]) -> bool:
    """True if every joint is inside its position limit."""
    joints = np.asarray(joints, dtype=float)
    return bool(np.all(joints >= JOINT_LIMITS[:, 0]) and np.all(joints <= JOINT_LIMITS[:, 1]))


@dataclass(frozen=True)
class IKSolution:
    """One inverse-kinematics branch, with the labels that generated it."""

    joints: np.ndarray
    shoulder: str  # "left" or "right"
    wrist: str  # "up" or "down"
    elbow: str  # "up" or "down"
    residual: float

    @property
    def branch(self) -> str:
        return f"{self.shoulder}/{self.wrist}/{self.elbow}"

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"IKSolution({self.branch}, residual={self.residual:.2e})"


def _safe_acos(value: float) -> Optional[float]:
    """acos that returns None outside the domain instead of NaN.

    Values within 1e-9 of +-1 are clamped rather than rejected: that is the
    floating-point neighbourhood of an exactly-singular configuration, which is
    reachable, and rejecting it would put a hole in the reachable set at
    precisely the poses a straight-down approach tends to produce.
    """
    if value > 1.0:
        if value > 1.0 + 1e-9:
            return None
        value = 1.0
    elif value < -1.0:
        if value < -1.0 - 1e-9:
            return None
        value = -1.0
    return float(np.arccos(value))


def _wrap(joints: np.ndarray) -> np.ndarray:
    """Wrap to (-pi, pi].

    Every UR joint has +-2 pi of travel, so the wrapped value is always inside
    its limit and is always the shortest representation of the same physical
    configuration.
    """
    return (joints + np.pi) % (2.0 * np.pi) - np.pi


def inverse_kinematics(pose: np.ndarray, tolerance: float = 1e-6) -> List[IKSolution]:
    """All valid closed-form solutions for a flange pose in the base frame.

    ``pose`` is the desired *flange* pose, not the tool pose. Callers holding a
    tool transform should pass ``pose @ invert(tool)``; :class:`ArmModel` in
    :mod:`cell_motion.core.arm` does that for them.
    """
    pose = np.asarray(pose, dtype=float)
    solutions: List[IKSolution] = []

    # Wrist centre: the flange origin backed off along the tool z axis by d6.
    p05 = (pose @ np.array([0.0, 0.0, -D[5], 1.0]))[:3]
    p05_xy = float(np.hypot(p05[0], p05[1]))
    if p05_xy < 1e-9:
        # Wrist centre on the base axis: theta1 is free. Nothing in this cell
        # commands it, and picking one arbitrarily would hide the degeneracy.
        return []

    psi = np.arctan2(p05[1], p05[0])
    phi = _safe_acos(D[3] / p05_xy)
    if phi is None:
        return []  # Wrist centre inside the shoulder-offset cylinder.

    for shoulder_sign, shoulder in ((1.0, "left"), (-1.0, "right")):
        theta1 = psi + shoulder_sign * phi + np.pi / 2
        sin1, cos1 = np.sin(theta1), np.cos(theta1)

        # theta5 from the flange origin expressed relative to the shoulder.
        numerator = pose[0, 3] * sin1 - pose[1, 3] * cos1 - D[3]
        theta5_magnitude = _safe_acos(numerator / D[5])
        if theta5_magnitude is None:
            continue

        for wrist_sign, wrist in ((1.0, "up"), (-1.0, "down")):
            theta5 = wrist_sign * theta5_magnitude
            sin5 = np.sin(theta5)
            if abs(sin5) < 1e-7:
                # Wrist singularity: theta4 and theta6 trade off freely. Skip
                # rather than pick, for the same reason as the shoulder case.
                continue

            # theta6 from the tool x and y axes expressed in the frame the
            # shoulder rotation defines.
            rotation = pose[:3, :3]
            theta6 = float(
                np.arctan2(
                    (-rotation[0, 1] * sin1 + rotation[1, 1] * cos1) / sin5,
                    (rotation[0, 0] * sin1 - rotation[1, 0] * cos1) / sin5,
                )
            )

            # Reduce to the planar two-link problem between frames 1 and 4.
            t01 = dh_transform(theta1, D[0], A[0], ALPHA[0])
            t45 = dh_transform(theta5, D[4], A[4], ALPHA[4])
            t56 = dh_transform(theta6, D[5], A[5], ALPHA[5])
            t14 = np.linalg.inv(t01) @ pose @ np.linalg.inv(t56) @ np.linalg.inv(t45)

            p13 = (t14 @ np.array([0.0, -D[3], 0.0, 1.0]))[:3]
            reach = float(np.hypot(p13[0], p13[1]))
            cos3 = (reach**2 - A[1] ** 2 - A[2] ** 2) / (2.0 * A[1] * A[2])
            theta3_magnitude = _safe_acos(cos3)
            if theta3_magnitude is None:
                continue

            for elbow_sign, elbow in ((1.0, "up"), (-1.0, "down")):
                theta3 = elbow_sign * theta3_magnitude
                denominator = reach if reach > 1e-12 else 1e-12
                theta2 = -np.arctan2(p13[1], -p13[0]) + np.arcsin(
                    float(np.clip(A[2] * np.sin(theta3) / denominator, -1.0, 1.0))
                )
                t12 = dh_transform(theta2, D[1], A[1], ALPHA[1])
                t23 = dh_transform(theta3, D[2], A[2], ALPHA[2])
                t34 = np.linalg.inv(t12 @ t23) @ t14
                theta4 = float(np.arctan2(t34[1, 0], t34[0, 0]))

                joints = _wrap(np.array([theta1, theta2, theta3, theta4, theta5, theta6]))
                residual = float(np.linalg.norm(forward_kinematics(joints) - pose))
                if residual > tolerance or not within_limits(joints):
                    continue
                solutions.append(IKSolution(joints, shoulder, wrist, elbow, residual))

    return solutions


def nearest_solution(
    solutions: Sequence[IKSolution],
    reference: Sequence[float],
    weights: Optional[Sequence[float]] = None,
) -> Optional[IKSolution]:
    """The branch closest to ``reference`` in weighted joint distance.

    Weights default to descending by joint index: moving the base is slower and
    sweeps more of the cell than moving the wrist, so a branch that differs
    only at wrist 3 is much cheaper than one that flips the shoulder, and an
    unweighted L2 norm treats them as equivalent.
    """
    if not solutions:
        return None
    weights = np.array([4.0, 3.0, 2.0, 1.0, 1.0, 0.5]) if weights is None else np.asarray(weights, dtype=float)
    reference = np.asarray(reference, dtype=float)
    costs = [float(np.linalg.norm(weights * _wrap(candidate.joints - reference))) for candidate in solutions]
    return solutions[int(np.argmin(costs))]
