"""Rigid-body transform helpers shared across the cell packages.

Everything here operates on 4x4 homogeneous matrices in metres and radians. The
convention is the ROS one throughout: right-handed, z up in the world frame, and
a tool frame whose +z axis points out of the flange along the approach
direction, so "approach along -z of the world" means "tool z anti-parallel to
world z".

scipy.spatial.transform.Rotation does the quaternion and rotation-vector work
rather than a hand-rolled implementation. It is already a dependency of the
analysis code, it is correct at the singularities, and quaternion sign
conventions are exactly the kind of detail that is worth not re-deriving.
"""

from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np
from scipy.spatial.transform import Rotation


def rotation_matrix(axis: str, angle: float) -> np.ndarray:
    """3x3 rotation about a principal axis, ``axis`` in {'x', 'y', 'z'}."""
    return Rotation.from_euler(axis, angle).as_matrix()


def transform(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    """Assemble a 4x4 transform from a 3x3 rotation and a 3-vector."""
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(rotation, dtype=float)
    matrix[:3, 3] = np.asarray(translation, dtype=float)
    return matrix


def translation(xyz: Sequence[float]) -> np.ndarray:
    """4x4 pure translation."""
    return transform(np.eye(3), xyz)


def from_rpy(roll: float, pitch: float, yaw: float, xyz: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """4x4 transform from fixed-axis roll-pitch-yaw, the URDF convention."""
    return transform(Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix(), xyz)


def to_rpy(matrix: np.ndarray) -> np.ndarray:
    """Fixed-axis roll-pitch-yaw of a 4x4 transform, matching :func:`from_rpy`."""
    return Rotation.from_matrix(np.asarray(matrix)[:3, :3]).as_euler("xyz")


def invert(matrix: np.ndarray) -> np.ndarray:
    """Inverse of a 4x4 rigid transform, without a general matrix inverse."""
    matrix = np.asarray(matrix, dtype=float)
    rotation = matrix[:3, :3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ matrix[:3, 3]
    return inverse


def apply(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Transform an (N, 3) array of points by a 4x4 matrix."""
    points = np.atleast_2d(np.asarray(points, dtype=float))
    return points @ np.asarray(matrix)[:3, :3].T + np.asarray(matrix)[:3, 3]


def pose_error(a: np.ndarray, b: np.ndarray) -> tuple:
    """Translation (m) and rotation (rad) error between two 4x4 poses.

    The rotation term is the geodesic angle of ``a^-1 b`` rather than a
    difference of Euler angles, which would be discontinuous and would
    understate the error near gimbal lock.
    """
    delta = invert(a) @ b
    linear = float(np.linalg.norm(delta[:3, 3]))
    angular = float(np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec()))
    return linear, angular


def rotation_angle(matrix: np.ndarray) -> float:
    """Geodesic angle of a 3x3 rotation, in radians."""
    return float(np.linalg.norm(Rotation.from_matrix(np.asarray(matrix)[:3, :3]).as_rotvec()))


def look_at(eye: Sequence[float], target: Sequence[float], up: Sequence[float] = (0.0, 0.0, 1.0)) -> np.ndarray:
    """Camera-style pose at ``eye`` whose +z axis points towards ``target``.

    The optical convention is used, not the ROS body one: +z forward along the
    view direction, +x right, +y down. That is what the pinhole projection in
    :mod:`cell_grasping.core.camera` expects, and mixing the two up is the
    classic way to get a calibration that converges to a plausible but mirrored
    answer.
    """
    eye = np.asarray(eye, dtype=float)
    forward = np.asarray(target, dtype=float) - eye
    norm = np.linalg.norm(forward)
    if norm < 1e-12:
        raise ValueError("look_at: eye and target coincide")
    forward /= norm

    up = np.asarray(up, dtype=float)
    if abs(float(forward @ up)) > 0.999:
        # Degenerate: looking straight along `up`. Any perpendicular will do and
        # world x is as good as any, so pick it rather than returning NaNs.
        up = np.array([1.0, 0.0, 0.0])

    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return transform(np.column_stack([right, down, forward]), eye)


def interpolate(a: np.ndarray, b: np.ndarray, fraction: float) -> np.ndarray:
    """Screw-linear interpolation between two 4x4 poses."""
    fraction = float(np.clip(fraction, 0.0, 1.0))
    rotations = Rotation.from_matrix(np.stack([np.asarray(a)[:3, :3], np.asarray(b)[:3, :3]]))
    rotvec = (rotations[0].inv() * rotations[1]).as_rotvec()
    rotation = rotations[0] * Rotation.from_rotvec(rotvec * fraction)
    position = (1.0 - fraction) * np.asarray(a)[:3, 3] + fraction * np.asarray(b)[:3, 3]
    return transform(rotation.as_matrix(), position)


def wrap_to_pi(angles: Iterable[float]) -> np.ndarray:
    """Wrap angles into (-pi, pi]."""
    return (np.asarray(angles, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi
