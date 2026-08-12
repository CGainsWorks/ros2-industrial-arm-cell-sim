"""The arm as the rest of the cell sees it: world frame, tool frame, gripper.

:mod:`cell_motion.core.kinematics` works in the arm base frame and knows about
the flange. Everything else in this repository works in the world frame and
cares about the tool centre point between the fingertips. This module is the
adapter, and it owns the gripper's collision geometry because the gripper is
part of what has to be checked.

The tool frame convention: +z out of the flange along the approach direction,
+x along the finger closing direction. So a top-down grasp has tool z pointing
at the belt, and rotating the grasp about the approach axis is a rotation of
tool x. Getting that pair the wrong way round produces a planner that grasps
every part across its longest axis, which is both wrong and slow to notice.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from cell_common.cell_model import CellSpec, GripperSpec, load_cell
from cell_common.transforms import invert, transform, translation
from cell_motion.core.collision import Box, Capsule, CollisionScene, arm_capsules, scene_from_dict, self_collision
from cell_motion.core.kinematics import (
    IKSolution,
    forward_kinematics,
    inverse_kinematics,
    manipulability,
    nearest_solution,
)

# Links that are allowed to be near the pedestal. The shoulder capsule starts
# inside the mount by construction, so checking it against the mount reports a
# collision at every configuration including the home one.
PEDESTAL_EXEMPT = {"link_1", "link_2"}


@dataclass
class ArmModel:
    """Kinematics, gripper geometry and the collision scene, in world frame."""

    cell: CellSpec
    scene: CollisionScene
    base_pose: np.ndarray
    tool_transform: np.ndarray
    gripper: GripperSpec

    @classmethod
    def from_cell(cls, cell: Optional[CellSpec] = None) -> "ArmModel":
        cell = cell if cell is not None else load_cell()
        gripper = cell.gripper
        return cls(
            cell=cell,
            scene=scene_from_dict(cell.raw),
            base_pose=translation(cell.arm_base_xyz),
            tool_transform=translation([0.0, 0.0, gripper.tcp_offset]),
            gripper=gripper,
        )

    # -- frames ------------------------------------------------------------

    def flange_pose(self, joints: Sequence[float]) -> np.ndarray:
        """Flange pose in the world frame."""
        return self.base_pose @ forward_kinematics(joints)

    def tool_pose(self, joints: Sequence[float]) -> np.ndarray:
        """Tool-centre-point pose in the world frame."""
        return self.flange_pose(joints) @ self.tool_transform

    def camera_pose(self, joints: Sequence[float]) -> np.ndarray:
        """Eye-in-hand camera optical frame in the world frame.

        Uses the ground-truth extrinsic from the cell description. The
        calibration routine deliberately does not call this.
        """
        return self.flange_pose(joints) @ self.cell.camera.extrinsic

    def ik(self, tool_pose: np.ndarray) -> List[IKSolution]:
        """All IK branches for a world-frame *tool* pose."""
        flange_in_base = invert(self.base_pose) @ np.asarray(tool_pose) @ invert(self.tool_transform)
        return inverse_kinematics(flange_in_base)

    def ik_nearest(self, tool_pose: np.ndarray, reference: Sequence[float]) -> Optional[IKSolution]:
        return nearest_solution(self.ik(tool_pose), reference)

    def manipulability(self, joints: Sequence[float]) -> float:
        return manipulability(joints)

    # -- collision ---------------------------------------------------------

    def gripper_capsules(self, joints: Sequence[float], opening: float) -> List[Capsule]:
        """Palm and finger capsules at a joint configuration and jaw opening.

        THE CAPS ARE THE WHOLE PROBLEM

        A capsule is a segment swept by a sphere, so its ends bulge past the
        segment by one radius. Modelling a finger as a capsule from its root to
        its tip therefore puts a hemisphere *below* the fingertip, and a
        22 mm-wide pad becomes an 11 mm-deep dome hanging off the end of it.
        Grasping an 8 mm coupon needs the tips 4 mm above the belt, and that
        phantom dome is 7 mm inside it: every thin part in the catalogue came
        back as ``collision_conveyor`` and the benchmark reported them as
        ungraspable.

        So each rectangular pad is covered by *two* capsules of the pad's
        half-thickness, offset along the pad's width, with both segment ends
        inset by one radius. The union contains the pad exactly and its lowest
        point is the fingertip, where the fingertip actually is. The palm gets
        the same treatment across its longer horizontal axis.
        """
        flange = self.flange_pose(joints)
        approach = flange[:3, 2]
        closing = flange[:3, 0]
        lateral = flange[:3, 1]
        origin = flange[:3, 3]
        gripper = self.gripper

        capsules: List[Capsule] = []

        # Palm: a box palm_size, from 0 to palm_size[2] along the approach.
        palm_radius = float(gripper.palm_size[1]) / 2.0
        palm_spread = max(float(gripper.palm_size[0]) / 2.0 - palm_radius, 0.0)
        palm_start = origin + approach * palm_radius
        palm_end = origin + approach * (float(gripper.palm_size[2]) - palm_radius)
        for sign, name in ((1.0, "palm_a"), (-1.0, "palm_b")):
            capsules.append(
                Capsule(
                    start=palm_start + closing * sign * palm_spread,
                    end=palm_end + closing * sign * palm_spread,
                    radius=palm_radius,
                    name=name,
                )
            )

        half_gap = float(np.clip(opening, gripper.min_opening, gripper.max_opening)) / 2.0
        finger_radius = gripper.finger_thickness / 2.0
        finger_offset = half_gap + finger_radius
        finger_spread = max(gripper.finger_width / 2.0 - finger_radius, 0.0)
        base = origin + approach * (gripper.tcp_offset - gripper.finger_length + finger_radius)
        tip = origin + approach * (gripper.tcp_offset - finger_radius)
        for sign, jaw in ((1.0, "a"), (-1.0, "b")):
            for lateral_sign, side in ((1.0, "1"), (-1.0, "2")):
                shift = closing * sign * finger_offset + lateral * lateral_sign * finger_spread
                capsules.append(
                    Capsule(start=base + shift, end=tip + shift, radius=finger_radius, name=f"finger_{jaw}{side}")
                )
        return capsules

    def capsules(self, joints: Sequence[float], opening: float) -> List[Capsule]:
        """Every capsule that represents the robot: links plus gripper."""
        links = arm_capsules(joints)
        for capsule in links:
            capsule.start = (self.base_pose @ np.append(capsule.start, 1.0))[:3]
            capsule.end = (self.base_pose @ np.append(capsule.end, 1.0))[:3]
        return links + self.gripper_capsules(joints, opening)

    def clearance(
        self,
        joints: Sequence[float],
        opening: float,
        ignore: Sequence[str] = (),
    ) -> Tuple[float, str]:
        """Smallest clearance to any obstacle, and the obstacle responsible.

        The pedestal is exempt for the two shoulder capsules, which start
        inside the mount by construction.
        """
        capsules = self.capsules(joints, opening)
        exempt = [capsule for capsule in capsules if capsule.name in PEDESTAL_EXEMPT]
        rest = [capsule for capsule in capsules if capsule.name not in PEDESTAL_EXEMPT]

        worst, culprit = self.scene.clearance(rest, ignore)
        exempt_gap, exempt_culprit = self.scene.clearance(exempt, list(ignore) + ["pedestal"])
        if exempt_gap < worst:
            worst, culprit = exempt_gap, exempt_culprit
        return worst, culprit

    def is_valid(
        self,
        joints: Sequence[float],
        opening: float,
        ignore: Sequence[str] = (),
        margin: float = 0.005,
    ) -> bool:
        """Collision-free at ``margin`` and free of self-collision.

        The 5 mm default is the planner's safety margin, not a modelling
        fudge: it is what stops a trajectory that grazes the belt from being
        declared valid when the belt's position is only known to a millimetre.
        """
        if self_collision(joints):
            return False
        return self.clearance(joints, opening, ignore)[0] >= margin

    def attach_part(self, box: Box) -> None:
        """Add the grasped part to the scene as an attached object."""
        self.scene.replace(box)

    def set_part(self, box: Optional[Box]) -> None:
        self.scene.remove("part")
        if box is not None:
            self.scene.add(box)


def approach_pose(position: Sequence[float], yaw: float, tilt: float = 0.0) -> np.ndarray:
    """A top-down tool pose at ``position``, rotated ``yaw`` about the approach.

    ``tilt`` leans the approach axis away from vertical, about the tool y axis.
    Zero tilt is a pure top-down grasp; the grasp planner uses a small tilt for
    parts close to the conveyor's far rail, where a vertical approach puts the
    palm through the rail.
    """
    # Yaw is applied in the *world* frame and the flip after it, so that
    # ``yaw`` is the heading of the closing axis as seen from above. Composing
    # the other way round -- flip first, then yaw about the now-downward tool z
    # -- also produces a valid pose, but with the sense of yaw reversed, and
    # every grasp then closes across the axis the planner thinks it chose.
    down = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    yaw_rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    tilt_rotation = np.array([[np.cos(tilt), 0.0, np.sin(tilt)], [0.0, 1.0, 0.0], [-np.sin(tilt), 0.0, np.cos(tilt)]])
    return transform(yaw_rotation @ down @ tilt_rotation, position)
