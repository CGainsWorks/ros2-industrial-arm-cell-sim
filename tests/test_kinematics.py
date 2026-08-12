"""Kinematics: the closed form has to be exactly right or nothing above it is."""

from __future__ import annotations

import numpy as np
import pytest

from cell_common.cell_model import load_cell
from cell_common.transforms import invert, pose_error
from cell_motion.core.arm import ArmModel, approach_pose
from cell_motion.core.kinematics import (
    JOINT_NAMES,
    forward_kinematics,
    inverse_kinematics,
    jacobian,
    manipulability,
    nearest_solution,
)


def test_forward_kinematics_at_zero_matches_the_dh_table():
    """All joints at zero, checked against the table rather than a recorded run.

    The alpha twists decide which link offset lands on which world axis, and
    reading them off by eye is exactly how a transcribed DH table goes wrong:

        x = a2 + a3           the two long links, laid out along -x
        y = -(d4 + d6)        wrist offset and flange, after the +90 twists
        z = d1 - d5           shoulder height less the wrist-2 offset
    """
    pose = forward_kinematics(np.zeros(6))
    from cell_motion.core.kinematics import A, D

    assert pose[0, 3] == pytest.approx(A[1] + A[2], abs=1e-12)
    assert pose[1, 3] == pytest.approx(-(D[3] + D[5]), abs=1e-12)
    assert pose[2, 3] == pytest.approx(D[0] - D[4], abs=1e-12)


@pytest.mark.parametrize("seed", range(6))
def test_ik_recovers_every_random_pose(seed):
    rng = np.random.default_rng(seed)
    for _ in range(60):
        joints = rng.uniform(-np.pi, np.pi, 6)
        pose = forward_kinematics(joints)
        solutions = inverse_kinematics(pose)
        assert solutions, "a pose produced by forward kinematics must be solvable"
        assert any(np.allclose(forward_kinematics(s.joints), pose, atol=1e-8) for s in solutions)


def test_ik_returns_multiple_distinct_branches():
    """A generic pose has eight branches. Fewer than four means the enumeration
    is collapsing solutions, which would make reachability checks pessimistic."""
    rng = np.random.default_rng(3)
    counts = []
    for _ in range(40):
        pose = forward_kinematics(rng.uniform(-2.0, 2.0, 6))
        counts.append(len(inverse_kinematics(pose)))
    assert np.median(counts) >= 6
    assert max(counts) == 8


def test_ik_rejects_an_unreachable_pose():
    pose = np.eye(4)
    pose[:3, 3] = [3.0, 0.0, 0.5]  # Well outside an 850 mm reach.
    assert inverse_kinematics(pose) == []


def test_jacobian_matches_finite_differences():
    rng = np.random.default_rng(11)
    joints = rng.uniform(-1.5, 1.5, 6)
    analytic = jacobian(joints)

    step = 1e-6
    for index in range(6):
        perturbed = joints.copy()
        perturbed[index] += step
        difference = (forward_kinematics(perturbed)[:3, 3] - forward_kinematics(joints)[:3, 3]) / step
        assert np.allclose(analytic[:3, index], difference, atol=1e-5)


def test_manipulability_collapses_at_a_wrist_singularity():
    """Wrist 2 at zero aligns wrist 1 and wrist 3, losing a degree of freedom."""
    singular = np.array([0.0, -1.2, 1.4, -1.7, 0.0, 0.0])
    generic = np.array([0.0, -1.2, 1.4, -1.7, -1.5, 0.0])
    assert manipulability(singular) < 0.01
    assert manipulability(generic) > 0.05


def test_nearest_solution_prefers_a_wrist_move_over_a_shoulder_flip():
    rng = np.random.default_rng(5)
    joints = rng.uniform(-1.0, 1.0, 6)
    solutions = inverse_kinematics(forward_kinematics(joints))
    chosen = nearest_solution(solutions, joints)
    assert chosen is not None
    assert np.allclose(chosen.joints, joints, atol=1e-6)


def test_tool_and_flange_frames_differ_by_the_tcp_offset():
    arm = ArmModel.from_cell(load_cell())
    joints = arm.cell.home_joints
    offset = invert(arm.flange_pose(joints)) @ arm.tool_pose(joints)
    assert offset[2, 3] == pytest.approx(arm.gripper.tcp_offset, abs=1e-12)
    assert np.allclose(offset[:3, :3], np.eye(3))


def test_ik_round_trips_through_the_tool_frame():
    """The tool-frame IK wrapper has to invert the tool transform, not apply it.

    Getting that sign wrong puts every grasp 163 mm above the part, which looks
    entirely plausible in a plot of the trajectory.
    """
    arm = ArmModel.from_cell(load_cell())
    target = approach_pose([0.60, 0.05, 0.78], 0.4)
    solutions = arm.ik(target)
    assert solutions
    for solution in solutions:
        linear, angular = pose_error(arm.tool_pose(solution.joints), target)
        assert linear < 1e-9
        assert angular < 1e-9


def test_joint_names_are_the_ur_ones():
    assert JOINT_NAMES[0] == "shoulder_pan_joint"
    assert JOINT_NAMES[-1] == "wrist_3_joint"
    assert len(JOINT_NAMES) == 6
