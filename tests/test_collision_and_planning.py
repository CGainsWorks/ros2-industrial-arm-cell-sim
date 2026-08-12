"""Collision checking and planning.

The collision tests are mostly about *conservatism*: this checker is allowed to
be wrong in the direction of refusing a safe configuration and is not allowed
to be wrong the other way. Two of them exist because it was wrong the other way.
"""

from __future__ import annotations

import numpy as np
import pytest

from cell_common.cell_model import load_cell, part_box, sample_part_pose
from cell_common.transforms import from_rpy
from cell_motion.core.arm import ArmModel, approach_pose
from cell_motion.core.collision import Box, Capsule, CollisionScene, segment_distance, self_collision
from cell_motion.core.planner import make_validity, plan_cartesian, plan_rrt_connect, shortcut


def test_segment_distance_handles_parallel_segments():
    """Two parallel segments divide by zero in the naive formula, and two
    adjacent arm links are exactly parallel whenever a joint is at zero."""
    distance = segment_distance([0, 0, 0], [1, 0, 0], [0, 0.5, 0], [1, 0.5, 0])
    assert distance == pytest.approx(0.5)


def test_segment_distance_matches_a_brute_force_sample():
    rng = np.random.default_rng(0)
    for _ in range(50):
        p1, q1, p2, q2 = (rng.uniform(-1, 1, 3) for _ in range(4))
        exact = segment_distance(p1, q1, p2, q2)
        grid = np.linspace(0, 1, 220)[:, None]
        a = p1 + grid * (q1 - p1)
        b = p2 + grid * (q2 - p2)
        brute = float(np.min(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)))
        assert exact <= brute + 1e-9
        assert exact == pytest.approx(brute, abs=2e-2)


def test_capsule_endpoints_are_not_inflated():
    """The regression that made every thin part unpickable.

    Sampling a capsule and inflating *every* sample by half the sample spacing
    adds a phantom hemisphere beyond each end. A 28 mm fingertip capsule gained
    7 mm of imaginary finger below the tip, which is more clearance than a grasp
    on an 8 mm part has to the belt.
    """
    capsule = Capsule(start=np.array([0.0, 0.0, 0.03]), end=np.array([0.0, 0.0, 0.0]), radius=0.007)
    points, radii = capsule.sample_points()
    assert radii[0] == pytest.approx(0.007)
    assert radii[-1] == pytest.approx(0.007)
    assert radii[1] > 0.007  # Interior samples are inflated.

    # The lowest surface of the capsule is exactly one true radius below the tip.
    floor = Box(name="floor", size=np.array([1.0, 1.0, 0.1]), pose=from_rpy(0, 0, 0, xyz=[0, 0, -0.05 - 0.007]))
    scene = CollisionScene([floor])
    gap, _ = scene.clearance([capsule])
    assert gap == pytest.approx(0.0, abs=1e-9)


def test_sampled_capsule_never_reports_more_clearance_than_the_truth():
    """Conservatism, stated as a property. The sampled check may under-report
    clearance; it must never over-report it."""
    rng = np.random.default_rng(4)
    box = Box(name="b", size=np.array([0.2, 0.3, 0.1]), pose=from_rpy(0.0, 0.0, 0.4, xyz=[0.1, 0.0, 0.0]))
    scene = CollisionScene([box])
    for _ in range(60):
        start = rng.uniform(-0.4, 0.4, 3)
        end = start + rng.uniform(-0.3, 0.3, 3)
        capsule = Capsule(start=start, end=end, radius=0.02)
        reported, _ = scene.clearance([capsule])

        dense = np.linspace(0, 1, 400)[:, None]
        points = start + dense * (end - start)
        exact = float(np.min(box.distance_to_points(points))) - 0.02
        assert reported <= exact + 1e-9


def test_home_configuration_is_collision_free():
    arm = ArmModel.from_cell(load_cell())
    clearance, culprit = arm.clearance(arm.cell.home_joints, arm.gripper.max_opening)
    assert clearance > 0.05, f"home is {clearance * 1000:.1f} mm from {culprit}"
    assert not self_collision(arm.cell.home_joints)


def test_a_configuration_driven_into_the_belt_is_rejected():
    arm = ArmModel.from_cell(load_cell())
    below = approach_pose([0.60, 0.0, arm.cell.conveyor_top - 0.05], 0.0)
    solution = arm.ik_nearest(below, arm.cell.home_joints)
    assert solution is not None, "the pose is reachable; it is just inside the conveyor"
    assert not arm.is_valid(solution.joints, arm.gripper.max_opening)


def test_a_grasp_on_a_thin_part_is_reachable():
    """The other side of the capsule-cap regression: a grasp 4 mm above the belt
    on an 8 mm part must be allowed, because it is what picking a coupon is."""
    cell = load_cell()
    arm = ArmModel.from_cell(cell)
    pose = approach_pose([0.60, 0.0, cell.conveyor_top + 0.004], 0.0)
    solution = arm.ik_nearest(pose, cell.home_joints)
    assert solution is not None
    clearance, culprit = arm.clearance(solution.joints, 0.03)
    assert clearance > 0.0, f"blocked by {culprit} at {clearance * 1000:.2f} mm"


def test_rrt_connect_finds_and_shortcuts_a_path():
    cell = load_cell()
    arm = ArmModel.from_cell(cell)
    rng = np.random.default_rng(0)
    validity = make_validity(arm, cell.gripper.max_opening)

    goal_pose = approach_pose([0.60, -0.15, 0.90], 0.3)
    goal = arm.ik_nearest(goal_pose, cell.home_joints)
    assert goal is not None and validity(goal.joints)

    result = plan_rrt_connect(cell.home_joints, goal.joints, validity, rng)
    assert result.ok, result.reason
    assert all(validity(point) for point in result.path)

    smoothed = shortcut(result.path, validity, rng)
    original = sum(float(np.linalg.norm(b - a)) for a, b in zip(result.path[:-1], result.path[1:]))
    shortened = sum(float(np.linalg.norm(b - a)) for a, b in zip(smoothed[:-1], smoothed[1:]))
    assert shortened <= original + 1e-9


def test_planner_refuses_a_goal_in_collision():
    cell = load_cell()
    arm = ArmModel.from_cell(cell)
    rng = np.random.default_rng(0)
    validity = make_validity(arm, cell.gripper.max_opening)
    goal = arm.ik_nearest(approach_pose([0.60, 0.0, cell.conveyor_top - 0.05], 0.0), cell.home_joints)
    result = plan_rrt_connect(cell.home_joints, goal.joints, validity, rng)
    assert not result.ok
    assert "goal" in result.reason


def test_cartesian_descent_is_a_straight_line():
    cell = load_cell()
    arm = ArmModel.from_cell(cell)
    start_pose = approach_pose([0.60, 0.0, 0.88], 0.2)
    start = arm.ik_nearest(start_pose, cell.home_joints)
    goal_pose = approach_pose([0.60, 0.0, 0.78], 0.2)

    result = plan_cartesian(arm, start.joints, goal_pose, 0.05)
    assert result.ok, result.reason

    positions = np.array([arm.tool_pose(point)[:3, 3] for point in result.path])
    lateral = np.linalg.norm(positions[:, :2] - positions[0, :2], axis=1)
    assert float(np.max(lateral)) < 1e-6
    assert positions[-1, 2] == pytest.approx(0.78, abs=1e-9)


def test_a_part_can_be_attached_and_ignored():
    cell = load_cell()
    arm = ArmModel.from_cell(cell)
    rng = np.random.default_rng(2)
    pose = sample_part_pose(cell, rng)
    arm.attach_part(part_box(pose))
    assert arm.scene.get("part") is not None
    arm.set_part(None)
    assert arm.scene.get("part") is None
