"""Perception and grasp planning, against poses whose truth is known."""

from __future__ import annotations

import numpy as np
import pytest

from cell_common.cell_model import load_cell, part_box, sample_part_pose
from cell_common.transforms import look_at, wrap_to_pi
from cell_grasping.core.camera import deproject, median_filter, project, render_depth
from cell_grasping.core.grasp import MIN_JAW_CLEARANCE, generate_candidates
from cell_grasping.core.perception import detect_scene, minimum_area_rectangle
from cell_motion.core.arm import ArmModel


@pytest.fixture(scope="module")
def cell():
    return load_cell()


@pytest.fixture(scope="module")
def arm(cell):
    return ArmModel.from_cell(cell)


def scan_camera(cell):
    return look_at(cell.scan_camera_xyz, cell.scan_target_xyz, up=(1.0, 0.0, 0.0))


def test_minimum_area_rectangle_is_exact_on_a_rotated_rectangle():
    rng = np.random.default_rng(0)
    for yaw in np.linspace(0.0, np.pi / 2, 7):
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        local = rng.uniform(-0.5, 0.5, size=(400, 2)) * np.array([0.08, 0.04])
        local = np.vstack([local, np.array([[0.04, 0.02], [-0.04, 0.02], [0.04, -0.02], [-0.04, -0.02]])])
        points = local @ rotation.T + np.array([0.3, -0.2])

        centre, angle, extents = minimum_area_rectangle(points)
        assert np.allclose(centre, [0.3, -0.2], atol=2e-3)
        assert extents[0] == pytest.approx(0.08, abs=2e-3)
        assert extents[1] == pytest.approx(0.04, abs=2e-3)
        delta = float(wrap_to_pi([angle - yaw])[0])
        assert abs((delta + np.pi / 2) % np.pi - np.pi / 2) < np.radians(2.0)


def test_minimum_area_rectangle_is_stable_on_a_square():
    """PCA is undefined here. The rectangle fit is not, and this is the case
    that cost every square-part grasp in the benchmark before it was used."""
    rng = np.random.default_rng(1)
    angles = []
    for trial in range(8):
        local = rng.uniform(-0.5, 0.5, size=(500, 2)) * 0.055
        local = np.vstack([local, np.array([[0.0275, 0.0275], [-0.0275, 0.0275],
                                            [0.0275, -0.0275], [-0.0275, -0.0275]])])
        centre, angle, extents = minimum_area_rectangle(local)
        angles.append(angle % (np.pi / 2))
        assert extents[0] == pytest.approx(0.055, abs=3e-3)
        assert extents[1] == pytest.approx(0.055, abs=3e-3)
    # Modulo 90 degrees, every fit agrees: a square has one answer, four times.
    spread = np.ptp(np.array(angles))
    assert min(spread, np.pi / 2 - spread) < np.radians(3.0)


def test_clustering_separates_two_parts_and_ignores_the_belt(cell, arm):
    rng = np.random.default_rng(7)
    # Both parts placed explicitly rather than sampled. A random draw can put a
    # 110 mm plate at either position, at which point the two are 20 mm apart
    # and clustering *correctly* merges them -- a true result about touching
    # parts, and not what this test is for.
    first = sample_part_pose(cell, rng, part=cell.part("bracket_a"))
    first["size"] = np.array([0.062, 0.038, 0.024])
    first["yaw"] = 0.0
    first["xyz"] = np.array([0.60, -0.07, cell.conveyor_top + 0.012])
    second = dict(first)
    second["xyz"] = np.array([0.60, 0.07, cell.conveyor_top + 0.012])

    image = median_filter(
        render_depth(
            scan_camera(cell),
            list(arm.scene.boxes) + [part_box(first), part_box(second, "part_1")],
            cell.camera,
            rng,
            stride=2,
        )
    )
    primary, clutter, plane = detect_scene(
        deproject(image), rng, cell.conveyor_top, workspace=cell.perception_workspace()
    )
    assert primary is not None
    assert len(clutter) >= 1
    assert plane.offset == pytest.approx(cell.conveyor_top, abs=0.004)


def test_perception_recovers_the_part_within_tolerance(cell, arm):
    """Centroid within 5 mm and yaw within 8 degrees, over 12 random poses.

    Loose thresholds on purpose: this is a regression guard, not the
    measurement. analysis/grasp_benchmark.py reports the distribution.
    """
    errors, yaws = [], []
    for seed in range(12):
        rng = np.random.default_rng(100 + seed)
        pose = sample_part_pose(cell, rng)
        image = median_filter(
            render_depth(scan_camera(cell), list(arm.scene.boxes) + [part_box(pose)], cell.camera, rng, stride=2)
        )
        estimate, _, _ = detect_scene(
            deproject(image), rng, cell.conveyor_top, workspace=cell.perception_workspace()
        )
        assert estimate is not None, f"no detection for {pose['part']}"
        errors.append(float(np.linalg.norm(estimate.centre - pose["xyz"])))

        size = pose["size"]
        true_yaw = pose["yaw"] if size[0] >= size[1] else pose["yaw"] + np.pi / 2
        delta = abs(((estimate.yaw - true_yaw + np.pi / 4) % (np.pi / 2)) - np.pi / 4)
        yaws.append(float(np.degrees(delta)))

    assert max(errors) < 0.005, f"worst centroid error {max(errors) * 1000:.1f} mm"
    assert np.median(yaws) < 8.0


def test_projection_and_deprojection_are_inverses(cell, arm):
    camera = scan_camera(cell)
    points = np.array([[0.60, 0.0, 0.72], [0.55, 0.10, 0.75], [0.66, -0.08, 0.71]])
    pixels = project(points, camera, cell.camera)
    assert np.all(np.isfinite(pixels))

    from cell_common.transforms import invert

    local = points @ invert(camera)[:3, :3].T + invert(camera)[:3, 3]
    rebuilt = np.column_stack(
        [
            (pixels[:, 0] - cell.camera.cx) / cell.camera.fx * local[:, 2],
            (pixels[:, 1] - cell.camera.cy) / cell.camera.fy * local[:, 2],
            local[:, 2],
        ]
    )
    rebuilt = rebuilt @ camera[:3, :3].T + camera[:3, 3]
    assert np.allclose(rebuilt, points, atol=1e-9)


def test_points_behind_the_camera_do_not_project(cell):
    camera = scan_camera(cell)
    behind = np.array([[0.58, 0.0, 1.60]])  # Above the camera, which looks down.
    assert np.all(np.isnan(project(behind, camera, cell.camera)))


def test_a_part_wider_than_the_stroke_is_rejected_not_scored(cell, arm):
    """The widest part must fail as 'wider_than_stroke', and it must fail on the
    long axis only -- the short axis is a perfectly ordinary tight grasp."""
    from cell_grasping.core.perception import PartEstimate

    estimate = PartEstimate(
        centre=np.array([0.60, 0.0, cell.conveyor_top + 0.006]),
        yaw=0.0,
        extents=np.array([0.110, 0.078, 0.012]),
        axis_ratio=110 / 78,
        support_height=cell.conveyor_top,
        point_count=2000,
        eigen_ratio=2.0,
    )
    plan = generate_candidates(estimate, arm)
    assert plan.rejections.get("wider_than_stroke", 0) >= 1
    assert plan.candidates, "the 78 mm axis is inside the 85 mm stroke and must be graspable"
    assert all(candidate.width <= cell.gripper.max_opening - 2 * MIN_JAW_CLEARANCE for candidate in plan.candidates)


def test_candidates_are_ranked_and_reachable(cell, arm):
    from cell_grasping.core.perception import PartEstimate

    estimate = PartEstimate(
        centre=np.array([0.60, 0.0, cell.conveyor_top + 0.012]),
        yaw=0.3,
        extents=np.array([0.062, 0.038, 0.024]),
        axis_ratio=62 / 38,
        support_height=cell.conveyor_top,
        point_count=1800,
        eigen_ratio=2.4,
    )
    plan = generate_candidates(estimate, arm)
    assert plan.candidates
    scores = [candidate.score for candidate in plan.candidates]
    assert scores == sorted(scores, reverse=True)
    for candidate in plan.candidates:
        assert candidate.joints is not None
        assert arm.is_valid(candidate.joints, candidate.opening, ignore=("part",), margin=0.0)


def test_the_best_candidate_closes_across_the_short_axis(cell, arm):
    """Given a choice, the planner should take the narrow grip, not the wide one."""
    from cell_grasping.core.perception import PartEstimate

    estimate = PartEstimate(
        centre=np.array([0.60, 0.0, cell.conveyor_top + 0.012]),
        yaw=0.0,
        extents=np.array([0.070, 0.030, 0.024]),
        axis_ratio=70 / 30,
        support_height=cell.conveyor_top,
        point_count=1800,
        eigen_ratio=2.6,
    )
    best = generate_candidates(estimate, arm).best
    assert best is not None
    assert best.width == pytest.approx(0.030, abs=1e-6)
