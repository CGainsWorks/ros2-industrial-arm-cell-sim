"""The grasp judge, the trajectory timing, and one end-to-end cycle."""

from __future__ import annotations

import numpy as np
import pytest

from cell_common.cell_model import load_cell, sample_part_pose
from cell_bringup.core.cycle import CycleRunner
from cell_bringup.core.outcome import evaluate_grasp, summarise, wilson_interval
from cell_motion.core.kinematics import ACCELERATION_LIMITS, VELOCITY_LIMITS
from cell_motion.core.trajectory import concatenate, resample, time_parameterise


@pytest.fixture(scope="module")
def cell():
    return load_cell()


def flat_part(cell, size=(0.062, 0.038, 0.024), yaw=0.0, mass=0.14, friction=0.7):
    return {
        "part": "bracket_a",
        "size": np.asarray(size, dtype=float),
        "resting_face": 0,
        "xyz": np.array([0.60, 0.0, cell.conveyor_top + size[2] / 2.0]),
        "yaw": yaw,
        "mass": mass,
        "friction": friction,
    }


def test_a_centred_square_on_grasp_holds(cell):
    part = flat_part(cell)
    outcome = evaluate_grasp(part["xyz"], np.pi / 2, 0.050, part, cell.gripper)
    assert outcome.held, outcome.failure
    assert outcome.width == pytest.approx(0.038, abs=1e-9)


def test_a_grasp_across_the_long_axis_of_a_wide_part_is_too_wide(cell):
    part = flat_part(cell, size=(0.110, 0.078, 0.012))
    outcome = evaluate_grasp(part["xyz"], 0.0, 0.085, part, cell.gripper)
    assert not outcome.held
    assert outcome.failure == "too_wide"


def test_a_diagonal_grasp_fails_the_friction_cone(cell):
    """45 degrees off the face normal is outside the cone for any real pad."""
    part = flat_part(cell)
    outcome = evaluate_grasp(part["xyz"], np.pi / 4, 0.085, part, cell.gripper)
    assert not outcome.held
    assert outcome.failure == "non_antipodal"
    assert outcome.contact_angle > outcome.friction_angle


def test_a_grasp_that_misses_the_part_is_reported_as_missed(cell):
    """Offset along the *jaw* axis, not the closing axis.

    With the jaws closing along y, sliding the tool along y keeps the grasp
    line through the part and is not a miss at all -- it is the same grasp.
    A miss is an offset along x, past the 31 mm half-length.
    """
    part = flat_part(cell)
    offset = part["xyz"] + np.array([0.05, 0.0, 0.0])
    outcome = evaluate_grasp(offset, np.pi / 2, 0.050, part, cell.gripper)
    assert not outcome.held
    assert outcome.failure == "missed"


def test_a_grasp_above_the_part_does_not_engage(cell):
    part = flat_part(cell)
    high = part["xyz"] + np.array([0.0, 0.0, 0.06])
    outcome = evaluate_grasp(high, np.pi / 2, 0.050, part, cell.gripper)
    assert not outcome.held
    assert outcome.failure == "shallow"


def test_jaws_narrower_than_the_part_collide_on_the_way_down(cell):
    part = flat_part(cell)
    outcome = evaluate_grasp(part["xyz"], np.pi / 2, 0.030, part, cell.gripper)
    assert not outcome.held
    assert outcome.failure == "finger_collision"


def test_the_friction_cone_is_the_part_property_not_a_constant(cell):
    """The same geometry holds on a grippy part and slips on a slick one."""
    angle = np.pi / 2 + np.radians(20.0)
    grippy = flat_part(cell, friction=0.7)  # atan(0.7) = 35 degrees.
    slick = flat_part(cell, friction=0.3)  # atan(0.3) = 17 degrees.
    assert evaluate_grasp(grippy["xyz"], angle, 0.060, grippy, cell.gripper).held
    assert not evaluate_grasp(slick["xyz"], angle, 0.060, slick, cell.gripper).held


def test_summarise_and_wilson_interval():
    part = flat_part(load_cell())
    held = [evaluate_grasp(part["xyz"], np.pi / 2, 0.050, part, load_cell().gripper) for _ in range(9)]
    lost = [evaluate_grasp(part["xyz"], np.pi / 4, 0.085, part, load_cell().gripper)]
    summary = summarise(held + lost)
    assert summary["attempts"] == 10
    assert summary["held"] == 9
    assert summary["success_rate"] == pytest.approx(0.9)
    assert summary["failures"] == {"non_antipodal": 1}

    interval = wilson_interval(180, 200)
    assert interval["low"] < 0.9 < interval["high"]
    assert interval["high"] - interval["low"] < 0.10


def test_time_parameterisation_respects_its_limits():
    rng = np.random.default_rng(0)
    path = [rng.uniform(-1.0, 1.0, 6)]
    for _ in range(12):
        path.append(path[-1] + rng.uniform(-0.4, 0.4, 6))

    trajectory = time_parameterise(path, velocity_scale=0.5, acceleration_scale=0.35)
    assert trajectory.duration > 0.0
    assert trajectory.peak_velocity_fraction(0.5) <= 1.0 + 1e-6

    # The profile starts and ends at rest, but `velocities()` is a finite
    # difference: the first entry is the *average* over the first segment,
    # which is not zero and should not be. What must hold is that the ends are
    # slower than the middle.
    speeds = np.linalg.norm(trajectory.velocities(), axis=1)
    assert speeds[0] < speeds.max()
    assert speeds[-1] < speeds.max()


def test_a_finely_sampled_short_move_does_not_take_forever():
    """The regression that made a 100 mm approach take nine seconds.

    A straight-line descent sampled every 5 mm is 20 nearly-collinear
    waypoints. Any parameterisation that treats each waypoint as a place to
    slow down turns it into a crawl, and the cycle time it produces is wrong in
    the direction that looks conservative.
    """
    start = np.array([0.0, -1.4, -1.4, -1.8, 1.57, -1.27])
    direction = np.array([0.01, 0.02, -0.02, 0.01, 0.0, 0.0])
    path = [start + direction * index for index in range(21)]

    trajectory = time_parameterise(path)
    assert trajectory.duration < 1.5, f"{trajectory.duration:.2f} s for a short move"
    assert trajectory.duration > 0.1


def test_a_single_long_segment_reaches_the_velocity_limit():
    path = [np.zeros(6), np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])]
    trajectory = time_parameterise(path, velocity_scale=0.5, acceleration_scale=0.35)
    expected_floor = 2.0 / (VELOCITY_LIMITS[0] * 0.5)
    ramp = (VELOCITY_LIMITS[0] * 0.5) / (ACCELERATION_LIMITS[0] * 0.35)
    assert expected_floor <= trajectory.duration <= expected_floor + ramp + 1e-6


def test_resample_and_concatenate_preserve_the_endpoints():
    path = [np.zeros(6), np.ones(6) * 0.5, np.ones(6) * 0.2]
    first = time_parameterise(path, label="a")
    second = time_parameterise(path[::-1], label="b")

    dense = resample(first, period=0.008)
    assert np.allclose(dense.positions[0], first.positions[0], atol=1e-9)
    assert dense.duration == pytest.approx(first.duration, abs=0.01)

    joined = concatenate([first, second])
    assert joined.duration == pytest.approx(first.duration + second.duration, abs=1e-9)
    assert np.allclose(joined.positions[0], first.positions[0])
    assert np.allclose(joined.positions[-1], second.positions[-1])


def test_one_full_cycle_completes(cell):
    """End to end, with a seed known to succeed. The rate over many seeds is
    what analysis/grasp_benchmark.py reports; this only guards the wiring."""
    runner = CycleRunner(cell)
    rng = np.random.default_rng(20260101)
    result = runner.run(sample_part_pose(cell, rng), rng)

    assert result.success, f"{result.stage}: {result.failure}"
    assert result.stage == "complete"
    assert 5.0 < result.cycle_time < 25.0
    assert result.outcome is not None and result.outcome.held
    assert [segment.name for segment in result.segments][:2] == ["cycle_start_handshake", "to_scan"]
    assert result.estimate_error["centre_mm"] < 10.0


def test_the_cycle_leaves_no_parts_in_the_scene(cell):
    """A failed cycle that leaks its part or clutter poisons every later trial."""
    runner = CycleRunner(cell)
    for seed in range(4):
        rng = np.random.default_rng(seed)
        first = sample_part_pose(cell, rng)
        runner.run(first, rng, distractors=[sample_part_pose(cell, rng)])
        leaked = [box.name for box in runner.arm.scene.boxes if box.kind in ("part", "clutter")]
        assert leaked == [], f"left {leaked} behind"
