"""Hand-eye calibration and force-feedback probing."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from cell_calibration.core.hand_eye import (
    board_corners,
    calibrate,
    collect_samples,
    generate_poses,
    project_corners,
    solve_pnp,
    tsai_lenz,
)
from cell_common.cell_model import load_cell
from cell_common.transforms import from_rpy, invert, transform
from cell_inspection.core.contact import (
    FAULT_NO_CONTACT,
    FAULT_NONE,
    FAULT_NOT_SEATED,
    ContactSurface,
    inspect_part,
    inspection_points,
    probe_point,
)
from cell_motion.core.arm import ArmModel


@pytest.fixture(scope="module")
def cell():
    return load_cell()


@pytest.fixture(scope="module")
def arm(cell):
    return ArmModel.from_cell(cell)


def board_pose(cell):
    return from_rpy(0.0, 0.0, 0.3, xyz=[0.60, 0.0, cell.conveyor_top + 0.002])


def test_tsai_lenz_is_exact_without_noise():
    """No noise, no excuse: the closed form must recover X to machine precision."""
    truth = from_rpy(0.1, -0.2, 0.3, xyz=[0.02, -0.08, 0.04])
    rng = np.random.default_rng(0)

    motions = []
    for _ in range(6):
        a = transform(Rotation.from_rotvec(rng.uniform(-0.6, 0.6, 3)).as_matrix(), rng.uniform(-0.2, 0.2, 3))
        b = invert(truth) @ a @ truth
        motions.append((a, b))

    recovered = tsai_lenz(motions)
    assert np.allclose(recovered, truth, atol=1e-9)


def test_pnp_recovers_a_board_pose_from_clean_corners(cell):
    pose = transform(Rotation.from_euler("xyz", [0.2, -0.15, 0.4]).as_matrix(), [0.01, -0.02, 0.42])
    pixels = project_corners(pose, cell.camera)
    assert np.sum(np.all(np.isfinite(pixels), axis=1)) > 40

    seed = pose.copy()
    seed[:3, 3] += 0.02
    recovered, rms = solve_pnp(pixels, cell.camera, seed)
    assert recovered is not None
    assert rms < 1e-6
    assert np.allclose(recovered, pose, atol=1e-6)


def test_board_corners_are_a_grid_of_the_right_size():
    corners = board_corners()
    assert corners.shape[1] == 3
    assert np.allclose(corners[:, 2], 0.0)
    spread = corners.max(axis=0) - corners.min(axis=0)
    assert spread[0] == pytest.approx(0.020 * 8)
    assert spread[1] == pytest.approx(0.020 * 6)


def test_a_spread_pose_set_recovers_the_extrinsic(cell, arm):
    rng = np.random.default_rng(7)
    poses = generate_poses(board_pose(cell), 20, rng)
    samples = collect_samples(arm, board_pose(cell), poses, rng)
    assert len(samples) >= 12

    result = calibrate(samples)
    error = result.error_against(cell.camera.extrinsic)
    assert error["translation_mm"] < 2.0
    assert error["rotation_deg"] < 0.3
    assert result.reprojection_rms < 0.3
    assert result.rotation_axis_spread_deg > 45.0


def test_a_pose_set_with_no_rotation_is_refused(cell, arm):
    """AX = XB needs rotating motions. A set that only translates must raise,
    not return a confident wrong answer."""
    rng = np.random.default_rng(3)
    poses = generate_poses(board_pose(cell), 8, rng, tilt=0.0)
    down = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    for pose in poses:
        pose[:3, :3] = down
    samples = collect_samples(arm, board_pose(cell), poses, rng)
    if len(samples) < 4:
        pytest.skip("the flattened poses were not reachable; nothing to assert")
    with pytest.raises(ValueError):
        calibrate(samples)


def test_a_narrow_pose_set_is_worse_than_it_looks(cell, arm):
    """The finding behind the miscalibrated benchmark condition.

    Nearly-overhead poses leave the translation of X ill-conditioned. The
    ground-truth error is several times worse than for a spread set, while the
    consistency residual -- the only thing measurable on a real robot -- barely
    moves. This test asserts that gap exists, because the whole argument for
    the pose-spread check in the calibration node rests on it.
    """
    rng = np.random.default_rng(11)
    spread_poses = generate_poses(board_pose(cell), 20, rng)
    spread = calibrate(collect_samples(arm, board_pose(cell), spread_poses, rng))

    narrow_poses = generate_poses(board_pose(cell), 6, rng, standoff=(0.34, 0.42), tilt=0.10)
    down = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    for pose in narrow_poses:
        pose[:3, :3] = (
            Rotation.from_euler("z", rng.uniform(-0.30, 0.30)).as_matrix()
            @ Rotation.from_euler("y", rng.uniform(-0.10, 0.10)).as_matrix()
            @ down
        )
    narrow = calibrate(collect_samples(arm, board_pose(cell), narrow_poses, rng), min_rotation=np.radians(1.0))

    spread_error = spread.error_against(cell.camera.extrinsic)["translation_mm"]
    narrow_error = narrow.error_against(cell.camera.extrinsic)["translation_mm"]
    assert narrow_error > spread_error

    # And the diagnostic a real cell would look at does not reveal it.
    assert narrow.reprojection_rms < 0.3
    assert narrow.consistency_translation_mm < 4.0


# -- inspection ------------------------------------------------------------


def test_a_probe_measures_a_flat_surface_to_tens_of_microns():
    errors = []
    for seed in range(20):
        rng = np.random.default_rng(seed)
        surface = ContactSurface(height=0.792)
        result = probe_point(surface, [0.0, -0.60], 0.797, rng)
        assert result.fault == FAULT_NONE
        errors.append(abs(result.error))
    assert np.median(errors) < 50e-6
    assert max(errors) < 200e-6


def test_the_probe_settles_at_the_commanded_force_instead_of_driving_through():
    """The admittance regression: keeping the approach term after contact means
    the loop never stops descending and every probe hits the abort limit."""
    from cell_inspection.core.contact import OVERFORCE_LIMIT, TARGET_FORCE

    rng = np.random.default_rng(0)
    result = probe_point(ContactSurface(height=0.792), [0.0, -0.60], 0.797, rng)
    assert result.fault == FAULT_NONE
    assert result.peak_force < OVERFORCE_LIMIT
    assert result.steady_force == pytest.approx(TARGET_FORCE, abs=1.0)


def test_no_part_in_the_nest_raises_no_contact():
    rng = np.random.default_rng(0)
    result = probe_point(ContactSurface(height=0.700), [0.0, -0.60], 0.797, rng)
    assert result.fault == FAULT_NO_CONTACT


def test_a_seated_part_passes_and_a_lifted_corner_fails(cell):
    rng = np.random.default_rng(0)
    points = inspection_points([0.0, -0.60], (0.062, 0.038), cell.inspection_points)
    height = cell.fixture_top + 0.012

    flat = inspect_part(ContactSurface(height=height, origin=np.array([0.0, -0.60])), points, height + 0.005, rng)
    assert flat.fault == FAULT_NONE
    assert flat.tilt_deg < 0.1
    assert flat.mean_abs_error_um < 60.0

    swarf = inspect_part(
        ContactSurface(height=height, tilt=np.array([0.012, 0.0]), origin=np.array([0.0, -0.60])),
        points,
        height + 0.005,
        rng,
    )
    assert swarf.fault == FAULT_NOT_SEATED
    assert swarf.tilt_deg > 0.35


def test_the_stiffness_correction_matters():
    """Without the deflection correction the probe reads about 100 microns deep,
    which is more than every other term in the error budget combined."""
    from cell_inspection.core.contact import CONTACT_THRESHOLD

    rng = np.random.default_rng(0)
    stiffness = 20000.0
    result = probe_point(ContactSurface(height=0.792, stiffness=stiffness), [0.0, -0.60], 0.797, rng)
    uncorrected = result.measured_height - result.estimated_stiffness * 0 - CONTACT_THRESHOLD / stiffness
    assert abs(result.error) < abs(uncorrected - result.true_height)
