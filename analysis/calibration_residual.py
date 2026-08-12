#!/usr/bin/env python3
"""Eye-in-hand calibration accuracy, and what the field diagnostics miss.

Runs the calibration routine over repeated randomised pose sets and reports two
things side by side::

    python analysis/calibration_residual.py --output results/calibration.json

* the **ground-truth error** against the transform the simulator used, which
  exists only here;
* the **consistency residual** -- the AX = XB mismatch and the spread of the
  reconstructed board position -- which is what you can compute on a real
  robot.

THE POINT OF RUNNING BOTH

A calibration is trusted on the strength of numbers like reprojection error and
board-position spread. This script exists to show how far that trust goes. The
"thorough" pose set (20 poses, well spread in orientation) and the "hurried"
one (6 nearly-overhead poses) produce field diagnostics that are hard to tell
apart -- both report a reprojection error around 0.15 px and a board spread
under a millimetre -- while their true errors differ by an order of magnitude.

The reason is conditioning, not noise. AX = XB fixes the translation of X only
from motions whose rotation axes span three dimensions. Nearly-overhead poses
rotate about one axis, so the solve is ill-conditioned, and an ill-conditioned
least-squares problem does not announce itself: it returns a confident answer
with a small residual.

``analysis/calibration_sensitivity.py`` then converts millimetres of hand-eye
error into percentage points of grasp success, which is the number that decides
how many poses are worth collecting.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parent.parent
for _package in ("cell_common", "cell_motion", "cell_calibration"):
    _path = str(REPO_ROOT / "src" / _package)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from cell_common.cell_model import load_cell  # noqa: E402
from cell_common.transforms import from_rpy  # noqa: E402
from cell_calibration.core.hand_eye import calibrate, collect_samples, generate_poses  # noqa: E402
from cell_motion.core.arm import ArmModel  # noqa: E402

# The pose sets compared. "thorough" is what the routine is meant to collect;
# "hurried" is what gets collected when the line is waiting.
POSE_SETS = {
    "thorough": dict(count=20, tilt=0.42, standoff=(0.30, 0.45), flatten=False, min_rotation_deg=8.0),
    "adequate": dict(count=10, tilt=0.30, standoff=(0.32, 0.44), flatten=False, min_rotation_deg=5.0),
    "hurried": dict(count=6, tilt=0.10, standoff=(0.34, 0.42), flatten=True, min_rotation_deg=1.0),
}


def run_one(arm: ArmModel, board: np.ndarray, specification: Dict, rng: np.random.Generator) -> Dict:
    poses = generate_poses(
        board, specification["count"], rng, standoff=specification["standoff"], tilt=specification["tilt"]
    )
    if specification["flatten"]:
        # Replace the spread orientations with near-overhead ones: still
        # rotating, but all about the vertical.
        down = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        for pose in poses:
            pose[:3, :3] = (
                Rotation.from_euler("z", rng.uniform(-0.30, 0.30)).as_matrix()
                @ Rotation.from_euler("y", rng.uniform(-0.10, 0.10)).as_matrix()
                @ down
            )

    samples = collect_samples(arm, board, poses, rng)
    if len(samples) < 4:
        return {"usable_poses": len(samples), "solved": False}

    result = calibrate(samples, min_rotation=np.radians(specification["min_rotation_deg"]))
    record = result.as_dict()
    record["usable_poses"] = len(samples)
    record["solved"] = True
    record["ground_truth"] = result.error_against(arm.cell.camera.extrinsic)
    return record


def run(repeats: int, seed: int) -> Dict:
    cell = load_cell()
    arm = ArmModel.from_cell(cell)
    board = from_rpy(0.0, 0.0, 0.3, xyz=[0.60, 0.0, cell.conveyor_top + 0.002])

    results: Dict[str, Dict] = {}
    for name, specification in POSE_SETS.items():
        records: List[Dict] = []
        for index in range(repeats):
            rng = np.random.default_rng(seed + 1000 * index)
            try:
                records.append(run_one(arm, board, specification, rng))
            except ValueError as error:
                records.append({"solved": False, "reason": str(error)})

        solved = [record for record in records if record.get("solved")]
        results[name] = {
            "specification": {key: value for key, value in specification.items()},
            "repeats": repeats,
            "solved": len(solved),
            "translation_error_mm": _distribution([record["ground_truth"]["translation_mm"] for record in solved]),
            "rotation_error_deg": _distribution([record["ground_truth"]["rotation_deg"] for record in solved]),
            "consistency_translation_mm": _distribution([record["consistency_translation_mm"] for record in solved]),
            "consistency_rotation_deg": _distribution([record["consistency_rotation_deg"] for record in solved]),
            "board_spread_mm": _distribution([record["board_spread_mm"] for record in solved]),
            "reprojection_rms_px": _distribution([record["reprojection_rms_px"] for record in solved]),
            "records": records,
        }
    return {"seed": seed, "repeats": repeats, "pose_sets": results}


def _distribution(values: List[float]) -> Dict:
    if not values:
        return {}
    array = np.asarray(values, dtype=float)
    return {
        "n": int(len(array)),
        "median": round(float(np.median(array)), 4),
        "mean": round(float(np.mean(array)), 4),
        "p90": round(float(np.percentile(array, 90)), 4),
        "max": round(float(np.max(array)), 4),
    }


def report(summary: Dict) -> str:
    lines = [
        f"Hand-eye calibration over {summary['repeats']} randomised pose sets (seed {summary['seed']})",
        "",
        f"{'pose set':<10} {'poses':>6} {'TRUE error':>22} {'   what the cell can measure':<44}",
        f"{'':<10} {'':>6} {'trans mm':>10} {'rot deg':>11}  "
        f"{'AX-XB mm':>9} {'spread mm':>10} {'reproj px':>10}",
    ]
    for name, entry in summary["pose_sets"].items():
        if not entry["translation_error_mm"]:
            lines.append(f"{name:<10}  no solution")
            continue
        lines.append(
            f"{name:<10} {entry['specification']['count']:>6} "
            f"{entry['translation_error_mm']['median']:>10.2f} "
            f"{entry['rotation_error_deg']['median']:>11.3f}  "
            f"{entry['consistency_translation_mm']['median']:>9.2f} "
            f"{entry['board_spread_mm']['median']:>10.2f} "
            f"{entry['reprojection_rms_px']['median']:>10.3f}"
        )
    lines.append("")
    lines.append(
        "Medians. The left pair needs the simulator's ground truth; the right three are\n"
        "what a real cell can compute. The right three barely move between pose sets."
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repeats", type=int, default=25)
    parser.add_argument("--seed", type=int, default=20260101)
    parser.add_argument("--output", type=Path, default=None)
    arguments = parser.parse_args()

    summary = run(arguments.repeats, arguments.seed)
    print(report(summary))
    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"\nwrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
