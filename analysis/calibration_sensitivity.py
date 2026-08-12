#!/usr/bin/env python3
"""How much hand-eye error the cell tolerates before grasps start failing.

Runs the grasp benchmark at a series of injected hand-eye translation errors
and reports the success rate at each::

    python analysis/calibration_sensitivity.py --trials 50 --output results/calibration_sensitivity.json

This is the number that decides how long to spend on a calibration. "As
accurate as possible" is not an engineering answer; "the cell holds above 95%
up to about a centimetre, and falls off a cliff past two" is, and it is what
tells you whether the twenty-pose routine is worth its four minutes over the
six-pose one.

The error is injected as a fixed displacement of the true extrinsic, mostly
lateral in the camera frame -- see
:func:`analysis.grasp_benchmark.perturbed_hand_eye` for why the direction
matters. Everything else about the cell is nominal, so the curve isolates the
calibration.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT / "analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "analysis"))

from grasp_benchmark import MISCALIBRATION_ROTATION, run  # noqa: E402

DEFAULT_ERRORS_MM = [0.0, 2.0, 5.0, 10.0, 16.0, 22.0, 30.0]


def sweep(errors_mm: List[float], trials: int, seed: int, stride: int) -> Dict:
    points = []
    for error in errors_mm:
        condition = "nominal" if error == 0.0 else "miscalibrated"
        summary = run(
            trials=trials,
            seed=seed,
            stride=stride,
            condition=condition,
            hand_eye_translation=error / 1000.0,
            hand_eye_rotation=MISCALIBRATION_ROTATION if error > 0 else 0.0,
        )
        rate = summary["success"]["rate"]
        points.append(
            {
                "hand_eye_error_mm": error,
                "trials": summary["trials"],
                "success_rate": rate["point"],
                "ci_low": rate["low"],
                "ci_high": rate["high"],
                "attempted_and_lost": summary["attempted_and_failed"]["rate"]["point"],
                "declined": summary["declined_before_attempt"]["rate"]["point"],
                "failure_reasons": summary["failure_reasons"],
                "perception_centre_median_mm": summary["perception_error"]["centre_mm"].get("median"),
            }
        )
    return {"seed": seed, "trials": trials, "points": points}


def report(summary: Dict) -> str:
    lines = [
        f"Grasp success against injected hand-eye error ({summary['trials']} trials each, seed {summary['seed']})",
        "",
        f"{'error mm':>9} {'success':>9} {'95% CI':>16} {'lost':>7} {'declined':>9}  perception centre err",
    ]
    for point in summary["points"]:
        lines.append(
            f"{point['hand_eye_error_mm']:>9.1f} "
            f"{point['success_rate'] * 100:>8.1f}% "
            f"{point['ci_low'] * 100:>7.1f}-{point['ci_high'] * 100:<8.1f} "
            f"{point['attempted_and_lost'] * 100:>6.1f}% "
            f"{point['declined'] * 100:>8.1f}%  "
            f"{point['perception_centre_median_mm']:>6.2f} mm"
        )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trials", type=int, default=50)
    parser.add_argument("--seed", type=int, default=20260101)
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--errors-mm", type=float, nargs="+", default=DEFAULT_ERRORS_MM)
    parser.add_argument("--output", type=Path, default=None)
    arguments = parser.parse_args()

    summary = sweep(arguments.errors_mm, arguments.trials, arguments.seed, arguments.stride)
    print(report(summary))
    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"\nwrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
