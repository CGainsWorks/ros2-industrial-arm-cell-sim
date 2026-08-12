#!/usr/bin/env python3
"""Grasp success rate and cycle time over a randomised part set.

This is the script behind the headline numbers in the README. It runs the whole
pick-and-place cycle -- render, perceive, plan, approach, grasp, transfer,
place -- over N randomised part presentations and reports what happened, with
a confidence interval and a failure breakdown::

    python analysis/grasp_benchmark.py --trials 200 --output results/grasp_benchmark.json

It needs numpy, scipy and pyyaml. No ROS, no Gazebo, no dataset download, and
about three minutes for 200 trials on one core.

WHY 200 AND WHY AN INTERVAL

A pick-and-place demo that works once is a video. A success rate over a
randomised set is a measurement, and a measurement without an interval is a
measurement quoted more precisely than it was taken. At a rate near 0.9,
200 trials gives a 95% Wilson interval about four percentage points wide, and
20 trials gives one about seventeen -- wide enough that most tuning changes
would be indistinguishable from noise. That is the reason for the number.

WHAT COUNTS AS A SUCCESS

The part is grasped, held through the transfer, and released in the nest. Every
stage can fail and each failure is attributed to the stage that produced it,
using the ground-truth criteria in :mod:`cell_bringup.core.outcome`. The
distinction that matters is between:

* **Perception and planning failures**, where the cell knew it could not do it.
  These are cheap: the part goes round again on the belt.
* **Grasp failures**, where the cell tried and the part did not come with it.
  These are expensive: a dropped part is a stopped line.

Both are reported, and the README quotes both, because a cell with a 95%
success rate made of 5% attempted-and-dropped is a worse cell than one at 90%
made of 10% declined.

CONDITIONS

The nominal condition is a clean cell: one part, a correct hand-eye transform,
a healthy camera. It is the right headline number and it is not, on its own,
evidence of much -- so the same poses are run under three degradations a real
cell meets, each reported separately:

``clutter``
    A second part on the belt. The cell has to segment it, plan around it, and
    not pick it up by mistake.
``miscalibrated``
    The hand-eye transform displaced by 5 mm and half a degree -- the size of
    error a hurried six-pose calibration produces, as measured by
    ``analysis/calibration_residual.py``, and one that every diagnostic
    available on a real robot reports as fine. This condition connects the
    calibration deliverable to the grasp result.
``noisy``
    Three times the depth noise, roughly what a dirty window or a
    low-reflectance part costs.

RANDOMISATION

Part type, resting face, position within the pick region and yaw are all drawn
per trial from a seeded generator. The seed is recorded in the output, so any
individual trial can be re-run with ``--only-trial``.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Dict, List

import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parent.parent
for _package in ("cell_common", "cell_motion", "cell_grasping", "cell_calibration", "cell_inspection", "cell_bringup"):
    _path = str(REPO_ROOT / "src" / _package)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from cell_common.cell_model import CellSpec, load_cell, sample_part_pose  # noqa: E402
from cell_bringup.core.cycle import CycleRunner  # noqa: E402
from cell_bringup.core.outcome import wilson_interval  # noqa: E402
from cell_motion.core.trajectory import duration_by_kind  # noqa: E402

CONDITIONS = ("nominal", "clutter", "miscalibrated", "noisy")

# Which stage each failure belongs to, for the "declined" versus "attempted"
# split. A failure at or after the grasp means the cell committed to a pick.
ATTEMPTED_STAGES = {"grasp", "lift", "transfer", "place", "retract", "retreat"}


def degraded_cell(cell: CellSpec, noise_factor: float) -> CellSpec:
    """A copy of the cell with the camera's noise terms scaled."""
    import copy

    raw = copy.deepcopy(cell.raw)
    raw["camera"]["depth_noise_base"] *= noise_factor
    raw["camera"]["depth_noise_quadratic"] *= noise_factor
    return CellSpec(raw=raw, path=cell.path)


# The hand-eye error the ``miscalibrated`` condition injects, in metres and
# radians. Not drawn at random and not the output of one calibration run: this
# condition has to answer "what does N mm of hand-eye error cost in grasp
# success", and that needs N fixed. ``analysis/calibration_residual.py`` reports
# the *distribution* of errors a hurried six-pose calibration produces, which is
# where these numbers come from -- a few millimetres and a few tenths of a
# degree, with every field-measurable diagnostic reading clean.
MISCALIBRATION_TRANSLATION = 0.005
MISCALIBRATION_ROTATION = 0.009  # About half a degree.


def perturbed_hand_eye(cell: CellSpec, translation: float, rotation: float) -> np.ndarray:
    """The true extrinsic, displaced by a fixed error.

    The displacement direction is fixed too, and chosen to be mostly lateral in
    the camera frame. A hand-eye error along the *optical axis* is the benign
    one -- it scales the apparent depth of a plane the cell already knows the
    height of -- while an error across it moves every part sideways by the same
    amount, which is what actually costs grasps.
    """
    direction = np.array([0.78, 0.58, 0.22])
    direction = direction / np.linalg.norm(direction)
    offset = np.eye(4)
    offset[:3, :3] = Rotation.from_rotvec(np.array([0.0, 0.0, 1.0]) * rotation).as_matrix()
    offset[:3, 3] = direction * translation
    return np.asarray(cell.camera.extrinsic) @ offset


def run(
    trials: int,
    seed: int,
    stride: int,
    condition: str = "nominal",
    only_trial: int = -1,
    verbose: bool = False,
    hand_eye_translation: float = MISCALIBRATION_TRANSLATION,
    hand_eye_rotation: float = MISCALIBRATION_ROTATION,
) -> Dict:
    cell = load_cell()
    hand_eye = None
    if condition == "noisy":
        cell = degraded_cell(cell, 3.0)
    elif condition == "miscalibrated":
        hand_eye = perturbed_hand_eye(cell, hand_eye_translation, hand_eye_rotation)
    runner = CycleRunner(cell, hand_eye=hand_eye, stride=stride)

    records: List[Dict] = []
    started = time.time()
    for index in range(trials):
        if only_trial >= 0 and index != only_trial:
            continue
        rng = np.random.default_rng(seed + index)
        part_pose = sample_part_pose(cell, rng)
        distractors = []
        if condition == "clutter":
            for _ in range(12):
                candidate = sample_part_pose(cell, rng)
                separation = float(np.linalg.norm(candidate["xyz"][:2] - part_pose["xyz"][:2]))
                clearance = float(np.max(candidate["size"][:2]) + np.max(part_pose["size"][:2])) / 2.0
                # A *near* neighbour: far enough not to intersect, close
                # enough that the jaws, the fingers and the approach have to
                # take account of it. A distractor dropped anywhere in the
                # region is almost always far away and tests nothing.
                if clearance + 0.008 < separation < clearance + 0.045:
                    distractors.append(candidate)
                    break
        result = runner.run(part_pose, rng, distractors=distractors)

        record = result.as_dict()
        record["trial"] = index
        record["seed"] = seed + index
        record["part"] = part_pose["part"]
        record["resting_face"] = int(part_pose["resting_face"])
        record["part_xy"] = [round(float(value), 4) for value in part_pose["xyz"][:2]]
        record["part_yaw_deg"] = round(float(np.degrees(part_pose["yaw"])), 2)
        record["part_size_mm"] = [round(float(value) * 1000.0, 1) for value in part_pose["size"]]
        record["by_kind"] = duration_by_kind(result.segments)
        record["distractors"] = len(distractors)
        records.append(record)

        if verbose:
            status = "ok" if result.success else f"{result.stage}:{result.failure}"
            print(f"  trial {index:3d}  {part_pose['part']:<16} {status:<34} {result.cycle_time:5.2f} s")

    summary = summarise(records, trials, seed, time.time() - started)
    summary["condition"] = condition
    if hand_eye is not None:
        summary["hand_eye_error_mm"] = round(
            float(np.linalg.norm(hand_eye[:3, 3] - cell.camera.extrinsic[:3, 3]) * 1000.0), 3
        )
    return summary


def summarise(records: List[Dict], trials: int, seed: int, wall_clock: float) -> Dict:
    successes = [record for record in records if record["success"]]
    failures = [record for record in records if not record["success"]]

    attempted_failures = [record for record in failures if record["stage"] in ATTEMPTED_STAGES]
    declined_failures = [record for record in failures if record["stage"] not in ATTEMPTED_STAGES]

    reasons = Counter(f"{record['stage']}:{record['failure']}" for record in failures)
    by_part: Dict[str, Dict] = {}
    for record in records:
        entry = by_part.setdefault(record["part"], {"trials": 0, "held": 0})
        entry["trials"] += 1
        entry["held"] += int(record["success"])
    for entry in by_part.values():
        entry["success_rate"] = round(entry["held"] / entry["trials"], 4)

    cycle_times = np.array([record["cycle_time"] for record in successes]) if successes else np.zeros(0)
    kinds: Dict[str, List[float]] = {}
    for record in successes:
        for kind, value in record["by_kind"].items():
            kinds.setdefault(kind, []).append(value)

    errors = [record["estimate_error"] for record in records if record.get("estimate_error")]

    return {
        "trials": len(records),
        "requested_trials": trials,
        "seed": seed,
        "wall_clock_s": round(wall_clock, 2),
        "success": {
            "held": len(successes),
            "rate": wilson_interval(len(successes), len(records)),
        },
        "attempted_and_failed": {
            "count": len(attempted_failures),
            "rate": wilson_interval(len(attempted_failures), len(records)),
        },
        "declined_before_attempt": {
            "count": len(declined_failures),
            "rate": wilson_interval(len(declined_failures), len(records)),
        },
        "failure_reasons": dict(reasons.most_common()),
        "by_part": dict(sorted(by_part.items())),
        "cycle_time_s": _distribution(cycle_times),
        "cycle_time_by_kind_s": {
            kind: round(float(np.mean(values)), 3) for kind, values in sorted(kinds.items())
        },
        "perception_error": {
            "centre_mm": _distribution(np.array([error["centre_mm"] for error in errors])),
            "yaw_deg": _distribution(np.array([error["yaw_deg"] for error in errors])),
        },
        "records": records,
    }


def _distribution(values: np.ndarray) -> Dict:
    if len(values) == 0:
        return {}
    return {
        "n": int(len(values)),
        "mean": round(float(np.mean(values)), 3),
        "median": round(float(np.median(values)), 3),
        "p05": round(float(np.percentile(values, 5)), 3),
        "p95": round(float(np.percentile(values, 95)), 3),
        "min": round(float(np.min(values)), 3),
        "max": round(float(np.max(values)), 3),
    }


def report(summary: Dict) -> str:
    lines = []
    rate = summary["success"]["rate"]
    lines.append(
        f"Trials:            {summary['trials']}  (seed {summary['seed']}, "
        f"condition {summary.get('condition', 'nominal')})"
    )
    if "hand_eye_error_mm" in summary:
        lines.append(f"Hand-eye error:    {summary['hand_eye_error_mm']:.2f} mm against the true transform")
    lines.append(
        f"Grasp success:     {summary['success']['held']}/{summary['trials']} = "
        f"{rate['point'] * 100:.1f}%  (95% CI {rate['low'] * 100:.1f}-{rate['high'] * 100:.1f}%)"
    )
    attempted = summary["attempted_and_failed"]
    declined = summary["declined_before_attempt"]
    lines.append(
        f"  attempted, lost: {attempted['count']} ({attempted['rate']['point'] * 100:.1f}%)   "
        f"declined first:  {declined['count']} ({declined['rate']['point'] * 100:.1f}%)"
    )
    if summary["cycle_time_s"]:
        cycle = summary["cycle_time_s"]
        lines.append(
            f"Cycle time:        median {cycle['median']:.2f} s  "
            f"(p05 {cycle['p05']:.2f}, p95 {cycle['p95']:.2f}) over {cycle['n']} successful cycles"
        )
        breakdown = ", ".join(f"{kind} {value:.2f}" for kind, value in summary["cycle_time_by_kind_s"].items())
        lines.append(f"  mean split:      {breakdown}")
    lines.append("")
    lines.append("Failures by stage and reason:")
    for reason, count in summary["failure_reasons"].items():
        lines.append(f"  {count:4d}  {reason}")
    lines.append("")
    lines.append("Success rate by part:")
    for part, entry in summary["by_part"].items():
        lines.append(f"  {part:<18} {entry['held']:3d}/{entry['trials']:<3d}  {entry['success_rate'] * 100:5.1f}%")
    if summary["perception_error"]["centre_mm"]:
        centre = summary["perception_error"]["centre_mm"]
        yaw = summary["perception_error"]["yaw_deg"]
        lines.append("")
        lines.append(
            f"Perception error:  centre median {centre['median']:.2f} mm (p95 {centre['p95']:.2f}), "
            f"yaw median {yaw['median']:.2f} deg (p95 {yaw['p95']:.2f})"
        )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20260101)
    parser.add_argument("--stride", type=int, default=2, help="Depth-image subsampling; 1 renders every pixel")
    parser.add_argument("--hand-eye-error-mm", type=float, default=MISCALIBRATION_TRANSLATION * 1000.0)
    parser.add_argument(
        "--condition",
        default="nominal",
        choices=list(CONDITIONS) + ["all"],
        help="Cell condition to run under; 'all' runs each in turn",
    )
    parser.add_argument("--only-trial", type=int, default=-1, help="Re-run a single trial by index")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--verbose", action="store_true")
    arguments = parser.parse_args()

    conditions = CONDITIONS if arguments.condition == "all" else (arguments.condition,)
    summaries = {}
    for condition in conditions:
        summary = run(
            arguments.trials,
            arguments.seed,
            arguments.stride,
            condition,
            arguments.only_trial,
            arguments.verbose,
            arguments.hand_eye_error_mm / 1000.0,
        )
        summaries[condition] = summary
        print(report(summary))
        print()

    if len(summaries) > 1:
        print("Success rate by condition:")
        for condition, summary in summaries.items():
            rate = summary["success"]["rate"]
            attempted = summary["attempted_and_failed"]["rate"]["point"]
            print(
                f"  {condition:<15} {rate['point'] * 100:5.1f}%  "
                f"(95% CI {rate['low'] * 100:4.1f}-{rate['high'] * 100:5.1f}%)   "
                f"attempted and lost {attempted * 100:4.1f}%"
            )

    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        payload = summaries[arguments.condition] if arguments.condition != "all" else summaries
        arguments.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"\nwrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
