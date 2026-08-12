#!/usr/bin/env python3
"""Force-feedback contact inspection: how accurately it measures, and what it catches.

Two questions, two experiments::

    python analysis/inspection_accuracy.py --output results/inspection.json

**How accurate is a probe?** Repeated probes of a flat, correctly-seated part,
reported as the distribution of measured-minus-true height. The error budget in
:mod:`cell_inspection.core.contact` claims three contributors -- the trigger
threshold against the contact stiffness, force-sensor noise, and the approach
speed against the control period -- and this sweeps each of them so the claim
is checkable rather than asserted.

**Does it catch a part that is not seated?** A detection sweep over lift
heights: a part with one corner raised by 0.05 to 1.0 mm, and the fraction of
runs where the routine raises ``FAULT_NOT_SEATED``. The useful output is the
lift at which detection becomes reliable, which is the specification this
routine can actually be held to.

The false-positive rate on correctly-seated parts is reported alongside it,
because a seating check that never misses and stops the line twice a shift is
worse than no seating check.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
for _package in ("cell_common", "cell_inspection"):
    _path = str(REPO_ROOT / "src" / _package)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from cell_common.cell_model import load_cell  # noqa: E402
from cell_inspection.core.contact import (  # noqa: E402
    FAULT_NONE,
    FAULT_NOT_SEATED,
    ContactSurface,
    inspect_part,
    inspection_points,
    probe_point,
)

STANDOFF = 0.005  # Probe starts this far above the expected surface.


def accuracy_sweep(repeats: int, seed: int) -> Dict:
    """Probe error against force noise, stiffness and approach speed."""
    base_height = 0.792
    results: Dict[str, List[Dict]] = {"force_noise": [], "stiffness": [], "approach_speed": []}

    for noise in (0.1, 0.35, 1.0, 2.0):
        errors = _probe_errors(repeats, seed, base_height, force_noise=noise)
        results["force_noise"].append({"force_noise_n": noise, **_distribution(errors)})

    for stiffness in (5000.0, 20000.0, 80000.0):
        errors = _probe_errors(repeats, seed, base_height, stiffness=stiffness)
        results["stiffness"].append({"stiffness_n_per_m": stiffness, **_distribution(errors)})

    for speed in (0.002, 0.005, 0.010, 0.020):
        errors = _probe_errors(repeats, seed, base_height, approach_speed=speed)
        results["approach_speed"].append({"approach_speed_mm_s": speed * 1000.0, **_distribution(errors)})

    return results


def _probe_errors(
    repeats: int,
    seed: int,
    height: float,
    force_noise: float = 0.35,
    stiffness: float = 20000.0,
    approach_speed: float = 0.005,
) -> np.ndarray:
    errors = []
    for index in range(repeats):
        rng = np.random.default_rng(seed + index)
        surface = ContactSurface(height=height, stiffness=stiffness)
        result = probe_point(
            surface, [0.0, -0.60], height + STANDOFF, rng,
            force_noise=force_noise, approach_speed=approach_speed,
        )
        if result.fault == FAULT_NONE:
            errors.append(result.error * 1e6)
    return np.asarray(errors)


def seating_sweep(repeats: int, seed: int, part_size=(0.062, 0.038)) -> Dict:
    """Detection rate against corner lift, plus the false-positive rate."""
    cell = load_cell()
    base_height = cell.fixture_top + 0.012
    points = inspection_points([0.0, -0.60], part_size, cell.inspection_points)
    span = float(np.hypot(*(np.asarray(part_size) * 0.6)))

    rows = []
    for lift_mm in (0.0, 0.05, 0.1, 0.2, 0.3, 0.5, 1.0):
        detections = 0
        tilts = []
        for index in range(repeats):
            rng = np.random.default_rng(seed + 7919 * index)
            gradient = (lift_mm / 1000.0) / max(span, 1e-6)
            surface = ContactSurface(
                height=base_height,
                tilt=np.array([gradient, 0.0]),
                origin=np.array([0.0, -0.60]),
            )
            report_ = inspect_part(surface, points, base_height + STANDOFF, rng)
            detections += int(report_.fault == FAULT_NOT_SEATED)
            if np.isfinite(report_.tilt_deg):
                tilts.append(report_.tilt_deg)
        rows.append(
            {
                "corner_lift_mm": lift_mm,
                "detection_rate": round(detections / repeats, 3),
                "median_measured_tilt_deg": round(float(np.median(tilts)), 4) if tilts else None,
            }
        )
    return {"rows": rows, "false_positive_rate": rows[0]["detection_rate"]}


def _distribution(values: np.ndarray) -> Dict:
    if len(values) == 0:
        return {"n": 0}
    return {
        "n": int(len(values)),
        "mean_um": round(float(np.mean(values)), 2),
        "std_um": round(float(np.std(values)), 2),
        "abs_median_um": round(float(np.median(np.abs(values))), 2),
        "abs_p95_um": round(float(np.percentile(np.abs(values), 95)), 2),
    }


def report(summary: Dict) -> str:
    lines = ["Probe accuracy (error in microns, measured minus true)", ""]
    for name, rows in summary["accuracy"].items():
        lines.append(f"  varying {name}:")
        for row in rows:
            key = next(iter(row))
            lines.append(
                f"    {key} = {row[key]:>8.4g}   mean {row['mean_um']:>7.2f}   "
                f"std {row['std_um']:>6.2f}   |err| p95 {row['abs_p95_um']:>7.2f}"
            )
        lines.append("")
    lines.append("Seating detection (part with one corner lifted)")
    lines.append(f"  {'lift mm':>8} {'detected':>9} {'measured tilt deg':>19}")
    for row in summary["seating"]["rows"]:
        tilt = row["median_measured_tilt_deg"]
        lines.append(
            f"  {row['corner_lift_mm']:>8.2f} {row['detection_rate'] * 100:>8.1f}% "
            f"{tilt if tilt is not None else float('nan'):>19.4f}"
        )
    lines.append(f"  false positives on a flat part: {summary['seating']['false_positive_rate'] * 100:.1f}%")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repeats", type=int, default=40)
    parser.add_argument("--seed", type=int, default=20260101)
    parser.add_argument("--output", type=Path, default=None)
    arguments = parser.parse_args()

    summary = {
        "seed": arguments.seed,
        "repeats": arguments.repeats,
        "accuracy": accuracy_sweep(arguments.repeats, arguments.seed),
        "seating": seating_sweep(max(arguments.repeats // 2, 10), arguments.seed),
    }
    print(report(summary))
    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"\nwrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
