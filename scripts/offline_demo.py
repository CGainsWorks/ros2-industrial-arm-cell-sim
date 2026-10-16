#!/usr/bin/env python3
"""One pick-and-place cycle, printed stage by stage. No ROS, no simulator.

    python scripts/offline_demo.py --seed 3

This is what ``scripts/run_demo.sh`` falls back to on a machine without ROS,
and it is the fastest way to see what the cell does: a randomised part on the
belt, a depth capture, the ranked grasp candidates, the trajectory durations,
the ground-truth verdict, and the fieldbus registers a PLC would read.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
for _package in ("cell_common", "cell_motion", "cell_grasping", "cell_calibration", "cell_inspection",
                 "cell_fieldbus", "cell_bringup"):
    _path = str(REPO_ROOT / "src" / _package)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from cell_bringup.core.cycle import CycleRunner  # noqa: E402
from cell_common.cell_model import load_cell, sample_part_pose  # noqa: E402
from cell_fieldbus.core.registers import FieldbusState, fault_for  # noqa: E402
from cell_inspection.core.contact import ContactSurface, inspect_part, inspection_points  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, default=20260101)
    parser.add_argument("--cycles", type=int, default=1)
    arguments, _ = parser.parse_known_args()

    cell = load_cell()
    runner = CycleRunner(cell)
    fieldbus = FieldbusState()

    for index in range(arguments.cycles):
        rng = np.random.default_rng(arguments.seed + index)
        part = sample_part_pose(cell, rng)

        print("=" * 72)
        print(f"cycle {index + 1}: {part['part']}")
        print(
            f"  presented at ({part['xyz'][0]:.3f}, {part['xyz'][1]:.3f}) m, "
            f"yaw {np.degrees(part['yaw']):+.0f} deg, "
            f"resting face {part['resting_face']}, "
            f"footprint {part['size'][0] * 1000:.0f} x {part['size'][1] * 1000:.0f} x "
            f"{part['size'][2] * 1000:.0f} mm"
        )

        result = runner.run(part, rng)

        if result.estimate_error:
            error = result.estimate_error
            print(
                f"  perception:  centre off by {error['centre_mm']:.2f} mm, "
                f"yaw off by {error['yaw_deg']:.2f} deg, from {error['points']} points"
            )
        if result.grasp:
            grasp = result.grasp
            terms = ", ".join(f"{name} {value:.2f}" for name, value in sorted(grasp.terms.items()))
            print(
                f"  grasp:       {result.candidate_count} candidates, best scores "
                f"{grasp.score:.3f} ({terms})"
            )
            print(
                f"               closing at {np.degrees(grasp.yaw):+.0f} deg across "
                f"{grasp.width * 1000:.1f} mm, jaws opened to {grasp.opening * 1000:.1f} mm"
            )
        if result.rejections:
            print(f"  rejected:    {result.rejections}")

        print("  timeline:")
        for segment in result.segments:
            print(f"    {segment.name:<24} {segment.duration:>6.2f} s   {segment.kind}")

        if result.outcome:
            outcome = result.outcome
            verdict = "HELD" if outcome.held else f"LOST ({outcome.failure})"
            print(
                f"  verdict:     {verdict} -- contact {np.degrees(outcome.contact_angle):.1f} deg "
                f"into a {np.degrees(outcome.friction_angle):.0f} deg friction cone, "
                f"engagement {outcome.engagement * 1000:.1f} mm"
            )

        if result.success:
            height = cell.fixture_top + float(part["size"][2])
            points = inspection_points(cell.place_xyz[:2], part["size"], cell.inspection_points)
            report = inspect_part(
                ContactSurface(height=height, origin=np.asarray(cell.place_xyz[:2])),
                points,
                height + 0.005,
                rng,
            )
            print(
                f"  inspection:  {len(report.probes)} probes, tilt {report.tilt_deg:.3f} deg, "
                f"mean error {report.mean_abs_error_um:.1f} um, fault {report.fault}"
            )

        fault = fault_for(result.failure)
        fieldbus.record_cycle(
            result.success, result.cycle_time, result.grasp.score if result.grasp else 0.0, index + 1
        )
        if not result.success:
            fieldbus.raise_fault(fault, result.stage)
        status = fieldbus.status
        print(
            f"  fieldbus:    state {int(status.state)} ({status.state.name}), "
            f"fault {int(status.fault)} ({status.fault.name}), "
            f"last cycle {status.last_cycle_ms} ms, "
            f"{status.success_count}/{status.cycle_count} held"
        )
        print(f"  TOTAL:       {result.cycle_time:.2f} s   {result.stage}")

    print("=" * 72)
    print("For the distribution rather than one draw:  make grasp")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
