#!/usr/bin/env python3
"""Collision-scene validation: what of the pick region can actually be grasped.

Sweeps a grid over the pick region and, at each point, counts how many top-down
grasp orientations have a collision-free IK solution::

    python analysis/reach_map.py --output results/reach_map.json

This is the check that would have caught two of the mistakes recorded in this
repository's history, before either of them reached a benchmark:

* a conveyor 120 mm taller in the URDF than in the collision checker, which
  showed up here as a region that was 100% reachable at every orientation
  including ones that go through the belt;
* fingertip capsules whose spherical end caps extended 7 mm past the tip, which
  showed up as the whole region being unreachable at grasp heights under 10 mm.

It also produces the honest answer to "how big is the working area", which is
not the same as the size of the belt. The far corners of the region are
marginal for a top-down grasp -- that is deliberate, and it is why the grasp
benchmark reports unreachable poses as their own failure class.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
for _package in ("cell_common", "cell_motion"):
    _path = str(REPO_ROOT / "src" / _package)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from cell_common.cell_model import load_cell  # noqa: E402
from cell_motion.core.arm import ArmModel, approach_pose  # noqa: E402


def run(resolution: int, orientations: int, heights_mm, expand: float = 0.0) -> Dict:
    cell = load_cell()
    arm = ArmModel.from_cell(cell)
    region = cell.part_region

    # ``expand`` grows the sampled area beyond the taught region. Sampling only
    # the taught region tells you it is reachable, which it had better be, and
    # tells you nothing about whether the checker works: a scene with the belt
    # accidentally left out also reads 100%. Sweeping past the guide rail and
    # past the arm's envelope produces cells that must fail, and names what
    # stopped them.
    xs = np.linspace(region["x"][0] - expand, region["x"][1] + expand, resolution)
    ys = np.linspace(region["y"][0] - expand, region["y"][1] + expand, resolution)
    yaws = np.linspace(-np.pi / 2, np.pi / 2, orientations, endpoint=False)

    layers = {}
    for height_mm in heights_mm:
        height = cell.conveyor_top + height_mm / 1000.0
        grid = np.zeros((len(ys), len(xs)))
        blockers: Dict[str, int] = {}
        for row, y in enumerate(ys):
            for column, x in enumerate(xs):
                reachable = 0
                for yaw in yaws:
                    solution = arm.ik_nearest(approach_pose([x, y, height], yaw), cell.home_joints)
                    if solution is None:
                        blockers["unreachable"] = blockers.get("unreachable", 0) + 1
                        continue
                    clearance, culprit = arm.clearance(solution.joints, 0.05)
                    if clearance < 0.002:
                        blockers[culprit or "collision"] = blockers.get(culprit or "collision", 0) + 1
                        continue
                    reachable += 1
                grid[row, column] = reachable / len(yaws)

        layers[f"{height_mm:g}mm"] = {
            "grasp_height_mm": height_mm,
            "fully_reachable_fraction": round(float(np.mean(grid >= 1.0 - 1e-9)), 4),
            "any_orientation_fraction": round(float(np.mean(grid > 0.0)), 4),
            "mean_orientation_fraction": round(float(np.mean(grid)), 4),
            "worst_cell": round(float(np.min(grid)), 4),
            "blocked_by": dict(sorted(blockers.items(), key=lambda item: -item[1])),
            "grid": [[round(float(value), 3) for value in row] for row in grid],
        }

    return {
        "region": {key: list(value) for key, value in region.items()},
        "expand_m": expand,
        "resolution": resolution,
        "orientations": orientations,
        "x": [round(float(value), 4) for value in xs],
        "y": [round(float(value), 4) for value in ys],
        "layers": layers,
    }


def report(summary: Dict) -> str:
    lines = [
        f"Pick region {summary['region']['x']} x {summary['region']['y']} m"
        + (f", swept {summary['expand_m'] * 1000:.0f} mm beyond it" if summary["expand_m"] else "")
        + f", {summary['resolution']}x{summary['resolution']} grid, "
        f"{summary['orientations']} orientations each",
        "",
        f"{'grasp height':>13} {'all yaws':>10} {'some yaw':>10} {'mean':>7}   blocked by",
    ]
    for name, layer in summary["layers"].items():
        blockers = ", ".join(f"{key} {value}" for key, value in list(layer["blocked_by"].items())[:3]) or "-"
        lines.append(
            f"{name:>13} {layer['fully_reachable_fraction'] * 100:>9.1f}% "
            f"{layer['any_orientation_fraction'] * 100:>9.1f}% "
            f"{layer['mean_orientation_fraction'] * 100:>6.1f}%   {blockers}"
        )
    lines.append("")
    if summary["expand_m"]:
        lines.append(
            "The taught region should read 100%; the swept margin should not. A run in which\n"
            "everything is reachable at every height means the obstacles are not in the scene\n"
            "the checker is using, which is the failure this sweep exists to catch."
        )
    else:
        lines.append("Run with --expand-m 0.10 to sweep past the region and exercise the obstacles.")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--resolution", type=int, default=9)
    parser.add_argument("--orientations", type=int, default=8)
    parser.add_argument("--heights-mm", type=float, nargs="+", default=[4.0, 12.0, 28.0])
    parser.add_argument("--expand-m", type=float, default=0.10, help="Sweep this far beyond the taught region")
    parser.add_argument("--output", type=Path, default=None)
    arguments = parser.parse_args()

    summary = run(arguments.resolution, arguments.orientations, arguments.heights_mm, arguments.expand_m)
    print(report(summary))
    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"\nwrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
