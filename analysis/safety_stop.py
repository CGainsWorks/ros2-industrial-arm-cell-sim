#!/usr/bin/env python3
"""Protective stop on a light curtain break: how long, how far, and is the curtain far enough away.

    python analysis/safety_stop.py --cycles 20 --output results/safety_stop.json

SIMULATED, NOT CERTIFIED. Every figure this prints comes from the models in
``cell_safety.core`` driven by trajectories the real planner produced for real
benchmark cycles. None of it is a measurement of a robot, a curtain or a
safety controller, and none of it supports a performance level claim.

WHAT IT DOES

1. Plans ``--cycles`` nominal pick and place cycles with the same seed as the
   grasp benchmark and keeps every motion segment of every successful cycle.
2. Executes each segment at the 500 Hz controller rate with the curtain
   broken at ten instants along it (5%, 15%, ... 95% of the segment), and
   records the time from beam broken to standstill and how far the tool moved
   after the stop request.
3. Checks every stop against the budget :func:`stop_time_budget` states from
   the parameters alone, and fails if any stop exceeds it.
4. Sweeps spheres of the curtain's rated resolution, and smaller, across the
   protected field to check the detection claim.
5. Applies ISO 13855 to the worst stopping time to get the minimum distance
   from the curtain to the hazard, finds how close the arm can actually get to
   the curtain plane by optimisation over the joint space, and reports whether
   the placement in ``simulation/cell.yaml`` leaves that distance.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np
from scipy.optimize import minimize

REPO_ROOT = Path(__file__).resolve().parent.parent
for _package in ("cell_common", "cell_motion", "cell_grasping", "cell_calibration", "cell_inspection",
                 "cell_bringup", "cell_safety"):
    _path = str(REPO_ROOT / "src" / _package)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from cell_bringup.core.cycle import CycleRunner  # noqa: E402
from cell_common.cell_model import load_cell, sample_part_pose  # noqa: E402
from cell_safety.core.light_curtain import (  # noqa: E402
    Intruder,
    LightCurtain,
    LightCurtainSpec,
    interrupted_beams,
    minimum_separation_distance,
)
from cell_safety.core.protective_stop import (  # noqa: E402
    SafetyParameters,
    execute_with_protective_stop,
    stop_time_budget,
)

BREAK_FRACTIONS = [0.05 + 0.1 * index for index in range(10)]


def collect_trajectories(runner: CycleRunner, cycles: int, seed: int) -> List:
    cell = runner.cell
    trajectories = []
    attempted = 0
    for trial in range(cycles):
        rng = np.random.default_rng(seed + trial)
        result = runner.run(sample_part_pose(cell, rng), rng)
        attempted += 1
        if result.success:
            trajectories.extend(result.trajectories)
    return trajectories


def stop_sweep(runner: CycleRunner, trajectories: List, spec: LightCurtainSpec, parameters: SafetyParameters) -> Dict:
    budget = stop_time_budget(spec.response_time, parameters)
    by_segment: Dict[str, Dict] = {}
    worst = None
    stops = 0
    over_budget = 0
    completed_anyway = 0
    peak_deceleration = 0.0

    def tool(joints):
        return runner.arm.tool_pose(joints)[:3, 3]

    for trajectory in trajectories:
        for fraction in BREAK_FRACTIONS:
            break_time = fraction * trajectory.duration
            report = execute_with_protective_stop(
                trajectory.times,
                trajectory.positions,
                LightCurtain(spec),
                lambda t, b=break_time: t >= b,
                parameters,
                tool_position=tool,
            )
            stops += 1
            if report.completed or report.stopping_time is None:
                completed_anyway += 1
                continue
            if report.stopping_time > budget + 1e-9:
                over_budget += 1
            peak_deceleration = max(peak_deceleration, report.peak_joint_deceleration)
            entry = by_segment.setdefault(
                trajectory.label, {"stops": 0, "max_stopping_time": 0.0, "max_tcp_distance": 0.0}
            )
            entry["stops"] += 1
            entry["max_stopping_time"] = max(entry["max_stopping_time"], report.stopping_time)
            entry["max_tcp_distance"] = max(entry["max_tcp_distance"], report.stopping_distance_tcp)
            if worst is None or report.stopping_time > worst["stopping_time"]:
                worst = dict(report.as_dict(), segment=trajectory.label, break_fraction=round(fraction, 2))

    for entry in by_segment.values():
        entry["max_stopping_time"] = round(entry["max_stopping_time"], 4)
        entry["max_tcp_distance"] = round(entry["max_tcp_distance"], 4)

    return {
        "budget_s": round(budget, 4),
        "stops": stops,
        "stops_over_budget": over_budget,
        "trajectory_finished_before_the_stop": completed_anyway,
        "max_stopping_time_s": worst["stopping_time"] if worst else None,
        "max_tcp_stopping_distance_m": round(max((e["max_tcp_distance"] for e in by_segment.values()), default=0.0), 4),
        "peak_commanded_joint_deceleration": round(peak_deceleration, 3),
        "by_segment": dict(sorted(by_segment.items())),
        "worst_case": worst,
    }


def detection_sweep(spec: LightCurtainSpec, diameters: List[float], step: float = 0.0005) -> Dict:
    """Fraction of positions in the field at which each diameter is detected."""
    heights = np.arange(spec.bottom_z + 0.02, spec.top_z - 0.02, step)
    alongs = np.linspace(0.05, spec.width - 0.05, 5)
    emitter = np.asarray(spec.emitter_xy)
    axis = (np.asarray(spec.receiver_xy) - emitter) / spec.width
    results = {}
    for diameter in diameters:
        detected = 0
        total = 0
        for along in alongs:
            xy = emitter + axis * along
            for height in heights:
                total += 1
                if interrupted_beams(spec, [Intruder((xy[0], xy[1], float(height)), diameter)]):
                    detected += 1
        results[f"{diameter * 1000:.0f}mm"] = round(detected / total, 4)
    return {"positions": int(len(heights) * len(alongs)), "detected_fraction": results}


def closest_approach(runner: CycleRunner, spec: LightCurtainSpec, starts: int = 60, seed: int = 7) -> Dict:
    """How close any part of the arm or gripper can get to the curtain plane.

    Maximises, over the whole joint space, the furthest extent of any capsule
    of the arm and gripper towards the curtain, from ``starts`` random seeds.
    Collisions with the cell are ignored on purpose: a fault can drive the arm
    anywhere it can physically reach, and that is the envelope ISO 13855 is
    applied to.
    """
    emitter = np.asarray(spec.emitter_xy)
    axis = (np.asarray(spec.receiver_xy) - emitter) / spec.width
    normal = np.array([-axis[1], axis[0]])
    plane_offset = float(emitter @ normal)
    # The normal pointing from the arm towards the curtain.
    if plane_offset < 0:
        normal, plane_offset = -normal, -plane_offset
    opening = runner.cell.gripper.max_opening

    def extent(joints) -> float:
        best = -np.inf
        for capsule in runner.arm.capsules(joints, opening):
            for point in (capsule.start, capsule.end):
                best = max(best, float(point[:2] @ normal) + capsule.radius)
        return best

    rng = np.random.default_rng(seed)
    best_value = -np.inf
    best_joints = None
    for _ in range(starts):
        start = rng.uniform(-np.pi, np.pi, size=6)
        result = minimize(lambda q: -extent(q), start, method="Nelder-Mead",
                          options={"maxiter": 1500, "xatol": 1e-4, "fatol": 1e-6})
        if -result.fun > best_value:
            best_value = -result.fun
            best_joints = result.x
    return {
        "curtain_plane_distance_from_base_m": round(plane_offset, 4),
        "max_arm_extent_towards_curtain_m": round(best_value, 4),
        "clearance_m": round(plane_offset - best_value, 4),
        "optimisation_starts": starts,
        "joints": [round(float(value), 4) for value in best_joints],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20260101)
    parser.add_argument("--reach-starts", type=int, default=60)
    parser.add_argument("--output", type=Path, default=None)
    arguments = parser.parse_args()

    cell = load_cell()
    spec = LightCurtainSpec.from_dict(cell.raw["light_curtain"])
    parameters = SafetyParameters()
    runner = CycleRunner(cell)

    trajectories = collect_trajectories(runner, arguments.cycles, arguments.seed)
    stops = stop_sweep(runner, trajectories, spec, parameters)
    detection = detection_sweep(spec, [spec.resolution, 0.010, 0.008])
    approach = closest_approach(runner, spec, starts=arguments.reach_starts)

    # The placement is judged on the stated budget, not on the worst stop
    # this run happened to find: the budget is the figure the cell is
    # designed to, and it does not move when the sample of cycles does.
    stopping_time = stops["max_stopping_time_s"]
    required = minimum_separation_distance(stops["budget_s"], spec.resolution)
    required_observed = minimum_separation_distance(stopping_time, spec.resolution)
    placement_ok = bool(approach["clearance_m"] >= required)

    report = {
        "simulated": True,
        "certified": False,
        "cycles": arguments.cycles,
        "seed": arguments.seed,
        "segments_executed": len(trajectories),
        "curtain": {
            "resolution_mm": round(spec.resolution * 1000, 1),
            "response_time_s": spec.response_time,
            "restart_delay_s": spec.restart_delay,
            "beams": int(len(spec.beam_heights())),
        },
        "controller": {
            "period_s": parameters.controller_period,
            "stop_category": int(parameters.category),
            "stop_deceleration_rad_s2": list(parameters.stop_deceleration),
            "discrepancy_time_s": parameters.discrepancy_time,
        },
        "stops": stops,
        "detection": detection,
        "iso13855": {
            "stopping_time_budget_s": stops["budget_s"],
            "minimum_distance_m": round(required, 4),
            "worst_observed_stopping_time_s": stopping_time,
            "minimum_distance_at_worst_observed_m": round(required_observed, 4),
            **approach,
            "placement_ok": placement_ok,
        },
    }

    print(f"curtain: {report['curtain']['beams']} beams, {report['curtain']['resolution_mm']} mm resolution")
    print(f"segments: {len(trajectories)} from {arguments.cycles} cycles, {stops['stops']} stops")
    print(f"budget:   {stops['budget_s'] * 1000:.1f} ms beam broken to standstill")
    print(f"worst:    {stopping_time * 1000:.1f} ms ({stops['worst_case']['segment']}), "
          f"{stops['stops_over_budget']} over budget")
    print(f"tool:     at most {stops['max_tcp_stopping_distance_m'] * 1000:.1f} mm after the stop request")
    print(f"detected: {detection['detected_fraction']}")
    print(f"ISO 13855: S = {required * 1000:.0f} mm at the budget; "
          f"arm reaches to {approach['max_arm_extent_towards_curtain_m']:.3f} m, "
          f"curtain at {approach['curtain_plane_distance_from_base_m']:.3f} m, "
          f"clearance {approach['clearance_m'] * 1000:.0f} mm -> {'OK' if placement_ok else 'TOO CLOSE'}")

    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8", newline="\n")

    failed = stops["stops_over_budget"] > 0 or not placement_ok
    failed = failed or detection["detected_fraction"][f"{spec.resolution * 1000:.0f}mm"] < 1.0
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
