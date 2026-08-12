#!/usr/bin/env python3
"""PID joint tuning: relay auto-tune, step responses, and stability margins.

Produces the controller tuning record::

    python analysis/pid_tuning.py --output results/pid_tuning.json --plots docs/plots

Three gain sets are compared on the shoulder-lift axis: classic
Ziegler-Nichols from a relay experiment, the same detuned by half, and the
hand-tuned gains the cell runs. Each is reported as a step response (rise time,
overshoot, settling, steady-state error), as gain and phase margin from the
linearised loop, and across an inertia sweep covering the arm folded and
extended.

WHAT THE RESULT SAYS, IN ADVANCE

Ziegler-Nichols does badly here, and not by a little. The rules were fitted to
first-order-plus-dead-time plants; a robot joint with light viscous friction is
close to a double integrator, and the ZN gains land it at around ten degrees of
phase margin. Halving them, which is the usual response to an aggressive
auto-tune, makes it worse rather than better -- the crossover drops to where the
PID's zeros contribute less phase lead, and the margin goes negative. The
simulated response then limit-cycles at full torque, which is exactly what the
margin predicts.

That is worth measuring rather than asserting, and it is worth measuring *two
ways*: the frequency-domain margin and the time-domain step have to agree, and
if they do not, one of them is wrong.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
_path = str(REPO_ROOT / "src" / "cell_motion")
if _path not in sys.path:
    sys.path.insert(0, _path)

from cell_motion.core.joint_control import (  # noqa: E402
    JointPlant,
    PIDGains,
    relay_autotune,
    simulate_step,
    stability_margins,
    ziegler_nichols,
)

# The gains the cell runs. Reached by hand: raise kd until the step is critically
# damped, then raise kp to the point where the torque saturates on a full-speed
# move, then add the least integral action that removes the Coulomb-friction
# offset within a second.
CELL_GAINS = PIDGains(kp=900.0, ki=1200.0, kd=45.0, name="hand-tuned")

# Inertia seen by the shoulder-lift axis, folded and extended.
INERTIA_SWEEP = (0.8, 1.6, 3.2)


def run(step_target: float, disturbance: float) -> Dict:
    plant = JointPlant()
    ultimate_gain, period = relay_autotune(plant)

    gain_sets = [
        ziegler_nichols(ultimate_gain, period),
        ziegler_nichols(ultimate_gain, period, detune=0.5),
        CELL_GAINS,
    ]

    responses = []
    for gains in gain_sets:
        for feedforward in (0.9, 0.0):
            for inertia in INERTIA_SWEEP:
                variant = JointPlant(inertia=inertia)
                response = simulate_step(
                    variant, gains, target=step_target, disturbance=disturbance,
                    gravity_feedforward=feedforward,
                    label=f"{gains.name} | J={inertia} | ff={feedforward}",
                )
                record = response.as_dict()
                record.update(
                    {
                        "gains": gains.as_dict(),
                        "inertia": inertia,
                        "gravity_feedforward": feedforward,
                        "margins": stability_margins(variant, gains),
                    }
                )
                responses.append(record)

    return {
        "relay": {
            "ultimate_gain": round(ultimate_gain, 2),
            "ultimate_period_s": round(period, 4),
            "ultimate_frequency_rad_s": round(2 * np.pi / period, 2),
        },
        "plant": {
            "inertia": plant.inertia,
            "viscous": plant.viscous,
            "coulomb": plant.coulomb,
            "gravity_amplitude": plant.gravity_amplitude,
            "torque_limit": plant.torque_limit,
            "current_lag_s": plant.current_lag,
        },
        "step_target_rad": step_target,
        "disturbance_nm": disturbance,
        "gain_sets": [gains.as_dict() for gains in gain_sets],
        "responses": responses,
    }


def write_plots(summary: Dict, directory: Path, step_target: float, disturbance: float) -> List[Path]:
    """Step-response figures. Skipped, with a message, if matplotlib is absent.

    Not a hard dependency: the numbers in the JSON are the result, and the plots
    are how a person reads them. A repository whose benchmark cannot run without
    a plotting library has the dependency the wrong way round.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed; skipping plots (the JSON output is unaffected)")
        return []

    directory.mkdir(parents=True, exist_ok=True)
    plant = JointPlant()
    ultimate_gain, period = relay_autotune(plant)
    gain_sets = [
        ziegler_nichols(ultimate_gain, period),
        ziegler_nichols(ultimate_gain, period, detune=0.5),
        CELL_GAINS,
    ]
    written = []

    # Figure 1: the three gain sets at nominal inertia.
    figure, axes = plt.subplots(2, 1, figsize=(8.0, 6.0), sharex=True, height_ratios=[2, 1])
    for gains in gain_sets:
        response = simulate_step(plant, gains, target=step_target, disturbance=disturbance)
        margins = stability_margins(plant, gains)
        label = f"{gains.name} (PM {margins['phase_margin_deg']:.0f}°)"
        axes[0].plot(response.time, response.position, label=label, linewidth=1.4)
        axes[1].plot(response.time, response.torque, linewidth=1.0)
    axes[0].axhline(step_target, color="0.5", linestyle="--", linewidth=0.8)
    axes[0].axhline(step_target * 1.02, color="0.8", linewidth=0.6)
    axes[0].axhline(step_target * 0.98, color="0.8", linewidth=0.6)
    axes[0].set_ylabel("position (rad)")
    axes[0].set_title(
        f"Shoulder-lift step response, {step_target} rad, "
        f"{disturbance:g} N m load applied at 1.2 s"
    )
    axes[0].legend(loc="lower right", fontsize=9)
    axes[0].grid(alpha=0.3)
    axes[1].axhline(plant.torque_limit, color="r", linestyle=":", linewidth=0.8)
    axes[1].axhline(-plant.torque_limit, color="r", linestyle=":", linewidth=0.8)
    axes[1].set_ylabel("torque (N m)")
    axes[1].set_xlabel("time (s)")
    axes[1].grid(alpha=0.3)
    figure.tight_layout()
    path = directory / "step_response.png"
    figure.savefig(path, dpi=110)
    plt.close(figure)
    written.append(path)

    # Figure 2: the cell's gains across the inertia sweep.
    figure, axis = plt.subplots(figsize=(8.0, 4.0))
    for inertia in INERTIA_SWEEP:
        variant = JointPlant(inertia=inertia)
        response = simulate_step(variant, CELL_GAINS, target=step_target, disturbance=disturbance)
        axis.plot(
            response.time, response.position, linewidth=1.4,
            label=f"J = {inertia} kg m² (overshoot {response.overshoot_percent:.1f}%)",
        )
    axis.axhline(step_target, color="0.5", linestyle="--", linewidth=0.8)
    axis.set_xlabel("time (s)")
    axis.set_ylabel("position (rad)")
    axis.set_title("Hand-tuned gains against configuration-dependent inertia")
    axis.legend(loc="lower right", fontsize=9)
    axis.grid(alpha=0.3)
    figure.tight_layout()
    path = directory / "inertia_sweep.png"
    figure.savefig(path, dpi=110)
    plt.close(figure)
    written.append(path)

    return written


def report(summary: Dict) -> str:
    lines = [
        f"Relay auto-tune: Ku = {summary['relay']['ultimate_gain']:.1f} N m/rad, "
        f"Tu = {summary['relay']['ultimate_period_s']:.3f} s "
        f"({summary['relay']['ultimate_frequency_rad_s']:.1f} rad/s)",
        "",
        f"{'gain set':<22} {'ff':>4} {'J':>5} {'rise ms':>8} {'over %':>7} "
        f"{'settle ms':>10} {'sse mrad':>9} {'PM deg':>8} {'stable':>7}",
    ]
    for record in summary["responses"]:
        lines.append(
            f"{record['gains']['name']:<22} {record['gravity_feedforward']:>4.1f} "
            f"{record['inertia']:>5.1f} {record['rise_time_ms']:>8.1f} "
            f"{record['overshoot_percent']:>7.1f} {record['settling_time_ms']:>10.0f} "
            f"{record['steady_state_error_mrad']:>9.2f} "
            f"{record['margins']['phase_margin_deg']:>8.1f} "
            f"{str(record['margins']['stable']):>7}"
        )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--target", type=float, default=0.35, help="Step size in radians")
    parser.add_argument("--disturbance", type=float, default=12.0, help="Load step in N m at 1.2 s")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--plots", type=Path, default=None)
    arguments = parser.parse_args()

    summary = run(arguments.target, arguments.disturbance)
    print(report(summary))

    if arguments.plots is not None:
        for path in write_plots(summary, arguments.plots, arguments.target, arguments.disturbance):
            print(f"wrote {path}")

    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"wrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
