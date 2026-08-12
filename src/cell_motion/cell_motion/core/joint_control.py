"""A joint model, a PID controller, and a harness for tuning one against the other.

WHAT IS BEING TUNED

The plant is one axis of the arm, as the position loop sees it:

    J q_ddot + b q_dot + tau_coulomb * sign(q_dot) + tau_gravity(q) = tau_motor

with the motor torque passed through a first-order current-loop lag and clipped
at the drive's limit. Those four terms are there because each one breaks a
different tuning rule:

* **Inertia** varies with configuration -- a UR5e's shoulder sees several times
  more of it with the arm out than tucked -- so a gain set at one pose is the
  wrong gain at another. The harness sweeps it.
* **Viscous friction** is the benign term. It is what makes the open loop
  stable and what Ziegler-Nichols implicitly assumes dominates.
* **Coulomb friction** is not smooth, so it produces a steady-state error that
  proportional gain cannot remove and integral gain removes only by winding up
  first. It is the reason the "steady-state error" column in the tuning table
  is not zero for the PD row.
* **Gravity**, being configuration-dependent, is a *load disturbance that moves
  with the setpoint*. A step in position is therefore also a step in load, and
  a controller tuned against a gravity-free plant undershoots every upward move.

THE TUNING METHODS COMPARED

1. **Ziegler-Nichols** from a relay experiment (Astrom-Hagglund), which is the
   auto-tune every industrial drive ships. It finds the ultimate gain and
   period without needing a model and without driving the axis unstable.
2. **The same, detuned**, which is what anyone actually commissions: classic ZN
   is deliberately aggressive and gives roughly 25% overshoot, which on a robot
   axis is a collision margin spent for nothing.
3. **Hand-tuned**, the settings this cell runs.

``analysis/pid_tuning.py`` runs all three across the inertia sweep and writes
the step responses to ``docs/plots/``. The point of the comparison is not that
one wins -- it is that the ZN row's overshoot is the same on paper as it is in
the plot, so the harness is measuring what it claims to.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

CONTROL_RATE = 500.0
CONTROL_PERIOD = 1.0 / CONTROL_RATE
SIMULATION_SUBSTEPS = 8  # Integrator steps per control period.


@dataclass
class JointPlant:
    """One robot axis, reflected to the joint side of the gearbox."""

    inertia: float = 1.6  # kg m^2
    viscous: float = 3.0  # N m s / rad
    coulomb: float = 4.0  # N m
    gravity_amplitude: float = 55.0  # N m, peak, at horizontal.
    torque_limit: float = 150.0  # N m
    current_lag: float = 0.002  # s, first-order.

    def gravity(self, position: float) -> float:
        return self.gravity_amplitude * float(np.cos(position))

    def derivative(self, position: float, velocity: float, torque: float) -> float:
        friction = self.viscous * velocity + self.coulomb * np.tanh(velocity / 0.002)
        return (torque - friction - self.gravity(position)) / self.inertia


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    derivative_cutoff: float = 40.0  # Hz.
    name: str = ""

    def as_dict(self) -> Dict[str, float]:
        return {"name": self.name, "kp": round(self.kp, 3), "ki": round(self.ki, 3), "kd": round(self.kd, 4)}


@dataclass
class StepResponse:
    """A recorded step, and the numbers extracted from it."""

    time: np.ndarray
    position: np.ndarray
    setpoint: np.ndarray
    torque: np.ndarray
    rise_time: float
    overshoot_percent: float
    settling_time: float
    steady_state_error: float
    integral_absolute_error: float
    saturated_fraction: float
    label: str = ""

    def as_dict(self) -> Dict:
        return {
            "label": self.label,
            "rise_time_ms": round(self.rise_time * 1000.0, 1),
            "overshoot_percent": round(self.overshoot_percent, 2),
            "settling_time_ms": round(self.settling_time * 1000.0, 1),
            "steady_state_error_mrad": round(self.steady_state_error * 1000.0, 3),
            "iae_rad_s": round(self.integral_absolute_error, 4),
            "saturated_fraction": round(self.saturated_fraction, 3),
        }


def simulate_step(
    plant: JointPlant,
    gains: PIDGains,
    start: float = 0.0,
    target: float = 0.35,
    duration: float = 2.0,
    disturbance: float = 0.0,
    disturbance_time: float = 1.2,
    gravity_feedforward: float = 0.9,
    label: str = "",
) -> StepResponse:
    """Closed-loop step response, with optional load disturbance part-way.

    The controller runs at ``CONTROL_RATE``; the plant is integrated with RK4 at
    ``SIMULATION_SUBSTEPS`` times that. Integrating the plant at the control
    rate as well would hide exactly the thing a tuning harness is for: with
    derivative gain, an under-resolved plant looks better damped than it is.

    ``gravity_feedforward`` is the fraction of the modelled gravity torque added
    ahead of the PID, and the default of 0.9 -- not 1.0 -- is the point of it: a
    real feedforward model is built from estimated link masses and is a few per
    cent wrong, so the loop still has a load to reject. Setting it to 0.0
    removes it entirely, which is worth doing once. Without feedforward the
    integrator alone has to supply the 52 N m that holds this axis up, and the
    detuned Ziegler-Nichols gains wind up against the drive limit on the way to
    the setpoint and never settle -- a full-amplitude limit cycle at *half* the
    gain that was stable. It is the clearest demonstration in this repository
    that a tuning rule derived for a linear plant says nothing about one with a
    saturating actuator, and ``analysis/pid_tuning.py`` reports both columns
    for exactly that reason.
    """
    steps = int(round(duration / CONTROL_PERIOD))
    times = np.zeros(steps)
    positions = np.zeros(steps)
    setpoints = np.zeros(steps)
    torques = np.zeros(steps)

    position, velocity = float(start), 0.0
    applied_torque = plant.gravity(start)
    integral = 0.0
    previous_error = target - start
    filtered_derivative = 0.0
    alpha = float(np.clip(2.0 * np.pi * gains.derivative_cutoff * CONTROL_PERIOD, 0.0, 1.0))
    saturated_steps = 0

    for index in range(steps):
        time = index * CONTROL_PERIOD
        error = target - position

        integral += error * CONTROL_PERIOD
        raw_derivative = (error - previous_error) / CONTROL_PERIOD
        filtered_derivative += alpha * (raw_derivative - filtered_derivative)
        previous_error = error

        feedforward = gravity_feedforward * plant.gravity(position)
        command = gains.kp * error + gains.ki * integral + gains.kd * filtered_derivative + feedforward
        limited = float(np.clip(command, -plant.torque_limit, plant.torque_limit))
        if abs(command - limited) > 1e-9:
            saturated_steps += 1
            # Clamping anti-windup: stop integrating while the drive is against
            # its limit. Without it a 0.35 rad step on this axis winds up about
            # 40 N m of integral that then has to be unwound *through* the
            # target, which reads as an oscillation and gets misdiagnosed as
            # too much proportional gain.
            integral -= error * CONTROL_PERIOD

        load = disturbance if time >= disturbance_time else 0.0

        for _ in range(SIMULATION_SUBSTEPS):
            dt = CONTROL_PERIOD / SIMULATION_SUBSTEPS
            # Current loop: the commanded torque arrives through a first-order lag.
            applied_torque += (limited - applied_torque) * (dt / max(plant.current_lag, 1e-6))
            position, velocity = _rk4(plant, position, velocity, applied_torque - load, dt)

        times[index] = time
        positions[index] = position
        setpoints[index] = target
        torques[index] = applied_torque

    return _analyse(times, positions, setpoints, torques, start, target, saturated_steps / steps, label)


def _rk4(plant: JointPlant, position: float, velocity: float, torque: float, dt: float) -> Tuple[float, float]:
    def derivatives(p: float, v: float) -> Tuple[float, float]:
        return v, plant.derivative(p, v, torque)

    k1 = derivatives(position, velocity)
    k2 = derivatives(position + dt / 2 * k1[0], velocity + dt / 2 * k1[1])
    k3 = derivatives(position + dt / 2 * k2[0], velocity + dt / 2 * k2[1])
    k4 = derivatives(position + dt * k3[0], velocity + dt * k3[1])
    return (
        position + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
        velocity + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]),
    )


def _analyse(
    times: np.ndarray,
    positions: np.ndarray,
    setpoints: np.ndarray,
    torques: np.ndarray,
    start: float,
    target: float,
    saturated_fraction: float,
    label: str,
) -> StepResponse:
    span = target - start
    if abs(span) < 1e-9:
        return StepResponse(times, positions, setpoints, torques, 0, 0, 0, 0, 0, saturated_fraction, label)

    normalised = (positions - start) / span
    rise_time = _crossing(times, normalised, 0.9) - _crossing(times, normalised, 0.1)
    overshoot = float(max(np.max(normalised) - 1.0, 0.0) * 100.0)

    # Settling time: the last moment the response left a +-2% band around its
    # own *final* value, which is the textbook definition and the one that
    # means something here. Measuring the band around the setpoint instead
    # conflates settling with steady-state error: this axis has Coulomb
    # friction, so a well-damped response that stops 3% short never "settles",
    # and the column reads as the full simulation duration for the controller
    # that is actually behaving best.
    final = float(np.mean(positions[-int(0.05 * len(positions)) :]))
    band = 0.02 * abs(span)
    outside = np.flatnonzero(np.abs(positions - final) > band)
    settling = float(times[outside[-1]]) if len(outside) else 0.0

    steady = float(final - target)
    iae = float(np.trapz(np.abs(setpoints - positions), times))
    return StepResponse(
        times, positions, setpoints, torques,
        max(rise_time, 0.0), overshoot, settling, steady, iae, saturated_fraction, label,
    )


def _crossing(times: np.ndarray, normalised: np.ndarray, level: float) -> float:
    """First time the response reaches ``level``, linearly interpolated."""
    above = np.flatnonzero(normalised >= level)
    if len(above) == 0:
        return float(times[-1])
    index = int(above[0])
    if index == 0:
        return float(times[0])
    previous, current = normalised[index - 1], normalised[index]
    if abs(current - previous) < 1e-12:
        return float(times[index])
    fraction = (level - previous) / (current - previous)
    return float(times[index - 1] + fraction * (times[index] - times[index - 1]))


def relay_autotune(
    plant: JointPlant,
    amplitude: float = 30.0,
    hysteresis: float = 0.002,
    position: float = 0.0,
    duration: float = 6.0,
) -> Tuple[float, float]:
    """Ultimate gain and period from a relay experiment (Astrom-Hagglund).

    Drives the axis with a bang-bang torque about the setpoint and measures the
    resulting limit cycle. The describing-function approximation gives
    ``Ku = 4 d / (pi a)`` from the relay amplitude ``d`` and the oscillation
    amplitude ``a``.

    The hysteresis band is not optional. Without it the relay chatters on
    sensor noise at the control rate and the "limit cycle" it measures is the
    sample period, which yields an ultimate gain about two orders of magnitude
    too high -- an auto-tune that confidently returns a set of gains that tear
    the axis apart.
    """
    steps = int(round(duration / CONTROL_PERIOD))
    setpoint = position
    velocity = 0.0
    output = amplitude
    applied = plant.gravity(position)
    bias = plant.gravity(position)

    history = np.zeros(steps)
    switch_times: List[float] = []
    for index in range(steps):
        error = setpoint - position
        if error > hysteresis:
            output = amplitude
        elif error < -hysteresis:
            output = -amplitude
        command = float(np.clip(bias + output, -plant.torque_limit, plant.torque_limit))

        for _ in range(SIMULATION_SUBSTEPS):
            dt = CONTROL_PERIOD / SIMULATION_SUBSTEPS
            applied += (command - applied) * (dt / max(plant.current_lag, 1e-6))
            position, velocity = _rk4(plant, position, velocity, applied, dt)

        history[index] = position
        if index > 0 and np.sign(history[index] - setpoint) != np.sign(history[index - 1] - setpoint):
            switch_times.append(index * CONTROL_PERIOD)

    settled = history[int(0.4 * steps):]
    oscillation = float((np.max(settled) - np.min(settled)) / 2.0)
    if oscillation < 1e-6 or len(switch_times) < 4:
        raise RuntimeError("the relay did not produce a limit cycle; raise the amplitude")

    # Use only the second half of the crossings: the first few include the
    # transient from the initial condition.
    usable = switch_times[len(switch_times) // 2:]
    period = float(2.0 * np.mean(np.diff(usable)))
    ultimate_gain = 4.0 * amplitude / (np.pi * oscillation)
    return ultimate_gain, period


def ziegler_nichols(ultimate_gain: float, period: float, detune: float = 1.0) -> PIDGains:
    """Classic ZN PID rule, optionally detuned by a factor.

    ``detune=1.0`` is textbook ZN and overshoots by roughly a quarter of the
    step. ``detune=0.5`` is what gets commissioned.
    """
    return PIDGains(
        kp=0.6 * ultimate_gain * detune,
        ki=1.2 * ultimate_gain * detune / period,
        kd=0.075 * ultimate_gain * period * detune,
        name=f"ziegler-nichols x{detune:g}",
    )


def stability_margins(plant: JointPlant, gains: PIDGains) -> Dict[str, float]:
    """Gain and phase margin of the linearised loop.

    The linearisation drops Coulomb friction and the actuator limit and keeps
    the inertia, viscous friction, current-loop lag and the filtered PID:

        L(s) = (kd' s^2 + kp s + ki) / (s * (J s^2 + b s)) * 1 / (1 + s * tau)

    This exists to make a claim in ``docs/controller-tuning.md`` checkable
    rather than asserted. The claim is that Ziegler-Nichols does badly on this
    axis for a structural reason: with ``b`` small, the plant is close to a
    double integrator, which is outside the first-order-plus-dead-time family
    the ZN rules were fitted to, and the resulting loop has *negative* phase
    margin. It survives in simulation only because friction and torque
    saturation limit the amplitude -- which is exactly the kind of "it seems to
    work" that hands you an axis that rings on every long move.

    Returns NaN margins where the response never crosses, which is a real
    outcome and not an error.
    """
    omega = np.logspace(-1, 3.5, 4000)
    s = 1j * omega
    controller = gains.kd * s**2 + gains.kp * s + gains.ki
    loop = controller / (s * (plant.inertia * s**2 + plant.viscous * s)) / (1.0 + s * plant.current_lag)

    magnitude = np.abs(loop)
    phase = np.unwrap(np.angle(loop))
    # At low frequency this loop looks like ki / (b s^2), whose phase is exactly
    # -180 degrees -- the branch cut of ``np.angle``. Unwrapping from there can
    # settle on the +180 branch and report a phase margin of 371 degrees, which
    # is 11.6 degrees wearing a disguise. Pull the whole curve down to the
    # branch that starts at or below zero.
    while phase[0] > 1e-9:
        phase -= 2.0 * np.pi

    crossings = np.flatnonzero(np.diff(np.sign(magnitude - 1.0)))
    if len(crossings):
        index = int(crossings[-1])
        crossover = float(np.interp(0.0, [np.log(magnitude[index + 1]), np.log(magnitude[index])],
                                    [omega[index + 1], omega[index]]))
        phase_at_crossover = float(np.interp(crossover, omega, phase))
        phase_margin = float(np.degrees(phase_at_crossover) + 180.0)
    else:
        crossover, phase_margin = float("nan"), float("nan")

    phase_crossings = np.flatnonzero(np.diff(np.sign(phase + np.pi)))
    if len(phase_crossings):
        index = int(phase_crossings[0])
        gain_margin = float(-20.0 * np.log10(magnitude[index]))
    else:
        gain_margin = float("nan")

    return {
        "crossover_rad_s": round(crossover, 3),
        "phase_margin_deg": round(phase_margin, 2),
        "gain_margin_db": round(gain_margin, 2),
        "stable": bool(np.isfinite(phase_margin) and phase_margin > 0.0),
    }


def sweep(
    plant: JointPlant,
    gain_sets: Sequence[PIDGains],
    inertias: Sequence[float],
    **kwargs,
) -> List[StepResponse]:
    """Every gain set against every inertia, for the robustness table."""
    responses = []
    for gains in gain_sets:
        for inertia in inertias:
            variant = JointPlant(
                inertia=inertia,
                viscous=plant.viscous,
                coulomb=plant.coulomb,
                gravity_amplitude=plant.gravity_amplitude,
                torque_limit=plant.torque_limit,
                current_lag=plant.current_lag,
            )
            responses.append(
                simulate_step(variant, gains, label=f"{gains.name} @ J={inertia:g}", **kwargs)
            )
    return responses
