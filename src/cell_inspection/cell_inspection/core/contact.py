"""Force-feedback contact inspection: touching a part to find out where it is.

After the part is placed in the nest, the tool probes a list of points on its
top face and measures the height at each. Five points give a plane, and the
plane says whether the part is seated: a part sitting on a chip of swarf, or
dropped into the nest at an angle, shows up as a tilt of a few tenths of a
degree, which no camera at this standoff resolves and which a probe measures
in a couple of seconds.

THE LOOP

Velocity-based admittance control at ``CONTROL_RATE``, one law throughout:

    v_z = (F_measured - F_target) / damping

In free space ``F`` is zero, so the probe descends at ``F_target / damping``,
and the damping is chosen to make that the intended approach speed. As the
force rises the descent slows, and the loop settles where ``F = F_target``.

It is worth writing it as one law rather than as "approach, then switch to
force control on contact". The switched version needs a mode flag; the version
that keeps the approach term *and* adds the force term -- which is the obvious
reading of the admittance equation, and what the first draft of this did --
never stops descending, because at ``F = F_target`` it is still commanding the
full approach speed. Every probe drove to the 45 N abort limit.

Contact is declared when the *filtered* force crosses ``CONTACT_THRESHOLD``.
That declaration affects only the measurement, not the control, so a late or
early trigger costs accuracy and cannot cost the part. The surface height is
the tip height at the trigger, corrected for the elastic deflection already
present, ``F / k``.

WHAT LIMITS THE ACCURACY, IN ORDER

1. **The trigger threshold against the contact stiffness.** Declaring contact
   at 2 N into a 20 kN/m contact means the probe is already 0.1 mm in. That
   term is *corrected for* rather than ignored -- the correction needs a
   stiffness estimate, which is what ``estimate_stiffness`` measures from the
   force ramp during the settle, rather than assuming the nominal value.
2. **Force noise.** A wrist force-torque sensor is good to a few tenths of a
   newton; the UR built-in estimator is closer to 1 N. Noise on the trigger
   maps to height error through the same stiffness, so at 1 N RMS it is worth
   50 microns on its own. The filter trades that against latency.
3. **Approach speed against the control period.** At 5 mm/s and 500 Hz, one
   period is 10 microns of overshoot before the loop can react.

None of these is a modelling flourish: they are the three numbers anyone tuning
a real touch-probe routine ends up trading off, and
``analysis/inspection_accuracy.py`` reports the measured error against each.

WHAT IS NOT MODELLED

The probe is a point. Real fingertips have a radius, so a probe landing near an
edge reads the height of the edge rather than the face, and a chamfer reads
somewhere in between. Robot structural compliance is lumped into the contact
stiffness rather than modelled as a separate series spring, which is fine for a
vertical probe and wrong for a lateral one. ``docs/limitations.md`` says so.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

CONTROL_RATE = 500.0  # Hz
CONTROL_PERIOD = 1.0 / CONTROL_RATE

APPROACH_SPEED = 0.005  # m/s, downward, in free space.
CONTACT_THRESHOLD = 2.0  # N, on the filtered force.
TARGET_FORCE = 5.0  # N, held once in contact.
# Chosen so that the free-space descent is exactly APPROACH_SPEED. Raising it
# makes the loop gentler and slower; the stability margin is generous either
# way, since the contact time constant is damping / stiffness = 50 ms against a
# 2 ms control period.
ADMITTANCE_DAMPING = TARGET_FORCE / APPROACH_SPEED  # N per (m/s).
FORCE_FILTER_CUTOFF = 30.0  # Hz, first-order low pass.
# Probe travel allowance. The routine starts 5 mm above where the part model
# says the face is, so 12 mm covers a part sitting well proud of the nest and
# still aborts in under three seconds when there is no part at all.
MAX_TRAVEL = 0.012  # m, from the start of the descent.
OVERFORCE_LIMIT = 45.0  # N, the abort threshold.

# Fault codes, mirrored in the Modbus register map so the PLC sees the same
# vocabulary the inspection routine uses.
FAULT_NONE = 0
FAULT_NO_CONTACT = 21
FAULT_OVERFORCE = 22
FAULT_NOT_SEATED = 23


@dataclass
class ContactSurface:
    """The thing being probed: a plane with a height offset per region.

    ``tilt`` is a two-vector of gradients in x and y, which is how a part that
    is not seated presents itself -- one corner up, the face no longer level.
    """

    height: float
    stiffness: float = 20000.0
    tilt: np.ndarray = field(default_factory=lambda: np.zeros(2))
    origin: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def height_at(self, point: Sequence[float]) -> float:
        offset = np.asarray(point[:2], dtype=float) - self.origin
        return float(self.height + self.tilt @ offset)


@dataclass
class ProbeResult:
    """One probe: where it touched, how hard, and whether it worked."""

    point: np.ndarray
    measured_height: float
    true_height: float
    contact_time: float
    peak_force: float
    steady_force: float
    fault: int
    estimated_stiffness: float
    samples: int

    @property
    def error(self) -> float:
        return self.measured_height - self.true_height

    def as_dict(self) -> Dict:
        return {
            "x": round(float(self.point[0]), 5),
            "y": round(float(self.point[1]), 5),
            "measured_height_mm": round(self.measured_height * 1000.0, 4),
            "true_height_mm": round(self.true_height * 1000.0, 4),
            "error_um": round(self.error * 1e6, 2),
            "contact_time_s": round(self.contact_time, 4),
            "peak_force_n": round(self.peak_force, 3),
            "steady_force_n": round(self.steady_force, 3),
            "estimated_stiffness_n_per_mm": round(self.estimated_stiffness / 1000.0, 2),
            "fault": self.fault,
        }


def probe_point(
    surface: ContactSurface,
    point: Sequence[float],
    start_height: float,
    rng: np.random.Generator,
    force_noise: float = 0.35,
    settle_time: float = 0.25,
    approach_speed: float = APPROACH_SPEED,
) -> ProbeResult:
    """Descend onto ``point`` under admittance control and measure the height.

    Returns as soon as the settle window after contact has elapsed, or with a
    fault if the probe travelled its full allowance without touching anything
    or drove into something too hard.
    """
    point = np.asarray(point, dtype=float)
    true_height = surface.height_at(point)

    position = float(start_height)
    filtered = 0.0
    alpha = float(np.clip(2.0 * np.pi * FORCE_FILTER_CUTOFF * CONTROL_PERIOD, 0.0, 1.0))

    contact_time: Optional[float] = None
    contact_position = 0.0
    contact_force = 0.0
    peak_force = 0.0
    forces: List[Tuple[float, float]] = []  # (penetration, force) after contact.
    time = 0.0
    steps = int(np.ceil((MAX_TRAVEL / approach_speed + settle_time) / CONTROL_PERIOD)) + 2

    for _ in range(steps):
        penetration = max(true_height - position, 0.0)
        true_force = surface.stiffness * penetration
        measured = true_force + rng.normal(0.0, force_noise)
        filtered += alpha * (measured - filtered)
        peak_force = max(peak_force, true_force)

        if true_force > OVERFORCE_LIMIT:
            return ProbeResult(
                point, float("nan"), true_height, time, peak_force, true_force,
                FAULT_OVERFORCE, surface.stiffness, len(forces),
            )

        if contact_time is None:
            if filtered > CONTACT_THRESHOLD:
                contact_time = time
                contact_position = position
                contact_force = filtered
            elif start_height - position > MAX_TRAVEL:
                return ProbeResult(
                    point, float("nan"), true_height, time, peak_force, 0.0,
                    FAULT_NO_CONTACT, surface.stiffness, 0,
                )
        else:
            forces.append((max(true_height - position, 0.0), filtered))
            if time - contact_time > settle_time:
                break

        damping = TARGET_FORCE / approach_speed
        position += (filtered - TARGET_FORCE) / damping * CONTROL_PERIOD
        time += CONTROL_PERIOD

    stiffness = estimate_stiffness(forces, surface.stiffness)
    # The tip was already `contact_force / stiffness` into the surface when the
    # trigger fired. Correcting for it is worth about 100 microns at these
    # settings, which is more than everything else in the error budget put
    # together.
    measured_height = contact_position + contact_force / stiffness
    steady = float(np.mean([force for _, force in forces[-40:]])) if forces else 0.0
    return ProbeResult(
        point, measured_height, true_height, contact_time or 0.0, peak_force, steady,
        FAULT_NONE, stiffness, len(forces),
    )


def estimate_stiffness(samples: Sequence[Tuple[float, float]], fallback: float) -> float:
    """Least-squares slope of force against penetration during the settle.

    Measuring it beats assuming it. The nominal contact stiffness of a part in
    a nest is a guess -- it depends on the part, the nest, and how much of the
    robot's own compliance is in series -- and the deflection correction is
    directly proportional to it.
    """
    usable = [(depth, force) for depth, force in samples if depth > 1e-6]
    if len(usable) < 20:
        return float(fallback)
    depths = np.array([depth for depth, _ in usable])
    forces = np.array([force for _, force in usable])
    slope = float((depths @ forces) / max(depths @ depths, 1e-12))
    if not np.isfinite(slope) or slope < 1e3:
        return float(fallback)
    return slope


@dataclass
class InspectionReport:
    """The result of probing a part: the fitted plane and the verdict."""

    probes: List[ProbeResult]
    tilt_deg: float
    plane_residual_um: float
    mean_abs_error_um: float
    max_abs_error_um: float
    fault: int
    duration: float

    def as_dict(self) -> Dict:
        return {
            "probes": [probe.as_dict() for probe in self.probes],
            "tilt_deg": round(self.tilt_deg, 4),
            "plane_residual_um": round(self.plane_residual_um, 2),
            "mean_abs_error_um": round(self.mean_abs_error_um, 2),
            "max_abs_error_um": round(self.max_abs_error_um, 2),
            "fault": self.fault,
            "duration_s": round(self.duration, 3),
        }


def inspect_part(
    surface: ContactSurface,
    points: Sequence[Sequence[float]],
    start_height: float,
    rng: np.random.Generator,
    tilt_limit_deg: float = 0.35,
    force_noise: float = 0.35,
) -> InspectionReport:
    """Probe every point, fit a plane, and decide whether the part is seated.

    ``tilt_limit_deg`` is the acceptance threshold. 0.35 degrees over a 60 mm
    part is 0.37 mm of lift at the corner -- about the thickness of a chip of
    swarf, which is the thing this is looking for.
    """
    probes = [probe_point(surface, point, start_height, rng, force_noise=force_noise) for point in points]
    duration = float(sum(probe.contact_time + 0.25 + 0.35 for probe in probes))

    good = [probe for probe in probes if probe.fault == FAULT_NONE]
    if len(good) < 3:
        fault = next((probe.fault for probe in probes if probe.fault != FAULT_NONE), FAULT_NO_CONTACT)
        return InspectionReport(probes, float("nan"), float("nan"), float("nan"), float("nan"), fault, duration)

    design = np.array([[probe.point[0], probe.point[1], 1.0] for probe in good])
    heights = np.array([probe.measured_height for probe in good])
    coefficients, *_ = np.linalg.lstsq(design, heights, rcond=None)
    residual = heights - design @ coefficients
    tilt = float(np.degrees(np.arctan(np.hypot(coefficients[0], coefficients[1]))))

    errors = np.array([abs(probe.error) for probe in good])
    fault = FAULT_NONE if tilt <= tilt_limit_deg else FAULT_NOT_SEATED
    return InspectionReport(
        probes=probes,
        tilt_deg=tilt,
        plane_residual_um=float(np.sqrt(np.mean(residual**2)) * 1e6),
        mean_abs_error_um=float(np.mean(errors) * 1e6),
        max_abs_error_um=float(np.max(errors) * 1e6),
        fault=fault,
        duration=duration,
    )


def inspection_points(centre: Sequence[float], size: Sequence[float], fractions: Sequence[Sequence[float]]):
    """Absolute probe points from fractional offsets over a part's footprint."""
    centre = np.asarray(centre, dtype=float)
    size = np.asarray(size, dtype=float)
    return [centre[:2] + np.asarray(fraction, dtype=float) * size[:2] for fraction in fractions]
