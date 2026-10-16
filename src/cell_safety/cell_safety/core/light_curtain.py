"""A simulated safety light curtain: beam geometry, detection and OSSD outputs.

SIMULATED, NOT CERTIFIED. Nothing in this package is a safety function in the
sense of ISO 13849-1 or IEC 62061. It models the behaviour a type 4 curtain
(IEC 61496-1 and -2) presents to the rest of the cell, so that the cell's
reaction to it can be designed and tested. A real cell gets its protective
stop from certified hardware and a safety PLC or safety relay, and a
performance level is claimed from that hardware's data, not from a simulation.

THE CURTAIN

Two posts stand in a vertical plane: an emitter and a receiver. Between them
run horizontal beams, one every ``beam_pitch`` metres from ``bottom_z`` to
``top_z``. Each beam is a cylinder of light ``beam_diameter`` wide. A beam is
*interrupted* when an object covers the whole of it, which is the conservative
reading: a partly shadowed beam may or may not trip a real receiver.

DETECTION CAPABILITY

The resolution ``d`` is the smallest object guaranteed to interrupt at least
one beam wherever it is in the protected field. For this geometry it is
``pitch + beam_diameter``: an object of diameter ``d`` always has a beam
centre within ``pitch / 2`` of its own centre, and then covers that beam
completely. ``tests/test_safety.py`` sweeps an object of exactly ``d`` across
the field to check the claim, and shows a smaller object slipping through.

OSSD OUTPUTS

A real curtain reports through two output signal switching devices (OSSDs),
two independent channels that are both ON when the field is clear. Both go
OFF within the *response time* of an interruption. Coming back ON is slower
on purpose: the field has to stay clear for ``restart_delay`` first. The
model also lets a test break one channel (``stuck_channel``), which is how
the dual-channel discrepancy check in :mod:`cell_safety.core.protective_stop`
is exercised.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class LightCurtainSpec:
    """Geometry and timing of one curtain. Metres and seconds.

    The defaults are model parameters, chosen to sit inside the range type 4
    finger-protection curtains are sold with. They are assumptions, not
    measurements of any product.
    """

    emitter_xy: Tuple[float, float] = (0.25, 1.60)
    receiver_xy: Tuple[float, float] = (1.05, 1.60)
    bottom_z: float = 0.30
    top_z: float = 1.80
    beam_pitch: float = 0.010
    beam_diameter: float = 0.004
    response_time: float = 0.020
    restart_delay: float = 0.100

    @classmethod
    def from_dict(cls, entry: Dict) -> "LightCurtainSpec":
        return cls(
            emitter_xy=tuple(float(value) for value in entry["emitter_xy"]),
            receiver_xy=tuple(float(value) for value in entry["receiver_xy"]),
            bottom_z=float(entry["bottom_z"]),
            top_z=float(entry["top_z"]),
            beam_pitch=float(entry["beam_pitch"]),
            beam_diameter=float(entry["beam_diameter"]),
            response_time=float(entry["response_time"]),
            restart_delay=float(entry["restart_delay"]),
        )

    @property
    def resolution(self) -> float:
        """Detection capability ``d``: the smallest object always detected."""
        return self.beam_pitch + self.beam_diameter

    @property
    def width(self) -> float:
        return float(np.hypot(self.receiver_xy[0] - self.emitter_xy[0], self.receiver_xy[1] - self.emitter_xy[1]))

    def beam_heights(self) -> np.ndarray:
        count = int(np.floor((self.top_z - self.bottom_z) / self.beam_pitch + 1e-9)) + 1
        return self.bottom_z + self.beam_pitch * np.arange(count)

    def plane_coordinates(self, point: Sequence[float]) -> Tuple[float, float, float]:
        """A world point as (along the curtain, height, signed distance off the plane)."""
        emitter = np.asarray(self.emitter_xy, dtype=float)
        receiver = np.asarray(self.receiver_xy, dtype=float)
        axis = (receiver - emitter) / self.width
        normal = np.array([-axis[1], axis[0]])
        offset = np.asarray(point[:2], dtype=float) - emitter
        return float(offset @ axis), float(point[2]), float(offset @ normal)


@dataclass(frozen=True)
class Intruder:
    """Something entering the protected field: a sphere, or a rod end-on.

    A finger or an arm crossing the curtain presents roughly a disc in the
    plane, so a sphere of the same diameter is the model.
    """

    centre: Tuple[float, float, float]
    diameter: float


def interrupted_beams(spec: LightCurtainSpec, intruders: Sequence[Intruder]) -> List[int]:
    """Indices of the beams completely covered by any intruder."""
    heights = spec.beam_heights()
    blocked: set = set()
    for intruder in intruders:
        along, height, off_plane = spec.plane_coordinates(intruder.centre)
        radius = intruder.diameter / 2.0
        if abs(off_plane) > radius:
            continue  # Not touching the plane.
        # The sphere's cross-section in the plane is a disc of this radius.
        disc = float(np.sqrt(max(radius**2 - off_plane**2, 0.0)))
        if along < -disc or along > spec.width + disc:
            continue  # Outside the span between the posts.
        for index, beam in enumerate(heights):
            if abs(height - beam) + spec.beam_diameter / 2.0 <= disc + 1e-9:  # 1 nm: rounding, not optics.
                blocked.add(index)
    return sorted(blocked)


@dataclass
class LightCurtain:
    """The curtain over time: field state in, two OSSD channels out.

    Call :meth:`step` once per tick with the current intruders. The OSSDs
    switch OFF ``response_time`` after the first interrupted tick and back ON
    ``restart_delay`` after the field last cleared.
    """

    spec: LightCurtainSpec = field(default_factory=LightCurtainSpec)
    stuck_channel: Optional[int] = None  # 1 or 2: that OSSD never switches off.
    _blocked_since: Optional[float] = None
    _clear_since: Optional[float] = 0.0
    _ossd_on: bool = True

    def step(self, time: float, intruders: Sequence[Intruder] = ()) -> Tuple[bool, bool]:
        return self.step_blocked(time, bool(interrupted_beams(self.spec, intruders)))

    def step_blocked(self, time: float, blocked: bool) -> Tuple[bool, bool]:
        """The same, with the field state given directly."""
        if blocked:
            self._clear_since = None
            if self._blocked_since is None:
                self._blocked_since = time
            if time - self._blocked_since >= self.spec.response_time - 1e-12:
                self._ossd_on = False
        else:
            self._blocked_since = None
            if self._clear_since is None:
                self._clear_since = time
            if not self._ossd_on and time - self._clear_since >= self.spec.restart_delay - 1e-12:
                self._ossd_on = True
        ossd1 = self._ossd_on or self.stuck_channel == 1
        ossd2 = self._ossd_on or self.stuck_channel == 2
        return ossd1, ossd2


def minimum_separation_distance(stopping_time: float, resolution: float) -> float:
    """ISO 13855 minimum distance from the curtain to the hazard, in metres.

    ``S = K * T + C`` with ``T`` the overall stopping performance (curtain
    response plus machine stopping time) and ``C = 8 (d - 14)`` millimetres
    for a resolution ``d`` of 40 mm or less. ``K`` is 2000 mm/s; if that gives
    more than 500 mm the standard recalculates with 1600 mm/s, with 500 mm as
    the floor. The floor below 500 mm is 100 mm.

    The formula is the standard's; the inputs here come from a simulation, so
    the output is a design figure for a simulated cell and nothing more.
    """
    d_mm = resolution * 1000.0
    if d_mm > 40.0:
        raise ValueError("the C = 8 (d - 14) term only applies to a resolution of 40 mm or less")
    c_mm = max(8.0 * (d_mm - 14.0), 0.0)
    t_ms = stopping_time * 1000.0
    s_mm = 2.0 * t_ms + c_mm
    if s_mm > 500.0:
        s_mm = max(1.6 * t_ms + c_mm, 500.0)
    return max(s_mm, 100.0) / 1000.0
