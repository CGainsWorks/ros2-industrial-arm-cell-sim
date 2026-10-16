"""The protective stop: dual-channel input, latching, reset, and the stop itself.

SIMULATED, NOT CERTIFIED. See :mod:`cell_safety.core.light_curtain`.

WHAT IS MODELLED

ISO 10218-1 requires a robot to have a protective stop that an external
safeguard, such as a light curtain, can trigger, and it calls up the stop
categories of IEC 60204-1:

* category 0: remove power from the drives at once and let the brakes stop it;
* category 1: a controlled stop with power available, then remove power once
  the robot is at a standstill;
* category 2: a controlled stop with power left on.

This cell uses **category 1**, which is the usual choice for a protective stop
on an arm carrying a part: stopping along the path under control keeps the
part in the jaws and the tool off the fixtures, and removing power at
standstill means the stop does not depend on the controller holding position.

THE CONTROLLER, :class:`SafetyController`

* **Two channels.** Both OSSDs must be ON to run. Either going OFF stops the
  arm. If they disagree for longer than ``discrepancy_time`` the controller
  locks out, because one channel stuck ON is exactly the fault a two-channel
  design exists to catch.
* **The stop latches.** Clearing the curtain does not restart the arm.
* **Manual reset, with a restart interlock.** The reset is a rising edge on a
  separate input, accepted only once the field is clear and the arm is at a
  standstill. Even then it only re-arms the safety function: the cycle does
  not resume until the PLC starts it again.

THE STOP, :func:`execute_with_protective_stop`

A timed trajectory is executed at the controller rate. When the stop is
requested the arm keeps following its planned path but the rate at which it
moves along that path ramps down to zero, as fast as the joint deceleration
limit allows. Stopping on the path rather than braking each joint on its own
is what keeps a category 1 stop from swinging the tool into something the
planned path avoided.

THE BUDGET

:func:`stop_time_budget` states, from the parameters alone, how long the cell
may take from a beam being broken to the arm standing still. Tests and
``analysis/safety_stop.py`` then check every simulated stop against it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional

import numpy as np

from cell_safety.core.light_curtain import LightCurtain

# Joint limits, duplicated from cell_motion.core.kinematics rather than
# imported so that the safety package depends on nothing but numpy. A test
# asserts the two copies agree.
VELOCITY_LIMITS = np.full(6, np.pi)
ACCELERATION_LIMITS = np.full(6, 8.0)


class StopCategory(IntEnum):
    """IEC 60204-1 stop categories."""

    CATEGORY_0 = 0
    CATEGORY_1 = 1
    CATEGORY_2 = 2


class SafetyState(IntEnum):
    RUN = 0
    STOPPING = 1  # Stop requested, decelerating with power on.
    STOPPED = 2  # Standstill. Category 1: power removed.
    LOCKOUT = 3  # Channel discrepancy.


@dataclass(frozen=True)
class SafetyParameters:
    """Timing and limits of the simulated safety chain. Seconds and rad/s^2.

    ``controller_period`` is 2 ms, the 500 Hz rate the ros2_control
    controller manager runs at in ``cell_motion/config/ros2_controllers.yaml``.
    ``stop_deceleration`` is the joint acceleration limit the trajectory
    generator plans to, not a figure from a UR5e safety manual: the real
    controller brakes harder, so this is the conservative side.
    """

    controller_period: float = 0.002
    discrepancy_time: float = 0.050
    stop_deceleration: tuple = tuple(ACCELERATION_LIMITS)
    category: StopCategory = StopCategory.CATEGORY_1


@dataclass
class SafetyOutputs:
    stop_requested: bool
    drives_enabled: bool
    state: SafetyState
    reset_required: bool
    curtain_clear: bool


@dataclass
class SafetyController:
    """Dual-channel evaluation of the OSSDs, latching, and manual reset."""

    parameters: SafetyParameters = field(default_factory=SafetyParameters)
    state: SafetyState = SafetyState.RUN
    _discrepant_since: Optional[float] = None
    _previous_reset: bool = False
    stop_time: Optional[float] = None
    trips: int = 0

    def step(self, time: float, ossd1: bool, ossd2: bool, reset: bool, standstill: bool) -> SafetyOutputs:
        """Evaluate one controller tick."""
        reset_edge = reset and not self._previous_reset
        self._previous_reset = reset
        clear = ossd1 and ossd2

        if ossd1 != ossd2:
            if self._discrepant_since is None:
                self._discrepant_since = time
            if time - self._discrepant_since >= self.parameters.discrepancy_time - 1e-12:
                self.state = SafetyState.LOCKOUT
        else:
            self._discrepant_since = None

        if self.state == SafetyState.RUN and not clear:
            self.state = SafetyState.STOPPING
            self.stop_time = time
            self.trips += 1

        if self.state == SafetyState.STOPPING:
            if self.parameters.category == StopCategory.CATEGORY_0 or standstill:
                self.state = SafetyState.STOPPED

        if reset_edge and clear and standstill and self.state in (SafetyState.STOPPED, SafetyState.LOCKOUT):
            # A lockout is only reset once the channels agree again, which
            # `clear` already requires. A real safety relay would usually
            # demand a power cycle for it as well.
            self.state = SafetyState.RUN
            self.stop_time = None

        return self.outputs(clear)

    def outputs(self, clear: bool) -> SafetyOutputs:
        stopping = self.state != SafetyState.RUN
        if self.parameters.category == StopCategory.CATEGORY_2:
            drives = self.state != SafetyState.LOCKOUT
        else:
            drives = self.state in (SafetyState.RUN, SafetyState.STOPPING)
        return SafetyOutputs(
            stop_requested=stopping,
            drives_enabled=drives,
            state=self.state,
            reset_required=self.state in (SafetyState.STOPPED, SafetyState.LOCKOUT) and clear,
            curtain_clear=clear,
        )


def stop_time_budget(
    response_time: float,
    parameters: SafetyParameters = SafetyParameters(),
    peak_joint_velocity: Optional[np.ndarray] = None,
    velocity_scale: float = 0.5,
) -> float:
    """The longest a protective stop may take, from beam broken to standstill.

    Curtain response, plus two controller periods (one to sample the OSSDs,
    one to act on them), plus the time to bring the fastest joint to rest
    from its peak speed at the stop deceleration. With no trajectory given,
    the peak speed is the velocity limit at the cell's 50% velocity scale.
    """
    if peak_joint_velocity is None:
        peak_joint_velocity = VELOCITY_LIMITS * velocity_scale
    deceleration = np.asarray(parameters.stop_deceleration, dtype=float)
    braking = float(np.max(np.abs(np.asarray(peak_joint_velocity, dtype=float)) / deceleration))
    return response_time + 2.0 * parameters.controller_period + braking


@dataclass
class StopReport:
    """What happened when a trajectory was executed under the safety chain."""

    duration: float  # Planned trajectory duration.
    break_time: Optional[float]  # Beam broken.
    ossd_off_time: Optional[float]  # Both channels OFF (or the first, on a discrepancy).
    stop_request_time: Optional[float]  # The controller acted.
    standstill_time: Optional[float]  # Path rate reached zero.
    drives_off_time: Optional[float]  # Category 1: power removed.
    completed: bool  # Reached the end of the trajectory.
    path_fraction: float  # How far along the path the arm stopped, 0..1.
    stopping_distance_tcp: float  # Metres the tool travelled after the stop request.
    peak_joint_deceleration: float  # rad/s^2, commanded by the stop ramp.
    final_joints: np.ndarray = field(default_factory=lambda: np.zeros(6))
    samples: List[Dict] = field(default_factory=list)

    @property
    def stopping_time(self) -> Optional[float]:
        """Beam broken to standstill, which is what a stop is judged on."""
        if self.break_time is None or self.standstill_time is None:
            return None
        return self.standstill_time - self.break_time

    def as_dict(self) -> Dict:
        return {
            "duration": round(self.duration, 4),
            "break_time": self.break_time,
            "ossd_off_time": self.ossd_off_time,
            "stop_request_time": self.stop_request_time,
            "standstill_time": self.standstill_time,
            "drives_off_time": self.drives_off_time,
            "stopping_time": None if self.stopping_time is None else round(self.stopping_time, 4),
            "completed": self.completed,
            "path_fraction": round(self.path_fraction, 4),
            "stopping_distance_tcp": round(self.stopping_distance_tcp, 5),
            "peak_joint_deceleration": round(self.peak_joint_deceleration, 3),
        }


def _path_velocity(positions: np.ndarray, times: np.ndarray, tau: float) -> np.ndarray:
    """dq/dtau of the piecewise-linear trajectory at time tau."""
    index = int(np.clip(np.searchsorted(times, tau, side="right") - 1, 0, len(times) - 2))
    span = max(times[index + 1] - times[index], 1e-9)
    return (positions[index + 1] - positions[index]) / span


def _sample(positions: np.ndarray, times: np.ndarray, tau: float) -> np.ndarray:
    return np.array([np.interp(tau, times, positions[:, joint]) for joint in range(positions.shape[1])])


def execute_with_protective_stop(
    times: np.ndarray,
    positions: np.ndarray,
    curtain: LightCurtain,
    blocked: Callable[[float], bool],
    parameters: SafetyParameters = SafetyParameters(),
    tool_position: Optional[Callable[[np.ndarray], np.ndarray]] = None,
    record: bool = False,
    settle: float = 0.05,
) -> StopReport:
    """Execute a timed joint trajectory at the controller rate, under the curtain.

    ``blocked(t)`` says whether something is in the protected field at time
    ``t``: that is the simulated world. The curtain turns it into OSSD
    signals, the controller into a stop request, and the stop request into a
    ramp-down of the rate along the path.

    ``tool_position(joints)`` returns the tool centre point, for the stopping
    distance; omit it to skip that figure.
    """
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    period = parameters.controller_period
    deceleration = np.asarray(parameters.stop_deceleration, dtype=float)
    duration = float(times[-1])

    controller = SafetyController(parameters)
    tau = 0.0
    rate = 1.0
    time = 0.0
    peak_deceleration = 0.0
    joints = positions[0].copy()

    break_time = ossd_off_time = stop_request_time = standstill_time = drives_off_time = None
    tcp_at_request: Optional[np.ndarray] = None
    samples: List[Dict] = []
    stopped_for = 0.0

    while True:
        is_blocked = bool(blocked(time))
        if is_blocked and break_time is None:
            break_time = time
        ossd1, ossd2 = curtain.step_blocked(time, is_blocked)
        if not (ossd1 and ossd2) and ossd_off_time is None:
            ossd_off_time = time

        standstill = rate <= 0.0 or tau >= duration
        outputs = controller.step(time, ossd1, ossd2, reset=False, standstill=standstill)

        if outputs.stop_requested and stop_request_time is None:
            # The controller samples on this tick and acts on the next one.
            stop_request_time = time + period
            tcp_at_request = tool_position(joints) if tool_position else None

        # Advance one controller period.
        if stop_request_time is not None and time + 1e-12 >= stop_request_time and rate > 0.0:
            path_speed = np.abs(_path_velocity(positions, times, tau))
            moving = path_speed > 1e-9
            previous_rate = rate
            if parameters.category == StopCategory.CATEGORY_0:
                rate = 0.0
            elif np.any(moving):
                rate_deceleration = float(np.min(deceleration[moving] / path_speed[moving]))
                rate = max(0.0, rate - rate_deceleration * period)
            else:
                rate = 0.0
            # The joint deceleration the stop itself commands: path speed
            # times the drop in rate. The executed polyline also has corners,
            # which change joint velocity whether or not a stop is under way;
            # those belong to the path, not to the stop, so are not counted.
            if parameters.category != StopCategory.CATEGORY_0:
                commanded = float(np.max(path_speed)) * (previous_rate - rate) / period
                peak_deceleration = max(peak_deceleration, commanded)
        tau = min(tau + rate * period, duration)
        time += period
        joints = _sample(positions, times, tau)

        if record:
            samples.append(
                {"t": round(time, 4), "tau": round(tau, 5), "rate": round(rate, 5),
                 "ossd": [bool(ossd1), bool(ossd2)], "state": int(controller.state)}
            )

        if rate <= 0.0 and standstill_time is None and stop_request_time is not None:
            standstill_time = time
        if standstill_time is not None and drives_off_time is None and not controller.outputs(True).drives_enabled:
            drives_off_time = time
        if tau >= duration and stop_request_time is None:
            stopped_for += period
            if stopped_for >= settle or break_time is None:
                break
        if drives_off_time is not None:
            break
        if time > duration + 5.0:  # pragma: no cover - a bug, not a result
            raise RuntimeError("protective stop simulation did not terminate")

    # One more tick, so the controller sees the standstill and removes power.
    if standstill_time is not None and drives_off_time is None:
        controller.step(time, *curtain.step_blocked(time, bool(blocked(time))), reset=False, standstill=True)
        drives_off_time = time

    stopping_distance = 0.0
    if tool_position is not None and tcp_at_request is not None:
        stopping_distance = float(np.linalg.norm(tool_position(joints) - tcp_at_request))

    return StopReport(
        duration=duration,
        break_time=break_time,
        ossd_off_time=ossd_off_time,
        stop_request_time=stop_request_time,
        standstill_time=standstill_time,
        drives_off_time=drives_off_time,
        completed=tau >= duration and stop_request_time is None,
        path_fraction=tau / duration if duration > 0 else 1.0,
        stopping_distance_tcp=stopping_distance,
        peak_joint_deceleration=peak_deceleration,
        final_joints=joints,
        samples=samples,
    )
