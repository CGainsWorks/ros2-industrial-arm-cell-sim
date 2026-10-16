"""The simulated light curtain and the category 1 protective stop.

Simulated, not certified: these tests check that the models in
``cell_safety.core`` behave as ``docs/safety.md`` says they do. They say
nothing about a real curtain or a real safety controller.

The one the rest exist to support is
``test_breaking_the_curtain_mid_motion_halts_the_arm_within_the_budget``: a
trajectory the real planner produced for a real cycle, executed at the 500 Hz
controller rate, with the curtain broken half way along it.
"""

from __future__ import annotations

import numpy as np
import pytest

from cell_bringup.core.cycle import CycleRunner
from cell_common.cell_model import load_cell, sample_part_pose
from cell_fieldbus.core.registers import (
    COIL_ENABLE,
    COIL_FAULT_RESET,
    SAFETY_CURTAIN_CLEAR,
    SAFETY_DRIVES_ENABLED,
    SAFETY_PROTECTIVE_STOP,
    SAFETY_RESET_REQUIRED,
    Fault,
    FieldbusState,
    State,
)
from cell_motion.core import kinematics
from cell_safety.core.light_curtain import (
    Intruder,
    LightCurtain,
    LightCurtainSpec,
    interrupted_beams,
    minimum_separation_distance,
)
from cell_safety.core.protective_stop import (
    ACCELERATION_LIMITS,
    VELOCITY_LIMITS,
    SafetyController,
    SafetyParameters,
    SafetyState,
    StopCategory,
    execute_with_protective_stop,
    stop_time_budget,
)

SPEC = LightCurtainSpec.from_dict(load_cell().raw["light_curtain"])


@pytest.fixture(scope="module")
def cycle():
    """One successful nominal cycle, planned by the real planner."""
    cell = load_cell()
    runner = CycleRunner(cell)
    for seed in range(20260101, 20260111):
        rng = np.random.default_rng(seed)
        result = runner.run(sample_part_pose(cell, rng), rng)
        if result.success:
            return runner, result
    pytest.fail("no successful cycle in ten seeds")


def _field_point(along: float, height: float, off_plane: float = 0.0):
    emitter = np.asarray(SPEC.emitter_xy)
    axis = (np.asarray(SPEC.receiver_xy) - emitter) / SPEC.width
    normal = np.array([-axis[1], axis[0]])
    xy = emitter + axis * along + normal * off_plane
    return (float(xy[0]), float(xy[1]), float(height))


# -- the curtain -----------------------------------------------------------


def test_an_object_of_the_rated_resolution_is_detected_everywhere():
    assert SPEC.resolution == pytest.approx(0.014)
    heights = np.arange(SPEC.bottom_z + 0.01, SPEC.top_z - 0.01, 0.0005)
    for along in (0.05, SPEC.width / 2, SPEC.width - 0.05):
        for height in heights:
            assert interrupted_beams(SPEC, [Intruder(_field_point(along, height), SPEC.resolution)]), (
                f"a {SPEC.resolution * 1000:.0f} mm object at {height:.4f} m slipped through"
            )


def test_a_smaller_object_can_slip_between_the_beams():
    # A thin rod centred between two beams covers neither completely.
    between = SPEC.beam_heights()[10] + SPEC.beam_pitch / 2
    assert not interrupted_beams(SPEC, [Intruder(_field_point(0.4, between), 0.008)])


def test_an_object_beside_the_plane_or_outside_the_posts_is_not_seen():
    assert not interrupted_beams(SPEC, [Intruder(_field_point(0.4, 1.0, off_plane=0.05), 0.05)])
    assert not interrupted_beams(SPEC, [Intruder(_field_point(-0.10, 1.0), 0.05)])
    assert not interrupted_beams(SPEC, [Intruder(_field_point(0.4, 1.95), 0.05)])


def test_the_ossds_switch_off_after_the_response_time_and_back_after_the_restart_delay():
    curtain = LightCurtain(SPEC)
    period = 0.002
    times = np.arange(0.0, 0.5, period)
    off_at = on_at = None
    for time in times:
        blocked = 0.100 <= time < 0.200
        ossd1, ossd2 = curtain.step_blocked(float(time), blocked)
        assert ossd1 == ossd2
        if not ossd1 and off_at is None:
            off_at = time
        if off_at is not None and ossd1 and on_at is None:
            on_at = time
    assert off_at == pytest.approx(0.100 + SPEC.response_time, abs=period)
    assert on_at == pytest.approx(0.200 + SPEC.restart_delay, abs=period)


def test_iso_13855_minimum_distance_follows_the_standard():
    assert minimum_separation_distance(0.100, 0.014) == pytest.approx(0.200)
    # Above 500 mm the standard recalculates with K = 1600 mm/s, floor 500.
    assert minimum_separation_distance(0.300, 0.014) == pytest.approx(0.500)
    assert minimum_separation_distance(0.400, 0.014) == pytest.approx(0.640)
    # The resolution term: C = 8 (d - 14) mm.
    assert minimum_separation_distance(0.100, 0.030) == pytest.approx(0.328)
    with pytest.raises(ValueError):
        minimum_separation_distance(0.1, 0.050)


# -- the controller --------------------------------------------------------


def test_the_stop_latches_and_needs_a_manual_reset_once_clear_and_stopped():
    controller = SafetyController()
    assert controller.step(0.0, True, True, reset=False, standstill=False).stop_requested is False
    out = controller.step(0.002, False, False, reset=False, standstill=False)
    assert out.stop_requested and out.drives_enabled  # Category 1: power on while braking.
    out = controller.step(0.004, False, False, reset=False, standstill=True)
    assert out.state == SafetyState.STOPPED and not out.drives_enabled

    # Clearing the curtain does not restart anything.
    out = controller.step(0.010, True, True, reset=False, standstill=True)
    assert out.stop_requested and out.reset_required

    # A reset held from before is not an edge; a press while blocked is refused.
    controller.step(0.012, False, False, reset=True, standstill=True)
    out = controller.step(0.014, True, True, reset=True, standstill=True)
    assert out.stop_requested
    controller.step(0.016, True, True, reset=False, standstill=True)
    out = controller.step(0.018, True, True, reset=True, standstill=True)
    assert out.state == SafetyState.RUN and not out.stop_requested and out.drives_enabled


def test_a_stuck_channel_is_caught_as_a_discrepancy():
    parameters = SafetyParameters()
    curtain = LightCurtain(SPEC, stuck_channel=2)
    controller = SafetyController(parameters)
    states = []
    for tick in range(100):
        time = tick * parameters.controller_period
        ossd1, ossd2 = curtain.step_blocked(time, tick >= 10)
        states.append(controller.step(time, ossd1, ossd2, reset=False, standstill=True).state)
    # One channel off is enough to stop; the disagreement then locks out.
    assert SafetyState.LOCKOUT in states
    assert states[-1] == SafetyState.LOCKOUT


def test_the_joint_limits_agree_with_the_kinematics():
    assert np.allclose(VELOCITY_LIMITS, kinematics.VELOCITY_LIMITS)
    assert np.allclose(ACCELERATION_LIMITS, kinematics.ACCELERATION_LIMITS)


# -- the stop, on planned trajectories ---------------------------------------


def test_breaking_the_curtain_mid_motion_halts_the_arm_within_the_budget(cycle):
    runner, result = cycle
    budget = stop_time_budget(SPEC.response_time)
    # Stated in docs/safety.md: 20 ms response, two 2 ms controller periods,
    # and 196 ms to stop a joint at half of 180 deg/s at 8 rad/s^2.
    assert budget == pytest.approx(0.2203, abs=1e-3)

    for trajectory in result.trajectories:
        break_time = 0.5 * trajectory.duration
        report = execute_with_protective_stop(
            trajectory.times,
            trajectory.positions,
            LightCurtain(SPEC),
            lambda t: t >= break_time,
            tool_position=lambda q: runner.arm.tool_pose(q)[:3, 3],
        )
        label = trajectory.label
        assert not report.completed, f"{label}: the arm finished the move despite the stop"
        assert report.path_fraction < 1.0
        assert report.ossd_off_time == pytest.approx(break_time + SPEC.response_time, abs=0.0021)
        assert report.stopping_time <= budget, f"{label}: {report.stopping_time:.3f} s > {budget:.3f} s"
        # Category 1: power stays on until standstill, then goes.
        assert report.drives_off_time >= report.standstill_time
        assert report.drives_off_time - report.standstill_time <= 0.0041
        assert report.peak_joint_deceleration <= ACCELERATION_LIMITS.max() + 1e-6


def test_a_motion_that_is_not_interrupted_completes(cycle):
    _, result = cycle
    trajectory = result.trajectories[0]
    report = execute_with_protective_stop(
        trajectory.times, trajectory.positions, LightCurtain(SPEC), lambda t: False
    )
    assert report.completed and report.stopping_time is None
    assert np.allclose(report.final_joints, trajectory.positions[-1])


def test_category_0_cuts_power_at_once_and_stops_sooner(cycle):
    _, result = cycle
    trajectory = max(result.trajectories, key=lambda candidate: candidate.duration)
    break_time = 0.5 * trajectory.duration
    reports = {}
    for category in (StopCategory.CATEGORY_0, StopCategory.CATEGORY_1):
        reports[category] = execute_with_protective_stop(
            trajectory.times,
            trajectory.positions,
            LightCurtain(SPEC),
            lambda t: t >= break_time,
            SafetyParameters(category=category),
        )
    assert reports[StopCategory.CATEGORY_0].stopping_time < reports[StopCategory.CATEGORY_1].stopping_time
    assert reports[StopCategory.CATEGORY_0].drives_off_time <= reports[StopCategory.CATEGORY_1].drives_off_time


# -- what the PLC sees -------------------------------------------------------


def test_a_protective_stop_reaches_the_fieldbus_and_a_plc_cannot_reset_it():
    state = FieldbusState()
    state.update({COIL_ENABLE: True})
    assert state.status.state == State.READY

    state.set_safety(SAFETY_PROTECTIVE_STOP | SAFETY_DRIVES_ENABLED, "transfer")
    assert state.status.state == State.FAULTED
    assert state.status.fault == Fault.PROTECTIVE_STOP
    assert state.status.last_fault_stage == 8

    # The PLC pulses fault reset. Nothing happens: the stop is still latched.
    state.update({COIL_ENABLE: True, COIL_FAULT_RESET: True})
    state.update({COIL_ENABLE: True, COIL_FAULT_RESET: False})
    assert state.status.fault == Fault.PROTECTIVE_STOP

    # Curtain clear, still latched, waiting for the reset at the cell.
    state.set_safety(SAFETY_CURTAIN_CLEAR | SAFETY_PROTECTIVE_STOP | SAFETY_RESET_REQUIRED)
    state.update({COIL_ENABLE: True, COIL_FAULT_RESET: True})
    state.update({COIL_ENABLE: True, COIL_FAULT_RESET: False})
    assert state.status.fault == Fault.PROTECTIVE_STOP

    # Reset at the cell: now the PLC's fault reset is accepted.
    state.set_safety(SAFETY_CURTAIN_CLEAR | SAFETY_DRIVES_ENABLED)
    state.update({COIL_ENABLE: True, COIL_FAULT_RESET: True})
    assert state.status.fault == Fault.NONE
    assert state.status.state == State.READY
