"""The Modbus register map, and the state machine behind it.

This is the contract between the cell and the line PLC. It is deliberately in
one file, in one table, with the semantics next to the addresses, because the
single most expensive class of bug on an integration job is two documents that
disagree about what register 40012 means.

DESIGN RULES, AND THE REASONING

* **The PLC writes coils; the cell writes registers.** Nothing is written from
  both ends. A bidirectional word needs an arbitration scheme, and every
  arbitration scheme invented on site at 2 a.m. has a race in it.
* **Commands are edge-triggered, state is level.** ``CYCLE_START`` is taken on
  a rising edge, so a PLC that leaves the bit high does not start a second
  cycle. The cell's ``STATE`` register is a level: read it at any time and it
  tells you where the cell is.
* **A fault latches.** It stays until ``FAULT_RESET`` is pulsed, even if the
  condition clears. A fault that clears itself is a fault nobody sees, and the
  cell does not restart from it on its own.
* **The heartbeat counts, and rolls over at 65535.** A PLC that watches for the
  value *changing*, rather than for a particular value, cannot be fooled by a
  cell that has stopped scanning.
* **Everything is 16-bit and unsigned.** Two registers per 32-bit value, high
  word first, matching the convention almost every PLC library defaults to.
  Times are milliseconds and lengths are micrometres, so nothing needs a float
  and nothing needs a scaling factor written on a Post-it.

REGISTER NUMBERING

Addresses here are zero-based protocol addresses, which is what pymodbus takes.
The PLC-facing documentation in ``docs/fieldbus.md`` gives the conventional
4xxxx equivalents, since that is what an engineer with a Siemens or Rockwell
manual expects, and getting that off by one is the other most expensive class
of bug.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

# -- coils, written by the PLC ---------------------------------------------
COIL_CYCLE_START = 0
COIL_FAULT_RESET = 1
COIL_ENABLE = 2
COIL_PAUSE = 3

COIL_NAMES = {
    COIL_CYCLE_START: "cycle_start",
    COIL_FAULT_RESET: "fault_reset",
    COIL_ENABLE: "enable",
    COIL_PAUSE: "pause",
}

# -- holding registers, written by the cell --------------------------------
REG_STATE = 0
REG_FAULT_CODE = 1
REG_HEARTBEAT = 2
REG_CYCLE_COUNT = 3
REG_SUCCESS_COUNT = 4
REG_LAST_CYCLE_MS = 5
REG_LAST_GRASP_SCORE = 6  # 0-1000, fixed point.
REG_PART_ID = 7
REG_INSPECTION_TILT_MDEG = 8  # Millidegrees.
REG_INSPECTION_RESULT = 9
REG_LAST_FAULT_STAGE = 10
REG_PROTOCOL_VERSION = 11

REGISTER_NAMES = {
    REG_STATE: "state",
    REG_FAULT_CODE: "fault_code",
    REG_HEARTBEAT: "heartbeat",
    REG_CYCLE_COUNT: "cycle_count",
    REG_SUCCESS_COUNT: "success_count",
    REG_LAST_CYCLE_MS: "last_cycle_ms",
    REG_LAST_GRASP_SCORE: "last_grasp_score",
    REG_PART_ID: "part_id",
    REG_INSPECTION_TILT_MDEG: "inspection_tilt_mdeg",
    REG_INSPECTION_RESULT: "inspection_result",
    REG_LAST_FAULT_STAGE: "last_fault_stage",
    REG_PROTOCOL_VERSION: "protocol_version",
}

REGISTER_COUNT = 16
COIL_COUNT = 8
PROTOCOL_VERSION = 2


class State(IntEnum):
    """Cell state, in ``REG_STATE``."""

    IDLE = 0
    READY = 1
    SCANNING = 2
    PLANNING = 3
    PICKING = 4
    TRANSFERRING = 5
    PLACING = 6
    INSPECTING = 7
    COMPLETE = 8
    FAULTED = 9
    PAUSED = 10


class Fault(IntEnum):
    """Fault codes, in ``REG_FAULT_CODE``.

    Grouped by decade so that a PLC can branch on the decade alone: 1x is
    perception, 2x is inspection, 3x is motion, 4x is grasping, 9x is the cell
    itself. The inspection codes deliberately match the constants in
    :mod:`cell_inspection.core.contact` -- the same number means the same thing
    on the fieldbus as it does in the routine that raised it, and a translation
    table between them is a translation table that will be wrong one day.
    """

    NONE = 0

    NO_PART_FOUND = 11
    PART_OUTSIDE_REGION = 12
    CAMERA_TIMEOUT = 13

    NO_CONTACT = 21
    OVERFORCE = 22
    NOT_SEATED = 23

    NO_IK_SOLUTION = 31
    PLANNING_FAILED = 32
    COLLISION_DETECTED = 33
    TRAJECTORY_ABORTED = 34

    NO_GRASP_CANDIDATE = 41
    GRASP_LOST = 42
    PART_TOO_WIDE = 43

    NOT_ENABLED = 91
    ESTOP = 92
    PROTOCOL_MISMATCH = 93


# Mapping from a cycle's failure string to the fault code the PLC sees. Every
# failure the cycle can report has an entry; ``fault_for`` raises rather than
# defaulting, so a new failure mode cannot quietly arrive on the fieldbus as
# "no fault".
FAILURE_TO_FAULT: Dict[str, Fault] = {
    "": Fault.NONE,
    "no cluster found": Fault.NO_PART_FOUND,
    "no feasible grasp candidate": Fault.NO_GRASP_CANDIDATE,
    "missed": Fault.GRASP_LOST,
    "too_wide": Fault.PART_TOO_WIDE,
    "shallow": Fault.GRASP_LOST,
    "non_antipodal": Fault.GRASP_LOST,
    "torque_slip": Fault.GRASP_LOST,
    "finger_collision": Fault.COLLISION_DETECTED,
    "no path found within the iteration budget": Fault.PLANNING_FAILED,
    "start configuration is in collision": Fault.COLLISION_DETECTED,
    "goal configuration is in collision": Fault.COLLISION_DETECTED,
    "no IK solution along the straight line": Fault.NO_IK_SOLUTION,
    "IK branch change along the straight line": Fault.TRAJECTORY_ABORTED,
    "straight line is in collision": Fault.COLLISION_DETECTED,
    "pre-place pose unreachable": Fault.NO_IK_SOLUTION,
}


def fault_for(failure: str) -> Fault:
    """The fault code for a cycle failure string."""
    if failure not in FAILURE_TO_FAULT:
        raise KeyError(
            f"no fault code mapped for cycle failure {failure!r}. "
            "Add it to FAILURE_TO_FAULT rather than letting it reach the PLC as 0."
        )
    return FAILURE_TO_FAULT[failure]


@dataclass
class CellStatus:
    """What the cell publishes. One place, so the map cannot drift."""

    state: State = State.IDLE
    fault: Fault = Fault.NONE
    heartbeat: int = 0
    cycle_count: int = 0
    success_count: int = 0
    last_cycle_ms: int = 0
    last_grasp_score: int = 0
    part_id: int = 0
    inspection_tilt_mdeg: int = 0
    inspection_result: int = 0
    last_fault_stage: int = 0

    def to_registers(self) -> List[int]:
        """Render to the holding-register block, clamped to 16 bits unsigned.

        Clamping rather than wrapping: a cycle time of 70 seconds should read
        as 65535 ms and obviously wrong, not as 4464 ms and plausible.
        """
        values = [0] * REGISTER_COUNT
        values[REG_STATE] = int(self.state)
        values[REG_FAULT_CODE] = int(self.fault)
        values[REG_HEARTBEAT] = self.heartbeat % 65536
        values[REG_CYCLE_COUNT] = _clamp(self.cycle_count)
        values[REG_SUCCESS_COUNT] = _clamp(self.success_count)
        values[REG_LAST_CYCLE_MS] = _clamp(self.last_cycle_ms)
        values[REG_LAST_GRASP_SCORE] = _clamp(self.last_grasp_score)
        values[REG_PART_ID] = _clamp(self.part_id)
        values[REG_INSPECTION_TILT_MDEG] = _clamp(self.inspection_tilt_mdeg)
        values[REG_INSPECTION_RESULT] = _clamp(self.inspection_result)
        values[REG_LAST_FAULT_STAGE] = _clamp(self.last_fault_stage)
        values[REG_PROTOCOL_VERSION] = PROTOCOL_VERSION
        return values

    @classmethod
    def from_registers(cls, values: List[int]) -> "CellStatus":
        return cls(
            state=State(values[REG_STATE]),
            fault=Fault(values[REG_FAULT_CODE]),
            heartbeat=values[REG_HEARTBEAT],
            cycle_count=values[REG_CYCLE_COUNT],
            success_count=values[REG_SUCCESS_COUNT],
            last_cycle_ms=values[REG_LAST_CYCLE_MS],
            last_grasp_score=values[REG_LAST_GRASP_SCORE],
            part_id=values[REG_PART_ID],
            inspection_tilt_mdeg=values[REG_INSPECTION_TILT_MDEG],
            inspection_result=values[REG_INSPECTION_RESULT],
            last_fault_stage=values[REG_LAST_FAULT_STAGE],
        )


def _clamp(value: int) -> int:
    return int(max(0, min(65535, int(value))))


# Stages, for REG_LAST_FAULT_STAGE. Numbered in cycle order so that a PLC can
# compare them: a fault at stage 6 got further than one at stage 3.
STAGE_CODES = {
    "": 0,
    "scan_move": 1,
    "perception": 2,
    "grasp_planning": 3,
    "approach": 4,
    "descend": 5,
    "grasp": 6,
    "lift": 7,
    "transfer": 8,
    "place": 9,
    "retract": 10,
    "retreat": 11,
    "inspection": 12,
    "complete": 13,
}


@dataclass
class FieldbusState:
    """The cell side of the protocol: edge detection, latching, heartbeat.

    Deliberately free of any I/O. It is driven by :meth:`update` from the
    Modbus server in :mod:`cell_fieldbus.core.bridge` and, identically, by the
    tests -- so the protocol behaviour is tested without a socket, and the
    socket is tested without the protocol.
    """

    status: CellStatus = field(default_factory=CellStatus)
    previous_coils: Dict[int, bool] = field(default_factory=dict)
    start_requested: bool = False

    def update(self, coils: Dict[int, bool]) -> None:
        """Consume one scan of the coil block."""
        rising = {
            index: value and not self.previous_coils.get(index, False) for index, value in coils.items()
        }
        self.previous_coils = dict(coils)

        self.status.heartbeat = (self.status.heartbeat + 1) % 65536

        if rising.get(COIL_FAULT_RESET, False):
            self.status.fault = Fault.NONE
            self.status.last_fault_stage = 0
            if self.status.state == State.FAULTED:
                self.status.state = State.IDLE

        if not coils.get(COIL_ENABLE, False):
            # Not enabled is a state, not a fault -- until something asks the
            # cell to run, at which point it becomes one.
            if rising.get(COIL_CYCLE_START, False):
                self.raise_fault(Fault.NOT_ENABLED, "")
            self.status.state = State.IDLE if self.status.state != State.FAULTED else State.FAULTED
            return

        if coils.get(COIL_PAUSE, False):
            self.status.state = State.PAUSED
            return

        if self.status.state in (State.IDLE, State.PAUSED, State.COMPLETE):
            self.status.state = State.READY

        if rising.get(COIL_CYCLE_START, False) and self.status.state == State.READY:
            self.start_requested = True

    def take_start_request(self) -> bool:
        """Consume a pending start request, if there is one."""
        requested, self.start_requested = self.start_requested, False
        return requested

    def raise_fault(self, fault: Fault, stage: str) -> None:
        self.status.fault = fault
        self.status.state = State.FAULTED
        self.status.last_fault_stage = STAGE_CODES.get(stage, 0)

    def record_cycle(self, success: bool, cycle_seconds: float, grasp_score: Optional[float], part_id: int) -> None:
        self.status.cycle_count += 1
        if success:
            self.status.success_count += 1
        self.status.last_cycle_ms = int(round(cycle_seconds * 1000.0))
        self.status.last_grasp_score = int(round((grasp_score or 0.0) * 1000.0))
        self.status.part_id = part_id
        self.status.state = State.COMPLETE if success else State.FAULTED


def describe_map() -> List[Tuple[str, int, str, str]]:
    """The register map as a table, for ``docs/fieldbus.md`` and the CLI.

    Generated from the constants above rather than transcribed, because a
    fieldbus document that disagrees with the code is worse than no document.
    """
    rows: List[Tuple[str, int, str, str]] = []
    for address, name in sorted(COIL_NAMES.items()):
        rows.append(("coil", address, name, "PLC writes, cell reads"))
    for address, name in sorted(REGISTER_NAMES.items()):
        rows.append(("holding", address, name, "cell writes, PLC reads"))
    return rows
