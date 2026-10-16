"""EtherNet/IP: the assembly layout, and the cell side of the adapter.

The adapter is OpENer, an open-source EtherNet/IP stack in C, built with the
application in ``cell_fieldbus/enip/adapter/cell_application.c``. OpENer does
the protocol: encapsulation, the connection manager, Class 1 implicit I/O and
Class 3 explicit messaging. This module does the cell: it is the other end of
a loopback UDP link the adapter forwards its output assembly over every 10 ms
tick, and it answers with the status block for the input assembly.

THE LAYOUT, WHICH IS THE CONTRACT

Input assembly 100, T->O, cell to PLC, 32 bytes. Sixteen UINTs, the same
sixteen words as the Modbus holding registers and in the same order, so word
``n`` of the assembly is holding register ``n``. CIP is little-endian, Modbus
is big-endian; the byte order is the transport's business and the word
meanings are identical.

Output assembly 150, O->T, PLC to cell, 4 bytes. Word 0 is the command word,
bit ``n`` being Modbus coil ``n``: bit 0 ``cycle_start``, bit 1
``fault_reset``, bit 2 ``enable``, bit 3 ``pause``. Word 1 is reserved and
must be zero. Commands keep their Modbus semantics: start and reset are taken
on a rising edge, enable and pause are levels.

Configuration assembly 151 is empty. 152 and 153 are the heartbeat assemblies
for input-only and listen-only connections. The requested packet interval is
20 ms, the cell's scan period.

Explicit messaging reaches the same data by Get_Attribute_Single on assembly
100 attribute 3, and the vendor-specific cell diagnostics object, class 0x64
instance 1, carries the headline status words as separate attributes (see
:data:`CELL_OBJECT_ATTRIBUTES`) for a PLC or HMI that wants one value without
the whole block.

WHY THE SEMANTICS STAY IN PYTHON

The edge detection, fault latching and heartbeat are in
:class:`cell_fieldbus.core.registers.FieldbusState`, shared with the Modbus
bridge. Porting them to C beside OpENer would give the cell two
implementations of its PLC contract, one per protocol, which agree until the
day they do not.
"""

from __future__ import annotations

import asyncio
import logging
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from cell_fieldbus.core.registers import (
    COIL_CYCLE_START,
    COIL_ENABLE,
    COIL_FAULT_RESET,
    COIL_NAMES,
    COIL_PAUSE,
    REGISTER_COUNT,
    SAFETY_CURTAIN_CLEAR,
    SAFETY_DRIVES_ENABLED,
    SAFETY_LOCKOUT,
    SAFETY_PROTECTIVE_STOP,
    SAFETY_RESET_REQUIRED,
    STAGE_CODES,
    CellStatus,
    Fault,
    FieldbusState,
    State,
)

LOGGER = logging.getLogger(__name__)

INPUT_ASSEMBLY = 100
OUTPUT_ASSEMBLY = 150
CONFIG_ASSEMBLY = 151
HEARTBEAT_INPUT_ONLY = 152
HEARTBEAT_LISTEN_ONLY = 153

INPUT_SIZE = 2 * REGISTER_COUNT  # 32 bytes.
OUTPUT_SIZE = 4
REQUESTED_PACKET_INTERVAL_MS = 20

ASSEMBLY_CLASS = 0x04
ASSEMBLY_DATA_ATTRIBUTE = 3
CELL_OBJECT_CLASS = 0x64  # Vendor-specific range is 0x64 to 0xC7.
CELL_OBJECT_INSTANCE = 1
CELL_OBJECT_ATTRIBUTES: Dict[int, str] = {
    1: "state",
    2: "fault_code",
    3: "last_fault_stage",
    4: "cycle_count",
    5: "success_count",
    6: "last_cycle_ms",
    7: "safety_status",
    8: "protocol_version",
    9: "heartbeat",
}
# Simulation hooks, USINT, settable. Not part of the PLC contract: a real cell
# wires the curtain and the safety reset to the safety controller.
SIM_CURTAIN_BLOCKED_ATTRIBUTE = 100
SIM_SAFETY_RESET_ATTRIBUTE = 101

# The loopback link between OpENer and this module.
DEFAULT_LINK_PORT = 47100  # This side listens here.
DEFAULT_ADAPTER_PORT = 47101  # The C side listens here.
FRAME_FROM_ADAPTER = struct.Struct("<2sBBB4s")
FRAME_TO_ADAPTER_MAGIC = b"CI"


def pack_input(status: CellStatus) -> bytes:
    """The status block as input assembly 100: sixteen little-endian UINTs."""
    return struct.pack(f"<{REGISTER_COUNT}H", *status.to_registers())


def unpack_input(data: bytes) -> CellStatus:
    if len(data) != INPUT_SIZE:
        raise ValueError(f"input assembly is {INPUT_SIZE} bytes, got {len(data)}")
    return CellStatus.from_registers(list(struct.unpack(f"<{REGISTER_COUNT}H", data)))


def pack_output(coils: Dict[int, bool]) -> bytes:
    """Command bits as output assembly 150: command word, reserved word."""
    word = 0
    for index in COIL_NAMES:
        if coils.get(index, False):
            word |= 1 << index
    return struct.pack("<HH", word, 0)


def unpack_output(data: bytes) -> Dict[int, bool]:
    if len(data) != OUTPUT_SIZE:
        raise ValueError(f"output assembly is {OUTPUT_SIZE} bytes, got {len(data)}")
    word, _reserved = struct.unpack("<HH", data)
    return {index: bool(word & (1 << index)) for index in COIL_NAMES}


def describe_assemblies() -> List[Tuple[int, str, int, str]]:
    """The assembly table for ``docs/fieldbus.md``: instance, direction, size, content."""
    return [
        (INPUT_ASSEMBLY, "input, T->O", INPUT_SIZE, "status block: holding registers 0-15 as UINT"),
        (OUTPUT_ASSEMBLY, "output, O->T", OUTPUT_SIZE, "command word (bit n = coil n), reserved word"),
        (CONFIG_ASSEMBLY, "configuration", 0, "empty"),
        (HEARTBEAT_INPUT_ONLY, "heartbeat", 0, "input-only connections"),
        (HEARTBEAT_LISTEN_ONLY, "heartbeat", 0, "listen-only connections"),
    ]


# Which cycle segment means which state and which stage code, so the PLC sees
# the cell move through the cycle in real time, and a protective stop during a
# segment is reported against the stage it interrupted.
SEGMENT_STATE: Dict[str, Tuple[State, str]] = {
    "cycle_start_handshake": (State.SCANNING, "scan_move"),
    "to_scan": (State.SCANNING, "scan_move"),
    "camera_settle": (State.SCANNING, "perception"),
    "camera_capture": (State.SCANNING, "perception"),
    "to_pregrasp": (State.PICKING, "approach"),
    "descend": (State.PICKING, "descend"),
    "close_gripper": (State.PICKING, "grasp"),
    "lift": (State.PICKING, "lift"),
    "transfer": (State.TRANSFERRING, "transfer"),
    "place": (State.PLACING, "place"),
    "open_gripper": (State.PLACING, "place"),
    "retract": (State.PLACING, "retract"),
    "to_home": (State.PLACING, "retreat"),
}


@dataclass
class EnipCellConfig:
    link_port: int = DEFAULT_LINK_PORT
    adapter_port: int = DEFAULT_ADAPTER_PORT
    scan_period: float = 0.02
    # Wall-clock seconds per simulated second when a cycle is played out. 1.0
    # is real time; the integration test runs faster so a ten-second cycle
    # does not cost ten seconds of CI.
    time_scale: float = 1.0
    # How long the simulated arm takes to stop once a protective stop is
    # requested while it is moving. The stop itself is modelled in
    # cell_safety.core.protective_stop; this is its budget, in simulated time.
    stop_time: float = 0.22


class SimulatedSafety:
    """The light curtain and safety controller behind the fieldbus, simulated.

    Imported lazily so that the fieldbus package does not need cell_safety
    unless the safety layer is actually used.
    """

    def __init__(self) -> None:
        from cell_safety.core.light_curtain import LightCurtain
        from cell_safety.core.protective_stop import SafetyController

        self.curtain = LightCurtain()
        self.controller = SafetyController()

    def step(self, now: float, blocked: bool, reset: bool, standstill: bool) -> int:
        from cell_safety.core.protective_stop import SafetyState

        ossd1, ossd2 = self.curtain.step_blocked(now, blocked)
        outputs = self.controller.step(now, ossd1, ossd2, reset=reset, standstill=standstill)
        word = 0
        if outputs.curtain_clear:
            word |= SAFETY_CURTAIN_CLEAR
        if outputs.stop_requested:
            word |= SAFETY_PROTECTIVE_STOP
        if outputs.reset_required:
            word |= SAFETY_RESET_REQUIRED
        if outputs.drives_enabled:
            word |= SAFETY_DRIVES_ENABLED
        if outputs.state == SafetyState.LOCKOUT:
            word |= SAFETY_LOCKOUT
        return word


class EnipCell:
    """The cell behind the EtherNet/IP adapter.

    ``run_cycle`` is called on a worker thread when the PLC starts a cycle and
    must return a :class:`cell_bringup.core.cycle.CycleResult`-like object
    (``success``, ``failure``, ``stage``, ``segments``, ``grasp``) and a part
    id. The cycle is then *played out* segment by segment in scaled wall-clock
    time, so the PLC watches the state move and a curtain break lands in a
    real segment.
    """

    def __init__(
        self,
        config: Optional[EnipCellConfig] = None,
        run_cycle: Optional[Callable[[int], Tuple[object, int]]] = None,
    ) -> None:
        self.config = config or EnipCellConfig()
        self.state = FieldbusState()
        self.safety = SimulatedSafety()
        self.run_cycle = run_cycle
        self.lock = threading.Lock()
        self.connected = False
        self.frames = 0
        self._coils: Dict[int, bool] = {index: False for index in COIL_NAMES}
        self._sim_blocked = False
        self._sim_reset = False
        self._busy = False
        self._segment = ""
        self._stop_requested_at: Optional[float] = None
        self._cycles_started = 0
        self._transport: Optional[asyncio.DatagramTransport] = None
        self._scan_task: Optional[asyncio.Task] = None
        self._start = time.monotonic()

    # -- the link to the adapter --------------------------------------------

    def datagram_received(self, data: bytes, _address) -> None:
        if len(data) != FRAME_FROM_ADAPTER.size:
            return
        magic, flags, blocked, reset, output = FRAME_FROM_ADAPTER.unpack(data)
        if magic != b"CO":
            return
        with self.lock:
            self.frames += 1
            self.connected = bool(flags & 1)
            self._coils = unpack_output(output) if self.connected else {index: False for index in COIL_NAMES}
            self._sim_blocked = bool(blocked)
            self._sim_reset = bool(reset)

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        cell = self

        class _Protocol(asyncio.DatagramProtocol):
            def datagram_received(self, data, address):
                cell.datagram_received(data, address)

        self._transport, _ = await loop.create_datagram_endpoint(
            _Protocol, local_addr=("127.0.0.1", self.config.link_port)
        )
        self._scan_task = asyncio.create_task(self._scan_loop())

    async def stop(self) -> None:
        if self._scan_task is not None:
            self._scan_task.cancel()
            try:
                await self._scan_task
            except asyncio.CancelledError:
                pass
        if self._transport is not None:
            self._transport.close()

    async def _scan_loop(self) -> None:
        while True:
            self.scan_once()
            await asyncio.sleep(self.config.scan_period)

    # -- one scan -----------------------------------------------------------

    def scan_once(self) -> None:
        now = time.monotonic() - self._start
        with self.lock:
            coils = dict(self._coils)
            blocked, reset = self._sim_blocked, self._sim_reset
            busy = self._busy

            # The arm is at a standstill unless a cycle is moving it; once a
            # stop is requested it takes the stop budget to come to rest.
            if self._stop_requested_at is None:
                standstill = not busy
            else:
                elapsed = (now - self._stop_requested_at) / max(self.config.time_scale, 1e-6)
                standstill = elapsed >= self.config.stop_time
            safety_word = self.safety.step(now, blocked, reset, standstill)
            stopping = bool(safety_word & SAFETY_PROTECTIVE_STOP)
            if stopping and self._stop_requested_at is None:
                self._stop_requested_at = now
            if not stopping:
                self._stop_requested_at = None

            self.state.set_safety(safety_word, self._segment_stage())
            self.state.update(coils)
            if busy and self.state.status.state != State.FAULTED:
                # update() would otherwise report READY over a running cycle.
                self.state.status.state = SEGMENT_STATE.get(self._segment, (State.SCANNING, ""))[0]
            start = self.state.take_start_request() and not busy and not stopping
            if start:
                self._busy = True
                self._segment = "cycle_start_handshake"
                self.state.status.state = State.SCANNING
                self._cycles_started += 1
            frame = FRAME_TO_ADAPTER_MAGIC + pack_input(self.state.status)

        if self._transport is not None:
            self._transport.sendto(frame, ("127.0.0.1", self.config.adapter_port))
        if start:
            threading.Thread(target=self._cycle_worker, daemon=True).start()

    def _segment_stage(self) -> str:
        return SEGMENT_STATE.get(self._segment, (State.SCANNING, ""))[1] if self._busy else ""

    # -- the cycle ----------------------------------------------------------

    def _cycle_worker(self) -> None:
        try:
            if self.run_cycle is None:
                raise RuntimeError("no cycle runner configured")
            result, part_id = self.run_cycle(self._cycles_started)
            interrupted = False
            for segment in result.segments:
                with self.lock:
                    self._segment = segment.name
                deadline = time.monotonic() + segment.duration * self.config.time_scale
                while time.monotonic() < deadline:
                    if self._protective_stop():
                        interrupted = True
                        break
                    time.sleep(0.005)
                if interrupted:
                    break
            with self.lock:
                if interrupted:
                    # The stop already faulted the cell, against this stage.
                    self.state.status.cycle_count += 1
                    self.state.status.last_cycle_ms = 0
                else:
                    seconds = sum(segment.duration for segment in result.segments)
                    score = result.grasp.score if getattr(result, "grasp", None) else 0.0
                    self.state.record_cycle(bool(result.success), seconds, score, part_id)
                    if not result.success:
                        from cell_fieldbus.core.registers import fault_for

                        self.state.raise_fault(fault_for(result.failure), result.stage)
        except Exception:  # noqa: B902 - a crashed cycle must still fault the cell
            LOGGER.exception("cycle failed")
            with self.lock:
                self.state.raise_fault(Fault.PLANNING_FAILED, "")
        finally:
            with self.lock:
                self._busy = False
                self._segment = ""

    def _protective_stop(self) -> bool:
        with self.lock:
            return bool(self.state.status.safety_status & SAFETY_PROTECTIVE_STOP)


def stage_code(stage: str) -> int:
    return STAGE_CODES.get(stage, 0)


__all__ = [
    "COIL_CYCLE_START",
    "COIL_ENABLE",
    "COIL_FAULT_RESET",
    "COIL_PAUSE",
    "EnipCell",
    "EnipCellConfig",
    "pack_input",
    "pack_output",
    "unpack_input",
    "unpack_output",
]
