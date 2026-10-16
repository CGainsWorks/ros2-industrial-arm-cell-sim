"""EtherNet/IP: the assembly layout, the cell behind the adapter, and the wire.

The first half runs anywhere: it checks the assembly layout, that the C
adapter and C++ scanner agree with it, and the cell-side logic with frames fed
in by hand.

The second half is the integration test. It needs the OpENer adapter running
the cell (``ENIP_ADAPTER_HOST``) and the EIPScanner-based client
(``ENIP_IO_SCANNER``), which is what ``docker/enip-compose.yml`` provides and
CI runs. It opens a real Class 1 connection, runs a cycle from the PLC side,
breaks the simulated light curtain mid-cycle, reads status over explicit
messaging with pycomm3, and kills the PLC to check the cell drops its
commands.
"""

from __future__ import annotations

import os
import re
import socket
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import pytest

from cell_fieldbus.core.enip import (
    CELL_OBJECT_ATTRIBUTES,
    CELL_OBJECT_CLASS,
    CONFIG_ASSEMBLY,
    FRAME_FROM_ADAPTER,
    HEARTBEAT_INPUT_ONLY,
    HEARTBEAT_LISTEN_ONLY,
    INPUT_ASSEMBLY,
    INPUT_SIZE,
    OUTPUT_ASSEMBLY,
    OUTPUT_SIZE,
    SIM_CURTAIN_BLOCKED_ATTRIBUTE,
    SIM_SAFETY_RESET_ATTRIBUTE,
    EnipCell,
    EnipCellConfig,
    pack_input,
    pack_output,
    unpack_input,
    unpack_output,
)
from cell_fieldbus.core.registers import (
    COIL_CYCLE_START,
    COIL_ENABLE,
    COIL_FAULT_RESET,
    COIL_PAUSE,
    PROTOCOL_VERSION,
    SAFETY_PROTECTIVE_STOP,
    SAFETY_RESET_REQUIRED,
    CellStatus,
    Fault,
    State,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
ADAPTER_SOURCE = REPO_ROOT / "src" / "cell_fieldbus" / "enip" / "adapter" / "cell_application.c"
SCANNER_SOURCE = REPO_ROOT / "src" / "cell_fieldbus" / "enip" / "scanner" / "io_scanner.cpp"

ENABLE = 1 << COIL_ENABLE
START = 1 << COIL_CYCLE_START
RESET = 1 << COIL_FAULT_RESET


# -- the layout --------------------------------------------------------------


def test_the_assemblies_round_trip():
    status = CellStatus(state=State.TRANSFERRING, fault=Fault.NONE, heartbeat=65535, cycle_count=7,
                        last_cycle_ms=9880, safety_status=0b1001)
    data = pack_input(status)
    assert len(data) == INPUT_SIZE == 32
    assert unpack_input(data) == status
    # Little-endian UINTs, word n = holding register n.
    assert data[0:2] == struct.pack("<H", int(State.TRANSFERRING))
    assert data[10:12] == struct.pack("<H", 9880)

    coils = {COIL_CYCLE_START: True, COIL_FAULT_RESET: False, COIL_ENABLE: True, COIL_PAUSE: False}
    output = pack_output(coils)
    assert len(output) == OUTPUT_SIZE == 4
    assert struct.unpack("<HH", output) == (START | ENABLE, 0)
    assert unpack_output(output) == coils


def _defines(path: Path) -> dict:
    return {name: int(value, 0) for name, value in re.findall(r"#define (\w+) (0x[0-9A-Fa-f]+|\d+)\b",
                                                              path.read_text(encoding="utf-8"))}


def test_the_c_adapter_agrees_with_the_layout():
    defines = _defines(ADAPTER_SOURCE)
    assert defines["CELL_INPUT_ASSEMBLY"] == INPUT_ASSEMBLY
    assert defines["CELL_OUTPUT_ASSEMBLY"] == OUTPUT_ASSEMBLY
    assert defines["CELL_CONFIG_ASSEMBLY"] == CONFIG_ASSEMBLY
    assert defines["CELL_HEARTBEAT_INPUT_ONLY"] == HEARTBEAT_INPUT_ONLY
    assert defines["CELL_HEARTBEAT_LISTEN_ONLY"] == HEARTBEAT_LISTEN_ONLY
    assert defines["CELL_INPUT_SIZE"] == INPUT_SIZE
    assert defines["CELL_OUTPUT_SIZE"] == OUTPUT_SIZE
    assert defines["CELL_OBJECT_CLASS"] == CELL_OBJECT_CLASS

    source = ADAPTER_SOURCE.read_text(encoding="utf-8")
    attributes = {int(number): name for number, name in
                  re.findall(r"InsertAttribute\(instance, (\d+), kCipUint, EncodeCipUint, NULL, &g_attr_(\w+)", source)}
    expected = {number: name.replace("fault_code", "fault") for number, name in CELL_OBJECT_ATTRIBUTES.items()}
    assert attributes == expected
    assert f"InsertAttribute(instance, {SIM_CURTAIN_BLOCKED_ATTRIBUTE}," in source
    assert f"InsertAttribute(instance, {SIM_SAFETY_RESET_ATTRIBUTE}," in source
    # The word indices the adapter reads the diagnostics from.
    assert defines["WORD_SAFETY_STATUS"] == 12 and defines["WORD_PROTOCOL_VERSION"] == 11


def test_the_scanner_client_agrees_with_the_layout():
    source = SCANNER_SOURCE.read_text(encoding="utf-8")
    constants = {name: int(value) for name, value in re.findall(r"constexpr int (\w+) = (\d+);", source)}
    assert constants["kInputAssembly"] == INPUT_ASSEMBLY
    assert constants["kOutputAssembly"] == OUTPUT_ASSEMBLY
    assert constants["kConfigAssembly"] == CONFIG_ASSEMBLY
    assert constants["kInputSize"] == INPUT_SIZE
    assert constants["kOutputSize"] == OUTPUT_SIZE
    assert constants["kRpiMicroseconds"] == 20000


# -- the cell behind the adapter, frames fed by hand -------------------------


@dataclass
class _Segment:
    name: str
    duration: float


@dataclass
class _Result:
    success: bool
    failure: str
    stage: str
    segments: List[_Segment]
    grasp: Optional[object] = None


def _frame(word: int, connected: bool = True, blocked: bool = False, reset: bool = False) -> bytes:
    return FRAME_FROM_ADAPTER.pack(b"CO", int(connected), int(blocked), int(reset), struct.pack("<HH", word, 0))


def _scan(cell: EnipCell, frame: bytes, scans: int = 1) -> CellStatus:
    for _ in range(scans):
        cell.datagram_received(frame, None)
        cell.scan_once()
        time.sleep(0.003)
    return cell.state.status


def _cell(segments, success=True, failure=""):
    result = _Result(success, failure, "complete" if success else "transfer", segments)
    return EnipCell(EnipCellConfig(time_scale=1.0), lambda index: (result, 3))


def test_a_cycle_started_over_the_assembly_is_played_out_and_recorded():
    cell = _cell([_Segment("to_scan", 0.05), _Segment("transfer", 0.05)])
    assert _scan(cell, _frame(ENABLE), 2).state == State.READY
    _scan(cell, _frame(ENABLE | START))
    status = _scan(cell, _frame(ENABLE))
    assert status.state in (State.SCANNING, State.TRANSFERRING)
    deadline = time.time() + 2.0
    while cell.state.status.cycle_count == 0 and time.time() < deadline:
        _scan(cell, _frame(ENABLE))
    status = cell.state.status
    assert status.cycle_count == 1 and status.success_count == 1
    assert status.last_cycle_ms == 100 and status.part_id == 3


def test_a_lost_connection_means_no_commands():
    cell = _cell([_Segment("to_scan", 0.01)])
    assert _scan(cell, _frame(ENABLE), 2).state == State.READY
    # The adapter reports the connection gone and forwards zeros; even if the
    # bytes it forwards were stale, they are ignored.
    assert _scan(cell, _frame(ENABLE | START, connected=False), 2).state == State.IDLE
    assert cell.state.status.cycle_count == 0


def test_a_curtain_break_mid_cycle_is_a_protective_stop_on_the_wire():
    cell = _cell([_Segment("to_scan", 0.02), _Segment("transfer", 2.0)])
    _scan(cell, _frame(ENABLE), 2)
    _scan(cell, _frame(ENABLE | START))
    deadline = time.time() + 2.0
    while cell.state.status.state != State.TRANSFERRING and time.time() < deadline:
        _scan(cell, _frame(ENABLE))
    status = _scan(cell, _frame(ENABLE, blocked=True), 15)  # Past the 20 ms response time.
    assert status.fault == Fault.PROTECTIVE_STOP
    assert status.state == State.FAULTED
    assert status.last_fault_stage == 8  # transfer
    assert status.safety_status & SAFETY_PROTECTIVE_STOP

    # The PLC cannot reset it, blocked or clear.
    _scan(cell, _frame(ENABLE | RESET, blocked=True), 2)
    # Clear: the arm finishes stopping (0.22 s simulated), then waits for the
    # reset at the cell.
    deadline = time.time() + 3.0
    while not cell.state.status.safety_status & SAFETY_RESET_REQUIRED and time.time() < deadline:
        _scan(cell, _frame(ENABLE))
    assert cell.state.status.safety_status & SAFETY_RESET_REQUIRED
    assert cell.state.status.fault == Fault.PROTECTIVE_STOP
    _scan(cell, _frame(ENABLE | RESET), 2)
    assert _scan(cell, _frame(ENABLE), 2).fault == Fault.PROTECTIVE_STOP

    # The reset at the cell, then the PLC's fault reset.
    _scan(cell, _frame(ENABLE, reset=True), 2)
    status = _scan(cell, _frame(ENABLE), 2)
    assert not status.safety_status & SAFETY_PROTECTIVE_STOP
    _scan(cell, _frame(ENABLE | RESET), 2)
    status = _scan(cell, _frame(ENABLE), 2)
    assert status.fault == Fault.NONE and status.state == State.READY


# -- over the wire, against OpENer -------------------------------------------

ADAPTER_HOST = os.environ.get("ENIP_ADAPTER_HOST")
IO_SCANNER = os.environ.get("ENIP_IO_SCANNER")

wire = pytest.mark.skipif(
    not (ADAPTER_HOST and IO_SCANNER),
    reason="needs the OpENer adapter and the EIPScanner client; run docker/enip-compose.yml",
)


@pytest.fixture(scope="module")
def adapter_ip():
    ip = socket.gethostbyname(ADAPTER_HOST)
    deadline = time.time() + 90.0
    while time.time() < deadline:
        try:
            with socket.create_connection((ip, 44818), timeout=1.0):
                break
        except OSError:
            time.sleep(0.5)
    else:
        pytest.fail(f"nothing listening on {ip}:44818")
    time.sleep(1.0)  # Let the cell side's first frames reach the adapter.
    return ip


class IoScanner:
    """The EIPScanner client as a subprocess: a PLC holding a Class 1 connection."""

    def __init__(self, ip: str) -> None:
        self.process = subprocess.Popen(
            [IO_SCANNER, ip], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
        self.lines: List[str] = []
        self.latest: Optional[CellStatus] = None
        self.latest_words: Optional[tuple] = None
        self.frames = 0
        self.opened = threading.Event()
        self.closed = threading.Event()
        threading.Thread(target=self._read, daemon=True).start()
        assert self.opened.wait(15.0), "the Class 1 connection did not open:\n" + "\n".join(self.lines)

    def _read(self) -> None:
        for line in self.process.stdout:
            line = line.rstrip()
            self.lines.append(line)
            if line == "@@ open":
                self.opened.set()
            elif line.startswith("@@ in "):
                data = bytes.fromhex(line[6:])
                self.latest_words = struct.unpack("<16H", data)
                self.latest = unpack_input(data)
                self.frames += 1
            elif line in ("@@ closed", "@@ failed"):
                self.closed.set()
                self.opened.set()

    def send(self, word: int) -> None:
        self.process.stdin.write(f"out {struct.pack('<HH', word, 0).hex()}\n")
        self.process.stdin.flush()

    def pulse(self, held: int, bit: int, hold: float = 0.1) -> None:
        self.send(held | bit)
        time.sleep(hold)
        self.send(held)

    def wait_for(self, predicate, timeout: float = 10.0, what: str = "") -> CellStatus:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.latest is not None and predicate(self.latest):
                return self.latest
            time.sleep(0.01)
        raise AssertionError(f"timed out waiting for {what}; last status {self.latest}")

    def close(self, graceful: bool = True) -> None:
        if self.process.poll() is None:
            self.process.stdin.write("close\n" if graceful else "quit\n")
            self.process.stdin.flush()
            self.process.wait(timeout=10)
        self.closed.wait(5.0)


def _explicit(ip: str):
    from pycomm3 import CIPDriver

    driver = CIPDriver(ip)
    driver.open()
    return driver


def _get(driver, class_code: int, instance: int, attribute: int) -> bytes:
    from pycomm3 import Services

    response = driver.generic_message(
        service=Services.get_attribute_single, class_code=class_code, instance=instance, attribute=attribute,
        connected=False, unconnected_send=False, route_path=False, name="get",
    )
    assert response.error is None, response.error
    return response.value


def _set(driver, class_code: int, instance: int, attribute: int, data: bytes) -> None:
    from pycomm3 import Services

    response = driver.generic_message(
        service=Services.set_attribute_single, class_code=class_code, instance=instance, attribute=attribute,
        request_data=data, connected=False, unconnected_send=False, route_path=False, name="set",
    )
    assert response.error is None, response.error


def _ready(scanner: IoScanner) -> CellStatus:
    """Enable, clear any fault left by an earlier test, and wait for READY."""
    scanner.send(ENABLE)
    scanner.pulse(ENABLE, RESET)
    return scanner.wait_for(lambda s: s.state == State.READY, what="READY")


@wire
def test_the_adapter_identifies_itself_over_explicit_messaging(adapter_ip):
    driver = _explicit(adapter_ip)
    try:
        vendor = struct.unpack("<H", _get(driver, 0x01, 1, 1))[0]
        name_bytes = _get(driver, 0x01, 1, 7)
        product = name_bytes[1:1 + name_bytes[0]].decode("ascii")
        version = struct.unpack("<H", _get(driver, CELL_OBJECT_CLASS, 1, 8))[0]
    finally:
        driver.close()
    assert vendor == 65535
    assert product == "Arm cell (simulated)"
    assert version == PROTOCOL_VERSION


@wire
def test_a_plc_runs_a_cycle_over_implicit_io(adapter_ip):
    scanner = IoScanner(adapter_ip)
    try:
        status = scanner.wait_for(lambda s: True, what="the first input frame")
        assert scanner.latest_words[11] == PROTOCOL_VERSION
        first_heartbeat = status.heartbeat
        scanner.wait_for(lambda s: s.heartbeat != first_heartbeat, what="the heartbeat to count")

        before = _ready(scanner).cycle_count
        scanner.pulse(ENABLE, START)
        scanner.wait_for(lambda s: s.state not in (State.READY, State.IDLE), what="the cycle to start")
        status = scanner.wait_for(lambda s: s.cycle_count == before + 1, timeout=60.0, what="the cycle to finish")
        assert status.last_cycle_ms > 0

        # Explicit messaging sees the same block as the implicit connection.
        driver = _explicit(adapter_ip)
        try:
            explicit = unpack_input(_get(driver, 0x04, INPUT_ASSEMBLY, 3))
            cycles = struct.unpack("<H", _get(driver, CELL_OBJECT_CLASS, 1, 4))[0]
        finally:
            driver.close()
        assert explicit.cycle_count == status.cycle_count == cycles
        assert explicit.last_cycle_ms == status.last_cycle_ms
    finally:
        scanner.close()


@wire
def test_breaking_the_curtain_mid_cycle_stops_the_cell_and_the_plc_cannot_reset_it(adapter_ip):
    scanner = IoScanner(adapter_ip)
    driver = _explicit(adapter_ip)
    try:
        _ready(scanner)
        scanner.pulse(ENABLE, START)
        scanner.wait_for(lambda s: s.state in (State.PICKING, State.TRANSFERRING, State.PLACING),
                         timeout=60.0, what="the arm to be moving a part")

        _set(driver, CELL_OBJECT_CLASS, 1, SIM_CURTAIN_BLOCKED_ATTRIBUTE, b"\x01")
        status = scanner.wait_for(lambda s: s.fault == Fault.PROTECTIVE_STOP, timeout=2.0, what="fault 94")
        assert status.state == State.FAULTED
        assert status.safety_status & SAFETY_PROTECTIVE_STOP
        assert status.last_fault_stage > 0

        scanner.pulse(ENABLE, RESET)
        time.sleep(0.2)
        assert scanner.latest.fault == Fault.PROTECTIVE_STOP, "a PLC fault reset cleared a protective stop"

        _set(driver, CELL_OBJECT_CLASS, 1, SIM_CURTAIN_BLOCKED_ATTRIBUTE, b"\x00")
        scanner.wait_for(lambda s: s.safety_status & SAFETY_RESET_REQUIRED, what="reset required")
        scanner.pulse(ENABLE, RESET)
        time.sleep(0.2)
        assert scanner.latest.fault == Fault.PROTECTIVE_STOP

        _set(driver, CELL_OBJECT_CLASS, 1, SIM_SAFETY_RESET_ATTRIBUTE, b"\x01")
        time.sleep(0.1)
        _set(driver, CELL_OBJECT_CLASS, 1, SIM_SAFETY_RESET_ATTRIBUTE, b"\x00")
        scanner.wait_for(lambda s: not s.safety_status & SAFETY_PROTECTIVE_STOP, what="the safety reset")
        scanner.pulse(ENABLE, RESET)
        status = scanner.wait_for(lambda s: s.fault == Fault.NONE and s.state == State.READY, what="READY again")
        assert status.fault == Fault.NONE
    finally:
        driver.close()
        scanner.close()


@wire
def test_a_plc_that_disappears_takes_its_commands_with_it(adapter_ip):
    scanner = IoScanner(adapter_ip)
    _ready(scanner)
    scanner.close(graceful=False)  # No Forward_Close: the adapter has to time it out.

    # When the connection times out, OpENer also closes the encapsulation
    # sessions from the same originator address, so an explicit session
    # opened in the meantime is dropped with it. Retry with a fresh one.
    from pycomm3.exceptions import CommError

    deadline = time.time() + 10.0
    state = None
    while time.time() < deadline:
        try:
            driver = _explicit(adapter_ip)
            try:
                state = struct.unpack("<H", _get(driver, CELL_OBJECT_CLASS, 1, 1))[0]
            finally:
                try:
                    driver.close()
                except CommError:
                    pass
        except CommError:
            state = None
        if state == State.IDLE:
            break
        time.sleep(0.1)
    assert state == State.IDLE, f"the cell is still {state} after its PLC went away"
