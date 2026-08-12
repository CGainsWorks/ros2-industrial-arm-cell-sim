"""The fieldbus, tested over a real socket.

Half of these drive :class:`FieldbusState` directly, because the protocol
behaviour -- edge detection, fault latching, heartbeat rollover -- deserves
tests that cannot be flaky. The other half start the pymodbus server on a
loopback port and talk to it with the real client, because the errors that cost
time on an integration job are transport errors: an address off by one, a value
that wrapped instead of clamping, a command pulse shorter than the scan.
"""

from __future__ import annotations

import asyncio

import pytest

from cell_fieldbus.core.bridge import BridgeConfig, ModbusBridge, ModbusCellClient, join_u32, split_u32
from cell_fieldbus.core.registers import (
    COIL_NAMES,
    REGISTER_NAMES,
    STAGE_CODES,
    COIL_CYCLE_START,
    COIL_ENABLE,
    COIL_FAULT_RESET,
    COIL_PAUSE,
    PROTOCOL_VERSION,
    REG_PROTOCOL_VERSION,
    CellStatus,
    Fault,
    FieldbusState,
    State,
    describe_map,
    fault_for,
)

pytest.importorskip("pymodbus", reason="pymodbus is needed for the fieldbus transport tests")

# Not 5020: a developer running `make demo` on the same machine has the real
# bridge on that port, and a test that fails because the demo is up is a test
# nobody trusts.
TEST_PORT = 55020


# -- protocol, no socket ---------------------------------------------------


def test_cycle_start_is_edge_triggered_not_level():
    """A PLC that leaves the start bit high must not start a second cycle."""
    state = FieldbusState()
    state.update({COIL_ENABLE: True, COIL_CYCLE_START: False})
    state.update({COIL_ENABLE: True, COIL_CYCLE_START: True})
    assert state.take_start_request()

    state.update({COIL_ENABLE: True, COIL_CYCLE_START: True})
    assert not state.take_start_request()

    state.update({COIL_ENABLE: True, COIL_CYCLE_START: False})
    state.update({COIL_ENABLE: True, COIL_CYCLE_START: True})
    assert state.take_start_request()


def test_a_fault_latches_until_it_is_reset():
    state = FieldbusState()
    state.update({COIL_ENABLE: True})
    state.raise_fault(Fault.NO_PART_FOUND, "perception")
    for _ in range(5):
        state.update({COIL_ENABLE: True})
    assert state.status.fault == Fault.NO_PART_FOUND
    assert state.status.state == State.FAULTED

    state.update({COIL_ENABLE: True, COIL_FAULT_RESET: True})
    assert state.status.fault == Fault.NONE
    assert state.status.state != State.FAULTED


def test_starting_while_disabled_raises_a_fault():
    state = FieldbusState()
    state.update({COIL_ENABLE: False, COIL_CYCLE_START: False})
    state.update({COIL_ENABLE: False, COIL_CYCLE_START: True})
    assert state.status.fault == Fault.NOT_ENABLED
    assert not state.take_start_request()


def test_pause_holds_the_cell_without_faulting_it():
    state = FieldbusState()
    state.update({COIL_ENABLE: True})
    state.update({COIL_ENABLE: True, COIL_PAUSE: True})
    assert state.status.state == State.PAUSED
    assert state.status.fault == Fault.NONE


def test_the_heartbeat_advances_and_rolls_over():
    state = FieldbusState()
    state.status.heartbeat = 65534
    for _ in range(4):
        state.update({COIL_ENABLE: True})
    assert state.status.heartbeat == 2


def test_registers_clamp_rather_than_wrap():
    """A 70 second cycle should read as an obviously-wrong 65535, not as a
    plausible 4464."""
    status = CellStatus(last_cycle_ms=70000, cycle_count=100000)
    values = status.to_registers()
    assert values[5] == 65535
    assert values[3] == 65535


def test_the_protocol_version_is_published():
    assert CellStatus().to_registers()[REG_PROTOCOL_VERSION] == PROTOCOL_VERSION


def test_every_cycle_failure_maps_to_a_fault_code():
    """A new failure mode must not arrive at the PLC as 'no fault'."""
    for failure in ("missed", "too_wide", "non_antipodal", "no cluster found"):
        assert fault_for(failure) != Fault.NONE
    assert fault_for("") == Fault.NONE
    with pytest.raises(KeyError):
        fault_for("something nobody has mapped yet")


def test_inspection_fault_codes_match_the_inspection_module():
    """One number, one meaning, on both sides of the bridge."""
    from cell_inspection.core import contact

    assert int(Fault.NO_CONTACT) == contact.FAULT_NO_CONTACT
    assert int(Fault.OVERFORCE) == contact.FAULT_OVERFORCE
    assert int(Fault.NOT_SEATED) == contact.FAULT_NOT_SEATED


def test_u32_word_order_round_trips():
    assert split_u32(0x1234ABCD) == [0x1234, 0xABCD]
    assert join_u32(split_u32(123456789)) == 123456789


def test_the_map_description_covers_coils_and_registers():
    rows = describe_map()
    kinds = {row[0] for row in rows}
    assert kinds == {"coil", "holding"}
    names = {row[2] for row in rows}
    assert {"cycle_start", "fault_reset", "state", "fault_code", "heartbeat"} <= names


# -- transport -------------------------------------------------------------


def _run(coroutine):
    return asyncio.run(coroutine)


def test_a_client_can_drive_a_cycle_over_the_wire():
    async def scenario():
        config = BridgeConfig(port=TEST_PORT)
        started = []
        bridge = ModbusBridge(config)
        await bridge.start(on_start=lambda: started.append(1))
        client = ModbusCellClient(config)
        try:
            await client.connect()

            status = await client.read_status()
            assert status.state == State.IDLE
            assert status.fault == Fault.NONE

            # Enable, then pulse start. The pulse must outlast one scan or the
            # rising edge is never seen -- which is the classic "it starts
            # sometimes" symptom.
            await client.write_coil(COIL_ENABLE, True)
            await asyncio.sleep(0.06)
            assert (await client.read_status()).state == State.READY

            await client.pulse_coil(COIL_CYCLE_START)
            await asyncio.sleep(0.06)
            assert len(started) == 1

            bridge.state.record_cycle(True, 10.31, 0.72, 3)
            bridge.publish(bridge.state.status)
            status = await client.read_status()
            assert status.cycle_count == 1
            assert status.success_count == 1
            assert status.last_cycle_ms == 10310
            assert status.last_grasp_score == 720
            assert status.part_id == 3

            first = (await client.read_status()).heartbeat
            await asyncio.sleep(0.12)
            assert (await client.read_status()).heartbeat != first

            coils = await client.read_coils()
            assert coils[COIL_ENABLE] is True
            assert coils[COIL_CYCLE_START] is False
        finally:
            await client.close()
            await bridge.stop()

    _run(scenario())


def test_a_fault_is_visible_and_clearable_over_the_wire():
    async def scenario():
        config = BridgeConfig(port=TEST_PORT + 1)
        bridge = ModbusBridge(config)
        await bridge.start()
        client = ModbusCellClient(config)
        try:
            await client.connect()
            await client.write_coil(COIL_ENABLE, True)
            await asyncio.sleep(0.05)

            bridge.state.raise_fault(Fault.NOT_SEATED, "inspection")
            bridge.publish(bridge.state.status)
            status = await client.read_status()
            assert status.fault == Fault.NOT_SEATED
            assert status.state == State.FAULTED
            assert status.last_fault_stage == 12

            await client.pulse_coil(COIL_FAULT_RESET)
            await asyncio.sleep(0.06)
            status = await client.read_status()
            assert status.fault == Fault.NONE
            assert status.state != State.FAULTED
        finally:
            await client.close()
            await bridge.stop()

    _run(scenario())


def test_the_fieldbus_document_matches_the_register_map():
    """docs/fieldbus.md is the PLC-facing contract. Assert it is true.

    The document claims to come from the same constants the code uses. Claims
    like that decay: someone renames a register, the code and the tests agree,
    and the document a controls engineer is reading from does not. Two
    documents that disagree about what 40012 means is the most expensive class
    of bug on an integration job, so the tables are parsed and compared here.
    """
    import re
    from pathlib import Path

    document = (Path(__file__).resolve().parent.parent / "docs" / "fieldbus.md").read_text(encoding="utf-8")

    # Rows look like: | 0 | 00001 | `cycle_start` | ... |
    documented = {}
    for match in re.finditer(r"^\|\s*(\d+)\s*\|\s*[04]\d{4}\s*\|\s*`(\w+)`", document, re.M):
        documented.setdefault(int(match.group(1)), set()).add(match.group(2))

    for address, name in COIL_NAMES.items():
        assert name in documented.get(address, set()), f"coil {address} ({name}) is not documented"
    for address, name in REGISTER_NAMES.items():
        assert name in documented.get(address, set()), f"register {address} ({name}) is not documented"

    # Every fault code and state in the enums must appear in the document.
    for fault in Fault:
        assert re.search(rf"^\|\s*{int(fault)}\s*\|\s*{fault.name}\s*\|", document, re.M), (
            f"fault {int(fault)} {fault.name} is missing from docs/fieldbus.md"
        )
    for state in State:
        assert re.search(rf"^\|\s*{int(state)}\s*\|\s*{state.name}\s*\|", document, re.M), (
            f"state {int(state)} {state.name} is missing from docs/fieldbus.md"
        )

    # And the stage codes, which the PLC compares as ordinals.
    for stage, code in STAGE_CODES.items():
        if stage:
            assert re.search(rf"\|\s*{code}\s*\|\s*{stage}\s*\|", document), (
                f"stage {code} {stage} is missing from docs/fieldbus.md"
            )
