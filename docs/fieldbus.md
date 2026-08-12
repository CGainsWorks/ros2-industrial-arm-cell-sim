# Fieldbus

The contract between the cell and the line PLC. This document is generated from
the same constants the code uses — run `python -c "from cell_fieldbus.core.registers
import describe_map; [print(r) for r in describe_map()]"` to print the live map —
because a fieldbus document that disagrees with the code is worse than no
document.

## Transport

Modbus/TCP, unit ID 1, port 5020 by default. Not 502: binding below 1024 needs
root, and a fieldbus bridge is not a thing to run as root. The bridge scans the
command coils every 20 ms, which is a typical PLC scan period.

**Requires pymodbus 3.6–3.8.** Version 3.9 renamed `ModbusSlaveContext` to
`ModbusDeviceContext` and the `slave=` keyword to `device_id=`, and 4.0 removed
the aliases. The bridge is written against and tested on the 3.6–3.8 API, and
raises an `ImportError` naming the problem on anything else rather than failing
three frames deeper. `requirements.txt` pins the range: the original constraint
was an unpinned `>=3.0`, CI duly installed a 4.x, and the transport tests died
on import — which is the argument for pinning an interface rather than a
numerical library.

## Addressing

Addresses below are **zero-based protocol addresses**, which is what pymodbus
and most Python libraries take. A PLC manual will number the same registers
from 40001 and the same coils from 00001, so the conventional equivalents are
given alongside. Getting this off by one is one of the two most expensive
classes of bug on an integration job.

## Coils — the PLC writes, the cell reads

| Addr | PLC address | Name | Semantics |
|---:|---:|---|---|
| 0 | 00001 | `cycle_start` | **Rising edge.** Starts one cycle. Held high does not start a second. |
| 1 | 00002 | `fault_reset` | **Rising edge.** Clears a latched fault. |
| 2 | 00003 | `enable` | **Level.** Low means the cell will not run. A start while low raises fault 91. |
| 3 | 00004 | `pause` | **Level.** Holds the cell in state 10 without faulting it. |

Commands are edge-triggered because a level-triggered start bit that the PLC
forgets to clear starts cycles forever. The pulse must outlast one scan period
or the edge is never observed; `ModbusCellClient.pulse_coil` holds for 60 ms
against a 20 ms scan. Getting that wrong produces the classic symptom: *the
cell starts sometimes.*

## Holding registers — the cell writes, the PLC reads

| Addr | PLC address | Name | Units |
|---:|---:|---|---|
| 0 | 40001 | `state` | enum, see below |
| 1 | 40002 | `fault_code` | enum, see below |
| 2 | 40003 | `heartbeat` | counts, rolls over at 65535 |
| 3 | 40004 | `cycle_count` | cycles since start |
| 4 | 40005 | `success_count` | held parts since start |
| 5 | 40006 | `last_cycle_ms` | milliseconds |
| 6 | 40007 | `last_grasp_score` | 0–1000, fixed point |
| 7 | 40008 | `part_id` | index into the part catalogue, 1-based |
| 8 | 40009 | `inspection_tilt_mdeg` | millidegrees |
| 9 | 40010 | `inspection_result` | 0 pass, non-zero fault |
| 10 | 40011 | `last_fault_stage` | stage code, see below |
| 11 | 40012 | `protocol_version` | currently 2 |

Nothing is written from both ends. A bidirectional word needs an arbitration
scheme, and every arbitration scheme invented on site at 2 a.m. has a race in
it.

Everything is 16-bit unsigned. Times are milliseconds and lengths are
micrometres, so nothing needs a float and nothing needs a scaling factor
written on a Post-it. Values are **clamped, not wrapped**: a 70-second cycle
reads as 65535 ms and is obviously wrong, rather than as 4464 ms and plausible.

The heartbeat *counts*. A PLC that watches for the value changing, rather than
for a particular value, cannot be fooled by a cell that has stopped scanning.

### 32-bit values

There are none in the current map. `split_u32` and `join_u32` exist anyway,
using **high word first**, which is what almost every PLC library defaults to.
The first thing anyone adds is a 32-bit part counter, and the word order is
worth deciding once rather than discovering.

## States (`state`)

| Value | Name |
|---:|---|
| 0 | IDLE |
| 1 | READY |
| 2 | SCANNING |
| 3 | PLANNING |
| 4 | PICKING |
| 5 | TRANSFERRING |
| 6 | PLACING |
| 7 | INSPECTING |
| 8 | COMPLETE |
| 9 | FAULTED |
| 10 | PAUSED |

## Fault codes (`fault_code`)

Grouped by decade so a PLC can branch on the decade alone.

| Range | Class |
|---|---|
| 1x | perception |
| 2x | inspection |
| 3x | motion |
| 4x | grasping |
| 9x | the cell itself |

| Code | Name | Raised when |
|---:|---|---|
| 0 | NONE | — |
| 11 | NO_PART_FOUND | no cluster in the region of interest |
| 12 | PART_OUTSIDE_REGION | detected, but outside the taught area |
| 13 | CAMERA_TIMEOUT | no frame within the capture window |
| 21 | NO_CONTACT | probe travelled its allowance without touching |
| 22 | OVERFORCE | probe exceeded the 45 N abort threshold |
| 23 | NOT_SEATED | the fitted plane tilts past the acceptance limit |
| 31 | NO_IK_SOLUTION | pose unreachable |
| 32 | PLANNING_FAILED | no path within the iteration budget |
| 33 | COLLISION_DETECTED | start, goal or path in collision |
| 34 | TRAJECTORY_ABORTED | IK branch change along a straight line |
| 41 | NO_GRASP_CANDIDATE | nothing scored above the feasibility floor |
| 42 | GRASP_LOST | the jaws closed and the part did not come |
| 43 | PART_TOO_WIDE | wider than the jaw stroke |
| 91 | NOT_ENABLED | start requested while `enable` was low |
| 92 | ESTOP | reserved; no safety layer is implemented |
| 93 | PROTOCOL_MISMATCH | reserved |

**A fault latches.** It stays until `fault_reset` is pulsed, even if the
condition has cleared. A fault that clears itself is a fault nobody sees, and
the cell does not restart from it on its own.

Codes 21–23 are **the same numbers** as the constants in
`cell_inspection.core.contact`. One number, one meaning, on both sides of the
bridge; a translation table between them is a translation table that will be
wrong one day. `tests/test_fieldbus.py` asserts it.

Every failure string the cycle can produce has an entry in `FAILURE_TO_FAULT`,
and `fault_for` **raises** on an unmapped one rather than defaulting to zero. A
new failure mode must not arrive at the PLC as "no fault".

## Stage codes (`last_fault_stage`)

Numbered in cycle order, so a PLC can compare them: a fault at stage 6 got
further than one at stage 3.

| Code | Stage | | Code | Stage |
|---:|---|---|---:|---|
| 0 | — | | 7 | lift |
| 1 | scan_move | | 8 | transfer |
| 2 | perception | | 9 | place |
| 3 | grasp_planning | | 10 | retract |
| 4 | approach | | 11 | retreat |
| 5 | descend | | 12 | inspection |
| 6 | grasp | | 13 | complete |

## Trying it

```bash
docker compose -f docker/docker-compose.yml up fieldbus
```

then from a Python shell:

```python
import asyncio
from cell_fieldbus.core.bridge import BridgeConfig, ModbusCellClient
from cell_fieldbus.core.registers import COIL_ENABLE, COIL_CYCLE_START

async def main():
    client = ModbusCellClient(BridgeConfig(host="127.0.0.1", port=5020))
    await client.connect()
    await client.write_coil(COIL_ENABLE, True)
    await client.pulse_coil(COIL_CYCLE_START)
    print(await client.read_status())
    await client.close()

asyncio.run(main())
```

## EtherNet/IP

Not implemented. The brief allows an open simulator, and a faithful CIP
implicit-messaging implementation is a project of its own.

If it were added, the equivalent would be two assembly instances carrying the
same content as the tables above: an output assembly of four bits mirroring the
coils, and an input assembly of twelve 16-bit words mirroring the holding
registers, at an RPI of 20 ms to match the scan period. The register
*semantics* — edge-triggered commands, latched faults, a counting heartbeat,
clamped values — are protocol-independent and would carry over unchanged, which
is the reason they live in `cell_fieldbus/core/registers.py` with no transport
in the file at all.

Claiming a protocol that is not implemented would be the one thing worse than
not implementing it.
