# Fieldbus

The contract between the cell and the line PLC, over two transports: Modbus
TCP and EtherNet/IP. Both carry the same words with the same meanings, from
one table in `cell_fieldbus/core/registers.py`. Print the live map with
`python -c "from cell_fieldbus.core.registers import describe_map; [print(r) for r in describe_map()]"`.
`tests/test_fieldbus.py` parses the tables below and fails if they disagree
with the code, because a fieldbus document that disagrees with the code is
worse than no document.

## Transports

**Modbus TCP**, unit ID 1, port 5020 by default. Not 502: binding below 1024
needs root, and a fieldbus bridge is not a thing to run as root. The bridge
scans the command coils every 20 ms, which is a typical PLC scan period.
pymodbus 3.6 to 3.8 (`requirements.txt` pins it: 3.9 renamed
`ModbusSlaveContext` and the `slave=` keyword, and 4.0 removed the aliases).

**EtherNet/IP**, an adapter (target) on TCP and UDP 44818 with implicit I/O on
UDP 2222. The stack is [OpENer](https://github.com/EIPStackGroup/OpENer), built
from a pinned commit with the cell's application in
`cell_fieldbus/enip/adapter/cell_application.c`. See
[EtherNet/IP](#ethernetip) below for the assembly layout.

## Addressing (Modbus)

Addresses below are **zero-based protocol addresses**, which is what pymodbus
and most Python libraries take. A PLC manual will number the same registers
from 40001 and the same coils from 00001, so the conventional equivalents are
given alongside. Getting this off by one is one of the two most expensive
classes of bug on an integration job.

## Coils: the PLC writes, the cell reads

| Addr | PLC address | Name | Semantics |
|---:|---:|---|---|
| 0 | 00001 | `cycle_start` | **Rising edge.** Starts one cycle. Held high does not start a second. |
| 1 | 00002 | `fault_reset` | **Rising edge.** Clears a latched fault, except a protective stop that is still latched. |
| 2 | 00003 | `enable` | **Level.** Low means the cell will not run. A start while low raises fault 91. |
| 3 | 00004 | `pause` | **Level.** Holds the cell in state 10 without faulting it. |

Commands are edge-triggered because a level-triggered start bit that the PLC
forgets to clear starts cycles forever. The pulse must outlast one scan period
or the edge is never observed; `ModbusCellClient.pulse_coil` holds for 60 ms
against a 20 ms scan. Getting that wrong produces the classic symptom: *the
cell starts sometimes.*

## Holding registers: the cell writes, the PLC reads

| Addr | PLC address | Name | Units |
|---:|---:|---|---|
| 0 | 40001 | `state` | enum, see below |
| 1 | 40002 | `fault_code` | enum, see below |
| 2 | 40003 | `heartbeat` | counts, rolls over at 65535 |
| 3 | 40004 | `cycle_count` | cycles since start |
| 4 | 40005 | `success_count` | held parts since start |
| 5 | 40006 | `last_cycle_ms` | milliseconds |
| 6 | 40007 | `last_grasp_score` | 0 to 1000, fixed point |
| 7 | 40008 | `part_id` | index into the part catalogue, 1-based |
| 8 | 40009 | `inspection_tilt_mdeg` | millidegrees |
| 9 | 40010 | `inspection_result` | 0 pass, non-zero fault |
| 10 | 40011 | `last_fault_stage` | stage code, see below |
| 11 | 40012 | `protocol_version` | currently 3 |
| 12 | 40013 | `safety_status` | bit field, see below |

Nothing is written from both ends. A bidirectional word needs an arbitration
scheme, and every arbitration scheme invented on site at 2 a.m. has a race in
it.

Everything is 16-bit unsigned. Times are milliseconds and lengths are
micrometres, so nothing needs a float and nothing needs a scaling factor
written on a Post-it. Values are **clamped, not wrapped**: a 70-second cycle
reads as 65535 ms and is obviously wrong, rather than as 4464 ms and plausible.

The heartbeat *counts*. A PLC that watches for the value changing, rather than
for a particular value, cannot be fooled by a cell that has stopped scanning.

Protocol version 3 added `safety_status` and fault 94. Version 2 had twelve
registers and no safety layer.

### 32-bit values

There are none in the current map. `split_u32` and `join_u32` exist anyway,
using **high word first**, which is what almost every PLC library defaults to.
The first thing anyone adds is a 32-bit part counter, and the word order is
worth deciding once rather than discovering.

## Safety status (`safety_status`)

Reported, not controlled: the safety function is in `cell_safety` and is
simulated, not certified ([safety.md](safety.md)).

| Bit | Value | Meaning |
|---:|---:|---|
| 0 | 1 | curtain clear: both OSSD channels on |
| 1 | 2 | protective stop latched |
| 2 | 4 | curtain clear again, waiting for the manual reset at the cell |
| 3 | 8 | drives enabled (category 1: on while braking, off at standstill) |
| 4 | 16 | lockout: the two OSSD channels disagreed |

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
| 0 | NONE | no fault |
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
| 92 | ESTOP | reserved for an emergency stop circuit, which is not modelled |
| 93 | PROTOCOL_MISMATCH | reserved |
| 94 | PROTECTIVE_STOP | the light curtain was broken; simulated, see [safety.md](safety.md) |

**A fault latches.** It stays until `fault_reset` is pulsed, even if the
condition has cleared. A fault that clears itself is a fault nobody sees, and
the cell does not restart from it on its own.

**Fault 94 latches twice.** The safety controller latches the stop, and only
the manual reset at the cell clears that. Until it has, `fault_reset` from the
PLC is ignored. Neither Modbus nor standard EtherNet/IP is a safety protocol,
so neither may reset a safety function.

Codes 21 to 23 are **the same numbers** as the constants in
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
| 0 | none | | 7 | lift |
| 1 | scan_move | | 8 | transfer |
| 2 | perception | | 9 | place |
| 3 | grasp_planning | | 10 | retract |
| 4 | approach | | 11 | retreat |
| 5 | descend | | 12 | inspection |
| 6 | grasp | | 13 | complete |

## EtherNet/IP

The cell is an EtherNet/IP **adapter**. The PLC is the **scanner** and owns
the I/O connection. OpENer does the protocol: encapsulation, the connection
manager, Class 1 implicit I/O, Class 3 and unconnected explicit messaging, and
the identity, TCP/IP and Ethernet link objects. The cell's semantics stay in
Python, shared with the Modbus bridge: the adapter forwards its output
assembly to `cell_fieldbus/core/enip.py` over a loopback socket every 10 ms
and copies back the status block, so there is one implementation of the PLC
contract and not one per protocol.

### Assemblies

| Instance | Direction | Size | Content |
|---:|---|---:|---|
| 100 | input, T->O (cell to PLC) | 32 bytes | the 16 holding registers above, as UINT, word *n* = register *n* |
| 150 | output, O->T (PLC to cell) | 4 bytes | command word, bit *n* = coil *n*; reserved word, zero |
| 151 | configuration | 0 | empty |
| 152 | heartbeat | 0 | for input-only connections |
| 153 | heartbeat | 0 | for listen-only connections |

Requested packet interval 20 ms each way. The O->T data carries the 32-bit
run/idle header. CIP is little-endian where Modbus is big-endian; the bytes
differ and the words are identical.

Output word 0, bit by bit:

| Bit | Coil | Command |
|---:|---:|---|
| 0 | 0 | `cycle_start`, rising edge |
| 1 | 1 | `fault_reset`, rising edge |
| 2 | 2 | `enable`, level |
| 3 | 3 | `pause`, level |

Connection points: exclusive owner 150/100 with configuration 151; input only
152/100; listen only 153/100.

**A lost connection means no commands.** If the exclusive-owner connection
times out or is closed, or the PLC goes to Idle (Program mode), the adapter
tells the cell every command bit is zero: `enable` drops and the cell goes to
IDLE. Holding the last outputs is how a cell keeps running after the PLC that
was supervising it has gone away. `tests/test_enip.py` kills the scanner
without a Forward_Close and checks the cell is IDLE afterwards.

### Explicit messaging

| Path | Service | Gives |
|---|---|---|
| class 0x04, instance 100, attribute 3 | Get_Attribute_Single | the whole input assembly |
| class 0x01, instance 1 | Get_Attribute_Single / All | identity: vendor ID 65535 (a placeholder, not an ODVA assignment), device type 43, product name `Arm cell (simulated)` |
| class 0x64, instance 1, attributes 1 to 9 | Get_Attribute_Single / All | vendor-specific cell diagnostics, one UINT each |

Cell diagnostics object, class 0x64 instance 1:

| Attribute | Name |
|---:|---|
| 1 | state |
| 2 | fault_code |
| 3 | last_fault_stage |
| 4 | cycle_count |
| 5 | success_count |
| 6 | last_cycle_ms |
| 7 | safety_status |
| 8 | protocol_version |
| 9 | heartbeat |
| 100 | *simulation only*: curtain blocked (USINT, settable) |
| 101 | *simulation only*: safety reset button (USINT, settable) |

Attributes 100 and 101 exist so the integration test can break the simulated
curtain from the scanner's side of the network. A real cell wires the curtain
and the reset button to its safety controller, and nothing on a standard
EtherNet/IP network can reach either.

### How it is tested

`docker/enip-compose.yml` runs two containers from `docker/enip.Dockerfile` on
one network, a PLC and a cell:

- **cell:** OpENer built with the cell application, plus
  `scripts/enip_cell.py` running the offline pick and place cycle behind it.
  A cycle is planned in full when the PLC starts it, then played out segment
  by segment in scaled wall-clock time, so the PLC sees the state move through
  the cycle and a curtain break lands in a real segment.
- **scanner:** a small client on
  [EIPScanner](https://github.com/nimbuscontrols/EIPScanner) (C++) holding a
  real Class 1 exclusive-owner connection, and
  [pycomm3](https://github.com/ottowayi/pycomm3) for explicit messaging, both
  driven by `tests/test_enip.py`.

The tests read the identity, run a cycle from the scanner over implicit I/O
and compare the result with what explicit messaging reads, break the curtain
mid-cycle and check the stop cannot be reset from the PLC, and kill the
scanner to check the commands go with it. CI runs it on every push, and
`make enip-test` runs it locally.

What it does not show: interoperability with a commercial PLC or with ODVA's
conformance test. EIPScanner and pycomm3 are independent implementations of
the scanner side, which is better than testing OpENer against itself, and is
still not a Rockwell or Siemens controller on the other end.

## Talking to the cell from a PLC

### Over EtherNet/IP

In the PLC's I/O configuration, add a generic EtherNet/IP adapter at the
cell's IP address (in Studio 5000, a *Generic Ethernet Module*; in CODESYS, a
generic EtherNet/IP adapter under the scanner) with:

| Setting | Value |
|---|---|
| Connection | exclusive owner |
| Input (T->O) | assembly 100, 32 bytes (16 INT/UINT) |
| Output (O->T) | assembly 150, 4 bytes (2 INT/UINT) |
| Configuration | assembly 151, 0 bytes |
| RPI | 20 ms |
| Electronic keying | disabled (the identity is a placeholder) |

Map the 32 input bytes and 4 output bytes into a free area of the process
image and the controller program works on words. The handshake, in IEC
61131-3 Structured Text, assuming `CellIn : ARRAY[0..15] OF UINT` and
`CellOut : ARRAY[0..1] OF WORD` are mapped onto the two assemblies:

```iecst
(* Enable while the line runs; start on request when READY. *)
CellOut[0].2 := xLineRunning;                                (* enable *)
CellOut[0].0 := xRequestPick AND (CellIn[0] = 1);            (* cycle_start: READY is 1 *)

(* Heartbeat: fault the line if word 2 has not changed for 500 ms. *)
tonHeartbeat(IN := CellIn[2] = uiLastHeartbeat, PT := T#500MS);
uiLastHeartbeat := CellIn[2];
xCellLost := tonHeartbeat.Q;

(* Faults by decade; 94 is the light curtain and cannot be reset from here. *)
xCellFault := CellIn[1] <> 0;
xCellProtectiveStop := (CellIn[12] AND 16#0002) <> 0;
CellOut[0].1 := xOperatorReset AND xCellFault AND NOT xCellProtectiveStop;  (* fault_reset *)
```

Start and fault reset are edge-triggered on the cell side, so a bit held high
does not start a second cycle; drop it when the cell leaves READY.

### The sibling PLC project

[CGainsWorks/plc-cip-batch-control](https://github.com/CGainsWorks/plc-cip-batch-control)
is a PLC project: IEC 61131-3 Structured Text for a **clean-in-place** skid.
The "CIP" in its name is clean-in-place, not the Common Industrial Protocol.
It speaks **Modbus TCP**, not EtherNet/IP, and it has no cell interface
today. What follows is how its controller *would* supervise this cell; none
of it is implemented in that repository.

- **Over EtherNet/IP**, on a runtime with an EtherNet/IP scanner (CODESYS
  has one), the two assemblies above would be mapped into a free area of its
  process image. Its current map (its `docs/io_map.md`) uses `%IX0.0` to
  `%IX0.7`, `%ID0` to `%ID12`, `%QX2.0` to `%QX3.6` and `%QD16` to `%QD17`,
  so a block such as `%IB200` to `%IB231` for the cell's input assembly and
  `%QB200` to `%QB203` for its output assembly would collide with nothing.
  Those addresses are a proposal, not part of either project.
- **Over Modbus TCP**, which its harness already speaks, its controller would
  be the *client* of this cell's Modbus server on port 5020: write coils 0 to
  3, read holding registers 0 to 12. Its own Modbus server, on the same port
  number on its own host, carries its skid I/O (coils 16 to 30 for the
  outputs, holding registers 32 to 35 for the analogue outputs) and is
  unrelated to this cell's map.

Either way the handshake is the one above. The two projects share no code;
what they share is the convention that the PLC writes commands, the device
writes status, and nothing is written from both ends.

## Trying it

Modbus:

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

EtherNet/IP:

```bash
make enip-test     # builds OpENer and EIPScanner, runs the cell against the scanner
```
