# Safety: the light curtain and the protective stop

> **Simulated, not certified.** Nothing here is a safety function in the sense
> of ISO 13849-1 or IEC 62061, and nothing here supports a performance level,
> a safety integrity level or a CE mark. It models how a light curtain and a
> protective stop *behave*, so that the rest of the cell can be designed and
> tested against that behaviour. A real cell gets its protective stop from
> certified hardware, validated on the machine.

The README's cell drawing has always shown a light curtain. Until this
change nothing in `src/` modelled one. Now the curtain is in the cell table,
in the Gazebo world, in a model with beams and OSSD outputs, and in a
protective stop that halts the arm and is tested three ways: offline against
trajectories the planner produced, live against `ros2_control`, and over
EtherNet/IP as the PLC sees it.

## What is modelled

| Piece | Where | What it does |
|---|---|---|
| Curtain geometry | `simulation/generate_cell.py`, `light_curtain` in `cell.yaml` | Two posts across the operator opening at the infeed end, 0.30 to 1.80 m high, 151 horizontal beams |
| Beams and detection | `cell_safety/core/light_curtain.py` | A beam is interrupted when an object covers it completely; resolution `d = pitch + beam diameter` = 14 mm |
| OSSD outputs | same | Two channels, OFF within the response time of an interruption, back ON after the restart delay; either channel can be stuck for fault tests |
| Safety controller | `cell_safety/core/protective_stop.py` | Dual-channel evaluation, discrepancy lockout, latching stop, manual reset with a restart interlock |
| The stop | same, `execute_with_protective_stop` | IEC 60204-1 **category 1**: controlled deceleration along the planned path, then power removed at standstill |
| Live node | `cell_safety/nodes/safety_node.py` | The same controller on the ROS stack: sends the trajectory controller a deceleration trajectory, then deactivates it |
| Fieldbus | `cell_fieldbus/core/registers.py` | Fault 94 `PROTECTIVE_STOP`, register 12 `safety_status`; a PLC fault reset cannot clear a latched stop |

### Why category 1

ISO 10218-1 requires a protective stop that an external safeguard can
trigger, and calls up the stop categories of IEC 60204-1. Category 0 cuts
power and lets the brakes stop the arm wherever it is going. Category 1 stops
under control first and removes power at standstill. With a part in the jaws
next to a conveyor rail, stopping *on the planned path* is what keeps the tool
out of the things the path was planned around. The offline model can also run
category 0 (`tests/test_safety.py` compares them).

### Parameters

These are model parameters, chosen inside the range type 4 finger-protection
curtains are sold with. They are assumptions, not data for any product.

| Parameter | Value | Note |
|---|---:|---|
| Beam pitch | 10 mm | |
| Beam optical diameter | 4 mm | |
| Resolution `d` | 14 mm | pitch + diameter; finger protection |
| Response time | 20 ms | interruption to OSSDs OFF |
| Restart delay | 100 ms | field clear to OSSDs ON |
| Controller period | 2 ms | the 500 Hz ros2_control rate |
| Discrepancy time | 50 ms | channels may disagree this long |
| Stop deceleration | 8 rad/s² per joint | the planner's own acceleration limit, so conservative against a real UR5e |

## The stated stopping time

From the parameters alone, before any simulation:

```
budget = response time + 2 controller periods + max joint speed / stop deceleration
       = 20 ms + 4 ms + (0.5 x 180 deg/s) / 8 rad/s^2
       = 20 ms + 4 ms + 196.3 ms
       = 220.3 ms, beam broken to standstill
```

The 0.5 is the 50% velocity scale the cell plans at. Two controller periods:
one to sample the OSSDs, one to act.

## Results

All simulated. Reference run: `make safety` in Python 3.10 with numpy 1.21
and scipy 1.8, committed as
[`docs/results/safety_stop.json`](results/safety_stop.json).

**Offline, on planned trajectories.** Twenty nominal benchmark cycles give 160
planned motion segments. Each was executed at 500 Hz with the curtain broken
at ten points along it, 1,600 stops in all:

| | |
|---|---:|
| Stops over the 220.3 ms budget | **0 of 1,600** |
| Worst stop, beam broken to standstill | **190.0 ms** (`to_pregrasp`) |
| Furthest the tool travelled after the stop request | 78.2 mm |
| Peak joint deceleration commanded by the stop | 8.0 rad/s² |

The worst case is under the budget because no planned segment reaches the
full scaled velocity limit on every joint at once.

**Detection.** A sphere swept across the field in 0.5 mm steps at five points
along it:

| Object | Detected at |
|---|---:|
| 14 mm (rated resolution) | 100% of positions |
| 10 mm | 65% |
| 8 mm | 45% |

The 14 mm result is the claim the geometry makes; the smaller objects show
the claim has teeth.

**Live, on `ros2_control`.** `src/cell_safety/test/test_protective_stop.launch.py`
moves the shoulder 1.5 rad in two seconds through `joint_trajectory_controller`
on mock hardware, breaks the curtain half way, and times beam broken to
standstill from the joint state stamps. Four isolated local runs measured
119, 122, 122 and 125 ms, and the first CI run that included it measured
104 ms; CI runs it on every push. The launch test allows
500 ms, because it runs on a shared runner with a Python node ticking at
250 Hz and DDS between every process. It then checks the controller refuses
goals until the curtain is clear and the reset has been pressed.

**Over EtherNet/IP.** `tests/test_enip.py` starts a cycle from the scanner,
breaks the curtain while the arm is carrying a part, and checks the PLC sees
fault 94, that its fault reset is ignored while the stop is latched, and that
the cell comes back to READY only after the reset at the cell.

## Is the curtain far enough away?

ISO 13855 gives the minimum distance from a curtain to the hazard as
`S = K T + C`, with `K` = 2000 mm/s (recalculated at 1600 mm/s if that gives
over 500 mm), `T` the overall stopping time and `C = 8 (d - 14)` mm. With the
220.3 ms budget and `d` = 14 mm, **S = 441 mm**.

`analysis/safety_stop.py` finds how close any part of the arm or gripper can
get to the curtain plane by optimising over the whole joint space from 60
random starts, ignoring collisions (a fault can drive the arm anywhere it can
reach): **1.131 m** from the base axis. The curtain stands at 1.600 m, so the
clearance is **469 mm against 441 mm required**. The analysis, and CI, fail if
that stops being true.

## What a real cell would need instead

- A certified type 4 curtain and a safety relay or safety PLC, to the
  performance level a risk assessment (ISO 12100) asks for, typically PL d,
  category 3 for this hazard.
- Measured stopping time and distance on the real robot with a stop-time
  analyser, not a model, and the separation distance recalculated from it.
- Reach over, under and around the curtain addressed (ISO 13857), and the
  rest of the perimeter guarded. Only the operator opening is modelled here.
- The robot's own safety-rated stop input wired from the safety relay. The
  reset button wired to the safety relay, outside the hazard zone, with a
  view of it.
- If the stop is to travel over a network, CIP Safety or PROFIsafe. Standard
  EtherNet/IP and Modbus are not safety protocols, which is why neither can
  reset a stop in this cell.
