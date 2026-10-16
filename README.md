# ros2-industrial-arm-cell-sim

A simulated production cell built around a 6-DoF arm: camera-guided grasp
planning, inverse-kinematics trajectory execution, MoveIt2 collision-aware
planning with OMPL, simulated force and torque feedback for contact inspection,
eye-in-hand calibration, PLC integration over Modbus TCP and EtherNet/IP, and a
simulated light curtain with a category 1 protective stop.

> ### Clean-room reimplementation
> This repository is an **independent reimplementation of a problem class**,
> built from scratch on personal time using public or synthetic data and
> open-source simulation. It is **not** derived from, and contains no material
> belonging to, any employer or client: no employer source code, schematics,
> calibration data, client data or internal documentation. It exists to
> demonstrate the architecture and engineering approach only. The only
> externally-sourced data in it is the publicly published UR5e DH table.

```
          light curtain across the operator opening (simulated)
          ||||||||||||||||||||||||||||||||||||||
   +------------------------------------------------------------+
   |                              infeed conveyor               |
   |     [ UR5e on pedestal ]     ===================>          |
   |            |                 randomised part poses [camera |
   |            | eye-in-hand camera on the wrist       pillar] |
   |            v                                               |
   |     [ inspection nest on the outfeed bench ]               |
   +----------------------------|-------------------------------+
                                | Modbus TCP :5020  or  EtherNet/IP :44818
                          [ line PLC ]

  scan -> capture -> segment -> rank grasps -> approach -> close
       -> lift -> transfer -> place -> probe five points -> home
```

Everything here is simulated. No robot, PLC, light curtain or safety
controller hardware has been used, and the safety layer is a model, not a
certified safety function. [STATUS.md](STATUS.md) says what is done and what
is not.

---

## The results

Every number below is produced by a script in `analysis/`, needs nothing but
numpy, scipy and pyyaml, and takes minutes on one core. `make report`
regenerates all of them.

### Grasp success over 200 randomised part poses

200 randomised poses per condition, seed 20260101. Part type, resting face,
position within the pick region and yaw are all drawn per trial. The reference
run is committed as
[`docs/results/grasp_benchmark.json`](docs/results/grasp_benchmark.json),
produced by `make grasp` on Python 3.10 with numpy 1.21 and scipy 1.8 after
the safety and EtherNet/IP changes, which did not move any figure.

| condition | grasp success | 95% CI | attempted and lost | declined |
|---|---:|---|---:|---:|
| **nominal** | **96.0%** | 92.3 to 98.0 | 4.0% | 0.0% |
| clutter: a second part on the belt | 94.5% | 90.4 to 96.9 | 5.0% | 0.5% |
| miscalibrated: 5 mm hand-eye error | 95.0% | 91.0 to 97.3 | 4.0% | 1.0% |
| noisy: 3× depth noise | 94.0% | 89.8 to 96.5 | 5.0% | 1.0% |

These reproduce exactly on Python 3.10 with numpy 1.21 and scipy 1.8. On
Python 3.12 with numpy 2.5 and scipy 1.18 the noisy row comes out one trial
lower, 187 of 200: floating-point differences between numeric libraries are
enough to move one marginal grasp across a threshold. CI runs a 60-trial
version of the benchmark on every push to show it still runs; it does not
gate on the rate.

**The degradations barely move it, and that is the result, not a claim of
robustness I went looking for.** The four intervals overlap; on 200 trials each
this cell is not measurably worse with a neighbouring part on the belt, with
three times the depth noise, or with the hand-eye transform 5 mm out. What
*does* move it is part geometry:

| part | footprint | success |
|---|---|---:|
| bracket_a | 62 × 38 mm | 36/36, 100% |
| bracket_b | 48 × 48 mm | 34/34, 100% |
| connector_shell | 90 × 32 mm | 39/39, 100% |
| pcb_coupon | 100 × 70 × 8 mm | 31/32, 96.9% |
| spacer_block | 55 × 55 × 55 mm | 33/35, 94.3% |
| **wide_plate** | **110 × 78 mm** | **19/24, 79.2%** |

All eight nominal failures are `too_wide` (5) or `finger_collision` (3), and
five of the eight are the 78 mm plate against an 85 mm jaw stroke. With 3.5 mm
of clearance a side, a width estimate two millimetres light is the difference
between a grasp and a collision, and the perception's extent estimate has
about that much spread. This is a gripper problem, not an algorithm problem:
the fix is a wider stroke or a different part presentation, and no amount of
planner tuning recovers it.

**Nothing is declined under nominal conditions and everything that fails is
attempted.** That is the expensive kind of failure, so it is worth being
explicit: this cell drops 4% of parts rather than refusing them. A production
version would add a confidence floor that declines a marginal-width grasp
rather than trying it.

Perception, over the same 200 poses: centroid error median **1.01 mm**
(p95 2.58), yaw error median **0.41°** (p95 6.56).

### Cycle time

Median **9.88 s**, mean 10.39 s, p05 9.39, p95 14.79, over the 192 successful
nominal cycles. Not measured with a stopwatch: every stage contributes a named
segment, and each motion segment's duration comes from time-parameterising the
actual planned path against velocity and acceleration limits scaled to 50% and
35% of the UR5e datasheet figures, because nobody commissions a cell at 100%.

| by kind | | | slowest segments | |
|---|---:|---|---|---:|
| motion | 93.0% | | `to_scan` | 2.60 s |
| gripper | 4.0% | | `to_home` | 1.89 s |
| sensor | 2.3% | | `transfer` | 1.79 s |
| fieldbus | 0.8% | | `to_pregrasp` | 1.33 s |

**43% of the cycle is travelling empty.** The two free moves, `to_scan` at
2.60 s and `to_home` at 1.89 s, are 4.49 s of the 10.39 s mean, and neither
does anything with the part.
Deleting the home pose (it exists so benchmark trials are independent, not
because the cell needs it) and overlapping the scan with the previous retreat
would take out most of that. The fieldbus handshake, which everyone assumes is
the expensive part, is 80 ms.

The spread between the median and the p95 is the cost of a sampling planner:
RRT-Connect occasionally takes an unlucky route that shortcutting does not
fully recover. [docs/cycle-time.md](docs/cycle-time.md)

### Protective stop on a light curtain break (simulated)

The curtain across the operator opening has 151 beams at a 14 mm resolution.
Breaking it triggers an IEC 60204-1 category 1 stop: the arm decelerates along
its planned path, then the drives are switched off at standstill. From the
parameters alone (20 ms curtain response, two 2 ms controller periods, and the
fastest joint braking from half of 180°/s at 8 rad/s²) the stated budget is
**220.3 ms from beam broken to standstill.**

| | |
|---|---:|
| stops simulated on planned trajectories (160 segments × 10 break points) | 1,600 |
| stops over the 220.3 ms budget | **0** |
| worst stop | **190.0 ms** |
| furthest the tool travelled after the stop request | 78.2 mm |
| 14 mm object detected, swept across the field | 100% of positions |
| live arm on `ros2_control`: four local launch-test runs, one CI run | 119 to 125 ms; 104 ms |
| ISO 13855 minimum distance at the budget | 441 mm |
| curtain's actual clearance from the arm's furthest reach | **469 mm** |

These are model outputs with assumed parameters, not measurements of a curtain
or a UR5e, and they support no performance level claim. A protective stop is
reported to the PLC as fault 94, and a PLC fault reset cannot clear it: only
the manual reset at the cell can, once the curtain is clear and the arm is
still. [docs/safety.md](docs/safety.md)

### Hand-eye calibration

Twenty-five randomised pose sets of each kind:

| pose set | poses | **true** translation error | **true** rotation error | AX−XB residual | board spread | reprojection |
|---|---:|---:|---:|---:|---:|---:|
| thorough | 20 | **0.38 mm** | **0.049°** | 0.81 mm | 0.19 mm | 0.146 px |
| adequate | 10 | **0.61 mm** | **0.143°** | 0.85 mm | 0.21 mm | 0.146 px |
| hurried | 6 | **4.51 mm** | **0.791°** | 1.03 mm | 0.62 mm | 0.146 px |

The hurried calibration is **twelve times worse and looks 27% worse.** Its
reprojection error is identical to three decimal places. Every diagnostic
available on a real robot says it is fine. `AX = XB` fixes the translation of
`X` only from motions whose rotation axes span, nearly-overhead poses rotate
about one axis, and an ill-conditioned least-squares problem does not announce
itself: it returns a confident answer with a small residual. So the
calibration node **refuses to solve** below 25° of axis spread rather than
reporting a residual and letting someone commission it.
[docs/calibration.md](docs/calibration.md)

### What a calibration error costs in grasps

| hand-eye error | 0 mm | 2 mm | 5 mm | 10 mm | 16 mm | 22 mm | 30 mm |
|---|---:|---:|---:|---:|---:|---:|---:|
| grasp success | 100% | 100% | 100% | 98% | 90% | 76% | 54% |
| attempted and lost | 0% | 0% | 0% | 0% | 2% | 6% | 22% |

50 trials each. The cell tolerates about a centimetre and falls apart past two.
Note the shape as well as the size: up to 16 mm the failures are grasps the
cell *declined*, which cost a re-present; by 30 mm nearly half are parts it
tried to pick and dropped, which cost a stopped line.

### Force-feedback contact inspection

At 0.35 N sensor noise, 20 kN/m contact stiffness and 5 mm/s approach, a probe
measures surface height to **8 µm mean, 18 µm p95.**

Seating detection, 20 repeats per row, 0.35° acceptance threshold:

| corner lift | 0.00 | 0.05 | 0.10 | 0.20 | **0.30** | 0.50 | 1.00 mm |
|---|---:|---:|---:|---:|---:|---:|---:|
| detected | 0% | 0% | 0% | 0% | **100%** | 100% | 100% |

**False positives on a correctly seated part: 0%.** The specification this
routine can be held to is: it catches anything over about 0.3 mm and nothing
under 0.2, and it does not cry wolf. [docs/inspection.md](docs/inspection.md)

### Joint controller tuning

![Step response](docs/plots/step_response.png)

Relay auto-tune gives Ku = 1534 N·m/rad, Tu = 0.197 s. Then:

| gain set | overshoot at J=1.6 | phase margin | stable across the inertia sweep |
|---|---:|---:|---|
| Ziegler-Nichols | 35.9% | 11.6° | no: negative margin at J = 3.2 |
| ZN detuned ×0.5 | 64.7% | −5.6° | no: limit-cycles at full torque |
| Hand-tuned | 6.4% | 57.2° | yes: 43° to 68° across the sweep |

**Ziegler-Nichols fails here for a structural reason**, and detuning it makes
it worse rather than better. The ZN rules were fitted to
first-order-plus-dead-time plants; a robot joint with light viscous friction is
close to a double integrator. Halving the gains lowers the crossover to where
the PID's own zeros contribute less phase lead, and the margin goes negative.
Every row is measured twice, a frequency-domain margin and a time-domain step,
and they agree on every row.
[docs/controller-tuning.md](docs/controller-tuning.md)

### Collision-scene validation

An 11×11 grid over the pick region plus a 100 mm margin, at eight grasp
orientations and three heights. The taught region is fully reachable at every
orientation; the swept margin is not, blocked by the far guide rail. A run in
which *everything* is reachable means the obstacles are not in the scene the
checker is using, which is the failure this sweep exists to catch, and which
it did catch, twice.

---

## Talking to the cell from a PLC

Two transports, one contract. Both carry the same sixteen status words and
the same four command bits, with the same semantics: start and fault reset on
a rising edge, enable and pause as levels, faults that latch, a heartbeat that
counts.

**Modbus TCP** on port 5020: coils 0 to 3 from the PLC, holding registers 0
to 12 from the cell.

**EtherNet/IP** on port 44818, with the cell as the adapter and the PLC as the
scanner. The adapter is [OpENer](https://github.com/EIPStackGroup/OpENer),
built from a pinned commit with the cell's application in
`src/cell_fieldbus/enip/adapter/`.

| Assembly | Direction | Size | Content |
|---:|---|---:|---|
| 100 | input, cell to PLC | 32 bytes | the 16 status words (state, fault, heartbeat, counts, cycle time, safety status and the rest) as UINT |
| 150 | output, PLC to cell | 4 bytes | command word (bit 0 start, 1 fault reset, 2 enable, 3 pause), reserved word |
| 151 | configuration | 0 | empty |

Exclusive-owner connection 150/100/151 at a 20 ms RPI; input-only and
listen-only heartbeats 152 and 153. Explicit messaging reads the input
assembly, the identity, and a vendor-specific diagnostics object (class 0x64).
If the PLC's connection times out or it goes to Program, the cell is told
every command bit is zero and goes idle.

A PLC adds the cell as a generic EtherNet/IP adapter with those three
assemblies and maps 32 input bytes and 4 output bytes into its process image.
[docs/fieldbus.md](docs/fieldbus.md) has the full layout, the I/O
configuration and a Structured Text handshake. It also describes how the
sibling project [plc-cip-batch-control](https://github.com/CGainsWorks/plc-cip-batch-control)
would supervise this cell, with proposed addresses clear of its current I/O
map. That project is a clean-in-place skid controller ("CIP" there is
clean-in-place), speaks Modbus TCP, and is not wired to this cell today.

**How it is tested.** Two containers on one Docker network, a PLC and a
cell. The PLC side holds a real Class 1 connection through a client built on
[EIPScanner](https://github.com/nimbuscontrols/EIPScanner) and does explicit
messaging with [pycomm3](https://github.com/ottowayi/pycomm3). It runs a
cycle, breaks the simulated curtain mid-cycle and checks the stop cannot be
reset over the network, then kills the scanner and checks the cell goes idle.
CI runs it on every push. No commercial PLC has been on the other end.

---

## Quick start

Nothing here needs ROS.

```bash
./scripts/setup.sh          # a venv with numpy, scipy, pyyaml, pymodbus, matplotlib
source .venv/bin/activate
make check                  # lint, 108 tests, and the fast analyses: a few minutes
```

Four of the 108 tests drive the real EtherNet/IP adapter and skip without it.
With Docker:

```bash
make enip-test              # builds OpENer and EIPScanner, runs the cell against the scanner
```

One cycle, printed stage by stage:

```bash
python scripts/offline_demo.py --seed 4
```

```
cycle 1: pcb_coupon
  presented at (0.715, -0.184) m, yaw +39 deg, footprint 100 x 70 x 8 mm
  perception:  centre off by 0.58 mm, yaw off by 0.17 deg, from 2765 points
  grasp:       3 candidates, best scores 0.727 (approach 1.00, balance 1.00,
               clearance 0.22, contact 1.00, posture 1.00, width 0.33)
               closing at +129 deg across 67.7 mm, jaws opened to 79.7 mm
  timeline:
    to_scan                    2.61 s   motion
    camera_settle              0.15 s   sensor
    to_pregrasp                0.85 s   motion
    descend                    0.52 s   motion
    ...
  verdict:     HELD -- contact 0.2 deg into a 27 deg friction cone,
               engagement 8.0 mm
  inspection:  5 probes, tilt 0.014 deg, mean error 5.7 um, fault 0
  fieldbus:    state 8 (COMPLETE), fault 0 (NONE), last cycle 9595 ms, 1/1 held
  TOTAL:       9.59 s   complete
```

The full benchmark:

```bash
make grasp                  # 200 poses x 4 conditions, about 15 minutes
make safety                 # protective stop, detection and curtain placement
make report                 # every number above
```

### The simulated cell

ROS 2 Humble, MoveIt2 and Gazebo Harmonic are needed only to run it as a live
system. The Docker image is the supported way:

```bash
make docker-demo            # or: ros2 launch cell_bringup cell.launch.py
ros2 launch cell_bringup gazebo.launch.py   # the cell in Gazebo Harmonic, headless
make launch-tests           # the headless launch tests CI runs
```

`scripts/run_demo.sh` launches the full stack when ROS is present and falls
back to the offline cycle demo when it is not. A demo that only works on a
machine with a 3 GB installation is not a demo, it is a prerequisite list.

The live stack is verified, not assumed. CI runs three checks against it:

- `scripts/verify_ros_stack.py` asks MoveIt to plan a joint-space goal,
  executes it through `ros2_control`'s `joint_trajectory_controller`, checks
  the arm arrives within 0.009 rad, and checks that a goal 60 mm inside the
  conveyor is **refused**.
- `src/cell_safety/test/test_protective_stop.launch.py` breaks the light
  curtain while the arm is moving and checks it stands still within 500 ms,
  that the controller refuses goals until the reset, and that the arm moves
  again after it.
- `src/cell_bringup/test/test_gazebo_world.launch.py` loads the generated world
  in Gazebo Harmonic, headless, and checks the arm spawns, the curtain and
  fixtures are there, and simulation time advances.

The hardware underneath is `mock_components/GenericSystem`, so the arm tracks
perfectly: that proves the plan-and-execute path, the collision scene and the
stop chain, and nothing about tracking error. Gazebo moved off Classic, which
is end of life, to Harmonic rather than Fortress, which reached end of life in
September 2026. Gazebo shows the cell; the arm is not driven in it.

---

## How it is built

**The algorithm is in `<package>/core/`. The ROS node is a wrapper around it.**
Every core module imports numpy, scipy and its siblings, and nothing else: no
`rclpy`, no message types. That is what lets 200 full cycles run in minutes
with no ROS installed, and what makes the tests tests of the algorithms rather
than of the middleware.

| Package | Contents |
|---|---|
| `cell_common` | Transforms and the typed reader for the generated cell description |
| `cell_description` | Generated URDF/xacro for the arm and the cell furniture, and the Gazebo world |
| `cell_motion` | Kinematics, collision model, RRT-Connect, Cartesian planning, time parameterisation, tuning harness, MoveIt2 config |
| `cell_grasping` | Simulated depth camera, perception, grasp planner |
| `cell_calibration` | Eye-in-hand: pose generation, PnP, the AX = XB solve |
| `cell_inspection` | Admittance contact probe and the seating check |
| `cell_fieldbus` | Register map, Modbus TCP bridge, EtherNet/IP layout, OpENer adapter (C) and EIPScanner test client (C++) |
| `cell_safety` | Simulated light curtain, safety controller, category 1 protective stop, and the node that applies it live |
| `cell_bringup` | Launch (including Gazebo Harmonic), and the cycle state machine |

**One source of truth for the geometry.** `simulation/generate_cell.py` holds
the dimensions and emits `cell.yaml`, the Gazebo world and both xacro files.
CI fails if any is stale. The arm description is generated from the same DH
arrays the IK solver uses, so the URDF joint frames *are* the DH frames rather
than a transcription of them. The light curtain is in the same table, and
deliberately not in the collision scene: it stands outside the arm's reach,
and a safety device has no business moving a grasp-success figure.

**Closed-form IK, all eight branches.** A numerical solver returns whichever
branch its seed fell into, so a pose only reachable elbow-up looks unreachable
when the seed was elbow-down, which is worth several fictitious percentage
points of grasp success over 200 poses.

**The boundary that makes the benchmark mean anything.** The pick pipeline sees
the depth image and the *calibrated* hand-eye transform. Ground truth enters in
exactly one place: `evaluate_grasp`, at the end, to decide whether the grasp
held.

**One PLC contract, two transports.** The edge detection, fault latching and
heartbeat live once, in `cell_fieldbus/core/registers.py`. The EtherNet/IP
adapter is transport only: OpENer forwards the output assembly to the Python
side every 10 ms and takes the status block back, so there is no second
implementation of the contract in C to drift from the first.

[docs/architecture.md](docs/architecture.md)

---

## What a failure means

Every failure is attributed to the stage that produced it, and split into two
classes that cost very different amounts:

- **Declined**: perception found nothing, or no candidate scored above the
  feasibility floor. Cheap: the part goes round again.
- **Attempted and lost**: the jaws closed and the part did not come. Expensive:
  a dropped part is a stopped line.

A cell at 95% made of 5% dropped is worse than one at 90% made of 10%
declined, so both are reported. The six ground-truth criteria a grasp must
satisfy (the antipodal friction-cone condition, jaw stroke, pad engagement,
grasp-line intersection, torsional slip and finger clearance) are in
`cell_bringup/core/outcome.py`.

A protective stop is a third kind: not a failure of the cycle but the cell
doing what it should. It is reported as fault 94 against the stage it
interrupted, so the PLC can tell a stop during transfer from one at the scan.

---

## Documentation

| | |
|---|---|
| [architecture.md](docs/architecture.md) | How the pieces fit, and the DH-to-URDF off-by-one that a test caught |
| [grasp-planning.md](docs/grasp-planning.md) | The pipeline, the scoring, and three decisions that were wrong first |
| [cycle-time.md](docs/cycle-time.md) | Where the seconds go and what each responds to |
| [calibration.md](docs/calibration.md) | Why a good residual is not evidence of a good calibration |
| [inspection.md](docs/inspection.md) | The probe's error budget, measured term by term |
| [controller-tuning.md](docs/controller-tuning.md) | Why Ziegler-Nichols fails on a robot joint |
| [fieldbus.md](docs/fieldbus.md) | The register map, the EtherNet/IP assemblies, the fault codes, and how a PLC talks to the cell |
| [safety.md](docs/safety.md) | The light curtain, the protective stop, and what a real cell would need instead |
| [limitations.md](docs/limitations.md) | What this does not do |
| [STATUS.md](STATUS.md) | Done, not started, and the limitations in one page |

---

## Limitations

The short version; the full list is [docs/limitations.md](docs/limitations.md).

- **No dynamics.** The arm is kinematic. Trajectories respect velocity and
  acceleration limits; nothing computes joint torques during a move.
- **Rigid parts, rigid pads.** A grasp holds or does not, by geometry and a
  friction cone. Real jaws often *centre* a part as they close, which forgives
  a millimetre or two; this model does not, so its off-centre failures are
  pessimistic.
- **Matte boxes only.** The depth model has range-dependent noise, dropout,
  quantisation and a working range. It has no model of specular reflection or
  transparency, which are what actually break depth cameras in a factory.
- **The parts are boxes and the estimator fits a box.** Convenient, and it
  flatters the result.
- **Safety is simulated, not certified.** The curtain's response time and the
  stop deceleration are assumed parameters. A real cell needs certified
  hardware, a risk assessment and a measured stopping time.
- **No commercial PLC on the other end.** Both fieldbus transports are real
  and tested against open-source clients, not against a Rockwell, Siemens or
  CODESYS controller.
- **Gazebo shows the cell but does not drive the arm.** No gz_ros2_control
  bridge is wired.
- **The nominal benchmark is a benign scenario.** One part, no occlusion, a
  correct calibration, a healthy camera. That is why the other three conditions
  and the sensitivity curve are there.
- **Success is judged by a model, not by physics.** A Gazebo or MuJoCo grasp
  with real contact dynamics would disagree at the margins, and the direction
  of that disagreement is unknown.

---

## Licence

MIT. See [LICENSE](LICENSE).
