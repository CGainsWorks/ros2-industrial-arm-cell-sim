# Architecture

## The shape of it

The cell is a 6-DoF arm, an infeed conveyor, a wrist camera, a parallel gripper
and an inspection nest, with a line PLC on the other side of a Modbus link.

```
                    camera / light-curtain pillar
                              |
   +--------------------------|-----------------------------+
   |                          |    infeed conveyor           |
   |     [ UR5e on pedestal ] |    ===================>      |
   |            |             |    parts arrive here         |
   |            | eye-in-hand camera on the wrist            |
   |            v                                            |
   |     [ inspection nest on the outfeed bench ]            |
   |                                                         |
   +----------------------------|----------------------------+
                                | Modbus/TCP
                          [ line PLC ]
```

One cycle:

```
  PLC raises cycle_start
        |
        v
  move to scan pose ---> capture depth ---> segment and fit the part
        |                                          |
        |                                          v
        |                              rank grasp candidates
        |                                          |
        v                                          v
  plan to pre-grasp <--- pick the best candidate that plans
        |
        v
  straight-line descent -> close jaws -> lift -> transfer -> place
        |
        v
  probe five points -> fit a plane -> seated or not
        |
        v
  retreat home, publish state and fault code to the PLC
```

## Packages

| Package | What is in it |
|---|---|
| `cell_common` | Transforms, and the typed reader for the generated cell description. Depends on nothing else here, because everything else depends on it. |
| `cell_description` | Generated URDF/xacro for the arm and the cell furniture. |
| `cell_motion` | UR5e kinematics, the collision model, RRT-Connect and Cartesian planning, time parameterisation, the joint tuning harness, and the MoveIt2 configuration. |
| `cell_grasping` | The simulated depth camera, the perception pipeline, and the grasp planner. |
| `cell_calibration` | Eye-in-hand: pose generation, PnP board detection, the AX = XB solve. |
| `cell_inspection` | The admittance contact probe and the seating check. |
| `cell_fieldbus` | The Modbus register map and the bridge. |
| `cell_bringup` | Cell launch, and the cycle state machine that drives all of the above. |

## The rule that shapes everything

**The algorithm is in `<package>/core/`. The ROS node is a wrapper around it.**

Every core module imports numpy, scipy and its sibling core modules, and
nothing else. No `rclpy`, no message types, no parameter server. That is what
lets `analysis/grasp_benchmark.py` run two hundred full cycles in three minutes
on a laptop with no ROS installation, and it is what makes the tests in
`tests/` tests of the algorithms rather than tests of the middleware.

The nodes in `<package>/nodes/` do three things: convert messages to arrays,
call the core, convert arrays back to messages. Where a node does anything
else, it is because that thing genuinely cannot be done offline -- taring a
real force sensor, reading a real `CameraInfo`, spanning two event loops --
and it is commented as such.

## One source of truth for the geometry

`simulation/generate_cell.py` holds the cell's dimensions and emits four files:

- `simulation/cell.yaml` -- what the planner and the perception read
- `simulation/cell.world` -- what Gazebo simulates
- `src/cell_description/urdf/ur5e.urdf.xacro` -- the arm, from the DH table
- `src/cell_description/urdf/cell.urdf.xacro` -- the fixtures

CI runs `generate_cell.py --check` and fails if any of them is stale.

This is not tidiness. Three hand-maintained descriptions of one cell drift, and
when they do the failure is silent: an early revision had the conveyor 120 mm
taller in the URDF than in the collision checker, and the planner routed
approach trajectories straight through the belt while the benchmark reported a
grasp success rate the simulated cell would never have reproduced.

The arm description is generated from the same `D`, `A` and `ALPHA` arrays the
IK solver uses, so the URDF joint frames *are* the DH frames rather than a
transcription of them. `tests/test_description.py` walks the generated joint
chain with an XML parser and compares it against `forward_kinematics` at random
configurations. That test exists because the first version of the generator got
it wrong in a way nothing else would have caught: standard DH composes as
`Rz(theta) Tz(d) Tx(a) Rx(alpha)` -- rotation first -- while a URDF joint's
`<origin>` is applied *before* its axis rotation, so each link's offset belongs
on the *next* joint. Writing it on its own joint silently produces modified DH
instead: a robot that looks entirely plausible in RViz and whose forward
kinematics disagrees with the solver by up to 130 mm.

## What knows what

The boundary that makes the benchmark mean anything:

| Stage | Knows |
|---|---|
| Depth render | Everything. It is the simulator. |
| Perception | The depth image and the *calibrated* hand-eye transform. |
| Grasp planning | The perception estimate. Not the part's identity, not its true size. |
| Motion planning | The estimate, the cell geometry, and the clutter the camera saw. |
| Outcome | Ground truth. It is the judge, and it is the only stage allowed to look. |

The perception stage deprojects using the pose implied by the *calibrated*
extrinsic, not the true one, which is why a calibration error shows up as a
displaced point cloud rather than as a special case somewhere.

## Two planners, on purpose

`cell_motion` has both RRT-Connect and a straight-line Cartesian planner, and
the cycle uses each for what it is for.

Free-space moves -- home to scan, lift to nest, nest to home -- go through
RRT-Connect and a randomised shortcut pass. The approach and retreat do not: a
grasp approach must be a straight line along the tool axis, because anything
else sweeps the fingers sideways through the part it is about to pick up. The
Cartesian planner interpolates the tool pose, solves IK at each step, and
rejects the whole path if consecutive solutions are discontinuous. MoveIt's
`compute_cartesian_path` instead returns the *fraction* it managed, and a
caller that executes an 87% path drives the tool to somewhere that is not the
grasp.

## Three launch bugs that only exist at runtime

CI imports every launch file and calls `generate_launch_description()`. That
catches typos and missing imports, and it cannot catch any of these, because
building a launch description does not evaluate its substitutions or start
anything. All three were found by running the stack in the ROS container.

**`robot_description` must be wrapped.** A `Command()` substitution evaluates
to a string, and launch parses parameter values as YAML unless told otherwise.
A URDF is not YAML, so `move_group` exited immediately with "Unable to parse
the value of parameter robot_description as yaml". The fix is
`ParameterValue(..., value_type=str)`, and all four launch files needed it.

**MoveIt's YAML files are not ROS 2 parameter files.** They are bare mappings;
`--params-file` requires `<node>: ros__parameters: ...`. Passing the paths
directly gets "Failed to parse global arguments" and an abort -- an
unhelpful message for a YAML shape problem. They are now loaded in the launch
file and passed as dictionaries, which is what MoveIt's own config builder
does and which keeps the files in the shape a reader expects.

**The planner has to be *named*, not just configured.** With the OMPL settings
at the top level rather than namespaced under the pipeline name, `move_group`
found no `planning_plugin`, fell back to whatever it could load, and ran
**CHOMP** -- while the README, this document and `ompl_planning.yaml` all said
OMPL. It announces the substitution in exactly one line of its startup log:

```
[move_group]: Using planning interface 'OMPL'
```

That line is now the thing to check after any change to the MoveIt
configuration. It is the difference between the planner you documented and the
planner you shipped.

## Where the numbers come from

Every quantitative claim in the README is produced by a script in `analysis/`,
each of which writes JSON to `results/` and prints a summary:

| Claim | Script |
|---|---|
| Grasp success rate, failure breakdown | `analysis/grasp_benchmark.py` |
| Cycle time and its split | `analysis/cycle_time.py` |
| Hand-eye residual, and what it hides | `analysis/calibration_residual.py` |
| Grasp success against hand-eye error | `analysis/calibration_sensitivity.py` |
| Probe accuracy, seating threshold | `analysis/inspection_accuracy.py` |
| Controller tuning, stability margins | `analysis/pid_tuning.py` |
| Collision-scene validation | `analysis/reach_map.py` |

`make report` runs all of them.
