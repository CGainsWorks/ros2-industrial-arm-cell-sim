# Status

As of 2026-10-16. Everything below that produces a number does so in
simulation; no robot, PLC, light curtain or safety controller hardware has
been used.

## Complete

- **Pick, inspect and place cycle, offline.** Closed-form UR5e IK (all eight
  branches), capsule collision model, RRT-Connect and Cartesian planning, time
  parameterisation, simulated depth camera, perception, grasp scoring,
  AX = XB hand-eye calibration, admittance contact probe, joint controller
  tuning harness. 200-pose grasp benchmark over four conditions, reference run
  committed in `docs/results/`.
- **ROS 2 Humble stack.** MoveIt2 with OMPL plans and executes through
  `ros2_control` (`joint_trajectory_controller` on
  `mock_components/GenericSystem`); a goal inside the conveyor is refused.
  Checked live in CI by `scripts/verify_ros_stack.py`.
- **Modbus TCP.** pymodbus server with the register map, edge-triggered
  commands, latched faults and a counting heartbeat; tested over a socket.
- **EtherNet/IP.** Adapter on OpENer (pinned commit) with input assembly 100
  (32 bytes), output assembly 150 (4 bytes), configuration 151, heartbeat
  assemblies 152 and 153, and a vendor-specific diagnostics object, class
  0x64. Tested in CI against a Class 1 scanner built on EIPScanner and
  against pycomm3 explicit messaging, in two containers: a cycle from the PLC,
  a protective stop mid-cycle, and a lost connection.
- **Light curtain and protective stop (simulated, not certified).** 14 mm
  resolution curtain with dual-channel OSSD outputs, safety controller with
  discrepancy lockout and manual reset, IEC 60204-1 category 1 stop along the
  planned path. Offline: 1,600 stops on planned trajectories, none over the
  220.3 ms stated budget, worst 190.0 ms. Live: a launch test breaks the
  curtain mid-motion on `ros2_control` and checks the arm stops within 500 ms
  (four local runs measured 119 to 125 ms, the first CI run 104 ms). Curtain placement checked against
  ISO 13855: 469 mm clearance against 441 mm required. Fault 94 and register
  12 report it to the PLC.
- **Gazebo Harmonic.** Migrated off Gazebo Classic (end of life). The
  generated world, light curtain included, loads headless in gz-sim 8 and the
  arm spawns into it; checked by a launch test in CI.
- **Tests.** 108 pytest tests (4 of them need the EtherNet/IP containers and
  skip without them; CI runs them in the compose step), plus 2 launch test
  files (3 tests) run headless in CI.

## In progress

Nothing.

## Not started

- Gazebo driving the arm (gz_ros2_control); today Gazebo shows the cell and
  the arm is unactuated there.
- The EtherNet/IP adapter inside the ROS launch, alongside the Modbus bridge
  node; today it runs beside the offline cycle.
- An emergency stop circuit (fault 92 is reserved for it).
- An OEE view from the fieldbus registers.

## Limitations

- **No hardware.** The arm is kinematic on mock hardware; no torques, no
  tracking error. Grasp success is judged by a geometric and frictional model,
  not by physics.
- **Safety is simulated, not certified.** Curtain response time and stop
  deceleration are assumed parameters. A real cell needs certified hardware,
  a risk assessment and a measured stopping time. Only the operator opening
  is guarded.
- **No commercial PLC on the other end.** The EtherNet/IP adapter has been
  exercised by two open-source scanner implementations, not by a Rockwell,
  Siemens or CODESYS controller, and not by ODVA conformance testing. Its
  vendor ID is a placeholder.
- **The sibling PLC project does not speak EtherNet/IP.**
  CGainsWorks/plc-cip-batch-control is a clean-in-place skid controller using
  Modbus TCP. `docs/fieldbus.md` describes how it would supervise this cell;
  nothing is wired between them.
- **Benchmark results depend slightly on the numeric stack.** The committed
  200-pose reference run reproduces the README on Python 3.10, numpy 1.21 and
  scipy 1.8. On Python 3.12 with numpy 2.5 and scipy 1.18 the noisy condition
  comes out one trial lower (187 of 200 rather than 188); the other three
  conditions are identical.
- Full list: [docs/limitations.md](docs/limitations.md).
