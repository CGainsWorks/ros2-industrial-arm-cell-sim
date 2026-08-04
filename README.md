# ros2-industrial-arm-cell-sim

> **Status: planned. This repository is a brief, not an implementation.**
> Nothing here is built yet. The sections below define what goes in it.

> ### Clean-room reimplementation
> This repository is an **independent reimplementation of a problem class**, built from
> scratch on personal time using public or synthetic data and open-source simulation.
> It is **not** derived from, and contains no material belonging to, any employer or
> client: no employer source code, schematics, calibration data, client data or internal
> documentation. It exists to demonstrate the architecture and engineering approach only.

A simulated production cell built around a 6-DoF arm: camera-guided grasp planning, inverse-kinematics trajectory execution, MoveIt2 collision-aware planning with OMPL, simulated force and torque feedback for contact inspection, eye-in-hand calibration and PLC integration over Modbus. Uses the open-source UR5e description and open fieldbus simulators. No employer code, schematics or client material.

---

## What this covers

CV entries this repository is the evidence for:

- Fanuc LR Mate 200iD line integration (Convex Technology)
- UR5e cobot for automated PCB inspection (Printed Electronics)
- Industrial fieldbus debugging, EtherNet/IP and Modbus (Convex Technology)

---

## Requirements

### Hardware

- None required. Simulation only
- Optional physical target: UR5e or any 6-DoF arm, gripper, wrist camera, force and torque sensor, calibration board
- Simulation substitute: UR5e or Panda in Gazebo or Isaac Sim, simulated FT sensor

### Software

- ROS2 Humble and ros2_control
- MoveIt2 with OMPL
- UR ROS2 driver description packages
- Vision-guided grasp planner
- Eye-in-hand calibration routine
- Simulated force and torque controller
- pymodbus and an open EtherNet/IP simulator
- PID joint controller tuning harness

### Report and documentation deliverables

- Cycle-time analysis
- Grasp success rate over a randomised part set
- Hand-eye calibration procedure with residual error
- Controller tuning record with step responses
- Collision scene validation
- Fieldbus message catalogue and fault-code map

---

## Inputs

- Camera frames of the part
- Hand-eye calibration transform
- Part CAD or a grasp database
- Cell geometry for collision checking
- Cycle trigger signals over Modbus
- Inspection point list

## Outputs

- Grasp pose candidates with scores
- Collision-free joint trajectories
- Gripper commands
- Contact-force logs
- Cycle status and fault codes over the fieldbus
- Inspection images tagged with pose
- Cycle-time and success-rate logs

---

## Intended structure

```
src/
  cell_bringup/           launch, cell config
  cell_description/       arm, fixtures, conveyor URDF
  cell_grasping/          vision grasp planner
  cell_motion/            MoveIt2 config, IK trajectory execution
  cell_inspection/        force-feedback contact routines
  cell_calibration/       eye-in-hand
  cell_fieldbus/          Modbus and EtherNet/IP bridge nodes
simulation/               cell worlds, part models
analysis/                 cycle time, grasp success
docs/
```

Plus the standard baseline: `README.md`, `LICENSE`, `.gitignore`, `.gitattributes`
(Git LFS rules for weights, bags and media), `.github/workflows/ci.yml`, `docs/`,
`scripts/setup.sh`, `scripts/run_demo.sh` and `tests/`.

---

## Build checklist

- [ ] Simulated cell: arm, conveyor, fixtures, camera
- [ ] MoveIt2 config with a collision scene built from the cell CAD
- [ ] IK trajectory execution to arbitrary poses
- [ ] Vision grasp planner producing scored candidates
- [ ] Pick and place cycle with a randomised part pose
- [ ] Eye-in-hand calibration routine and residual reporting
- [ ] Force-feedback contact inspection routine
- [ ] Modbus bridge exposing cycle start, status and fault codes
- [ ] PID joint tuning harness with step-response plots
- [ ] Cycle-time and grasp-success benchmark run

---

## Definition of done

This repository is finished when:

- [ ] The whole thing runs from one command on a clean machine.
- [ ] The README opens with a diagram or a GIF, not a wall of text.
- [ ] Every number quoted in the README is reproducible from a script in the repo.
- [ ] Limitations are stated honestly in their own section.
- [ ] A licence is declared.
- [ ] Large binaries live in Releases or LFS, not in the Git history.
- [ ] The repository URL is added to the matching CV bullet point.

---

## Starter prompt

Paste this into your coding agent of choice to begin.

<details>
<summary>Expand</summary>

```
You are helping me build a portfolio project from scratch. Read this brief and then
propose a concrete implementation plan before writing any code.

PROJECT: ros2-industrial-arm-cell-sim
A simulated production cell built around a 6-DoF arm: camera-guided grasp planning, inverse-kinematics trajectory execution, MoveIt2 collision-aware planning with OMPL, simulated force and torque feedback for contact inspection, eye-in-hand calibration and PLC integration over Modbus. Uses the open-source UR5e description and open fieldbus simulators. No employer code, schematics or client material.

TARGET STACK
- ROS2 Humble and ros2_control
- MoveIt2 with OMPL
- UR ROS2 driver description packages
- Vision-guided grasp planner
- Eye-in-hand calibration routine
- Simulated force and torque controller
- pymodbus and an open EtherNet/IP simulator
- PID joint controller tuning harness

HARDWARE CONTEXT
- None required. Simulation only
- Optional physical target: UR5e or any 6-DoF arm, gripper, wrist camera, force and torque sensor, calibration board
- Simulation substitute: UR5e or Panda in Gazebo or Isaac Sim, simulated FT sensor

THE SYSTEM MUST TAKE THESE INPUTS
- Camera frames of the part
- Hand-eye calibration transform
- Part CAD or a grasp database
- Cell geometry for collision checking
- Cycle trigger signals over Modbus
- Inspection point list

AND PRODUCE THESE OUTPUTS
- Grasp pose candidates with scores
- Collision-free joint trajectories
- Gripper commands
- Contact-force logs
- Cycle status and fault codes over the fieldbus
- Inspection images tagged with pose
- Cycle-time and success-rate logs

INTENDED REPOSITORY LAYOUT
src/
  cell_bringup/           launch, cell config
  cell_description/       arm, fixtures, conveyor URDF
  cell_grasping/          vision grasp planner
  cell_motion/            MoveIt2 config, IK trajectory execution
  cell_inspection/        force-feedback contact routines
  cell_calibration/       eye-in-hand
  cell_fieldbus/          Modbus and EtherNet/IP bridge nodes
simulation/               cell worlds, part models
analysis/                 cycle time, grasp success
docs/

CONSTRAINTS
- Everything must run from a single command: `docker compose up` or `make demo`.
- No placeholder or stub code presented as working. If something is not implemented, say so.
- Every quantitative claim in the README must be reproducible from a script in this repo.
- Use only public or synthetic data. Record the licence of every dataset used.
- This is a clean-room reimplementation. Do not reproduce any proprietary design, protocol detail or dataset. Build the equivalent capability independently.

START BY
1. Confirming or challenging the proposed repository layout.
2. Listing the specific libraries and versions you intend to pin.
3. Identifying the single riskiest technical unknown and how to de-risk it first.
Then work through the milestones in the README build checklist, one at a time.

```

</details>

---

## Notes

- Do not backdate commits to match CV dates. Publishing older work today is normal and expected.
- Record dataset licences before training on anything. Several common research datasets are
  non-commercial only.
- Flip this repository to public once there is something in it worth showing.
