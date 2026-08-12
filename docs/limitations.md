# Limitations

What this repository does not do, stated plainly. Everything here is a real
gap between the simulation and a cell you could commission, and each one is
listed with what it would cost to close.

## The physics

**No dynamics.** The arm is kinematic. Trajectories are checked against
velocity and acceleration limits and timed accordingly, but nothing computes
joint torques during a move, and nothing simulates the arm deflecting under
load. The one place dynamics is modelled at all is the single-axis plant in
`cell_motion/core/joint_control.py`, which exists for the tuning harness and is
not connected to the cell.

**Rigid parts, rigid pads.** A grasp holds or does not, decided by geometry and
a friction cone. Nothing deforms, nothing rolls in the jaws, and the part does
not move as the fingers close on it. Real parallel-jaw grasps often *centre*
the part as they close, which forgives a millimetre or two of lateral error;
this model does not, so its off-centre failures are pessimistic.

**No conveyor motion.** Parts are stationary when scanned and stationary when
picked. A moving belt would add tracking, latency compensation and a second
class of failure entirely.

**Point-contact probing.** The inspection probe is a point. A real fingertip
has a radius, so a probe landing near an edge reads the edge and one landing on
a chamfer reads somewhere in between. Robot structural compliance is lumped
into the contact stiffness rather than modelled as a series spring, which is
acceptable for a vertical probe and wrong for a lateral one.

## The sensing

**Matte boxes only.** The depth camera model has range-dependent Gaussian
noise, grazing-incidence dropout, quantisation and a working range. It has no
model of specular reflection, transparency, ambient IR or stereo multi-path.
Those are the things that actually break depth cameras in a factory, and no
amount of Gaussian noise represents them. The parts here are matte cuboids on a
matte belt, which is the easy case.

**Parts are boxes, and the estimator fits a box.** The perception pipeline fits
a minimum-area rectangle to a cluster, and the parts are rectangular. That
agreement is convenient and it flatters the result. A bracket with a hole, a
moulding with a draft angle, or two parts touching would all need a different
estimator, and `docs/grasp-planning.md` says which failure each would produce.

**One camera, one view.** No multi-view fusion, no scanning of the region from
two poses. A part occluded by another is simply not seen.

## The planning

**Capsules, not meshes.** The offline collision model is capsules against
oriented boxes. It over-approximates the arm everywhere, so a configuration it
calls clear is clear for the mesh too -- but the converse does not hold, and
the offline planner therefore refuses some poses a mesh checker would allow.
MoveIt2 checks the real meshes through FCL when the full stack is up.

**KDL, not IKFast, inside MoveIt.** The offline solver is the closed form and
enumerates all eight branches. The MoveIt configuration ships with KDL, which
is numerical and seed-dependent, so MoveIt will occasionally refuse a grasp
pose the offline planner accepts. Generating an IKFast plugin needs OpenRAVE,
which is a heavier dependency than this repository takes on.

**No optimality.** RRT-Connect plus randomised shortcutting. The paths are
collision-free and roughly direct; they are not optimal in length, clearance or
time, and a CHOMP or TrajOpt pass would shorten the cycle.

**No corner blending in the executed path.** The time parameterisation bounds
the speed at each waypoint so the direction change fits inside a blend window,
but it does not generate the blend geometry. A real controller rounds the
corner; this reports the time as though it tracked the polyline exactly.

## The fieldbus

**Modbus is real. EtherNet/IP is not.** The Modbus/TCP bridge is a working
pymodbus server with a register map, edge-triggered commands, latched faults
and a heartbeat, and `tests/test_fieldbus.py` drives it over a socket. There is
no EtherNet/IP implementation. `docs/fieldbus.md` describes what the equivalent
CIP assembly instances would carry, which is not the same as having written
them, and claiming the protocol would be worse than not having it.

**No safety layer.** No dual-channel E-stop, no safety-rated speed monitoring,
no light-curtain logic beyond a pillar that exists as an obstacle. A cell with
a person near it needs all of that and none of it is here.

## The results

**The nominal grasp benchmark is a benign scenario.** One part, no occlusion,
a correct hand-eye transform, a healthy camera, a pick region comfortably
inside the arm's envelope. The success rate under those conditions is high, and
it should be. That is why the benchmark also reports clutter, degraded sensing
and calibration error as separate conditions, and why
`analysis/calibration_sensitivity.py` exists -- the sensitivity curve is a more
useful number than the headline.

**The cycle time excludes everything outside the cycle.** No PLC decision time
beyond the two handshakes, no part indexing on the belt, no tool changes, no
recovery from a failed pick. A real line takt would be longer.

**Success is judged by a model, not by physics.** `evaluate_grasp` applies six
geometric and frictional criteria. They are the right criteria and they are not
a rigid-body simulation. A Gazebo or MuJoCo grasp with real contact dynamics
would disagree with it at the margins, and the direction of that disagreement
is unknown.

**Everything is one machine, one core.** No timing measured on real hardware,
no controller latency, no network jitter between the vision PC and the robot.

## What is deliberately not here

- **No employer or client material.** No proprietary CAD, no calibration data,
  no cell drawings, no protocol details from any product. The part catalogue is
  synthetic, the cell is invented, and the only externally-sourced data is the
  publicly published UR5e DH table.
- **No large binaries.** No recorded bags, no meshes, no model weights. The
  figures under `docs/plots/` are matplotlib output of a few tens of kilobytes
  and are regenerated by `make tuning`.
- **No trained models.** The perception is classical throughout: RANSAC,
  Euclidean clustering, a rotating-calipers rectangle fit. A learned grasp
  quality network would be a reasonable next step and would need a dataset,
  which would need a licence, which would need recording.
