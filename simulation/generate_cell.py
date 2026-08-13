#!/usr/bin/env python3
"""Generate the cell: collision scene, URDF and Gazebo world, from one table.

Run it with ``make cell``. It writes four artefacts, all committed:

    simulation/cell.yaml                       the machine-readable cell
    simulation/cell.world                      Gazebo SDF for the same cell
    src/cell_description/urdf/ur5e.urdf.xacro  arm links, from the DH table
    src/cell_description/urdf/cell.urdf.xacro  fixtures, conveyor, camera mount
    src/cell_description/urdf/ros2_control.urdf.xacro  the hardware interface

WHY GENERATE RATHER THAN HAND-WRITE

Three descriptions of the same cell have to agree: the one the planner checks
collisions against, the one MoveIt2 and RViz draw, and the one Gazebo
simulates. Hand-maintained, they drift. In an early revision of this repository
the conveyor was 120 mm taller in the URDF than in the collision checker, and
the planner happily routed approach trajectories straight through the belt --
the offline benchmark reported a grasp success rate that the simulated cell
would not have reproduced, which is exactly the class of result this repository
exists to avoid.

The arm description is generated from the same ``D``, ``A`` and ``ALPHA``
arrays the inverse kinematics uses, so the URDF joint frames are the DH frames
by construction rather than by a transcription that has to be checked.

CI re-runs this script and fails if anything under ``simulation/`` or
``src/cell_description/urdf/`` changes, so a geometry edit that was not
regenerated cannot be merged.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
import sys  # noqa: E402

sys.path.insert(0, str(REPO_ROOT / "src" / "cell_motion"))
sys.path.insert(0, str(REPO_ROOT / "src" / "cell_common"))

from cell_motion.core.kinematics import A, ALPHA, D, JOINT_NAMES, LINK_RADII  # noqa: E402

# ---------------------------------------------------------------------------
# The cell. Metres and radians throughout, world origin on the floor at the
# centre of the robot pedestal, x towards the conveyor, z up.
# ---------------------------------------------------------------------------

BASE_HEIGHT = 0.75  # Top of the pedestal; the arm base frame sits here.
CONVEYOR_TOP = 0.70
FIXTURE_TOP = 0.78

OBSTACLES: List[Dict] = [
    # Robot mount. The arm's own first capsule overlaps this by design, so the
    # planner ignores it for link 1 and checks it for everything else.
    dict(name="pedestal", size=[0.30, 0.30, BASE_HEIGHT], xyz=[0.0, 0.0, BASE_HEIGHT / 2], colour="0.25 0.25 0.28 1"),
    # Infeed conveyor, running left to right across the front of the robot.
    dict(name="conveyor", size=[0.40, 1.10, CONVEYOR_TOP], xyz=[0.63, 0.0, CONVEYOR_TOP / 2],
         colour="0.35 0.35 0.38 1"),
    dict(name="conveyor_rail_far", size=[0.03, 1.10, 0.06], xyz=[0.815, 0.0, CONVEYOR_TOP + 0.03],
         colour="0.6 0.6 0.1 1"),
    # Camera and light-curtain pillar beside the belt. This is the obstacle the
    # approach trajectory has to plan around for parts at the far end of the
    # pick region, and it is responsible for most of the collision-class
    # failures in the benchmark.
    dict(name="camera_pillar", size=[0.08, 0.08, 1.30], xyz=[0.62, 0.62, 0.65], colour="0.5 0.5 0.55 1"),
    # Outfeed bench carrying the inspection nest.
    dict(name="bench", size=[0.50, 0.40, 0.72], xyz=[0.0, -0.60, 0.36], colour="0.35 0.35 0.38 1"),
    dict(name="inspection_nest", size=[0.30, 0.24, 0.06], xyz=[0.0, -0.60, 0.75], colour="0.2 0.45 0.6 1"),
    # Cell boundary. Not decoration: without the cabinet the planner is free to
    # swing the elbow through where the controller actually lives.
    dict(name="control_cabinet", size=[0.30, 0.50, 1.10], xyz=[-0.55, 0.30, 0.55], colour="0.3 0.32 0.36 1"),
    dict(name="rear_guard", size=[0.04, 1.80, 1.60], xyz=[-0.80, 0.0, 0.80], colour="0.45 0.5 0.45 0.4"),
]

# Where parts arrive: the region the part *centre* is drawn from.
#
# The upper x bound is set by the far guide rail, not by the arm. The longest
# part is 110 mm, so a centre at x = 0.72 puts its far edge at 0.775 and the
# rail's near face is at 0.800. Pushing the region out to the rail instead
# produced part poses that intersected it -- physically impossible, and the
# perception stage then clustered part and rail together and reported a 490 mm
# "part", which is how the bound came to be here.
PART_REGION = dict(x=[0.50, 0.72], y=[-0.22, 0.22], yaw=[-np.pi, np.pi])

# How far outside the region the perception stage looks. Enough to contain a
# part whose centre is at the region edge, and not so far as to reach the rail.
REGION_MARGIN = 0.07

# Synthetic part catalogue. No client CAD: these are generic sheet-metal and
# moulded shapes with dimensions chosen to span the gripper's stroke, including
# two that are marginal for it.
PARTS: List[Dict] = [
    dict(name="bracket_a", size=[0.062, 0.038, 0.024], mass=0.14, friction=0.7),
    dict(name="bracket_b", size=[0.048, 0.048, 0.030], mass=0.11, friction=0.7),
    dict(name="connector_shell", size=[0.090, 0.032, 0.026], mass=0.09, friction=0.6),
    dict(name="spacer_block", size=[0.055, 0.055, 0.055], mass=0.31, friction=0.8),
    dict(name="pcb_coupon", size=[0.100, 0.070, 0.008], mass=0.05, friction=0.5),
    dict(name="wide_plate", size=[0.110, 0.078, 0.012], mass=0.16, friction=0.55),
]

# Robotiq 2F-85 class parallel gripper.
GRIPPER = dict(
    tcp_offset=0.163,  # Flange to the point between the fingertips, along tool z.
    max_opening=0.085,
    min_opening=0.000,
    finger_length=0.042,
    finger_width=0.022,
    finger_thickness=0.014,
    palm_size=[0.086, 0.070, 0.098],
    close_time=0.55,  # Seconds for a full close, from the datasheet's 0.03-0.07 m/s.
    grip_force=90.0,  # Newtons at the default force setting.
)

# Eye-in-hand camera. The extrinsic here is ground truth: it is what the
# simulated camera actually uses, and what the calibration routine has to
# recover without being told. Optical convention, +z along the view axis.
CAMERA = dict(
    width=640,
    height=480,
    fx=617.2,
    fy=617.2,
    cx=319.5,
    cy=239.5,
    depth_noise_base=0.0012,  # Metres, constant term.
    depth_noise_quadratic=0.0026,  # Metres per metre squared, the D400-series shape.
    depth_min=0.16,
    depth_max=1.20,
    extrinsic_xyz=[0.0, -0.082, 0.041],
    extrinsic_rpy=[0.0, 0.0, -np.pi / 2],
)

# Inspection points on the part, as fractions of its footprint, probed after
# placing. Four corners and the centre: enough to catch a part sitting on swarf
# in the nest, which is the failure this routine exists to detect.
INSPECTION_POINTS = [[-0.3, -0.3], [0.3, -0.3], [0.0, 0.0], [-0.3, 0.3], [0.3, 0.3]]

# Joint configuration the cycle starts and ends at. Elbow up, wrist pointing
# down, clear of the belt and the pillar.
HOME_JOINTS = [0.3007, -1.4599, -1.4034, -1.8490, 1.5708, -1.2701]

# Where the wrist camera goes to look for the part, as a camera position and a
# point it looks at. Stored as a pose rather than as joint angles because the
# constraint is optical, not kinematic: the standoff is set by the field of
# view needed to cover the pick region.
#
#   horizontal half-angle  atan(320 / 617.2) = 27.4 deg
#   vertical half-angle    atan(240 / 617.2) = 21.3 deg
#
# At the 0.50 m standoff below that is +-0.259 m across the belt and +-0.195 m
# along it, which covers the pick region with roughly 40 mm of margin on the
# tight axis. Widen PART_REGION without raising the camera and parts at the
# edge fall outside the frame -- which the benchmark would report as a
# perception failure, correctly but unhelpfully.
SCAN_CAMERA_XYZ = [0.58, 0.0, CONVEYOR_TOP + 0.50]
SCAN_TARGET_XYZ = [0.58, 0.0, CONVEYOR_TOP]


def build_specification() -> Dict:
    """Assemble the dictionary that becomes ``simulation/cell.yaml``."""
    return {
        "frames": {
            "arm_base_xyz": [0.0, 0.0, BASE_HEIGHT],
            "conveyor_top": CONVEYOR_TOP,
            "fixture_top": FIXTURE_TOP,
            "place_xyz": [0.0, -0.60, FIXTURE_TOP],
            "scan_camera_xyz": [float(value) for value in SCAN_CAMERA_XYZ],
            "scan_target_xyz": [float(value) for value in SCAN_TARGET_XYZ],
        },
        "obstacles": [
            {k: (list(v) if isinstance(v, list) else v) for k, v in entry.items() if k != "colour"}
            for entry in OBSTACLES
        ],
        "part_region": {k: [float(v[0]), float(v[1])] for k, v in PART_REGION.items()},
        "region_margin": REGION_MARGIN,
        "parts": PARTS,
        "gripper": GRIPPER,
        "camera": CAMERA,
        "inspection_points": INSPECTION_POINTS,
        "home_joints": [float(value) for value in HOME_JOINTS],
        "joint_names": list(JOINT_NAMES),
    }


def write_yaml(path: Path, specification: Dict) -> None:
    header = (
        "# Generated by simulation/generate_cell.py. Do not edit by hand.\n"
        "#\n"
        "# This is the single source of truth for the cell geometry: the collision\n"
        "# checker, the URDF and the Gazebo world are all built from it.\n"
    )
    path.write_text(header + yaml.safe_dump(specification, sort_keys=True, default_flow_style=False), encoding="utf-8")


# ---------------------------------------------------------------------------
# URDF
# ---------------------------------------------------------------------------


def _xacro_header(name: str, comment: str) -> str:
    return (
        '<?xml version="1.0"?>\n'
        f"<!-- {comment} -->\n"
        f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{name}">\n'
    )


def arm_xacro() -> str:
    """UR5e links and joints, derived from the DH table.

    THE OFF-BY-ONE THAT MAKES THIS WORTH GENERATING

    Standard DH composes as ``Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)``: the
    joint rotation comes *first*, then the link offset. URDF is the other way
    round -- a joint's ``<origin>`` is applied before its axis rotation. So the
    fixed offset that belongs to DH link ``i`` has to be written onto URDF joint
    ``i+1``, and the last one onto a fixed flange joint. Writing link ``i``'s own
    offset onto joint ``i``, which is the obvious thing to do and what the first
    version of this function did, silently produces the *modified* (proximal) DH
    convention instead: a robot that looks entirely plausible in RViz and whose
    forward kinematics disagrees with the IK solver by up to 130 mm.

    ``tests/test_description.py`` walks the generated joint chain with an XML
    parser and compares it against :func:`forward_kinematics` at random
    configurations, which is what caught it.

    The visual and collision geometry is a cylinder spanning each link's offset,
    sized from the same ``LINK_RADII`` the collision capsules use, so what RViz
    draws is what the planner checks.
    """
    lines = [
        _xacro_header(
            "ur5e",
            "Generated by simulation/generate_cell.py from the public UR5e DH table. Do not edit by hand.",
        ),
        '  <xacro:macro name="ur5e" params="prefix parent *origin">\n',
        '    <joint name="${prefix}base_fixed_joint" type="fixed">\n',
        "      <xacro:insert_block name=\"origin\"/>\n",
        '      <parent link="${parent}"/>\n',
        '      <child link="${prefix}base_link"/>\n',
        "    </joint>\n",
        '    <link name="${prefix}base_link">\n',
        "      <visual>\n"
        '        <origin xyz="0 0 0.03" rpy="0 0 0"/>\n'
        '        <geometry><cylinder radius="0.075" length="0.06"/></geometry>\n'
        '        <material name="ur_grey"><color rgba="0.6 0.6 0.62 1"/></material>\n'
        "      </visual>\n",
        "      <inertial>\n"
        '        <mass value="4.0"/>\n'
        '        <inertia ixx="0.0072" ixy="0" ixz="0" iyy="0.0072" iyz="0" izz="0.0125"/>\n'
        "      </inertial>\n",
        "    </link>\n",
    ]

    parent = "${prefix}base_link"
    # Approximate UR5e link masses, in order, from the published payload sheet.
    masses = [3.7, 8.393, 2.33, 1.219, 1.219, 0.1879]
    for index, name in enumerate(JOINT_NAMES):
        child = f"${{prefix}}{name.replace('_joint', '')}_link"
        radius = float(LINK_RADII[index])

        # The fixed offset carried by *this* joint is the previous DH link's,
        # for the reason in the docstring. Joint 1 carries none.
        if index == 0:
            offset_a = offset_d = offset_alpha = 0.0
        else:
            offset_a, offset_d, offset_alpha = float(A[index - 1]), float(D[index - 1]), float(ALPHA[index - 1])

        # The visible body spans this link's own offset, from the joint frame
        # to (a_i, 0, d_i). A cylinder's axis is its local z, so the pitch that
        # takes z onto that direction is atan2(a_i, d_i).
        own_a, own_d = float(A[index]), float(D[index])
        length = float(np.hypot(own_a, own_d))
        pitch = float(np.arctan2(own_a, own_d))
        body = (
            f'        <origin xyz="{own_a / 2:.6f} 0 {own_d / 2:.6f}" rpy="0 {pitch:.6f} 0"/>\n'
            f'        <geometry><cylinder radius="{radius:.4f}" length="{max(length, 0.06):.4f}"/></geometry>\n'
        )
        lines.append(
            f'    <joint name="${{prefix}}{name}" type="revolute">\n'
            f'      <origin xyz="{offset_a:.6f} 0 {offset_d:.6f}" rpy="{offset_alpha:.6f} 0 0"/>\n'
            f'      <parent link="{parent}"/>\n'
            f'      <child link="{child}"/>\n'
            f'      <axis xyz="0 0 1"/>\n'
            f'      <limit lower="-6.283185" upper="6.283185" effort="150.0" velocity="3.141593"/>\n'
            f"    </joint>\n"
        )
        lines.append(
            f'    <link name="{child}">\n'
            "      <visual>\n"
            + body
            + '        <material name="ur_blue"><color rgba="0.3 0.45 0.65 1"/></material>\n'
            "      </visual>\n"
            "      <collision>\n"
            + body
            + "      </collision>\n"
            f"      <inertial>\n"
            f'        <mass value="{masses[index]:.4f}"/>\n'
            f'        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>\n'
            f"      </inertial>\n"
            f"    </link>\n"
        )
        parent = child

    # Flange, tool and the eye-in-hand camera, all fixed to wrist 3. The flange
    # joint carries the last DH link's offset -- the one that has nowhere else
    # to go.
    tcp = GRIPPER["tcp_offset"]
    flange_a, flange_d, flange_alpha = float(A[5]), float(D[5]), float(ALPHA[5])
    cam_xyz = " ".join(f"{value:.6f}" for value in CAMERA["extrinsic_xyz"])
    cam_rpy = " ".join(f"{value:.6f}" for value in CAMERA["extrinsic_rpy"])
    lines.append(
        f'    <joint name="${{prefix}}tool0_joint" type="fixed">\n'
        f'      <origin xyz="{flange_a:.6f} 0 {flange_d:.6f}" rpy="{flange_alpha:.6f} 0 0"/>\n'
        f'      <parent link="{parent}"/>\n'
        f'      <child link="${{prefix}}tool0"/>\n'
        f"    </joint>\n"
        f'    <link name="${{prefix}}tool0"/>\n'
        f'    <joint name="${{prefix}}tcp_joint" type="fixed">\n'
        f'      <origin xyz="0 0 {tcp:.6f}" rpy="0 0 0"/>\n'
        f'      <parent link="${{prefix}}tool0"/>\n'
        f'      <child link="${{prefix}}tcp_link"/>\n'
        f"    </joint>\n"
        f'    <link name="${{prefix}}tcp_link"/>\n'
        f'    <joint name="${{prefix}}camera_joint" type="fixed">\n'
        f'      <origin xyz="{cam_xyz}" rpy="{cam_rpy}"/>\n'
        f'      <parent link="${{prefix}}tool0"/>\n'
        f'      <child link="${{prefix}}camera_optical_frame"/>\n'
        f"    </joint>\n"
        f'    <link name="${{prefix}}camera_optical_frame"/>\n'
    )
    lines.append("  </xacro:macro>\n</robot>\n")
    return "".join(lines)


def ros2_control_xacro() -> str:
    """The ros2_control hardware interface for the arm.

    Generated for the same reason as everything else here: the joint list and
    the home configuration are already in this file, and a hand-maintained
    ros2_control block is one rename away from declaring a command interface
    for a joint the robot does not have.

    The hardware plugin is ``mock_components/GenericSystem``, which is the
    ros2_control fake: it accepts position commands and echoes them straight
    back as state. That is exactly enough to give MoveIt a real controller to
    execute onto -- a real ``joint_trajectory_controller``, a real
    FollowJointTrajectory action, real joint states -- without a physics
    engine. What it does *not* provide is any dynamics: the arm tracks commands
    perfectly and instantly, so it proves the plan-and-execute path works and
    proves nothing about tracking error. ``docs/limitations.md`` says so.

    Each joint's position state is seeded with the home configuration, so the
    arm starts where the cycle expects it to rather than at the all-zeros pose,
    which on this arm is stretched horizontally through the conveyor.
    """
    lines = [
        _xacro_header(
            "cell_ros2_control",
            "Generated by simulation/generate_cell.py. Do not edit by hand.",
        ),
        '  <xacro:macro name="cell_ros2_control" params="name:=cell_hardware">\n',
        '    <ros2_control name="${name}" type="system">\n',
        "      <hardware>\n"
        "        <plugin>mock_components/GenericSystem</plugin>\n"
        "        <param name=\"calculate_dynamics\">true</param>\n"
        "      </hardware>\n",
    ]
    for index, name in enumerate(JOINT_NAMES):
        lines.append(
            f'      <joint name="{name}">\n'
            f'        <command_interface name="position">\n'
            f'          <param name="min">-6.283185</param>\n'
            f'          <param name="max">6.283185</param>\n'
            f"        </command_interface>\n"
            f'        <state_interface name="position">\n'
            f'          <param name="initial_value">{HOME_JOINTS[index]:.6f}</param>\n'
            f"        </state_interface>\n"
            f'        <state_interface name="velocity"/>\n'
            f"      </joint>\n"
        )
    lines.append("    </ros2_control>\n  </xacro:macro>\n</robot>\n")
    return "".join(lines)


def cell_xacro() -> str:
    """Fixtures, conveyor and guarding as fixed links on the world frame."""
    lines = [
        _xacro_header(
            "cell",
            "Generated by simulation/generate_cell.py from OBSTACLES. Do not edit by hand.",
        ),
        '  <xacro:macro name="cell_fixtures" params="parent">\n',
    ]
    for entry in OBSTACLES:
        name = entry["name"]
        size = " ".join(f"{value:.4f}" for value in entry["size"])
        xyz = " ".join(f"{value:.4f}" for value in entry["xyz"])
        colour = entry.get("colour", "0.4 0.4 0.4 1")
        lines.append(
            f'    <joint name="{name}_joint" type="fixed">\n'
            f'      <origin xyz="{xyz}" rpy="0 0 0"/>\n'
            f'      <parent link="${{parent}}"/>\n'
            f'      <child link="{name}_link"/>\n'
            f"    </joint>\n"
            f'    <link name="{name}_link">\n'
            f'      <visual><geometry><box size="{size}"/></geometry>\n'
            f'        <material name="{name}_material"><color rgba="{colour}"/></material></visual>\n'
            f'      <collision><geometry><box size="{size}"/></geometry></collision>\n'
            '      <inertial><mass value="0.0"/>'
            '<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>\n'
            f"    </link>\n"
        )
    lines.append("  </xacro:macro>\n</robot>\n")
    return "".join(lines)


# ---------------------------------------------------------------------------
# Gazebo world
# ---------------------------------------------------------------------------


def world_sdf() -> str:
    """SDF for the static cell furniture. The arm is spawned separately."""
    models = []
    for entry in OBSTACLES:
        size = " ".join(f"{value:.4f}" for value in entry["size"])
        xyz = " ".join(f"{value:.4f}" for value in entry["xyz"])
        colour = entry.get("colour", "0.4 0.4 0.4 1")
        models.append(
            f'    <model name="{entry["name"]}">\n'
            f"      <static>true</static>\n"
            f"      <pose>{xyz} 0 0 0</pose>\n"
            f'      <link name="link">\n'
            f'        <collision name="collision">\n'
            f'          <geometry><box><size>{size}</size></box></geometry>\n'
            f"        </collision>\n"
            f'        <visual name="visual">\n'
            f"          <geometry><box><size>{size}</size></box></geometry>\n"
            f"          <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material>\n"
            f"        </visual>\n"
            f"      </link>\n"
            f"    </model>\n"
        )
    return (
        '<?xml version="1.0"?>\n'
        "<!-- Generated by simulation/generate_cell.py. Do not edit by hand. -->\n"
        '<sdf version="1.7">\n'
        '  <world name="arm_cell">\n'
        "    <include><uri>model://sun</uri></include>\n"
        "    <include><uri>model://ground_plane</uri></include>\n"
        "    <physics type=\"ode\">\n"
        "      <max_step_size>0.001</max_step_size>\n"
        "      <real_time_update_rate>1000</real_time_update_rate>\n"
        "    </physics>\n" + "".join(models) + "  </world>\n</sdf>\n"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="Fail if any generated file would change")
    arguments = parser.parse_args()

    targets = {
        REPO_ROOT / "simulation" / "cell.world": world_sdf(),
        REPO_ROOT / "src" / "cell_description" / "urdf" / "ur5e.urdf.xacro": arm_xacro(),
        REPO_ROOT / "src" / "cell_description" / "urdf" / "cell.urdf.xacro": cell_xacro(),
        REPO_ROOT / "src" / "cell_description" / "urdf" / "ros2_control.urdf.xacro": ros2_control_xacro(),
    }

    specification = build_specification()
    yaml_path = REPO_ROOT / "simulation" / "cell.yaml"

    if arguments.check:
        stale = []
        for path, content in targets.items():
            if not path.exists() or path.read_text(encoding="utf-8") != content:
                stale.append(path)
        previous = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) if yaml_path.exists() else None
        if previous != specification:
            stale.append(yaml_path)
        if stale:
            for path in stale:
                print(f"stale: {path.relative_to(REPO_ROOT)}")
            return 1
        print("cell description is up to date")
        return 0

    for path, content in targets.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        print(f"wrote {path.relative_to(REPO_ROOT)}")
    write_yaml(yaml_path, specification)
    print(f"wrote {yaml_path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
