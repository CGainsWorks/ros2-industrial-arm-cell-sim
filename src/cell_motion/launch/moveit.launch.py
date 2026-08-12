"""MoveIt2 with OMPL, planning against the cell's collision scene.

The scene is not built by hand in RViz and saved: it is published from
``simulation/cell.yaml``, which is the same file the offline planner reads. A
collision scene that lives in a saved RViz state is a collision scene that is
one edit away from disagreeing with the cell.

TWO LAUNCH GOTCHAS, BOTH OF WHICH ONLY APPEAR AT RUNTIME

Neither is caught by importing this file and calling
``generate_launch_description()``, which is what CI does, because that builds
the description without starting anything.

1. **``robot_description`` must be wrapped.** A ``Command()`` substitution
   evaluates to a string, and launch parses parameter values as YAML unless
   told otherwise. A URDF is not YAML, so an unwrapped value makes move_group
   exit at startup with "Unable to parse the value of parameter
   robot_description as yaml". Hence ``ParameterValue(..., value_type=str)``.

2. **MoveIt's YAML files are not ROS 2 parameter files.** They are bare
   mappings -- ``arm: {kinematics_solver: ...}`` -- whereas ``--params-file``
   requires ``<node_name>: ros__parameters: ...``. Passing the paths directly
   gets "Failed to parse global arguments" and an immediate abort, which is a
   spectacularly unhelpful message for a YAML shape problem. So the files are
   loaded here and passed as dictionaries, which is what MoveIt's own config
   builder does, and it keeps the files in the shape a person reading
   ``ompl_planning.yaml`` expects.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(package: str, *relative) -> dict:
    """Read a MoveIt configuration file into a dict.

    Read at launch-description build time rather than handed over as a path,
    for the reason in the module docstring. A missing or malformed file
    therefore fails here, loudly, instead of inside a node that aborts with a
    parser error.
    """
    path = os.path.join(get_package_share_directory(package), *relative)
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def generate_launch_description():
    description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("cell_description"), "urdf", "arm_cell.urdf.xacro"]),
        ]
    )
    semantic = Command(
        [
            "cat ",
            PathJoinSubstitution([FindPackageShare("cell_motion"), "moveit_config", "arm_cell.srdf"]),
        ]
    )

    kinematics = _load_yaml("cell_motion", "moveit_config", "kinematics.yaml")
    ompl = _load_yaml("cell_motion", "moveit_config", "ompl_planning.yaml")
    joint_limits = _load_yaml("cell_motion", "moveit_config", "joint_limits.yaml")
    controllers = _load_yaml("cell_motion", "moveit_config", "controllers.yaml")

    parameters = [
        {"robot_description": ParameterValue(description, value_type=str)},
        {"robot_description_semantic": ParameterValue(semantic, value_type=str)},
        {"robot_description_kinematics": kinematics},
        {"robot_description_planning": joint_limits},
        # The pipeline has to be named as well as configured. Without these two
        # move_group loads whatever planner it can find -- CHOMP, on a default
        # MoveIt install -- and says so only in one line of its startup log.
        {"planning_pipelines": ["ompl"]},
        {"default_planning_pipeline": "ompl"},
        ompl,
        controllers,
        # Without a controller manager MoveIt plans happily and then fails to
        # execute with a message about no controller being found, which reads
        # like a planning problem and is not one.
        {"publish_robot_description_semantic": True},
        {"use_sim_time": False},
    ]

    return LaunchDescription(
        [
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=parameters,
            ),
        ]
    )
