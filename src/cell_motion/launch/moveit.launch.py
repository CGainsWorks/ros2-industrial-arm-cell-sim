"""MoveIt2 with OMPL, planning against the cell's collision scene.

The scene is not built by hand in RViz and saved: it is published by
``cell_bringup``'s scene node from ``simulation/cell.yaml``, which is the same
file the offline planner reads. A collision scene that lives in a saved RViz
state is a collision scene that is one edit away from disagreeing with the cell.
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _config(package, *path):
    return PathJoinSubstitution([FindPackageShare(package), *path])


def generate_launch_description():
    description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            _config("cell_description", "urdf", "arm_cell.urdf.xacro"),
        ]
    )
    semantic = Command(["cat ", _config("cell_motion", "moveit_config", "arm_cell.srdf")])

    parameters = [
        {"robot_description": description},
        {"robot_description_semantic": semantic},
        _config("cell_motion", "moveit_config", "kinematics.yaml"),
        _config("cell_motion", "moveit_config", "ompl_planning.yaml"),
        _config("cell_motion", "moveit_config", "joint_limits.yaml"),
        _config("cell_motion", "moveit_config", "controllers.yaml"),
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
