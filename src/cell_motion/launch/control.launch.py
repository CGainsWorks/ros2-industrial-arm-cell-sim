"""The ros2_control stack: controller manager, joint states, trajectory controller.

    ros2 launch cell_motion control.launch.py

Brings up ``ros2_control_node`` against the description's hardware interface,
then spawns the joint state broadcaster and the joint trajectory controller.
This is what gives MoveIt something to execute onto: a real
``FollowJointTrajectory`` action at
``/joint_trajectory_controller/follow_joint_trajectory``, and real
``/joint_states``.

WHAT THE HARDWARE IS

``mock_components/GenericSystem`` -- the ros2_control fake. It accepts position
commands and echoes them back as state. That is enough to exercise the whole
plan-and-execute path end to end, and it has no dynamics whatsoever: the arm
tracks perfectly and instantly. It proves the *path* works and proves nothing
about tracking error, which ``docs/limitations.md`` states plainly.

The spawners are sequenced deliberately. A controller spawned before the
controller manager is listening fails with "Controller manager not available",
and a joint trajectory controller active before the broadcaster leaves MoveIt
without joint states to plan from.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("cell_description"), "urdf", "arm_cell.urdf.xacro"]),
        ]
    )

    controllers = os.path.join(
        get_package_share_directory("cell_motion"), "config", "ros2_controllers.yaml"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            # Humble's ros2_control_node takes the description as a parameter.
            # Later distributions read it from a topic; passing it here is what
            # this one needs, and it must be a string for the same reason
            # move_group's copy must be.
            {"robot_description": ParameterValue(description, value_type=str)},
            controllers,
        ],
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
    )

    broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            control_node,
            state_publisher,
            broadcaster_spawner,
            # Only once the broadcaster has finished spawning, so that joint
            # states exist before anything can be asked to move.
            RegisterEventHandler(
                OnProcessExit(target_action=broadcaster_spawner, on_exit=[trajectory_spawner])
            ),
        ]
    )
