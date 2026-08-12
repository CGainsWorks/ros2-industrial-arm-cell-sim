"""Publish the cell description, optionally with RViz and joint sliders."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_gui = LaunchConfiguration("use_gui")

    description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("cell_description"), "urdf", "arm_cell.urdf.xacro"]),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(use_gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                condition=IfCondition(use_rviz),
                arguments=[
                    "-d",
                    PathJoinSubstitution([FindPackageShare("cell_description"), "rviz", "cell.rviz"]),
                ],
            ),
        ]
    )
