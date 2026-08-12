"""The whole cell: description, MoveIt2, the fieldbus bridge and the cycle.

    ros2 launch cell_bringup cell.launch.py

Arguments
    use_rviz    (true)   start RViz with the cell view
    use_gazebo  (false)  spawn the cell in Gazebo as well
    auto_run    (false)  run cycles on a timer instead of waiting for the PLC
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
    auto_run = LaunchConfiguration("auto_run")

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
            DeclareLaunchArgument("use_gazebo", default_value="false"),
            DeclareLaunchArgument("auto_run", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": description}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare("cell_motion"), "launch", "moveit.launch.py"])]
                ),
            ),
            Node(
                package="cell_fieldbus",
                executable="modbus_bridge_node",
                output="screen",
                parameters=[PathJoinSubstitution([FindPackageShare("cell_fieldbus"), "config", "fieldbus.yaml"])],
            ),
            Node(
                package="cell_grasping",
                executable="grasp_planner_node",
                output="screen",
                parameters=[PathJoinSubstitution([FindPackageShare("cell_grasping"), "config", "grasping.yaml"])],
            ),
            Node(
                package="cell_inspection",
                executable="contact_probe_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("cell_inspection"), "config", "inspection.yaml"])
                ],
            ),
            Node(
                package="cell_motion",
                executable="trajectory_executor_node",
                output="screen",
            ),
            Node(
                package="cell_bringup",
                executable="cycle_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("cell_bringup"), "config", "cell.yaml"]),
                    {"auto_run": auto_run},
                ],
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
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                condition=IfCondition(use_gazebo),
                arguments=["-topic", "robot_description", "-entity", "arm_cell"],
            ),
        ]
    )
