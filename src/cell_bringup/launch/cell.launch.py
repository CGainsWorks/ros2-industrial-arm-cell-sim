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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# A Command() substitution evaluates to a string, and launch parses parameter
# values as YAML unless told otherwise. A URDF is not YAML, so an unwrapped
# robot_description makes the consuming node die at startup with "Unable to
# parse the value of parameter robot_description as yaml". Wrapping it in
# ParameterValue(..., value_type=str) is the fix.
#
# Worth knowing: this cannot be caught by importing the launch file and calling
# generate_launch_description(), which is what CI does -- the substitution is
# built there, not evaluated. It only appears when a node actually starts.


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
    auto_run = LaunchConfiguration("auto_run")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_gazebo", default_value="false"),
            DeclareLaunchArgument("auto_run", default_value="false"),
            # The control stack brings up robot_state_publisher as well as the
            # controller manager, so it is not started again here. Two
            # robot_state_publishers on one robot_description is a race for
            # /tf that resolves differently on every run.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare("cell_motion"), "launch", "control.launch.py"])]
                ),
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
