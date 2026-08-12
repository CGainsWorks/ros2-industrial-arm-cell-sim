"""Hand-eye calibration: the solver node and the description it needs."""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("cell_description"), "urdf", "arm_cell.urdf.xacro"]),
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description}],
            ),
            Node(
                package="cell_calibration",
                executable="hand_eye_node",
                output="screen",
                parameters=[
                    {
                        "base_frame": "base_link",
                        "flange_frame": "tool0",
                        "min_samples": 8,
                        # Refuse to solve from poses that all point the same way.
                        # See the node docstring; this is the check that turns an
                        # unobservable calibration into an error instead of a
                        # plausible number.
                        "min_axis_spread_deg": 25.0,
                    }
                ],
            ),
        ]
    )
