"""Hand-eye calibration: the solver node and the description it needs."""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
                parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
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
