#!/usr/bin/env python3
"""ROS 2 node that turns a tool pose into an executed joint trajectory.

Solves IK for the requested pose, plans to it with the same planner the offline
benchmark uses, time-parameterises the result, and sends it to
``/joint_trajectory_controller/follow_joint_trajectory``.

    ros2 run cell_motion trajectory_executor_node

Subscriptions
    ``~/target_pose`` (geometry_msgs/PoseStamped)   free-space move, planned
    ``~/linear_pose`` (geometry_msgs/PoseStamped)   straight line in tool space
    ``~/joint_states`` (sensor_msgs/JointState)     the current configuration
Publications
    ``~/planned`` (trajectory_msgs/JointTrajectory) what was sent, for logging

WHY THE PLANNING IS HERE AND NOT IN MOVEIT

It is in both. ``cell_motion/launch/moveit.launch.py`` brings up MoveIt2 with
OMPL and the collision scene built from ``simulation/cell.yaml``, and that is
what runs when the full stack is up. This node exists for the case MoveIt2
handles badly and this cell needs constantly: a guaranteed straight-line
approach along the tool axis with a hard rejection if the IK branch changes
anywhere along it. ``compute_cartesian_path`` returns a *fraction* of the path
it managed, and a caller that executes an 87% path drives the tool to
somewhere that is not the grasp.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ImportError as error:  # pragma: no cover - ROS is not present offline
    raise SystemExit(
        "rclpy is not available. This node needs a sourced ROS 2 Humble workspace; "
        "the offline half of this repository does not."
    ) from error

from scipy.spatial.transform import Rotation

from cell_common.cell_model import load_cell
from cell_motion.core.arm import ArmModel
from cell_motion.core.planner import make_validity, plan_cartesian, plan_rrt_connect, shortcut
from cell_motion.core.trajectory import Trajectory, time_parameterise


class TrajectoryExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_executor")
        self.declare_parameter("velocity_scale", 0.5)
        self.declare_parameter("acceleration_scale", 0.35)
        self.declare_parameter("gripper_opening", 0.085)
        self.declare_parameter("controller", "/joint_trajectory_controller/follow_joint_trajectory")

        self.cell = load_cell()
        self.arm = ArmModel.from_cell(self.cell)
        self.rng = np.random.default_rng(0)
        self.joints: Optional[np.ndarray] = None

        self.create_subscription(JointState, "~/joint_states", self._on_joint_state, 10)
        self.create_subscription(PoseStamped, "~/target_pose", self._on_target, 1)
        self.create_subscription(PoseStamped, "~/linear_pose", self._on_linear, 1)
        self.publisher = self.create_publisher(JointTrajectory, "~/planned", 1)
        self.client = ActionClient(self, FollowJointTrajectory, self.get_parameter("controller").value)
        self.get_logger().info("trajectory executor ready")

    def _on_joint_state(self, message: JointState) -> None:
        """Reorder the incoming state into the solver's joint order.

        Never index a JointState by position. The order is whatever the
        publisher felt like, it is allowed to change between messages, and a
        mis-ordered configuration produces a plan that is valid for a robot
        that does not exist.
        """
        lookup = dict(zip(message.name, message.position))
        try:
            self.joints = np.array([lookup[name] for name in self.cell.joint_names])
        except KeyError:
            missing = set(self.cell.joint_names) - set(message.name)
            self.get_logger().warn(f"joint state is missing {sorted(missing)}", throttle_duration_sec=5.0)

    def _on_target(self, message: PoseStamped) -> None:
        if self.joints is None:
            self.get_logger().warn("no joint state yet")
            return
        pose = _from_pose(message)
        solution = self.arm.ik_nearest(pose, self.joints)
        if solution is None:
            self.get_logger().error("target pose has no IK solution")
            return

        opening = float(self.get_parameter("gripper_opening").value)
        validity = make_validity(self.arm, opening)
        if not validity(solution.joints):
            self.get_logger().error("target configuration is in collision")
            return

        result = plan_rrt_connect(self.joints, solution.joints, validity, self.rng)
        if not result.ok:
            self.get_logger().error(f"planning failed: {result.reason}")
            return
        path = shortcut(result.path, validity, self.rng)
        self._send(time_parameterise(path, *self._scales(), label="free"))

    def _on_linear(self, message: PoseStamped) -> None:
        if self.joints is None:
            self.get_logger().warn("no joint state yet")
            return
        opening = float(self.get_parameter("gripper_opening").value)
        result = plan_cartesian(self.arm, self.joints, _from_pose(message), opening)
        if not result.ok:
            self.get_logger().error(f"cartesian move refused: {result.reason}")
            return
        self._send(time_parameterise(result.path, *self._scales(), label="linear"))

    def _scales(self):
        return (
            float(self.get_parameter("velocity_scale").value),
            float(self.get_parameter("acceleration_scale").value),
        )

    def _send(self, trajectory: Trajectory) -> None:
        message = JointTrajectory()
        message.joint_names = list(self.cell.joint_names)
        velocities = trajectory.velocities()
        for index, time in enumerate(trajectory.times):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in trajectory.positions[index]]
            point.velocities = [float(value) for value in velocities[index]]
            point.time_from_start.sec = int(time)
            point.time_from_start.nanosec = int((time - int(time)) * 1e9)
            message.points.append(point)

        self.publisher.publish(message)
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("trajectory controller is not available")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = message
        self.client.send_goal_async(goal)
        self.get_logger().info(
            f"sent {trajectory.label} trajectory: {len(message.points)} points, "
            f"{trajectory.duration:.2f} s, peak {trajectory.peak_velocity_fraction() * 100:.0f}% of the velocity limit"
        )


def _from_pose(message: PoseStamped) -> np.ndarray:
    pose = np.eye(4)
    orientation = message.pose.orientation
    pose[:3, :3] = Rotation.from_quat(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    ).as_matrix()
    pose[:3, 3] = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
    return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
