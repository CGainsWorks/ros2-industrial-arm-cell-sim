#!/usr/bin/env python3
"""ROS 2 node that collects hand-eye samples and solves for the extrinsic.

    ros2 run cell_calibration hand_eye_node
    ros2 service call /hand_eye/capture std_srvs/srv/Trigger
    ros2 service call /hand_eye/solve std_srvs/srv/Trigger

Each ``capture`` records the current flange pose from tf together with the
board pose the detector published. ``solve`` runs the Tsai-Lenz solve over the
accumulated samples and publishes the result, both as a transform on
``~/flange_to_camera`` and as a static tf broadcast so the rest of the stack
picks it up immediately.

WHAT THE NODE REFUSES TO DO

It refuses to solve from a pose set whose rotation axes do not span. That check
is the reason this node exists rather than a script: on a real robot the poses
are collected by a person jogging the arm, the natural set is a grid at roughly
constant orientation, and ``AX = XB`` then leaves the translation of X
unobservable while still returning a confident-looking answer with a small
residual. ``analysis/calibration_residual.py`` measures how large that error
gets -- several millimetres, with every field diagnostic reading clean -- and
``analysis/calibration_sensitivity.py`` converts it into lost grasps.

So: if the largest angle between any two motion axes is under
``min_axis_spread``, the service fails with a message saying which way to
rotate the wrist. Refusing is the useful behaviour. Reporting the residual and
letting someone commission it is not.
"""

from __future__ import annotations

from typing import List

import numpy as np

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    import tf2_ros
except ImportError as error:  # pragma: no cover - ROS is not present offline
    raise SystemExit(
        "rclpy is not available. This node needs a sourced ROS 2 Humble workspace; "
        "the offline half of this repository does not."
    ) from error

from scipy.spatial.transform import Rotation

from cell_calibration.core.hand_eye import CalibrationSample, calibrate


class HandEyeNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_eye")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("flange_frame", "tool0")
        self.declare_parameter("min_samples", 8)
        self.declare_parameter("min_axis_spread_deg", 25.0)

        self.samples: List[CalibrationSample] = []
        self.board_pose = None

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.create_subscription(PoseStamped, "~/board_pose", self._on_board, 1)
        self.publisher = self.create_publisher(TransformStamped, "~/flange_to_camera", 1)
        self.create_service(Trigger, "~/capture", self._capture)
        self.create_service(Trigger, "~/solve", self._solve)
        self.create_service(Trigger, "~/reset", self._reset)
        self.get_logger().info("hand-eye calibration ready; call ~/capture at each pose")

    def _on_board(self, message: PoseStamped) -> None:
        self.board_pose = _from_pose(message)

    def _capture(self, request, response):
        if self.board_pose is None:
            response.success = False
            response.message = "no board detection received"
            return response
        try:
            stamped = self.buffer.lookup_transform(
                self.get_parameter("base_frame").value,
                self.get_parameter("flange_frame").value,
                rclpy.time.Time(),
            )
        except tf2_ros.TransformException as error:
            response.success = False
            response.message = f"no flange transform: {error}"
            return response

        self.samples.append(
            CalibrationSample(
                flange_pose=_from_transform(stamped),
                board_in_camera=self.board_pose,
                reprojection_rms=float("nan"),
                visible_corners=0,
            )
        )
        response.success = True
        response.message = f"{len(self.samples)} samples"
        self.get_logger().info(response.message)
        return response

    def _solve(self, request, response):
        minimum = int(self.get_parameter("min_samples").value)
        if len(self.samples) < minimum:
            response.success = False
            response.message = f"{len(self.samples)} samples; need at least {minimum}"
            return response

        try:
            result = calibrate(self.samples)
        except ValueError as error:
            response.success = False
            response.message = str(error)
            return response

        spread = float(self.get_parameter("min_axis_spread_deg").value)
        if result.rotation_axis_spread_deg < spread:
            response.success = False
            response.message = (
                f"rotation axes span only {result.rotation_axis_spread_deg:.1f} degrees "
                f"(need {spread:.0f}). The poses are too alike in orientation: the translation "
                f"of the result is not observable from them. Roll the wrist between poses and "
                f"tilt the camera off vertical, then capture again."
            )
            self.get_logger().error(response.message)
            return response

        self._publish(result.transform)
        response.success = True
        response.message = (
            f"solved from {result.sample_count} samples / {result.pair_count} pairs: "
            f"consistency {result.consistency_translation_mm:.2f} mm, "
            f"{result.consistency_rotation_deg:.3f} deg; "
            f"board spread {result.board_spread_mm:.2f} mm; "
            f"axis spread {result.rotation_axis_spread_deg:.0f} deg"
        )
        self.get_logger().info(response.message)
        return response

    def _reset(self, request, response):
        self.samples = []
        response.success = True
        response.message = "samples cleared"
        return response

    def _publish(self, transform: np.ndarray) -> None:
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter("flange_frame").value
        message.child_frame_id = "camera_optical_frame"
        message.transform.translation.x = float(transform[0, 3])
        message.transform.translation.y = float(transform[1, 3])
        message.transform.translation.z = float(transform[2, 3])
        quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])
        self.publisher.publish(message)
        self.broadcaster.sendTransform(message)


def _from_pose(message: PoseStamped) -> np.ndarray:
    pose = np.eye(4)
    orientation = message.pose.orientation
    pose[:3, :3] = Rotation.from_quat(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    ).as_matrix()
    pose[:3, 3] = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
    return pose


def _from_transform(stamped) -> np.ndarray:
    pose = np.eye(4)
    rotation = stamped.transform.rotation
    pose[:3, :3] = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
    translation = stamped.transform.translation
    pose[:3, 3] = [translation.x, translation.y, translation.z]
    return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandEyeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
