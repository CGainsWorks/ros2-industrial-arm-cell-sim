#!/usr/bin/env python3
"""ROS 2 node exposing the grasp planner.

Takes a depth image and the camera's pose, runs the same perception and grasp
code the offline benchmark runs, and publishes the ranked candidates as a
``PoseArray`` with their scores alongside.

    ros2 run cell_grasping grasp_planner_node

Subscriptions
    ``~/depth`` (sensor_msgs/Image, 16UC1 or 32FC1)
    ``~/camera_info`` (sensor_msgs/CameraInfo)   overrides the intrinsics in
                                                 simulation/cell.yaml if given
Publications
    ``~/grasps`` (geometry_msgs/PoseArray)        best first, in the world frame
    ``~/scores`` (std_msgs/Float32MultiArray)     one score per pose, same order
    ``~/markers`` (visualization_msgs/MarkerArray) approach axes, for RViz

The camera pose comes from tf: ``world`` to the frame in the image header. That
is deliberately not the cell's ground-truth extrinsic -- the node uses whatever
the calibration published, which is the whole point of having calibrated it.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

try:
    import rclpy
    from geometry_msgs.msg import Pose, PoseArray
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Float32MultiArray
    from visualization_msgs.msg import Marker, MarkerArray
    import tf2_ros
except ImportError as error:  # pragma: no cover - ROS is not present offline
    raise SystemExit(
        "rclpy is not available. This node needs a sourced ROS 2 Humble workspace; "
        "the offline half of this repository does not."
    ) from error

from scipy.spatial.transform import Rotation

from cell_common.cell_model import load_cell
from cell_common.transforms import from_rpy
from cell_grasping.core.camera import DepthImage, deproject, median_filter
from cell_grasping.core.grasp import generate_candidates
from cell_grasping.core.perception import detect_scene
from cell_motion.core.arm import ArmModel


class GraspPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_planner")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("max_published", 8)
        self.declare_parameter("median_filter", True)

        self.cell = load_cell()
        self.arm = ArmModel.from_cell(self.cell)
        self.camera = self.cell.camera
        self.rng = np.random.default_rng(0)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.create_subscription(Image, "~/depth", self._on_depth, 1)
        self.create_subscription(CameraInfo, "~/camera_info", self._on_camera_info, 1)
        self.pose_publisher = self.create_publisher(PoseArray, "~/grasps", 1)
        self.score_publisher = self.create_publisher(Float32MultiArray, "~/scores", 1)
        self.marker_publisher = self.create_publisher(MarkerArray, "~/markers", 1)
        self.get_logger().info("grasp planner ready")

    def _on_camera_info(self, message: CameraInfo) -> None:
        """Replace the intrinsics from the cell file with the published ones.

        A node that silently keeps using a configured intrinsic while a camera
        driver publishes a different one produces grasps that are wrong by a
        few millimetres and correct in every log.
        """
        import dataclasses

        self.camera = dataclasses.replace(
            self.camera,
            fx=float(message.k[0]),
            fy=float(message.k[4]),
            cx=float(message.k[2]),
            cy=float(message.k[5]),
            width=int(message.width),
            height=int(message.height),
        )

    def _lookup_camera_pose(self, frame: str) -> Optional[np.ndarray]:
        try:
            stamped = self.buffer.lookup_transform(
                self.get_parameter("world_frame").value, frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException as error:
            self.get_logger().warn(f"no transform to {frame}: {error}", throttle_duration_sec=5.0)
            return None
        translation = stamped.transform.translation
        rotation = stamped.transform.rotation
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
        pose[:3, 3] = [translation.x, translation.y, translation.z]
        return pose

    def _on_depth(self, message: Image) -> None:
        camera_pose = self._lookup_camera_pose(message.header.frame_id)
        if camera_pose is None:
            return

        depth = _decode_depth(message)
        if depth is None:
            self.get_logger().warn(f"unsupported depth encoding {message.encoding!r}")
            return

        image = DepthImage(depth=depth, pose=camera_pose, camera=self.camera, stride=1)
        if bool(self.get_parameter("median_filter").value):
            image = median_filter(image)

        estimate, clutter, _ = detect_scene(
            deproject(image), self.rng, self.cell.conveyor_top, workspace=self.cell.perception_workspace()
        )
        if estimate is None:
            self.get_logger().info("no part found in the region of interest")
            self.pose_publisher.publish(PoseArray(header=message.header))
            return

        for index, other in enumerate(clutter):
            self.arm.scene.replace(
                _box(other, f"clutter_{index}")
            )
        try:
            plan = generate_candidates(estimate, self.arm, reference_joints=self.cell.home_joints)
        finally:
            for index in range(len(clutter)):
                self.arm.scene.remove(f"clutter_{index}")

        limit = int(self.get_parameter("max_published").value)
        candidates = plan.top(limit)

        poses = PoseArray()
        poses.header.stamp = message.header.stamp
        poses.header.frame_id = self.get_parameter("world_frame").value
        scores = Float32MultiArray()
        markers = MarkerArray()
        for index, candidate in enumerate(candidates):
            poses.poses.append(_to_pose(candidate.pose))
            scores.data.append(float(candidate.score))
            markers.markers.append(_to_marker(candidate, index, poses.header))

        self.pose_publisher.publish(poses)
        self.score_publisher.publish(scores)
        self.marker_publisher.publish(markers)
        self.get_logger().info(
            f"{len(plan.candidates)} candidates from {plan.considered} considered; "
            f"rejections {plan.rejections}"
        )


def _box(estimate, name: str):
    from cell_motion.core.collision import Box

    return Box(
        name=name,
        size=np.asarray(estimate.extents, dtype=float),
        pose=from_rpy(0.0, 0.0, estimate.yaw, xyz=estimate.centre),
        kind="clutter",
    )


def _decode_depth(message) -> Optional[np.ndarray]:
    """Depth image to metres, NaN where the sensor returned nothing.

    A zero in a 16UC1 depth image means "no return", not "zero metres". Treating
    it as a measurement puts a plane of phantom points at the optical centre,
    which the plane fit then happily latches onto.
    """
    if message.encoding == "16UC1":
        raw = np.frombuffer(message.data, dtype=np.uint16).reshape(message.height, message.width)
        depth = raw.astype(float) / 1000.0
        return np.where(raw == 0, np.nan, depth)
    if message.encoding == "32FC1":
        depth = np.frombuffer(message.data, dtype=np.float32).reshape(message.height, message.width)
        return np.where(depth <= 0.0, np.nan, depth.astype(float))
    return None


def _to_pose(matrix: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(value) for value in matrix[:3, 3])
    quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(value) for value in quaternion
    )
    return pose


def _to_marker(candidate, index: int, header) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = "grasp"
    marker.id = index
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose = _to_pose(candidate.pregrasp_pose)
    marker.scale.x, marker.scale.y, marker.scale.z = 0.08, 0.006, 0.006
    marker.color.a = 1.0
    # Best candidate green, fading to red down the ranking.
    marker.color.g = float(candidate.score)
    marker.color.r = float(1.0 - candidate.score)
    return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
