#!/usr/bin/env python3
"""ROS 2 node running the force-feedback contact probe against a live wrench.

    ros2 run cell_inspection contact_probe_node

Subscriptions
    ``~/wrench`` (geometry_msgs/WrenchStamped)  the wrist force-torque reading
    ``~/probe`` (geometry_msgs/PointStamped)    a point to probe; triggers a descent
Publications
    ``~/command`` (geometry_msgs/TwistStamped)  tool-frame velocity command
    ``~/height`` (std_msgs/Float32)             measured surface height, metres
    ``~/fault`` (std_msgs/UInt16)               fault code, matching the fieldbus map

The admittance law, the contact-trigger logic, the deflection correction and
the fault codes come from :mod:`cell_inspection.core.contact` unchanged, so the
loop that runs against a simulated surface in ``analysis/inspection_accuracy.py``
and the loop that runs against a real sensor are the same arithmetic. What this
node adds is the part that cannot be shared: reading a real wrench, minus a
tare, at whatever rate the sensor publishes rather than a fixed 500 Hz.

THE TARE IS NOT OPTIONAL

A wrist sensor reads the tool's own weight, which changes with orientation and
with whether a part is held. Probing without re-taring at the current pose puts
a constant offset -- several newtons for this gripper -- straight into the
contact trigger. The node tares on the first ``~/probe`` request while the tool
is stationary above the surface, and republishes the tare it used so a log can
show what was subtracted.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

try:
    import rclpy
    from geometry_msgs.msg import PointStamped, TwistStamped, WrenchStamped
    from rclpy.node import Node
    from std_msgs.msg import Float32, UInt16
except ImportError as error:  # pragma: no cover - ROS is not present offline
    raise SystemExit(
        "rclpy is not available. This node needs a sourced ROS 2 Humble workspace; "
        "the offline half of this repository does not."
    ) from error

from cell_inspection.core.contact import (
    CONTACT_THRESHOLD,
    FAULT_NO_CONTACT,
    FAULT_NONE,
    FAULT_OVERFORCE,
    FORCE_FILTER_CUTOFF,
    MAX_TRAVEL,
    OVERFORCE_LIMIT,
    TARGET_FORCE,
    estimate_stiffness,
)


class ContactProbeNode(Node):
    def __init__(self) -> None:
        super().__init__("contact_probe")
        self.declare_parameter("approach_speed", 0.005)
        self.declare_parameter("nominal_stiffness", 20000.0)
        self.declare_parameter("command_rate", 125.0)

        self.filtered = 0.0
        self.tare = 0.0
        self.raw: Optional[float] = None
        self.travel = 0.0
        self.active = False
        self.contact_travel: Optional[float] = None
        self.contact_force = 0.0
        self.samples = []
        self.last_stamp: Optional[float] = None

        self.create_subscription(WrenchStamped, "~/wrench", self._on_wrench, 20)
        self.create_subscription(PointStamped, "~/probe", self._on_probe, 1)
        self.command_publisher = self.create_publisher(TwistStamped, "~/command", 10)
        self.height_publisher = self.create_publisher(Float32, "~/height", 1)
        self.fault_publisher = self.create_publisher(UInt16, "~/fault", 1)
        self.create_timer(1.0 / float(self.get_parameter("command_rate").value), self._tick)
        self.get_logger().info("contact probe ready")

    def _on_wrench(self, message: WrenchStamped) -> None:
        """Filter the vertical force, with the time constant referred to the
        message timestamps rather than to an assumed rate.

        A first-order filter written as ``y += alpha * (x - y)`` with a constant
        alpha is only the cutoff it claims to be at the rate it was tuned for.
        A sensor publishing at 100 Hz instead of 500 gives five times the
        intended time constant, which delays the contact trigger by tens of
        milliseconds and drives the probe that much deeper before it notices.
        """
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        period = 0.002 if self.last_stamp is None else max(stamp - self.last_stamp, 1e-4)
        self.last_stamp = stamp

        self.raw = float(message.wrench.force.z)
        force = abs(self.raw - self.tare)
        alpha = float(np.clip(2.0 * np.pi * FORCE_FILTER_CUTOFF * period, 0.0, 1.0))
        self.filtered += alpha * (force - self.filtered)

    def _on_probe(self, message: PointStamped) -> None:
        if self.raw is None:
            self.get_logger().error("no wrench received; refusing to probe blind")
            return
        self.tare = self.raw
        self.filtered = 0.0
        self.travel = 0.0
        self.contact_travel = None
        self.samples = []
        self.active = True
        self.get_logger().info(
            f"probing ({message.point.x:.3f}, {message.point.y:.3f}), tared at {self.tare:.2f} N"
        )

    def _tick(self) -> None:
        if not self.active:
            return
        period = 1.0 / float(self.get_parameter("command_rate").value)
        speed = float(self.get_parameter("approach_speed").value)

        if self.filtered > OVERFORCE_LIMIT:
            self._finish(FAULT_OVERFORCE, None)
            return

        if self.contact_travel is None:
            if self.filtered > CONTACT_THRESHOLD:
                self.contact_travel = self.travel
                self.contact_force = self.filtered
                self.get_logger().info(f"contact at {self.travel * 1000:.2f} mm, {self.filtered:.2f} N")
            elif self.travel > MAX_TRAVEL:
                self._finish(FAULT_NO_CONTACT, None)
                return
        else:
            self.samples.append((self.travel - self.contact_travel, self.filtered))
            if len(self.samples) > int(0.25 / period):
                stiffness = estimate_stiffness(self.samples, float(self.get_parameter("nominal_stiffness").value))
                self._finish(FAULT_NONE, self.contact_travel - self.contact_force / stiffness)
                return

        velocity = (self.filtered - TARGET_FORCE) / (TARGET_FORCE / speed)
        self.travel += -velocity * period

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.twist.linear.z = float(velocity)
        self.command_publisher.publish(command)

    def _finish(self, fault: int, depth: Optional[float]) -> None:
        self.active = False
        self.command_publisher.publish(TwistStamped(header=self._stamp()))
        self.fault_publisher.publish(UInt16(data=int(fault)))
        if depth is not None:
            self.height_publisher.publish(Float32(data=float(depth)))
            self.get_logger().info(f"surface at {depth * 1000:.3f} mm of travel")
        else:
            self.get_logger().warn(f"probe finished with fault {fault}")

    def _stamp(self):
        from std_msgs.msg import Header

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        return header


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ContactProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
