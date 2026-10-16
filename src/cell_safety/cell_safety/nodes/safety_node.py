#!/usr/bin/env python3
"""ROS 2 node: the simulated light curtain and the category 1 protective stop.

SIMULATED, NOT CERTIFIED. A real protective stop is a hard-wired function of
certified hardware. This node models its behaviour on the live ROS stack so
that the cell's reaction can be exercised end to end against
``ros2_control``, and ``test/test_protective_stop.launch.py`` does that in CI.

    ros2 run cell_safety safety_node

Subscriptions
    ``~/curtain_blocked`` (std_msgs/Bool)  the simulated world: true while
                                           something is in the protected field
    ``/joint_states`` (sensor_msgs/JointState)
Publications
    ``~/protective_stop`` (std_msgs/Bool)   latched stop, for the fieldbus
    ``~/safety_status`` (std_msgs/UInt16)   the REG_SAFETY_STATUS bit field
    ``/joint_trajectory_controller/joint_trajectory`` (trajectory_msgs/JointTrajectory)
                                            the controlled stop
Services
    ``~/reset`` (std_srvs/Trigger)          the manual reset button

WHAT HAPPENS ON A BREAK

The curtain model turns the field state into two OSSD channels after its
response time; the safety controller in :mod:`cell_safety.core.protective_stop`
evaluates them every tick. On a stop request the node sends the joint
trajectory controller a short trajectory that brings every joint from its
current velocity to rest at the stop deceleration, which replaces whatever
trajectory was running (MoveIt's included). Once the arm is still, it
deactivates the controller through the controller manager: the category 1
"remove power" step, as far as mock hardware has power to remove. The reset
service re-activates it, but only once the field is clear and the arm is
still, and the cycle does not resume on its own.

The node ticks at ``rate`` (default 250 Hz). A Python timer is not a safety
controller's scan, and the launch test allows for that.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

try:
    import rclpy
    from controller_manager_msgs.srv import SwitchController
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, UInt16
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ImportError as error:  # pragma: no cover - ROS is not present offline
    raise SystemExit(
        "rclpy is not available. This node needs a sourced ROS 2 Humble workspace; "
        "the offline half of this repository does not."
    ) from error

from cell_safety.core.light_curtain import LightCurtain, LightCurtainSpec
from cell_safety.core.protective_stop import SafetyController, SafetyParameters, SafetyState

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# REG_SAFETY_STATUS bits, as in cell_fieldbus.core.registers. Repeated rather
# than imported so this node does not depend on the fieldbus package.
CURTAIN_CLEAR, PROTECTIVE_STOP, RESET_REQUIRED, DRIVES_ENABLED, LOCKOUT = 1, 2, 4, 8, 16

STANDSTILL_VELOCITY = 1e-3  # rad/s, every joint.
VELOCITY_WINDOW = 0.008  # s of joint-state time per velocity estimate.


class SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("safety")
        self.declare_parameter("rate", 250.0)
        self.declare_parameter("response_time", LightCurtainSpec().response_time)
        self.declare_parameter("restart_delay", LightCurtainSpec().restart_delay)
        self.declare_parameter("controller", "joint_trajectory_controller")

        spec = LightCurtainSpec(
            response_time=float(self.get_parameter("response_time").value),
            restart_delay=float(self.get_parameter("restart_delay").value),
        )
        period = 1.0 / float(self.get_parameter("rate").value)
        self.parameters = SafetyParameters(controller_period=period)
        self.curtain = LightCurtain(spec)
        self.safety = SafetyController(self.parameters)
        self.controller_name = str(self.get_parameter("controller").value)

        self.blocked = False
        self.reset_pending = False
        self.positions: Optional[np.ndarray] = None
        self.velocities = np.zeros(len(JOINTS))
        self._reference = None
        self.stop_sent = False
        self.drives_off = False
        self.switch_in_flight = False
        self.start = time.monotonic()

        self.create_subscription(Bool, "~/curtain_blocked", self._on_curtain, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)
        self.stop_publisher = self.create_publisher(Bool, "~/protective_stop", 10)
        self.status_publisher = self.create_publisher(UInt16, "~/safety_status", 10)
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, f"/{self.controller_name}/joint_trajectory", 10
        )
        self.switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"safety node up: curtain response {spec.response_time * 1000:.0f} ms, "
            f"tick {period * 1000:.1f} ms, category {int(self.parameters.category)} stop. Simulated, not certified."
        )

    # -- inputs ----------------------------------------------------------------

    def _on_curtain(self, message: Bool) -> None:
        self.blocked = bool(message.data)

    def _on_joint_state(self, message: JointState) -> None:
        """Positions as reported; velocities differenced from them.

        Not the message's velocity field: mock hardware with position command
        interfaces does not have to fill it, and a stop computed from a
        velocity of zero is an instant freeze, not a category 1 stop.
        Differenced over at least ``VELOCITY_WINDOW`` of header-stamp time, so
        a burst of messages delivered together does not divide by nothing.
        """
        lookup = dict(zip(message.name, message.position))
        if not all(name in lookup for name in JOINTS):
            return
        positions = np.array([lookup[name] for name in JOINTS])
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self._reference is None:
            self._reference = (stamp, positions)
        else:
            elapsed = stamp - self._reference[0]
            if elapsed >= VELOCITY_WINDOW:
                self.velocities = (positions - self._reference[1]) / elapsed
                self._reference = (stamp, positions)
            elif elapsed < 0:
                self._reference = (stamp, positions)
        self.positions = positions

    def _on_reset(self, _request, response):
        self.reset_pending = True
        response.success = True
        response.message = "reset requested; accepted only with the field clear and the arm still"
        return response

    # -- the tick ----------------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic() - self.start
        standstill = bool(np.all(np.abs(self.velocities) < STANDSTILL_VELOCITY))
        ossd1, ossd2 = self.curtain.step_blocked(now, self.blocked)
        reset, self.reset_pending = self.reset_pending, False
        was_stopped = self.safety.state != SafetyState.RUN
        outputs = self.safety.step(now, ossd1, ossd2, reset=reset, standstill=standstill)

        if outputs.stop_requested and not self.stop_sent:
            self._send_stop_trajectory()
            self.stop_sent = True
            self.get_logger().warn("light curtain broken: protective stop")
        if not outputs.drives_enabled and not self.drives_off:
            self._switch(activate=False)
            self.drives_off = True
        if was_stopped and outputs.state == SafetyState.RUN:
            self._switch(activate=True)
            self.stop_sent = False
            self.drives_off = False
            self.get_logger().info("protective stop reset; the cycle must be restarted")

        word = 0
        word |= CURTAIN_CLEAR if outputs.curtain_clear else 0
        word |= PROTECTIVE_STOP if outputs.stop_requested else 0
        word |= RESET_REQUIRED if outputs.reset_required else 0
        word |= DRIVES_ENABLED if outputs.drives_enabled else 0
        word |= LOCKOUT if outputs.state == SafetyState.LOCKOUT else 0
        self.stop_publisher.publish(Bool(data=outputs.stop_requested))
        self.status_publisher.publish(UInt16(data=word))

    # -- outputs -----------------------------------------------------------------

    def _send_stop_trajectory(self) -> None:
        """Bring every joint from its current velocity to rest, together."""
        if self.positions is None:
            return
        deceleration = np.asarray(self.parameters.stop_deceleration, dtype=float)
        velocity = self.velocities
        duration = max(float(np.max(np.abs(velocity) / deceleration)), 0.02)
        message = JointTrajectory()
        message.joint_names = list(JOINTS)
        for step in range(1, 11):
            t = duration * step / 10.0
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in self.positions + velocity * t - 0.5 * velocity / duration * t * t]
            point.velocities = [float(v) for v in velocity * (1.0 - t / duration)]
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            message.points.append(point)
        self.trajectory_publisher.publish(message)

    def _switch(self, activate: bool) -> None:
        if not self.switch_client.service_is_ready():
            self.get_logger().error("controller manager switch service is not available")
            return
        request = SwitchController.Request()
        if activate:
            request.activate_controllers = [self.controller_name]
        else:
            request.deactivate_controllers = [self.controller_name]
        request.strictness = SwitchController.Request.BEST_EFFORT
        self.switch_client.call_async(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # A shutdown signal can land while rclpy is taking a message, which
        # surfaces as a conversion error from the dead context. Only that.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
