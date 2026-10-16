#!/usr/bin/env python3
"""ROS 2 node wrapping the Modbus bridge.

Thin by design. Everything about the protocol -- the register map, the edge
detection, the fault latching, the heartbeat -- is in
:mod:`cell_fieldbus.core.registers` and :mod:`cell_fieldbus.core.bridge`, which
are tested over a real socket without ROS in ``tests/test_fieldbus.py``. This
file is the part that cannot be tested that way, and it is kept small so that
there is little in it to be wrong.

    ros2 run cell_fieldbus modbus_bridge_node --ros-args -p port:=5020

Topics
    ``~/state`` (std_msgs/UInt16)      the cell state register
    ``~/fault`` (std_msgs/UInt16)      the latched fault code
    ``~/cycle_start`` (std_msgs/Empty) published once per rising edge of the
                                       PLC's start coil
Subscriptions
    ``~/cycle_result`` (std_msgs/Float32MultiArray)
        [success, cycle_seconds, grasp_score, part_id, fault_code, stage_code]
        from the cycle node, which is what updates the status block.
    ``/safety/safety_status`` (std_msgs/UInt16)
        the safety node's status word, which becomes ``safety_status`` and,
        on a protective stop, fault 94.
"""

from __future__ import annotations

import asyncio
import threading

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty, Float32MultiArray, UInt16
except ImportError as error:  # pragma: no cover - ROS is not present offline
    raise SystemExit(
        "rclpy is not available. This node needs a sourced ROS 2 Humble workspace; "
        "the offline half of this repository does not."
    ) from error

from cell_fieldbus.core.bridge import BridgeConfig, ModbusBridge
from cell_fieldbus.core.registers import Fault, State


class ModbusBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("modbus_bridge")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5020)
        self.declare_parameter("unit_id", 1)
        self.declare_parameter("scan_period", 0.02)
        self.declare_parameter("publish_period", 0.1)

        self.bridge = ModbusBridge(
            BridgeConfig(
                host=self.get_parameter("host").value,
                port=int(self.get_parameter("port").value),
                unit_id=int(self.get_parameter("unit_id").value),
                scan_period=float(self.get_parameter("scan_period").value),
            )
        )

        self.state_publisher = self.create_publisher(UInt16, "~/state", 10)
        self.fault_publisher = self.create_publisher(UInt16, "~/fault", 10)
        self.start_publisher = self.create_publisher(Empty, "~/cycle_start", 10)
        self.create_subscription(Float32MultiArray, "~/cycle_result", self._on_result, 10)
        self.create_subscription(UInt16, "/safety/safety_status", self._on_safety, 10)
        self.create_timer(float(self.get_parameter("publish_period").value), self._publish)

        # The bridge is asyncio and rclpy is not. Rather than trying to marry
        # the two event loops -- which works until something in either blocks --
        # the server runs on its own thread with its own loop, and the two sides
        # exchange only the plain dataclass in `bridge.state`.
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"modbus bridge listening on {self.bridge.config.host}:{self.bridge.config.port} "
            f"unit {self.bridge.config.unit_id}"
        )

    def _serve(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self.bridge.start(on_start=self._on_start_request))
        self._loop.run_forever()

    def _on_start_request(self) -> None:
        self.start_publisher.publish(Empty())
        self.get_logger().info("cycle start requested by the PLC")

    def _on_result(self, message: Float32MultiArray) -> None:
        values = list(message.data)
        if len(values) < 6:
            self.get_logger().warn(f"cycle_result needs 6 fields, got {len(values)}")
            return
        success, seconds, score, part_id, fault_code, stage_code = values[:6]
        self.bridge.state.record_cycle(bool(success), float(seconds), float(score), int(part_id))
        if not success:
            self.bridge.state.status.fault = Fault(int(fault_code))
            self.bridge.state.status.state = State.FAULTED
            self.bridge.state.status.last_fault_stage = int(stage_code)
        self.bridge.publish(self.bridge.state.status)

    def _on_safety(self, message: UInt16) -> None:
        self.bridge.state.set_safety(int(message.data))

    def _publish(self) -> None:
        status = self.bridge.state.status
        self.state_publisher.publish(UInt16(data=int(status.state)))
        self.fault_publisher.publish(UInt16(data=int(status.fault)))

    def destroy_node(self) -> bool:
        try:
            asyncio.run_coroutine_threadsafe(self.bridge.stop(), self._loop).result(timeout=2.0)
            self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception:  # pragma: no cover - shutdown is best effort
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModbusBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
