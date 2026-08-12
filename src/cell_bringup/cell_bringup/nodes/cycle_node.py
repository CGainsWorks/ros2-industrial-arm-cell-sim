#!/usr/bin/env python3
"""ROS 2 node driving the pick-place-inspect cycle from the fieldbus.

    ros2 run cell_bringup cycle_node

Waits for a cycle-start edge from the Modbus bridge, runs a cycle, and reports
the result back to the bridge as the register block the PLC reads.

Subscriptions
    ``/modbus_bridge/cycle_start`` (std_msgs/Empty)
Publications
    ``/modbus_bridge/cycle_result`` (std_msgs/Float32MultiArray)
    ``~/state`` (std_msgs/String)  a human-readable stage, for the log

WHAT THIS NODE IS AND IS NOT

The cycle logic -- the stage order, what each stage is allowed to know, how a
failure is classified -- lives in :mod:`cell_bringup.core.cycle`, which runs
without ROS and is what the 200-trial benchmark exercises. This node is the
part that talks to the fieldbus and the clock.

It runs the cycle against the *simulated* cell by default, which is what makes
``make demo`` do something on a machine with no robot attached. Pointing it at
real hardware means replacing the ``CycleRunner`` with one whose motion and
gripper calls go to the driver, and the seam for that is deliberately narrow:
the runner's only outward-facing operations are "plan and execute this
trajectory", "set the jaws to this opening", and "capture a depth frame".
"""

from __future__ import annotations

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty, Float32MultiArray, String
except ImportError as error:  # pragma: no cover - ROS is not present offline
    raise SystemExit(
        "rclpy is not available. This node needs a sourced ROS 2 Humble workspace; "
        "the offline half of this repository does not."
    ) from error

from cell_bringup.core.cycle import CycleRunner
from cell_common.cell_model import load_cell, sample_part_pose
from cell_fieldbus.core.registers import STAGE_CODES, Fault, fault_for


class CycleNode(Node):
    def __init__(self) -> None:
        super().__init__("cycle")
        self.declare_parameter("seed", 20260101)
        self.declare_parameter("auto_run", False)
        self.declare_parameter("auto_period", 12.0)

        self.cell = load_cell()
        self.runner = CycleRunner(self.cell)
        self.counter = 0
        self.seed = int(self.get_parameter("seed").value)

        self.result_publisher = self.create_publisher(Float32MultiArray, "/modbus_bridge/cycle_result", 10)
        self.state_publisher = self.create_publisher(String, "~/state", 10)
        self.create_subscription(Empty, "/modbus_bridge/cycle_start", self._on_start, 10)

        if bool(self.get_parameter("auto_run").value):
            self.create_timer(float(self.get_parameter("auto_period").value), lambda: self._on_start(Empty()))
            self.get_logger().info("auto-run enabled; a cycle will start on the timer")
        self.get_logger().info("cycle node ready, waiting for a start edge")

    def _on_start(self, _message) -> None:
        rng = np.random.default_rng(self.seed + self.counter)
        part_pose = sample_part_pose(self.cell, rng)
        self.state_publisher.publish(String(data=f"running: {part_pose['part']}"))

        result = self.runner.run(part_pose, rng)
        self.counter += 1

        try:
            fault = fault_for(result.failure)
        except KeyError as error:
            # An unmapped failure must not reach the PLC as "no fault". Report
            # it as a generic planning fault and log loudly enough that the map
            # gets fixed.
            self.get_logger().error(str(error))
            fault = Fault.PLANNING_FAILED

        message = Float32MultiArray()
        message.data = [
            float(result.success),
            float(result.cycle_time),
            float(result.grasp.score if result.grasp else 0.0),
            float(self._part_id(part_pose["part"])),
            float(int(fault)),
            float(STAGE_CODES.get(result.stage, 0)),
        ]
        self.result_publisher.publish(message)
        self.state_publisher.publish(
            String(data=f"{result.stage}: {result.failure or 'ok'} in {result.cycle_time:.2f} s")
        )
        self.get_logger().info(
            f"cycle {self.counter}: {'ok' if result.success else result.stage + '/' + result.failure} "
            f"in {result.cycle_time:.2f} s (fault {int(fault)})"
        )

    def _part_id(self, name: str) -> int:
        names = [part.name for part in self.cell.parts]
        return names.index(name) + 1 if name in names else 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
