#!/usr/bin/env python3
"""Ask MoveIt to plan a motion, execute it, and check the arm arrived.

    ros2 launch cell_bringup cell.launch.py use_rviz:=false &
    python3 scripts/verify_ros_stack.py

This is the check that closes the gap between "the stack starts" and "the stack
works". Everything else in this repository is verified offline or by a build;
this one needs a running controller manager, a running move_group, and a real
action round trip, so it lives in a script that CI runs inside the ROS image
rather than in the pytest suite, which must stay ROS-free.

WHAT IT ASSERTS, IN ORDER

1. ``/joint_states`` is being published, and reports six joints.
2. The ``/move_action`` server is available (that is the action; the node
   is ``/move_group``).
3. A joint-space goal plans *and executes*, and the arm ends up within
   tolerance of the requested configuration.
4. A goal that is inside the conveyor is refused rather than executed. A
   planner that will happily drive the tool through the belt is worse than no
   planner, and "it planned something" is not evidence that it planned
   something safe.

The fourth is the one worth having. The first three prove the plumbing; the
fourth proves the collision scene reached MoveIt, which is the thing the whole
generated-description argument in docs/architecture.md rests on.
"""

from __future__ import annotations

import sys
import time
from typing import List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory  # noqa: F401  (import proves the action exists)
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
HOME = [0.3007, -1.4599, -1.4034, -1.8490, 1.5708, -1.2701]

# A modest joint-space move from home: enough that arriving proves execution,
# small enough to stay in free space.
TARGET = [0.10, -1.30, -1.30, -1.80, 1.5708, -1.20]

# A configuration whose tool sits 60 mm *below* the belt surface, at the middle
# of the pick region. The arm can reach it and the collision scene must refuse
# it.
#
# These numbers are not guessed. The first version of this check used a
# plausible-looking set of joint angles labelled "inside the conveyor", MoveIt
# planned and executed it, and the check duly failed -- because the pose was
# actually 1.47 m in the air with 177 mm of clearance. The offline checker in
# cell_motion.core.arm was asked for a configuration with genuinely negative
# clearance, and reported this one at -30 mm against the conveyor. A test whose
# fixture is wrong tests nothing, and reports it as a failure of the thing being
# tested.
INSIDE_THE_CONVEYOR = [2.9249, 0.5205, -1.0601, 2.1103, 1.5708, 1.3541]

GOAL_TOLERANCE = 0.05  # rad


class Verifier(Node):
    def __init__(self) -> None:
        super().__init__("verify_ros_stack")
        self.joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        # The action is /move_action. The *node* is /move_group, and reaching
        # for the node name is the obvious mistake: the client then waits
        # forever on a server that does not exist while move_group sits there
        # perfectly healthy. `ros2 action list` is the thing to check.
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def _on_joint_state(self, message: JointState) -> None:
        self.joint_state = message

    def current(self) -> Optional[List[float]]:
        if self.joint_state is None:
            return None
        lookup = dict(zip(self.joint_state.name, self.joint_state.position))
        if not all(name in lookup for name in JOINTS):
            return None
        return [lookup[name] for name in JOINTS]

    def wait_for_joint_states(self, timeout: float = 60.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.current() is not None:
                return True
        return False

    def plan_and_execute(self, target: List[float], timeout: float = 60.0):
        """Send a joint-space MoveGroup goal. Returns the MoveIt error code."""
        request = MotionPlanRequest()
        request.group_name = "arm"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.35

        constraints = Constraints()
        for name, position in zip(JOINTS, target):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(position)
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False  # Plan *and* execute.

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=timeout)
        handle = send.result()
        if handle is None or not handle.accepted:
            return None

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        outcome = result.result()
        return None if outcome is None else outcome.result.error_code.val

    def settle(self, seconds: float = 2.0) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)


def main() -> int:
    rclpy.init()
    node = Verifier()
    failures = []

    def check(label: str, ok: bool, detail: str = "") -> None:
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f" -- {detail}" if detail else ""))
        if not ok:
            failures.append(label)

    try:
        print("verifying the running ROS stack")

        ok = node.wait_for_joint_states()
        check("/joint_states published with all six arm joints", ok)
        if not ok:
            return 1
        start = node.current()
        print(f"         start: {[round(v, 3) for v in start]}")
        check(
            "the arm started at the home configuration",
            max(abs(a - b) for a, b in zip(start, HOME)) < 0.05,
            "initial_value in the generated ros2_control block",
        )

        ok = node.client.wait_for_server(timeout_sec=60.0)
        check("move_group action server available", ok)
        if not ok:
            return 1

        code = node.plan_and_execute(TARGET)
        check("a free-space joint goal plans and executes", code == 1, f"MoveIt error code {code}")

        node.settle()
        reached = node.current()
        error = max(abs(a - b) for a, b in zip(reached, TARGET)) if reached else float("inf")
        print(f"         reached: {[round(v, 3) for v in reached]}")
        check(
            "the arm arrived at the goal",
            error < GOAL_TOLERANCE,
            f"worst joint error {error:.4f} rad (tolerance {GOAL_TOLERANCE})",
        )

        code = node.plan_and_execute(INSIDE_THE_CONVEYOR, timeout=40.0)
        check(
            "a goal inside the conveyor is refused",
            code != 1,
            f"MoveIt error code {code} (1 would mean it happily planned through the belt)",
        )

        print()
        if failures:
            print(f"FAILED: {len(failures)} check(s): " + "; ".join(failures))
            return 1
        print("all checks passed: the stack plans, executes, arrives, and refuses a collision")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
