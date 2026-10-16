"""Launch test: break the light curtain mid-motion and the live arm stops.

    launch_test src/cell_safety/test/test_protective_stop.launch.py

Headless. Brings up ``ros2_control`` on mock hardware (controller manager,
joint state broadcaster, joint trajectory controller) and the safety node,
then plays the PLC and the world:

1. sends the trajectory controller a two-second, 1.5 rad move of the shoulder;
2. half way through, puts something in the curtain;
3. measures, from ``/joint_states``, the time from the break to the arm
   standing still, and checks it against ``LIVE_BUDGET``;
4. checks the controller was deactivated (category 1: power off at
   standstill): it refuses goals and the arm does not move;
5. clears the curtain, presses reset, and checks the arm moves again.

The offline model in ``tests/test_safety.py`` states a 220 ms budget at the
500 Hz controller rate. This test runs on a shared CI runner, with a Python
safety node ticking at 250 Hz and DDS between every process, so it allows
``LIVE_BUDGET`` = 500 ms and reports the measured time in its output. The
deterministic claim is the offline one; this one shows the same chain works
on the real controller stack.

Simulated, not certified.
"""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
LIVE_BUDGET = 0.5  # s, beam broken to standstill, on a shared runner.
MOVE = 1.5  # rad of shoulder pan.
MOVE_TIME = 2.0  # s.


@pytest.mark.launch_test
def generate_test_description():
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("cell_motion"), "launch", "control.launch.py"])]
        )
    )
    safety = Node(
        package="cell_safety",
        executable="safety_node",
        name="safety",
        output="screen",
        parameters=[PathJoinSubstitution([FindPackageShare("cell_safety"), "config", "safety.yaml"])],
    )
    return launch.LaunchDescription([control, safety, launch_testing.actions.ReadyToTest()])


def _duration(seconds: float) -> Duration:
    return Duration(sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9))


class TestProtectiveStop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("protective_stop_test")
        cls.samples = []  # (monotonic time, positions)
        cls.node.create_subscription(JointState, "/joint_states", cls._on_joint_state, 100)
        cls.curtain = cls.node.create_publisher(Bool, "/safety/curtain_blocked", 10)
        cls.reset = cls.node.create_client(Trigger, "/safety/reset")
        cls.action = ActionClient(
            cls.node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joint_state(cls, message):
        # Timed by the header stamp, which the broadcaster sets from the
        # system clock when the controller manager read the state. Receipt
        # time would add however far this Python process has fallen behind.
        lookup = dict(zip(message.name, message.position))
        if all(name in lookup for name in JOINTS):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            cls.samples.append((stamp, [lookup[name] for name in JOINTS]))

    # -- helpers ---------------------------------------------------------------

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def spin_until(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if predicate():
                return
        self.fail(f"timed out after {timeout} s waiting for {what}")

    # Whether the controller is active is read from its behaviour, not from
    # /controller_manager/list_controllers. Humble's controller manager dies
    # with "failed to send response" if a freshly created service client
    # calls before its response reader is matched, and taking the controller
    # manager down is a poor way to ask it a question. The trajectory
    # controller's own action server answers instead: it accepts goals only
    # while the controller is active.

    def accepts_goals(self):
        """Send a goal to stay where it is; True if the controller takes it."""
        here = self.current()
        handle = self.send_move(here, 0.0, 0.1)
        if handle is None or not handle.accepted:
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=3.0)
        return True

    def current(self):
        return list(self.samples[-1][1])

    def send_move(self, start, delta, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        steps = 40
        for step in range(1, steps + 1):
            point = JointTrajectoryPoint()
            fraction = step / steps
            target = list(start)
            target[0] = start[0] + delta * fraction
            point.positions = target
            point.time_from_start = _duration(duration * fraction)
            goal.trajectory.points.append(point)
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result()

    def standstill_since(self, after, window=0.05, tolerance=1e-5):
        """First time after ``after`` from which the arm moved less than ``tolerance`` for ``window`` s."""
        recent = [(t, q) for t, q in self.samples if t >= after]
        for index, (t0, q0) in enumerate(recent):
            later = [(t, q) for t, q in recent[index:] if t - t0 <= window]
            if later and later[-1][0] - t0 >= window * 0.8:
                if all(max(abs(a - b) for a, b in zip(q, q0)) < tolerance for _, q in later):
                    return t0
        return None

    # -- the test ----------------------------------------------------------------

    def test_breaking_the_curtain_mid_motion_halts_the_arm(self):
        self.spin_until(lambda: len(self.samples) > 10, 90.0, "/joint_states")
        self.assertTrue(self.action.wait_for_server(timeout_sec=90.0))
        self.spin_until(lambda: self.accepts_goals(), 90.0, "the trajectory controller to accept goals")
        self.assertTrue(self.reset.wait_for_service(timeout_sec=30.0))
        self.curtain.publish(Bool(data=False))
        self.spin_for(0.5)

        start = self.current()
        handle = self.send_move(start, MOVE, MOVE_TIME)
        self.assertTrue(handle is not None and handle.accepted, "the move was not accepted")

        # Half way, by position.
        self.spin_until(lambda: self.current()[0] - start[0] >= MOVE / 2, 10.0, "the arm to be half way")
        broken_at = time.time()  # The same clock as the joint state stamps.
        self.curtain.publish(Bool(data=True))
        self.spin_for(1.5)

        stopped_at = self.standstill_since(broken_at)
        self.assertIsNotNone(stopped_at, "the arm never stood still after the curtain was broken")
        stopping_time = stopped_at - broken_at
        travelled = self.current()[0] - start[0]
        print(f"\nprotective stop: beam broken to standstill {stopping_time * 1000:.0f} ms "
              f"(live budget {LIVE_BUDGET * 1000:.0f} ms), stopped at {travelled:.3f} of {MOVE} rad")
        self.assertLessEqual(stopping_time, LIVE_BUDGET)
        self.assertLess(travelled, MOVE - 0.2, "the arm finished the move despite the stop")

        # Category 1: once still, the controller is deactivated and refuses
        # goals, and the arm does not move.
        self.spin_until(lambda: not self.accepts_goals(), 5.0, "the controller to be deactivated")
        held = self.current()
        self.send_move(held, 0.3, 1.0)
        self.spin_for(1.5)
        self.assertLess(abs(self.current()[0] - held[0]), 1e-4, "the arm moved during a protective stop")

        # Reset while blocked does nothing.
        future = self.reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.spin_for(0.5)
        self.assertFalse(self.accepts_goals(), "a reset with the curtain blocked re-armed the arm")

        # Clear, wait out the restart delay, reset: the arm can move again.
        self.curtain.publish(Bool(data=False))
        self.spin_for(0.5)
        future = self.reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.spin_until(lambda: self.accepts_goals(), 5.0, "the controller to come back")
        resumed = self.current()
        handle = self.send_move(resumed, -0.3, 1.0)
        self.assertTrue(handle is not None and handle.accepted)
        self.spin_until(lambda: abs(self.current()[0] - resumed[0]) > 0.2, 5.0, "the arm to move after reset")
