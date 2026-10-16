"""Launch test: the generated world loads in Gazebo Harmonic and the arm spawns into it.

    launch_test src/cell_bringup/test/test_gazebo_world.launch.py

Headless (server only, no rendering). Runs ``gazebo.launch.py`` and checks:

1. the spawner (``ros_gz_sim create``) exits 0, so the arm description was
   accepted by gz-sim;
2. the world reports the cell furniture, the light curtain and the arm as
   models, asked of the running server with ``gz model --list``;
3. simulation time advances, so the server is stepping rather than stalled.

This is the check that the move off Gazebo Classic actually happened: the
world file is SDF for gz-sim 8 and would not load in Classic, and the
launch file uses ros_gz_sim rather than gazebo_ros.
"""

import re
import subprocess
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

EXPECTED_MODELS = {
    "ground_plane",
    "pedestal",
    "conveyor",
    "camera_pillar",
    "inspection_nest",
    "light_curtain_emitter",
    "light_curtain_receiver",
    "light_curtain_field",
    "arm_cell",
}


@pytest.mark.launch_test
def generate_test_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("cell_bringup"), "launch", "gazebo.launch.py"])]
        )
    )
    return launch.LaunchDescription([gazebo, launch_testing.actions.ReadyToTest()])


def _gz(*arguments, timeout=20.0):
    return subprocess.run(["gz", *arguments], capture_output=True, text=True, timeout=timeout)


class TestGazeboWorld(unittest.TestCase):
    def test_the_world_loads_and_the_arm_spawns(self, proc_output):
        # The wording differs between ros_gz_sim releases.
        proc_output.assertWaitFor(re.compile(r"OK creation of entity|Entity creation successful"), timeout=120)

        models = set()
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            listing = _gz("model", "--list")
            models = set(re.findall(r"^\s*-\s*(\S+)\s*$", listing.stdout, re.M))
            if EXPECTED_MODELS <= models:
                break
            time.sleep(1.0)
        print(f"\nmodels in the world: {sorted(models)}")
        self.assertTrue(EXPECTED_MODELS <= models, f"missing: {sorted(EXPECTED_MODELS - models)}")

    def test_simulation_time_advances(self):
        def sim_time():
            result = _gz("topic", "-e", "-n", "1", "-t", "/world/arm_cell/clock")
            match = re.search(r"sim\s*\{\s*(?:sec:\s*(\d+))?\s*(?:nsec:\s*(\d+))?", result.stdout)
            if not match:
                return None
            return int(match.group(1) or 0) + int(match.group(2) or 0) * 1e-9

        deadline = time.monotonic() + 120.0
        first = None
        while time.monotonic() < deadline and first is None:
            first = sim_time()
            time.sleep(0.5)
        self.assertIsNotNone(first, "no clock from the world")
        time.sleep(2.0)
        second = sim_time()
        self.assertIsNotNone(second)
        self.assertGreater(second, first, "simulation time is not advancing")
