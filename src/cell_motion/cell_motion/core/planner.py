"""Collision-free joint-space planning, and straight-line Cartesian moves.

Two planners, because the cycle needs two different things.

**RRT-Connect** for the free-space moves: home to pre-grasp, lift to the
inspection nest, nest back to home. This is the same algorithm OMPL runs under
MoveIt2 and the launch files in this package configure MoveIt2 to use it; this
implementation exists so the offline benchmark plans the same class of motion
without a ROS installation. It is not a reimplementation of OMPL's -- it is the
1500-line-shorter version of the same 2000 paper, with the same guarantees
(probabilistically complete, no optimality) and none of the tuning surface.

**Straight-line Cartesian descent** for the approach and the retreat. A grasp
approach must be a straight line along the tool axis: anything else sweeps the
fingers sideways through the part it is about to pick up. The Cartesian planner
interpolates the tool pose, solves IK at each step, and rejects the whole path
if consecutive solutions are not continuous -- which is what happens when the
straight line crosses a branch boundary, and what would otherwise produce a
trajectory containing a 180-degree wrist flip executed at full speed 30 mm
above the belt.

WHAT THIS DOES NOT DO

No optimisation objective, no clearance maximisation, no time-optimal
retiming along the path. Shortcutting the RRT output removes the worst of the
random detours and that is all. ``docs/limitations.md`` says what that costs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

import numpy as np

from cell_common.transforms import interpolate
from cell_motion.core.arm import ArmModel
from cell_motion.core.kinematics import JOINT_LIMITS

# Joint-space resolution at which an edge is checked, in radians. A coarser
# check tunnels through the guide rail, which is 30 mm thick and which a
# 0.1 rad step can straddle at the elbow.
EDGE_RESOLUTION = 0.04


@dataclass
class PlanResult:
    """A path, or the reason there is not one."""

    path: Optional[List[np.ndarray]]
    iterations: int
    reason: str = ""

    @property
    def ok(self) -> bool:
        return self.path is not None

    def joint_distance(self) -> float:
        if not self.path:
            return 0.0
        return float(sum(np.linalg.norm(b - a) for a, b in zip(self.path[:-1], self.path[1:])))


def _edge_is_valid(validity: Callable[[np.ndarray], bool], start: np.ndarray, goal: np.ndarray) -> bool:
    """Check an edge at :data:`EDGE_RESOLUTION`, endpoints included.

    Checked in a bisecting order rather than left to right: the midpoint of a
    bad edge is far more often the colliding sample than its second point, so
    the common case exits after one or two checks instead of a dozen.
    """
    delta = goal - start
    steps = int(np.ceil(float(np.max(np.abs(delta))) / EDGE_RESOLUTION))
    if steps <= 1:
        return validity(goal)
    order = sorted(range(1, steps + 1), key=lambda index: abs(index - steps / 2.0))
    for index in order:
        if not validity(start + delta * (index / steps)):
            return False
    return True


def plan_rrt_connect(
    start: Sequence[float],
    goal: Sequence[float],
    validity: Callable[[np.ndarray], bool],
    rng: np.random.Generator,
    max_iterations: int = 3000,
    step_size: float = 0.35,
    goal_bias: float = 0.1,
    limits: Optional[np.ndarray] = None,
) -> PlanResult:
    """Bidirectional RRT between two joint configurations."""
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    limits = JOINT_LIMITS if limits is None else limits

    if not validity(start):
        return PlanResult(None, 0, "start configuration is in collision")
    if not validity(goal):
        return PlanResult(None, 0, "goal configuration is in collision")
    if _edge_is_valid(validity, start, goal):
        return PlanResult([start, goal], 0)

    # Sampling is restricted to a box around the two endpoints rather than the
    # full +-2 pi joint range. The full range is 12 radians per joint and
    # almost all of it is behind the robot; sampling it uniformly wastes
    # essentially every sample.
    low = np.minimum(start, goal) - 1.6
    high = np.maximum(start, goal) + 1.6
    low = np.maximum(low, limits[:, 0])
    high = np.minimum(high, limits[:, 1])

    trees = [[start], [goal]]
    parents = [[-1], [-1]]

    def extend(tree_index: int, target: np.ndarray) -> int:
        tree = trees[tree_index]
        distances = [float(np.linalg.norm(node - target)) for node in tree]
        nearest = int(np.argmin(distances))
        direction = target - tree[nearest]
        norm = float(np.linalg.norm(direction))
        if norm < 1e-9:
            return nearest
        new_node = tree[nearest] + direction / norm * min(step_size, norm)
        if not _edge_is_valid(validity, tree[nearest], new_node):
            return -1
        tree.append(new_node)
        parents[tree_index].append(nearest)
        return len(tree) - 1

    for iteration in range(max_iterations):
        active = iteration % 2
        other = 1 - active
        target = goal if active == 0 else start
        sample = target if rng.random() < goal_bias else rng.uniform(low, high)

        added = extend(active, sample)
        if added < 0:
            continue

        # Connect: grow the other tree towards the new node until it arrives or
        # is blocked. This is what makes it RRT-*Connect* rather than a
        # bidirectional RRT, and it is worth roughly a factor of five here.
        node = trees[active][added]
        while True:
            connected = extend(other, node)
            if connected < 0:
                break
            if float(np.linalg.norm(trees[other][connected] - node)) < 1e-6:
                forward = _trace(trees[active], parents[active], added)
                backward = _trace(trees[other], parents[other], connected)
                path = forward[::-1] + backward if active == 0 else backward[::-1] + forward
                return PlanResult(path, iteration + 1)

    return PlanResult(None, max_iterations, "no path found within the iteration budget")


def _trace(tree: List[np.ndarray], parents: List[int], index: int) -> List[np.ndarray]:
    """Walk a tree back to its root, root last."""
    path = []
    while index != -1:
        path.append(tree[index])
        index = parents[index]
    return path


def shortcut(
    path: Sequence[np.ndarray],
    validity: Callable[[np.ndarray], bool],
    rng: np.random.Generator,
    attempts: int = 120,
) -> List[np.ndarray]:
    """Randomised shortcutting: replace a sub-path with a straight edge.

    Halves the joint distance of a typical RRT output here, and the joint
    distance is what the cycle time is made of.
    """
    path = [np.asarray(point, dtype=float) for point in path]
    for _ in range(attempts):
        if len(path) <= 2:
            break
        i, j = sorted(rng.choice(len(path), size=2, replace=False))
        if j - i < 2:
            continue
        if _edge_is_valid(validity, path[i], path[j]):
            path = path[: i + 1] + path[j:]
    return path


def plan_cartesian(
    arm: ArmModel,
    start_joints: Sequence[float],
    goal_pose: np.ndarray,
    opening: float,
    ignore: Sequence[str] = (),
    step: float = 0.005,
    max_jump: float = 0.35,
    margin: float = 0.002,
) -> PlanResult:
    """Straight-line tool-space move from a configuration to a pose.

    ``max_jump`` is the largest permitted joint step between consecutive
    waypoints. Exceeding it means the IK solution jumped branch: the tool pose
    is continuous but the arm is not, and executing it swings a link through
    whatever is beside it. Rejecting the path is the only safe response --
    there is no way to interpolate across a branch boundary.
    """
    start_joints = np.asarray(start_joints, dtype=float)
    start_pose = arm.tool_pose(start_joints)
    distance = float(np.linalg.norm(np.asarray(goal_pose)[:3, 3] - start_pose[:3, 3]))
    steps = max(2, int(np.ceil(distance / step)))

    path = [start_joints]
    for index in range(1, steps + 1):
        pose = interpolate(start_pose, goal_pose, index / steps)
        solution = arm.ik_nearest(pose, path[-1])
        if solution is None:
            return PlanResult(None, index, "no IK solution along the straight line")
        if float(np.max(np.abs(solution.joints - path[-1]))) > max_jump:
            return PlanResult(None, index, "IK branch change along the straight line")
        if not arm.is_valid(solution.joints, opening, ignore=ignore, margin=margin):
            return PlanResult(None, index, "straight line is in collision")
        path.append(solution.joints)
    return PlanResult(path, steps)


def make_validity(
    arm: ArmModel,
    opening: float,
    ignore: Sequence[str] = (),
    margin: float = 0.005,
) -> Callable[[np.ndarray], bool]:
    """A validity predicate bound to a gripper state and an ignore list."""

    def validity(joints: np.ndarray) -> bool:
        return arm.is_valid(joints, opening, ignore=ignore, margin=margin)

    return validity
