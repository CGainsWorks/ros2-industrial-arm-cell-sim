"""One pick-and-place cycle, end to end, without ROS.

This is the orchestration the ROS node in ``cell_bringup/nodes/cycle_node.py``
performs against live topics, factored so that the benchmark can run it two
hundred times in a minute. The stages are the real ones and they run in the
real order:

    scan -> perceive -> plan grasp -> approach -> close -> lift
         -> transfer -> place -> inspect -> retreat

Every stage that consumes time contributes a :class:`CycleSegment`, so the
cycle-time figure is a sum of named parts rather than a wall-clock measurement
of a simulation running at a speed that means nothing.

WHAT THE CYCLE IS ALLOWED TO KNOW

The pick pipeline sees the depth image and the calibrated hand-eye transform.
It does not see the part's true pose. The only place ground truth enters is
:func:`cell_bringup.core.outcome.evaluate_grasp`, at the end, to decide whether
the grasp held -- and, if it did, to attach the part to the tool so the
transfer motion is planned with the part in the gripper, which is where a part
that is picked successfully but too far off-centre still fails by clipping the
guide rail on the way out.

FAILURE IS NOT AN EXCEPTION

Every stage returns a reason rather than raising. A cycle that fails is a
result, not an error: the benchmark's whole output is the distribution of those
reasons.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from cell_common.cell_model import CellSpec, load_cell, part_box
from cell_common.transforms import look_at, invert
from cell_bringup.core.outcome import GraspOutcome, evaluate_grasp
from cell_grasping.core.camera import deproject, median_filter, render_depth
from cell_grasping.core.grasp import APPROACH_HEIGHT, GraspCandidate, generate_candidates
from cell_grasping.core.perception import detect_scene
from cell_motion.core.arm import ArmModel, approach_pose
from cell_motion.core.collision import Box
from cell_motion.core.planner import make_validity, plan_cartesian, plan_rrt_connect, shortcut
from cell_motion.core.trajectory import CycleSegment, Trajectory, time_parameterise, total_duration

# Fixed costs, in seconds, measured off the datasheets rather than invented:
# the camera's exposure plus transfer at 640x480, the PLC handshake at a 20 ms
# scan, and the settling allowance before a capture.
CAMERA_CAPTURE_TIME = 0.09
CAMERA_SETTLE_TIME = 0.15
FIELDBUS_HANDSHAKE_TIME = 0.04
PLACE_HEIGHT = 0.06  # Pre-place standoff above the nest, metres.


@dataclass
class CycleResult:
    """Everything one cycle produced."""

    success: bool
    stage: str  # Where it got to. "complete" when it finished.
    failure: str  # "" on success.
    segments: List[CycleSegment] = field(default_factory=list)
    trajectories: List[Trajectory] = field(default_factory=list)
    grasp: Optional[GraspCandidate] = None
    outcome: Optional[GraspOutcome] = None
    estimate_error: Optional[Dict[str, float]] = None
    candidate_count: int = 0
    clutter_count: int = 0
    rejections: Dict[str, int] = field(default_factory=dict)
    plan_iterations: int = 0
    inspection: Optional[Dict] = None

    @property
    def cycle_time(self) -> float:
        return total_duration(self.segments)

    def as_dict(self) -> Dict:
        return {
            "success": self.success,
            "stage": self.stage,
            "failure": self.failure,
            "cycle_time": round(self.cycle_time, 4),
            "segments": [segment.as_dict() for segment in self.segments],
            "candidate_count": self.candidate_count,
            "clutter_count": self.clutter_count,
            "rejections": self.rejections,
            "plan_iterations": self.plan_iterations,
            "grasp_score": round(self.grasp.score, 4) if self.grasp else None,
            "outcome": self.outcome.as_dict() if self.outcome else None,
            "estimate_error": self.estimate_error,
            "inspection": self.inspection,
        }


class CycleRunner:
    """Runs pick-and-place cycles against a cell.

    ``hand_eye`` is the flange-to-camera transform the cycle believes in. Pass
    the output of the calibration routine to see what a calibration residual
    costs in grasp success; pass ``None`` to use the true one, which is what
    the grasp benchmark does so that its failure classes are about grasping
    rather than about calibration.
    """

    def __init__(
        self,
        cell: Optional[CellSpec] = None,
        hand_eye: Optional[np.ndarray] = None,
        stride: int = 2,
        velocity_scale: float = 0.5,
        acceleration_scale: float = 0.35,
    ) -> None:
        self.cell = cell if cell is not None else load_cell()
        self.arm = ArmModel.from_cell(self.cell)
        self.hand_eye = self.cell.camera.extrinsic if hand_eye is None else np.asarray(hand_eye, dtype=float)
        self.stride = stride
        self.velocity_scale = velocity_scale
        self.acceleration_scale = acceleration_scale
        self.scan_joints = self._solve_scan_pose()

    # -- setup -------------------------------------------------------------

    def _solve_scan_pose(self) -> np.ndarray:
        """Joint configuration that puts the camera at the scan pose.

        Uses the *true* extrinsic: this is where the robot physically goes, and
        the robot goes where its program tells it, not where its calibration
        thinks the camera is. A calibration error moves the interpretation of
        the image, not the arm.
        """
        camera_pose = look_at(self.cell.scan_camera_xyz, self.cell.scan_target_xyz, up=(1.0, 0.0, 0.0))
        flange = camera_pose @ invert(self.cell.camera.extrinsic)
        tool = flange @ self.arm.tool_transform
        solutions = self.arm.ik(tool)
        home = self.cell.home_joints
        valid = [
            solution
            for solution in solutions
            if self.arm.is_valid(solution.joints, self.cell.gripper.max_opening)
        ]
        if not valid:
            raise RuntimeError("the scan pose is not reachable; check simulation/cell.yaml")
        weights = np.array([4.0, 3.0, 2.0, 1.0, 1.0, 0.5])
        return min(valid, key=lambda solution: float(np.linalg.norm(weights * (solution.joints - home)))).joints

    # -- stages ------------------------------------------------------------

    def _move(
        self,
        name: str,
        start: Sequence[float],
        goal: Sequence[float],
        opening: float,
        rng: np.random.Generator,
        ignore: Sequence[str] = (),
    ):
        """Free-space move, planned and timed. Returns (trajectory, reason)."""
        validity = make_validity(self.arm, opening, ignore=ignore)
        result = plan_rrt_connect(start, goal, validity, rng)
        if not result.ok:
            return None, result.reason, result.iterations
        path = shortcut(result.path, validity, rng)
        trajectory = time_parameterise(path, self.velocity_scale, self.acceleration_scale, label=name)
        return trajectory, "", result.iterations

    def _linear(
        self,
        name: str,
        start: Sequence[float],
        pose: np.ndarray,
        opening: float,
        ignore: Sequence[str] = (),
        margin: float = 0.002,
    ):
        result = plan_cartesian(self.arm, start, pose, opening, ignore=ignore, margin=margin)
        if not result.ok:
            return None, result.reason
        trajectory = time_parameterise(result.path, self.velocity_scale, self.acceleration_scale, label=name)
        return trajectory, ""

    # -- the cycle ---------------------------------------------------------

    def run(
        self,
        part_pose: Dict,
        rng: np.random.Generator,
        distractors: Sequence[Dict] = (),
    ) -> CycleResult:
        """Run one cycle against a ground-truth part presentation.

        ``distractors`` are additional real parts on the belt. The cell is not
        told which is which: it detects them all, picks the largest cluster,
        and the outcome is judged against whichever true part the grasp was
        actually aimed at.

        The scene is restored on the way out whichever branch the cycle takes.
        A cycle that fails half way through and leaves its clutter behind
        poisons every subsequent trial in the benchmark with obstacles from a
        part that is no longer there -- and does it invisibly, since the
        failures it causes are all plausible ones.
        """
        try:
            return self._run(part_pose, rng, distractors)
        finally:
            for box in list(self.arm.scene.boxes):
                if box.kind in ("part", "clutter"):
                    self.arm.scene.remove(box.name)

    def _run(
        self,
        part_pose: Dict,
        rng: np.random.Generator,
        distractors: Sequence[Dict] = (),
    ) -> CycleResult:
        cell = self.cell
        arm = self.arm
        gripper = cell.gripper
        open_jaws = gripper.max_opening
        segments: List[CycleSegment] = []
        trajectories: List[Trajectory] = []

        truth = part_box(part_pose)
        all_parts = [part_pose] + list(distractors)
        truth_boxes = [truth] + [part_box(other, f"distractor_{index}") for index, other in enumerate(distractors)]
        home = cell.home_joints

        segments.append(CycleSegment("cycle_start_handshake", FIELDBUS_HANDSHAKE_TIME, "fieldbus"))

        # 1. Move to the scan pose.
        trajectory, reason, iterations = self._move("to_scan", home, self.scan_joints, open_jaws, rng)
        if trajectory is None:
            return CycleResult(False, "scan_move", reason, segments, trajectories)
        segments.append(CycleSegment("to_scan", trajectory.duration, "motion"))
        trajectories.append(trajectory)

        # 2. Capture.
        segments.append(CycleSegment("camera_settle", CAMERA_SETTLE_TIME, "sensor"))
        segments.append(CycleSegment("camera_capture", CAMERA_CAPTURE_TIME, "sensor"))
        true_camera_pose = arm.camera_pose(self.scan_joints)
        image = median_filter(
            render_depth(
                true_camera_pose, list(arm.scene.boxes) + truth_boxes, cell.camera, rng, stride=self.stride
            )
        )

        # 3. Perceive. The cloud is deprojected using the *believed* camera
        #    pose, which is the flange pose composed with the calibrated
        #    hand-eye transform.
        believed_camera_pose = arm.flange_pose(self.scan_joints) @ self.hand_eye
        cloud = deproject(image, believed_camera_pose)
        estimate, clutter, _ = detect_scene(cloud, rng, cell.conveyor_top, workspace=cell.perception_workspace())
        if estimate is None:
            return CycleResult(False, "perception", "no cluster found", segments, trajectories)

        # Anything else the camera saw standing above the belt is an obstacle,
        # whatever it is. Added before grasp planning so the approach is
        # checked against it, and removed on the way out.
        clutter_names = []
        for index, other in enumerate(clutter):
            name = f"clutter_{index}"
            clutter_names.append(name)
            arm.scene.replace(_box_from_estimate(other, name))

        result = CycleResult(False, "perception", "", segments, trajectories)
        result.estimate_error = _estimate_error(estimate, _nearest_part(all_parts, estimate.centre))
        result.clutter_count = len(clutter)

        # 4. Plan the grasp.
        plan = generate_candidates(estimate, arm, reference_joints=self.scan_joints)
        result.candidate_count = len(plan.candidates)
        result.rejections = plan.rejections
        if plan.best is None:
            result.stage = "grasp_planning"
            result.failure = "no feasible grasp candidate"
            return result
        grasp = plan.best
        result.grasp = grasp

        # 5. Move to the pre-grasp, then descend in a straight line.
        trajectory, reason, iterations = self._move(
            "to_pregrasp", self.scan_joints, grasp.pregrasp_joints, grasp.opening, rng
        )
        result.plan_iterations = iterations
        if trajectory is None:
            result.stage, result.failure = "approach", reason
            return result
        segments.append(CycleSegment("to_pregrasp", trajectory.duration, "motion"))
        trajectories.append(trajectory)

        arm.set_part(_estimated_box_from(grasp, estimate))
        descent, reason = self._linear("descend", grasp.pregrasp_joints, grasp.pose, grasp.opening, ignore=("part",))
        arm.set_part(None)
        if descent is None:
            result.stage, result.failure = "descend", reason
            return result
        segments.append(CycleSegment("descend", descent.duration, "motion"))
        trajectories.append(descent)

        # 6. Close the jaws. Timed from the stroke actually travelled.
        travel = max(grasp.opening - grasp.width, 0.0)
        close_duration = gripper.close_time * float(np.clip(travel / max(gripper.max_opening, 1e-6), 0.15, 1.0))
        segments.append(CycleSegment("close_gripper", close_duration, "gripper"))

        # 7. Did it hold?
        achieved = arm.tool_pose(descent.positions[-1])
        target_part = _nearest_part(all_parts, achieved[:3, 3])
        outcome = evaluate_grasp(achieved[:3, 3], grasp.yaw, grasp.opening, target_part, gripper)
        result.outcome = outcome
        if not outcome.held:
            result.stage, result.failure = "grasp", outcome.failure
            return result

        # 8. Lift, with the part now attached to the tool.
        attached = _attached_box(target_part, achieved, grasp)
        arm.attach_part(attached)
        try:
            lift_pose = approach_pose(grasp.position + np.array([0.0, 0.0, APPROACH_HEIGHT]), grasp.yaw)
            lift, reason = self._linear("lift", descent.positions[-1], lift_pose, outcome.width, ignore=("part",))
            if lift is None:
                result.stage, result.failure = "lift", reason
                return result
            segments.append(CycleSegment("lift", lift.duration, "motion"))
            trajectories.append(lift)

            # 9. Transfer to the pre-place pose.
            place_position = cell.place_xyz + np.array([0.0, 0.0, float(target_part["size"][2]) / 2.0])
            preplace_pose = approach_pose(place_position + np.array([0.0, 0.0, PLACE_HEIGHT]), grasp.yaw)
            preplace = arm.ik_nearest(preplace_pose, lift.positions[-1])
            if preplace is None or not arm.is_valid(preplace.joints, outcome.width, ignore=("part",)):
                result.stage, result.failure = "transfer", "pre-place pose unreachable"
                return result
            transfer, reason, _ = self._move(
                "transfer", lift.positions[-1], preplace.joints, outcome.width, rng, ignore=("part",)
            )
            if transfer is None:
                result.stage, result.failure = "transfer", reason
                return result
            segments.append(CycleSegment("transfer", transfer.duration, "motion"))
            trajectories.append(transfer)

            # 10. Place.
            place, reason = self._linear(
                "place", transfer.positions[-1], approach_pose(place_position, grasp.yaw), outcome.width,
                ignore=("part", "inspection_nest"),
            )
            if place is None:
                result.stage, result.failure = "place", reason
                return result
            segments.append(CycleSegment("place", place.duration, "motion"))
            trajectories.append(place)
            segments.append(CycleSegment("open_gripper", gripper.close_time * 0.6, "gripper"))

            # 11. Straight up, out of the nest, before any free-space motion.
            #     The fingers finish the place inside the nest's footprint with
            #     their tips level with its rim; asking RRT-Connect to plan from
            #     there fails at the first check with "start configuration is in
            #     collision", which is true and useless. A real cell retracts
            #     along the tool axis first, and so does this one.
            retract_pose = approach_pose(place_position + np.array([0.0, 0.0, PLACE_HEIGHT]), grasp.yaw)
            retract, reason = self._linear(
                "retract", place.positions[-1], retract_pose, open_jaws,
                ignore=("part", "inspection_nest"),
            )
            if retract is None:
                result.stage, result.failure = "retract", reason
                return result
            segments.append(CycleSegment("retract", retract.duration, "motion"))
            trajectories.append(retract)
        finally:
            arm.set_part(None)

        # 12. Retreat to home.
        retreat, reason, _ = self._move("to_home", retract.positions[-1], home, open_jaws, rng)
        if retreat is None:
            result.stage, result.failure = "retreat", reason
            return result
        segments.append(CycleSegment("to_home", retreat.duration, "motion"))
        trajectories.append(retreat)
        segments.append(CycleSegment("cycle_done_handshake", FIELDBUS_HANDSHAKE_TIME, "fieldbus"))

        result.success = True
        result.stage = "complete"
        return result


def _nearest_part(parts: Sequence[Dict], position: np.ndarray) -> Dict:
    """Whichever true part is closest to a world position, in the horizontal plane.

    Used to decide what the cell actually grabbed. With one part it is trivially
    that part; with clutter it is the honest answer to "what did the jaws close
    on", and it is the reason the clutter condition can fail by picking up the
    neighbour, which is a failure worth being able to see.
    """
    position = np.asarray(position, dtype=float)[:2]
    return min(parts, key=lambda part: float(np.linalg.norm(np.asarray(part["xyz"])[:2] - position)))


def _box_from_estimate(estimate, name: str) -> Box:
    from cell_common.transforms import from_rpy

    return Box(
        name=name,
        size=np.asarray(estimate.extents, dtype=float),
        pose=from_rpy(0.0, 0.0, estimate.yaw, xyz=estimate.centre),
        kind="clutter",
    )


def _estimated_box_from(grasp: GraspCandidate, estimate) -> Box:
    from cell_common.transforms import from_rpy

    return Box(
        name="part",
        size=np.asarray(estimate.extents, dtype=float),
        pose=from_rpy(0.0, 0.0, estimate.yaw, xyz=estimate.centre),
        kind="part",
    )


def _attached_box(part_pose: Dict, tool_pose: np.ndarray, grasp: GraspCandidate) -> Box:
    """The part, where it actually is once the jaws have closed on it.

    A successful grasp does not teleport the part into the gripper's centre: it
    stays where it was, and the tool is wherever the descent put it. Modelling
    it as centred on the tool would erase exactly the offsets that make a
    lifted part clip the guide rail.
    """
    from cell_common.transforms import from_rpy

    centre = np.asarray(part_pose["xyz"], dtype=float).copy()
    centre[2] = float(part_pose["xyz"][2])
    return Box(
        name="part",
        size=np.asarray(part_pose["size"], dtype=float),
        pose=from_rpy(0.0, 0.0, float(part_pose["yaw"]), xyz=centre),
        kind="part",
    )


def _estimate_error(estimate, part_pose: Dict) -> Dict[str, float]:
    """Perception error against ground truth, for reporting only.

    Yaw error is reported modulo 90 degrees. A rectangle grasped across its
    other axis is the same grasp, and a metric that calls a 90-degree "error"
    on a square part a failure is measuring the parameterisation, not the
    perception.
    """
    size = np.asarray(part_pose["size"], dtype=float)
    true_yaw = float(part_pose["yaw"]) if size[0] >= size[1] else float(part_pose["yaw"]) + np.pi / 2
    delta = abs(((estimate.yaw - true_yaw + np.pi / 4) % (np.pi / 2)) - np.pi / 4)
    long_true, short_true = max(size[0], size[1]), min(size[0], size[1])
    return {
        "centre_mm": round(float(np.linalg.norm(estimate.centre - np.asarray(part_pose["xyz"]))) * 1000.0, 3),
        "yaw_deg": round(float(np.degrees(delta)), 3),
        "long_extent_mm": round(float(estimate.extents[0] - long_true) * 1000.0, 3),
        "short_extent_mm": round(float(estimate.extents[1] - short_true) * 1000.0, 3),
        "height_mm": round(float(estimate.extents[2] - size[2]) * 1000.0, 3),
        "eigen_ratio": round(float(estimate.eigen_ratio), 3),
        "points": int(estimate.point_count),
    }
