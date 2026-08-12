"""The generated robot description has to agree with the IK solver's model.

These tests parse ``src/cell_description/urdf/ur5e.urdf.xacro`` with an XML
parser -- no ROS, no xacro binary -- walk the joint chain, and compare the pose
it produces against :func:`forward_kinematics` at random configurations.

This is the test that caught the DH-to-URDF off-by-one described in
``simulation/generate_cell.py``: joint offsets written one link too early
produce modified-DH kinematics, which agrees with standard DH at the home
configuration and diverges by over 100 mm elsewhere. A visual check in RViz
would not have found it.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from cell_common.transforms import from_rpy
from cell_motion.core.kinematics import JOINT_NAMES, forward_kinematics

REPO_ROOT = Path(__file__).resolve().parent.parent
ARM_XACRO = REPO_ROOT / "src" / "cell_description" / "urdf" / "ur5e.urdf.xacro"
CELL_XACRO = REPO_ROOT / "src" / "cell_description" / "urdf" / "cell.urdf.xacro"


def _joint_chain(path: Path):
    """(origin, axis) for every joint in the arm macro, in chain order.

    ``prefix`` substitutions are stripped: the chain is a straight line, so the
    order the joints appear in the file is the order they compose in.
    """
    root = ET.parse(path).getroot()
    chain = []
    for joint in root.iter("joint"):
        name = joint.attrib["name"].replace("${prefix}", "")
        origin = joint.find("origin")
        xyz = [float(value) for value in origin.attrib.get("xyz", "0 0 0").split()] if origin is not None else [0.0] * 3
        rpy = [float(value) for value in origin.attrib.get("rpy", "0 0 0").split()] if origin is not None else [0.0] * 3
        axis = joint.find("axis")
        axis_xyz = [float(value) for value in axis.attrib["xyz"].split()] if axis is not None else None
        chain.append((name, joint.attrib["type"], from_rpy(*rpy, xyz=xyz), axis_xyz))
    return chain


def test_arm_xacro_parses():
    assert ARM_XACRO.exists(), "run `make cell` to generate the description"
    assert CELL_XACRO.exists()
    ET.parse(ARM_XACRO)
    ET.parse(CELL_XACRO)


def test_joint_names_and_order_match_the_solver():
    chain = _joint_chain(ARM_XACRO)
    revolute = [name for name, kind, _, _ in chain if kind == "revolute"]
    assert tuple(revolute) == JOINT_NAMES


def test_every_revolute_joint_turns_about_z():
    """Standard DH puts every joint axis on local z. Anything else means the
    generator invented a frame that the solver does not know about."""
    for name, kind, _, axis in _joint_chain(ARM_XACRO):
        if kind == "revolute":
            assert axis == [0.0, 0.0, 1.0], f"{name} does not rotate about z"


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
def test_urdf_forward_kinematics_matches_the_dh_model(seed):
    """Walk the URDF chain to tool0 and compare against the DH solver."""
    rng = np.random.default_rng(seed)
    joints = rng.uniform(-np.pi, np.pi, 6)

    chain = _joint_chain(ARM_XACRO)
    pose = np.eye(4)
    index = 0
    for name, kind, origin, _ in chain:
        pose = pose @ origin
        if kind == "revolute":
            angle = joints[index]
            pose = pose @ from_rpy(0.0, 0.0, angle)
            index += 1
        if name == "tool0_joint":
            break

    expected = forward_kinematics(joints)
    assert np.allclose(pose, expected, atol=1e-9), (
        "URDF and DH kinematics disagree by "
        f"{np.linalg.norm(pose[:3, 3] - expected[:3, 3]) * 1000:.1f} mm at this configuration"
    )


def test_cell_fixtures_cover_every_obstacle():
    """Every obstacle in cell.yaml must exist as a link in the cell macro,
    otherwise the planner avoids something the simulator does not draw."""
    import yaml

    specification = yaml.safe_load((REPO_ROOT / "simulation" / "cell.yaml").read_text(encoding="utf-8"))
    links = {link.attrib["name"] for link in ET.parse(CELL_XACRO).getroot().iter("link")}
    for obstacle in specification["obstacles"]:
        assert f"{obstacle['name']}_link" in links
