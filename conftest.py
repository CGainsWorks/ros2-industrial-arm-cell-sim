"""Make the ROS package sources importable without a colcon build.

Each ``src/<pkg>`` directory is an ament_python package whose importable module
lives one level down. Putting those directories on ``sys.path`` here means the
test suite, the analysis scripts and CI all exercise exactly the code that gets
installed, with no copy step and no ROS installation required.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent

PACKAGE_DIRS = [
    REPO_ROOT / "src" / "cell_common",
    REPO_ROOT / "src" / "cell_motion",
    REPO_ROOT / "src" / "cell_grasping",
    REPO_ROOT / "src" / "cell_calibration",
    REPO_ROOT / "src" / "cell_inspection",
    REPO_ROOT / "src" / "cell_fieldbus",
    REPO_ROOT / "src" / "cell_bringup",
    REPO_ROOT,
]

for _path in PACKAGE_DIRS:
    entry = str(_path)
    if _path.is_dir() and entry not in sys.path:
        sys.path.insert(0, entry)
