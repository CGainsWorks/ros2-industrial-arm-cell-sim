#!/usr/bin/env bash
# Set up the offline half of this repository: a virtual environment with
# numpy, scipy, pyyaml, pymodbus and matplotlib.
#
# This is all that `make check`, `make grasp` and every other analysis target
# needs. ROS 2 Humble, MoveIt2 and Gazebo are only needed for `make demo`, and
# the Docker image in docker/ is the supported way to get those.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PYTHON="${PYTHON:-python3}"
VENV="${VENV:-.venv}"

if [ ! -d "$VENV" ]; then
  echo "creating $VENV"
  "$PYTHON" -m venv "$VENV"
fi

# shellcheck disable=SC1090
source "$VENV/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

echo
echo "Generating the cell description from simulation/generate_cell.py"
python simulation/generate_cell.py

echo
echo "Done. Activate with:  source $VENV/bin/activate"
echo "Then:                 make check"
