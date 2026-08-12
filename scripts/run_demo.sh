#!/usr/bin/env bash
# The one-command demo.
#
# With ROS 2 Humble sourced, this launches the simulated cell. Without it, it
# runs the offline demo instead -- one full pick-and-place cycle, printed stage
# by stage -- rather than failing with a message about a missing rclpy.
#
# A demo that only works on a machine with a 3 GB ROS installation is not a
# demo, it is a prerequisite list.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if command -v ros2 >/dev/null 2>&1 && python3 -c "import rclpy" >/dev/null 2>&1; then
  echo "ROS 2 found; launching the full cell."
  exec ros2 launch cell_bringup cell.launch.py "$@"
fi

echo "ROS 2 is not on this machine. Running the offline cycle demo instead."
echo "For the simulated cell:  make docker-demo"
echo
exec python3 scripts/offline_demo.py "$@"
