#!/usr/bin/env bash
# Source the ROS environment and the built workspace, then run whatever was
# asked for. `set -e` after the sourcing, not before: the ROS setup scripts
# reference unset variables and would abort under `set -u`.
source /opt/ros/humble/setup.bash
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi
set -e
exec "$@"
