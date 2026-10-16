# Two stages, because the two halves of this repository have nothing in common
# except the source tree.
#
#   offline  python:3.10-slim plus numpy and scipy. Runs the tests, the grasp
#            benchmark and every analysis script. About 200 MB.
#   ros      ros:humble-ros-base plus MoveIt2 and Gazebo Harmonic. Runs the
#            simulated cell. About 3 GB.
#
# `make docker-check` uses the first and `make docker-demo` the second. Building
# a 3 GB image to run a benchmark that needs numpy would be its own small
# statement about how the project is put together.

FROM python:3.10-slim AS offline

RUN apt-get update \
    && apt-get install -y --no-install-recommends build-essential make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .
CMD ["make", "check"]


FROM ros:humble-ros-base-jammy AS ros

SHELL ["/bin/bash", "-c"]

# Gazebo Harmonic, not Classic (end of life January 2025) and not Fortress
# (end of life September 2026). Humble's own apt repository pairs with
# Fortress, so Harmonic and its ros_gz build for Humble come from the OSRF
# Gazebo repository.
RUN apt-get update \
    && apt-get install -y --no-install-recommends curl ca-certificates \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions python3-rosdep python3-pip \
        python3-numpy python3-scipy python3-yaml \
        ros-humble-moveit ros-humble-moveit-ros-move-group \
        ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager \
        ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
        ros-humble-xacro ros-humble-rviz2 \
        ros-humble-ros2-control ros-humble-ros2-controllers \
        ros-humble-launch-testing-ros \
        ros-humble-ros-gzharmonic-sim gz-sim8-cli gz-transport13-cli \
        make \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir "pymodbus>=3.6,<3.9"

WORKDIR /workspace
COPY . .

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "cell_bringup", "cell.launch.py", "use_rviz:=false", "auto_run:=true"]
