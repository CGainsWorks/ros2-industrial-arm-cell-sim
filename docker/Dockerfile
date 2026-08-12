# Two stages, because the two halves of this repository have nothing in common
# except the source tree.
#
#   offline  python:3.10-slim plus numpy and scipy. Runs the tests, the grasp
#            benchmark and every analysis script. About 200 MB.
#   ros      ros:humble-ros-base plus MoveIt2 and Gazebo. Runs the simulated
#            cell. About 3 GB.
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

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions python3-rosdep python3-pip \
        python3-numpy python3-scipy python3-yaml \
        ros-humble-moveit ros-humble-moveit-ros-move-group \
        ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager \
        ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
        ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-rviz2 \
        ros-humble-ros2-control ros-humble-ros2-controllers \
        make \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir "pymodbus>=3.0"

WORKDIR /workspace
COPY . .

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "cell_bringup", "cell.launch.py", "use_rviz:=false", "auto_run:=true"]
