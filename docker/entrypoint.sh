#!/usr/bin/env bash
# Source ROS 2, the PX4 underlay, this workspace and the venv, then run the
# given command (an interactive bash by default).
set -e
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /root/px4_ros_ws/install/setup.bash
# shellcheck disable=SC1091
source /root/fixed-wing-uav-px4-sitl-ros2/install/setup.bash
# shellcheck disable=SC1091
source /root/fixed-wing-uav-px4-sitl-ros2/.venv/bin/activate
exec "$@"