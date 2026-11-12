#!/usr/bin/env bash
# Build every package and run every test, exactly as CI does. Run from the
# repository root (the colcon workspace) with px4_msgs v1.17.0 in src/.
set -eo pipefail
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

colcon build --event-handlers console_cohesion+ --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-skip px4_msgs --event-handlers console_cohesion+
colcon test-result --verbose