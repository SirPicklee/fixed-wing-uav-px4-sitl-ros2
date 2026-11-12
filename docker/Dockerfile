# Complete SITL environment for this project: ROS 2 Jazzy, Gazebo Harmonic,
# PX4 v1.17.0, Micro-XRCE-DDS Agent v2.4.3, px4_msgs v1.17.0, the reference
# Python model and this workspace, built the same way as on the host.
#
# Build (from the repository root; about an hour, about 15 GB):
#   docker build -f docker/Dockerfile -t fw-uav-sitl .
# Fly one mission headless, 4x faster than real time, and print the result:
#   docker run --rm fw-uav-sitl python scripts/run_flight.py --controller lqi --out /tmp/flight
# Interactive shell with everything sourced:
#   docker run --rm -it fw-uav-sitl
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      bc ca-certificates curl git gnupg lsb-release procps python3-venv sudo wget \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /root

# PX4 v1.17.0 (full clone: the build derives its version from the git tags),
# its Ubuntu toolchain and Gazebo Harmonic (simulation tools, no NuttX).
RUN git clone --branch v1.17.0 --recursive https://github.com/PX4/PX4-Autopilot.git \
    && bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx \
    && rm -rf /var/lib/apt/lists/*
RUN cd PX4-Autopilot && make px4_sitl_default

# uXRCE-DDS agent and the PX4 message definitions, in an underlay workspace.
RUN mkdir -p px4_ros_ws/src && cd px4_ros_ws/src \
    && git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && git clone --depth 1 --branch v1.17.0 https://github.com/PX4/px4_msgs.git \
    && cd .. && source /opt/ros/jazzy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Reference Python 6-DoF model (used by the design and validation tools).
RUN git clone --depth 1 https://github.com/SirPicklee/uav-6dof-autopilot.git

# This workspace and its Python environment.
COPY . /root/fixed-wing-uav-px4-sitl-ros2
WORKDIR /root/fixed-wing-uav-px4-sitl-ros2
RUN source /root/px4_ros_ws/install/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && python3 -m venv --system-site-packages .venv \
    && .venv/bin/pip install --no-cache-dir -r requirements.txt

ENTRYPOINT ["/root/fixed-wing-uav-px4-sitl-ros2/docker/entrypoint.sh"]
CMD ["bash"]