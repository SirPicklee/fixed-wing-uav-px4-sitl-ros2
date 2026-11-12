#!/usr/bin/env bash
# Install the build and test dependencies on Ubuntu 24.04 with ROS 2 Jazzy
# (the ros:jazzy-ros-base image): Gazebo Harmonic development libraries from
# the OSRF repository, Eigen, GoogleTest and the ament Python linters.
# Run as root. Used by the GitHub Actions workflow and for local checks.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y --no-install-recommends ca-certificates curl git gnupg lsb-release

curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/gazebo-stable.list

apt-get update
apt-get install -y --no-install-recommends \
  libgz-sim8-dev libeigen3-dev libgtest-dev \
  python3-numpy python3-pytest python3-colcon-common-extensions \
  ros-jazzy-ament-cmake ros-jazzy-ament-flake8 ros-jazzy-ament-pep257 \
  ros-jazzy-rclpy ros-jazzy-rosidl-default-generators