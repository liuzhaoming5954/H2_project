#!/usr/bin/env bash
# Interactive prerequisite install only. Does not start any robot process.
set -euo pipefail
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 ]] || {
  echo 'Ubuntu 24.04 is required.' >&2; exit 2;
}
[[ -f /opt/ros/jazzy/setup.bash ]] || {
  echo 'Install ROS 2 Jazzy first (scripts/install_ros_jazzy.sh).' >&2; exit 2;
}
h2_sudo=()
if [[ "$EUID" -ne 0 ]]; then h2_sudo=(sudo); sudo -v; fi
"${h2_sudo[@]}" apt-get update
h2_rtab_packages=(rtabmap rtabmap-conversions rtabmap-msgs rtabmap-sync rtabmap-util rtabmap-slam)
h2_pinned=()
for h2_name in "${h2_rtab_packages[@]}"; do
  h2_package="ros-jazzy-${h2_name}"
  h2_version=$(apt-cache madison "$h2_package" | awk '$3 ~ /^0\.23\.13-/ {print $3; exit}')
  [[ -n "$h2_version" ]] || {
    echo "No 0.23.13 build available for $h2_package; review the lock before upgrading." >&2
    exit 2
  }
  h2_pinned+=("${h2_package}=${h2_version}")
done
printf 'RTAB-Map version selections:\n%s\n' "${h2_pinned[@]}"
"${h2_sudo[@]}" apt-get install -y \
  build-essential cmake git pkg-config python3-vcstool python3-yaml \
  python3-colcon-common-extensions libpcl-dev libeigen3-dev python3-dev \
  ros-jazzy-ament-cmake-auto ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-common-interfaces \
  "${h2_pinned[@]}"
echo 'Installed mapping prerequisites only; LIO still requires a source build and validation.'
