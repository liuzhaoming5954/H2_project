#!/usr/bin/env bash
# Run interactively on Ubuntu 24.04. No password or token is stored by this script.
set -euo pipefail

source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  echo "This installer supports Ubuntu 24.04 only." >&2
  exit 2
fi
case "$(dpkg --print-architecture)" in
  amd64|arm64) ;;
  *) echo "Only amd64 and arm64 are supported." >&2; exit 2 ;;
esac

h2_sudo=()
if [[ "$EUID" -ne 0 ]]; then
  h2_sudo=(sudo)
  sudo -v
fi

"${h2_sudo[@]}" apt-get update
"${h2_sudo[@]}" apt-get install -y ca-certificates curl software-properties-common
"${h2_sudo[@]}" add-apt-repository -y universe

h2_source_version=1.3.0
h2_source_sha=f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296
h2_tmp_dir=$(mktemp -d)
trap 'rm -rf -- "$h2_tmp_dir"' EXIT
curl --fail --location --retry 3 --output "$h2_tmp_dir/ros2-apt-source.deb" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${h2_source_version}/ros2-apt-source_${h2_source_version}.noble_all.deb"
printf '%s  %s\n' "$h2_source_sha" "$h2_tmp_dir/ros2-apt-source.deb" | sha256sum --check -
"${h2_sudo[@]}" dpkg -i "$h2_tmp_dir/ros2-apt-source.deb"
"${h2_sudo[@]}" apt-get update
"${h2_sudo[@]}" apt-get install -y \
  build-essential cmake ninja-build git rsync \
  python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pytest \
  ros-jazzy-ros-base ros-jazzy-launch-ros ros-jazzy-tf2-ros \
  ros-jazzy-rosidl-default-generators ros-jazzy-ament-cmake-pytest

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${h2_sudo[@]}" rosdep init
fi
if [[ "$EUID" -ne 0 ]]; then
  rosdep update --rosdistro jazzy
fi
echo "Installed build/runtime prerequisites. In each new terminal run:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "No shell startup file, H2 firmware, or running robot service was changed."
