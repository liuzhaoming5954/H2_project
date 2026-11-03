#!/usr/bin/env bash
# Build pinned mapping dependencies locally; never launch sensors or robot code.
set -euo pipefail

h2_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
h2_jobs=${H2_MAPPING_JOBS:-2}
h2_prepare_only=0
h2_skip_sdk=0
h2_original_arguments=("$@")
h2_fail() { printf 'mapping build: %s\n' "$*" >&2; exit 2; }
h2_usage() {
  cat <<'HELP'
Usage: scripts/build_mapping_dependencies.sh [--jobs 1|2] [--skip-sdk] [--prepare-only]
  --jobs          Native compiler workers, at most 2 (default: 2).
  --skip-sdk      Reuse the project's installed Livox SDK; verify its native ELF.
  --prepare-only  Verify pinned sources and prepare the patch/ROS2 manifest only.
No apt installation, downloads, upstream build.sh, hardware startup, or motion commands.
HELP
}
while (($#)); do
  case "$1" in
    --jobs) [[ $# -ge 2 ]] || h2_fail '--jobs needs 1 or 2'; h2_jobs=$2; shift 2 ;;
    --skip-sdk) h2_skip_sdk=1; shift ;;
    --prepare-only) h2_prepare_only=1; shift ;;
    -h|--help) h2_usage; exit 0 ;;
    *) h2_usage >&2; h2_fail "unknown argument: $1" ;;
  esac
done
[[ "$h2_jobs" == 1 || "$h2_jobs" == 2 ]] || h2_fail '--jobs must be 1 or 2'

# Re-exec without a sourced foundation/mapping overlay or user compiler paths.
if [[ ${H2_MAPPING_CLEAN_ENV:-} != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 H2_MAPPING_CLEAN_ENV=1 H2_MAPPING_JOBS="$h2_jobs" \
    /bin/bash --noprofile --norc "$h2_root/scripts/build_mapping_dependencies.sh" "${h2_original_arguments[@]}"
fi

h2_summary="$h2_root/log/mapping/build-summary.txt"
if ((!h2_prepare_only)); then
  mkdir -p -- "$(dirname -- "$h2_summary")"
  # A failed new attempt must never leave a previous success marker in place.
  rm -f -- "$h2_summary"
fi

for h2_tool in git python3 cmake colcon pkg-config dpkg-query readelf sha256sum; do
  command -v "$h2_tool" >/dev/null || h2_fail "missing $h2_tool; install mapping prerequisites first"
done
[[ -f /opt/ros/jazzy/setup.bash ]] || h2_fail 'missing ROS 2 Jazzy; install it first'
set +u
source /opt/ros/jazzy/setup.bash
set -u
[[ ${ROS_DISTRO:-} == jazzy ]] || h2_fail 'only the ROS 2 Jazzy underlay is supported'
case "$(uname -m)" in x86_64|aarch64|arm64) ;; *) h2_fail 'supported native architectures: x86_64 or ARM64' ;; esac

h2_source_root="$h2_root/vendor/sources"
h2_patch="$h2_root/dependencies/patches/fast_lio_jazzy.patch"
[[ -f "$h2_patch" ]] || h2_fail "missing patch: $h2_patch"
python3 - "$h2_root/dependencies/navigation.repos" "$h2_source_root" <<'PY'
from pathlib import Path
import re
import subprocess
import sys
try:
    import yaml
except ImportError:
    sys.exit('mapping build: python3-yaml is required to validate dependencies/navigation.repos')
lock = yaml.safe_load(Path(sys.argv[1]).read_text())['repositories']
required = ('fast_lio', 'livox_ros_driver2', 'livox_sdk2', 'rtabmap', 'rtabmap_ros')
for name in required:
    entry = lock.get(name, {})
    expected = entry.get('version', '')
    if entry.get('type') != 'git' or not re.fullmatch(r'[0-9a-f]{40}', expected):
        sys.exit(f'mapping build: {name} needs a full immutable Git commit in the source lock')
    source = Path(sys.argv[2]) / name
    result = subprocess.run(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True, capture_output=True)
    if result.returncode or result.stdout.strip() != expected:
        sys.exit(f'mapping build: {name} is missing or not at pinned commit {expected}; '
                 'fetch/review the source, do not reset local changes automatically')
    submodules = subprocess.run(['git', '-C', str(source), 'submodule', 'status', '--recursive'],
                               text=True, capture_output=True)
    if submodules.returncode or any(line and line[0] != ' ' for line in submodules.stdout.splitlines()):
        sys.exit(f'mapping build: {name} has missing or mismatched submodules; initialize the pinned submodules')
    print(f'pinned source: {name} {expected}', flush=True)
PY

if git -C "$h2_source_root/fast_lio" apply --check "$h2_patch" 2>/dev/null; then
  git -C "$h2_source_root/fast_lio" apply "$h2_patch"
  printf 'Applied FAST-LIO Jazzy/covariance patch.\n'
elif git -C "$h2_source_root/fast_lio" apply --reverse --check "$h2_patch" 2>/dev/null; then
  printf 'FAST-LIO patch is already applied.\n'
else
  h2_fail 'FAST-LIO patch conflicts with local changes; review manually (no reset/clean performed)'
fi
h2_driver="$h2_source_root/livox_ros_driver2"
if [[ -e "$h2_driver/package.xml" || -L "$h2_driver/package.xml" ]]; then
  [[ ! -L "$h2_driver/package.xml" ]] && cmp -s "$h2_driver/package_ROS2.xml" "$h2_driver/package.xml" ||
    h2_fail 'driver package.xml differs from package_ROS2.xml; refusing to overwrite it'
else
  cp -- "$h2_driver/package_ROS2.xml" "$h2_driver/package.xml"
fi
if ((h2_prepare_only)); then
  printf 'Source preparation only: pins/patch/ROS2 manifest ready; no dependency or build acceptance claimed.\n'
  exit 0
fi

pkg-config --exists pcl_common pcl_io || h2_fail 'missing PCL development files; run scripts/install_mapping_dependencies.sh'
for h2_package in pcl_ros pcl_conversions ament_cmake_auto rclcpp_components rtabmap_conversions rtabmap_sync rtabmap_util rtabmap_slam; do
  [[ -f "/opt/ros/jazzy/share/$h2_package/cmake/${h2_package}Config.cmake" ]] ||
    h2_fail "missing Jazzy package $h2_package; run scripts/install_mapping_dependencies.sh"
done
h2_rtab_packages=(rtabmap rtabmap-conversions rtabmap-msgs rtabmap-sync rtabmap-util rtabmap-slam)
h2_rtab_versions=()
for h2_name in "${h2_rtab_packages[@]}"; do
  h2_package="ros-jazzy-$h2_name"
  h2_status=$(dpkg-query -W -f='${Status}\t${Version}' "$h2_package" 2>/dev/null) ||
    h2_fail "missing $h2_package; run scripts/install_mapping_dependencies.sh"
  [[ "$h2_status" == 'install ok installed'$'\t'* ]] || h2_fail "$h2_package is not fully installed"
  h2_version=${h2_status##*$'\t'}
  [[ "$h2_version" == 0.23.13-* ]] || h2_fail "$h2_package is $h2_version, expected 0.23.13-*; review the lock before upgrading"
  h2_rtab_versions+=("$h2_package=$h2_version")
done

h2_sdk_prefix="$h2_root/install/livox_sdk2"
h2_sdk_mode=reused_native_elf_only
export CMAKE_BUILD_PARALLEL_LEVEL="$h2_jobs"
export MAKEFLAGS="-j$h2_jobs -l$h2_jobs"
if ((!h2_skip_sdk)); then
  h2_sdk_mode=built_from_source_checkout
  cmake -S "$h2_source_root/livox_sdk2" -B "$h2_root/build/livox_sdk2" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$h2_sdk_prefix"
  cmake --build "$h2_root/build/livox_sdk2" --parallel "$h2_jobs" \
    --target livox_lidar_sdk_shared livox_lidar_sdk_static
  cmake --install "$h2_root/build/livox_sdk2"
else
  printf 'Reusing installed SDK: architecture is checked, its source provenance is not established by --skip-sdk.\n'
fi
for h2_file in include/livox_lidar_api.h lib/liblivox_lidar_sdk_shared.so lib/liblivox_lidar_sdk_static.a; do
  [[ -s "$h2_sdk_prefix/$h2_file" ]] || h2_fail "missing SDK output $h2_sdk_prefix/$h2_file; rebuild without --skip-sdk"
done
h2_sdk_elf=$(readelf -h "$h2_sdk_prefix/lib/liblivox_lidar_sdk_shared.so")
case "$(uname -m)" in
  x86_64) [[ "$h2_sdk_elf" == *'Advanced Micro Devices X86-64'* ]] || h2_fail 'SDK is not a native x86_64 ELF' ;;
  aarch64|arm64) [[ "$h2_sdk_elf" == *AArch64* ]] || h2_fail 'SDK is not a native ARM64 ELF' ;;
esac
export CMAKE_PREFIX_PATH="$h2_sdk_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"

# Build only these two vendor packages; the system RTAB-Map 0.23.13 is reused.
# Sequential package execution bounds total compiler concurrency to --jobs.
colcon --log-base "$h2_root/log/mapping" build \
  --base-paths "$h2_driver" "$h2_source_root/fast_lio" \
  --packages-select livox_ros_driver2 fast_lio --executor sequential \
  --build-base "$h2_root/build/mapping" --install-base "$h2_root/install/mapping" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF \
    "-DLIVOX_LIDAR_SDK_LIBRARY:FILEPATH=$h2_sdk_prefix/lib/liblivox_lidar_sdk_shared.so" \
    "-DLIVOX_LIDAR_SDK_INCLUDE_DIR:PATH=$h2_sdk_prefix/include" \
    "-DCMAKE_LIBRARY_PATH:PATH=$h2_sdk_prefix/lib" "-DCMAKE_INCLUDE_PATH:PATH=$h2_sdk_prefix/include" \
    "-DCMAKE_BUILD_RPATH=$h2_sdk_prefix/lib" "-DCMAKE_INSTALL_RPATH=$h2_sdk_prefix/lib" \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON

h2_driver_library="$h2_root/install/mapping/livox_ros_driver2/lib/liblivox_ros_driver2.so"
h2_lio_executable="$h2_root/install/mapping/fast_lio/lib/fast_lio/fastlio_mapping"
[[ -s "$h2_driver_library" && -x "$h2_lio_executable" ]] || h2_fail 'colcon did not produce both required driver/LIO outputs'
h2_dynamic=$(readelf -d "$h2_driver_library")
[[ "$h2_dynamic" == *"$h2_sdk_prefix/lib"* ]] || h2_fail 'installed Livox driver lacks the SDK runtime library path'
{
  printf 'status=build_succeeded\narchitecture=%s\ncompleted_utc=%s\n' "$(uname -m)" "$(date -u +%FT%TZ)"
  printf 'scope=local native compilation only; no sensor, ROS node or robot motion was started\n'
  printf 'sdk_mode=%s\n' "$h2_sdk_mode"
  printf 'fast_lio.logging=disabled by default; enabling runtime_pos_log_enable still writes under the source Log directory\n'
  printf '%s\n' "${h2_rtab_versions[@]}"
  for h2_name in fast_lio livox_ros_driver2 livox_sdk2 rtabmap rtabmap_ros; do
    printf '%s.head=%s\n' "$h2_name" "$(git -C "$h2_source_root/$h2_name" rev-parse HEAD)"
    printf '%s.worktree_diff.sha256=%s\n' "$h2_name" "$(git -C "$h2_source_root/$h2_name" diff --no-ext-diff --binary HEAD | sha256sum | cut -d ' ' -f 1)"
  done
  printf 'fast_lio.patch.sha256=%s\n' "$(sha256sum "$h2_patch" | cut -d ' ' -f 1)"
  printf 'driver.package_xml.sha256=%s\n' "$(sha256sum "$h2_driver/package.xml" | cut -d ' ' -f 1)"
  printf 'sdk.shared.sha256=%s\n' "$(sha256sum "$h2_sdk_prefix/lib/liblivox_lidar_sdk_shared.so" | cut -d ' ' -f 1)"
  printf 'driver.library=%s\nfast_lio.executable=%s\n' "$h2_driver_library" "$h2_lio_executable"
} > "$h2_summary"
printf 'Native mapping dependencies built. Evidence: %s\nNo hardware was started.\n' "$h2_summary"
