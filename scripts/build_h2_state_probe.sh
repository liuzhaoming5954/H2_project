#!/usr/bin/env bash
# Native non-ROS build. No device communication is started here.
set -euo pipefail
h2_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
mkdir -p "$h2_root/log/h2-state"
rm -f "$h2_root/log/h2-state/build-summary.txt"
h2_sdk_source=${1:-"$h2_root/vendor/sources/unitree_sdk2"}
[[ $# -le 1 ]] || { echo 'Usage: scripts/build_h2_state_probe.sh [SDK_SOURCE_DIR]' >&2; exit 2; }
[[ -d "$h2_sdk_source" ]] || { echo 'Fetch the SDK with scripts/fetch_h2_state_source.sh first.' >&2; exit 2; }
h2_sdk_source=$(cd -- "$h2_sdk_source" && pwd -P)
if [[ ${H2_STATE_CLEAN_ENV:-} != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 H2_STATE_CLEAN_ENV=1 \
    /bin/bash --noprofile --norc "$h2_root/scripts/build_h2_state_probe.sh" "$h2_sdk_source"
fi
for h2_tool in git cmake readelf sha256sum; do command -v "$h2_tool" >/dev/null; done
h2_pin=8e1b927586712edd6a4c29d50dd52dff1e163439
[[ "$(git -C "$h2_sdk_source" rev-parse HEAD)" == "$h2_pin" ]] || {
  echo "SDK must be at $h2_pin; local checkout was not changed." >&2; exit 2;
}
git -C "$h2_sdk_source" diff --quiet HEAD -- || {
  echo 'SDK has tracked modifications; review them before building (not reset).' >&2; exit 2;
}
h2_arch=$(uname -m)
case "$h2_arch" in x86_64|aarch64) ;; *) echo 'Only native x86_64/aarch64 builds are supported.' >&2; exit 2 ;; esac
[[ -s "$h2_sdk_source/lib/$h2_arch/libunitree_sdk2.a" ]] || { echo 'Missing native SDK static library.' >&2; exit 2; }
h2_dds="$h2_sdk_source/thirdparty/lib/$h2_arch/libddsc.so"
h2_elf=$(readelf -h "$h2_dds")
case "$h2_arch" in
  x86_64) [[ "$h2_elf" == *'Advanced Micro Devices X86-64'* ]] ;;
  aarch64) [[ "$h2_elf" == *AArch64* ]] ;;
esac
export CMAKE_BUILD_PARALLEL_LEVEL=2
cmake -S "$h2_root/tools/h2_state_probe" -B "$h2_root/build/h2_state_probe" \
  -DCMAKE_BUILD_TYPE=Release -DUNITREE_SDK_SOURCE_DIR="$h2_sdk_source" \
  -DCMAKE_INSTALL_PREFIX="$h2_root/install/h2_state_probe"
cmake --build "$h2_root/build/h2_state_probe" --parallel 2
cmake --install "$h2_root/build/h2_state_probe"
h2_binary="$h2_root/install/h2_state_probe/bin/h2_state_probe"
"$h2_binary" --version
mkdir -p "$h2_root/log/h2-state"
{
  printf 'scope=native compilation only; no DDS participant or hardware was started\n'
  printf 'architecture=%s\nsdk.head=%s\n' "$h2_arch" "$h2_pin"
  printf 'sdk.static.sha256=%s\n' "$(sha256sum "$h2_sdk_source/lib/$h2_arch/libunitree_sdk2.a" | cut -d ' ' -f 1)"
  printf 'binary.sha256=%s\n' "$(sha256sum "$h2_binary" | cut -d ' ' -f 1)"
} > "$h2_root/log/h2-state/build-summary.txt"
echo "Built passive state probe: $h2_binary"
