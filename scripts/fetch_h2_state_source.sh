#!/usr/bin/env bash
set -euo pipefail
h2_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
command -v vcs >/dev/null || { echo 'Install python3-vcstool first.' >&2; exit 2; }
mkdir -p "$h2_root/vendor/sources"
# Never force/reset/clean an existing SDK checkout.
GIT_TERMINAL_PROMPT=0 vcs import --recursive --workers 1 \
  --input "$h2_root/dependencies/h2_state.repos" "$h2_root/vendor/sources"
vcs export --exact "$h2_root/vendor/sources/unitree_sdk2" > "$h2_root/vendor/h2-state-actual.repos"
echo 'Optional SDK source fetched; no executable or robot process was started.'
