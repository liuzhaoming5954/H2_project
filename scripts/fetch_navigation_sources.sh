#!/usr/bin/env bash
set -euo pipefail
h2_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
command -v vcs >/dev/null || { echo 'Install python3-vcstool first.' >&2; exit 2; }
mkdir -p "$h2_root/vendor/sources"
# No --force, reset or clean: existing local changes must not be discarded.
GIT_TERMINAL_PROMPT=0 vcs import --recursive --workers 2 \
  --input "$h2_root/dependencies/navigation.repos" "$h2_root/vendor/sources"
vcs export --exact "$h2_root/vendor/sources" > "$h2_root/vendor/actual.repos"
echo "Source checkouts are in vendor/sources; no binaries or hardware processes were started."
