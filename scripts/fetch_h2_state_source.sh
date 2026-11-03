#!/usr/bin/env bash
set -euo pipefail
h2_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
for h2_tool in python3 git; do
  command -v "$h2_tool" >/dev/null || { echo "Missing $h2_tool." >&2; exit 2; }
done
GIT_TERMINAL_PROMPT=0 python3 - "$h2_root" <<'PY'
import json
from pathlib import Path
import re
import subprocess
import sys

root = Path(sys.argv[1])
try:
    lock = json.loads((root / 'dependencies/h2_state.repos').read_text())
    repositories = lock['repositories']
    if set(repositories) != {'unitree_sdk2'}:
        raise ValueError('the lock must contain exactly unitree_sdk2')
    entry = repositories['unitree_sdk2']
    expected = entry['version']
    if entry['type'] != 'git' or not re.fullmatch(r'[0-9a-f]{40}', expected):
        raise ValueError('the SDK needs a full immutable Git commit')
    url = entry['url']
    if not isinstance(url, str) or not url:
        raise ValueError('the SDK Git URL is missing')
    source = root / 'vendor/sources/unitree_sdk2'
    source.parent.mkdir(parents=True, exist_ok=True)
    if source.exists() or source.is_symlink():
        # Existing source is inspect-only: never checkout, fetch, reset or clean.
        top = subprocess.run(['git', '-C', str(source), 'rev-parse', '--show-toplevel'],
                             text=True, capture_output=True)
        if top.returncode or Path(top.stdout.strip()).resolve() != source.resolve():
            raise ValueError(f'existing SDK path is not its own Git checkout: {source}')
    else:
        subprocess.run(['git', 'clone', '--no-checkout', '--', url, str(source)], check=True)
        subprocess.run(['git', '-C', str(source), 'checkout', '--detach', expected], check=True)
        subprocess.run(['git', '-C', str(source), 'submodule', 'update', '--init', '--recursive'], check=True)
    head = subprocess.run(['git', '-C', str(source), 'rev-parse', '--verify', 'HEAD'],
                          text=True, capture_output=True)
    if head.returncode or head.stdout.strip() != expected:
        raise ValueError(f'SDK is not at pinned commit {expected}; local source was preserved for review')
    actual = {'repositories': {'unitree_sdk2': {'type': 'git', 'url': url, 'version': expected}}}
    (root / 'vendor/h2-state-actual.repos').write_text(json.dumps(actual, indent=2) + '\n')
except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as error:
    sys.exit(f'H2 state source: {error}')
print(f'Optional SDK verified at {expected}; existing local changes were preserved.')
print('No executable or robot process was started.')
PY
