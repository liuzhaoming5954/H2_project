#!/usr/bin/env python3
"""Collect first-test host inventory without opening a robot connection."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys

from h2ctl import doctor_report


ROOT = Path(__file__).resolve().parents[1]


def probe(command):
    try:
        result = subprocess.run(command, text=True, capture_output=True, timeout=15)
        return {"command": command, "returncode": result.returncode,
                "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"command": command, "returncode": None, "error": str(error)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path,
                        help="Save to a new JSON file; existing files are not overwritten")
    args = parser.parse_args()
    disk = shutil.disk_usage(ROOT)
    l4t = Path("/etc/nv_tegra_release")
    report = {
        "collected_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "host inventory only; no robot, lidar or SDK connection was opened",
        "architecture": platform.machine(),
        "target_arm64": platform.machine() in {"aarch64", "arm64"},
        "kernel": platform.release(),
        "ros_prerequisites": doctor_report("ros"),
        "free_bytes_at_workspace": disk.free,
        "network_addresses": probe(["ip", "-json", "address", "show"]),
        "network_routes": probe(["ip", "-json", "route", "show"]),
        "nvidia_packages": probe([
            "dpkg-query", "-W", "-f=${Package} ${Version} ${Status}\\n",
            "nvidia-jetpack", "nvidia-l4t-core"]),
        "nvidia_release": l4t.read_text().strip() if l4t.is_file() else None,
        "git_revision": probe(["git", "-C", str(ROOT), "rev-parse", "HEAD"]),
        "acceptance": "not_evaluated; inventory collection is not an H2 acceptance result",
    }
    data = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.report:
        try:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            with args.report.open("x", encoding="utf-8") as output:
                output.write(data)
        except OSError as error:
            print(f"Cannot save inventory: {error}", file=sys.stderr)
            return 2
        print(f"Host inventory saved: {args.report.resolve()}")
    else:
        print(data, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
