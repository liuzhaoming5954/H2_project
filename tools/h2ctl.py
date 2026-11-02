#!/usr/bin/env python3
"""Local development checks and source packaging for the H2 workspace.

This tool does not connect to a robot or send motion commands. Remote deployment
is deliberately preview-only until the target-side activation path is tested.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import stat
import subprocess
import sys
import tarfile
from typing import Any, Iterable
import xml.etree.ElementTree as ET


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIRECTORIES = frozenset(
    {"src", "tools", "scripts", "tests", "docs", "config", "launch", "params", "rviz", "cmake", "dependencies", ".github"}
)
SOURCE_FILES = frozenset(
    {
        "README.md", "CONTRIBUTING.md", "CMakeLists.txt", "CTestConfig.cmake",
        "package.xml", "pyproject.toml", "setup.cfg", "setup.py", "colcon.meta",
        "dependencies.repos", "requirements.txt", "requirements-dev.txt", "pytest.ini",
        ".gitignore", ".gitattributes", ".clang-format", ".clang-tidy", ".pre-commit-config.yaml",
        ".editorconfig", ".env.example", "LICENSE", "LICENSE.md", "LICENSE.txt",
    }
)
EXCLUDED_PARTS = frozenset(
    {
        ".git", ".aws", ".codex", ".agents", ".ssh", ".gnupg", ".venv", ".tools", ".tox", "venv", "__pycache__",
        ".pytest_cache", ".mypy_cache", ".ruff_cache", "build", "install", "log",
        "data", "bags", "datasets", "recordings", "runs", "artifacts", "local",
    }
)
EXCLUDED_SUFFIXES = frozenset({".bag", ".db3", ".mcap", ".pyc", ".pyo", ".o", ".a", ".so", ".dll", ".exe"})
MAX_SOURCE_FILE_BYTES = 64 * 1024 * 1024
ROS_PACKAGES = frozenset({"h2_core", "h2_interfaces", "h2_runtime", "h2_supervisor", "h2_bringup"})
ROS_REQUIRED_REPORTS = {
    "h2_core": "h2_core/Testing/*/Test.xml",
    "h2_runtime": "h2_runtime/test_results/h2_runtime/runtime_contract.xunit.xml",
    "h2_supervisor": "h2_supervisor/pytest.xml",
}


class ToolError(RuntimeError):
    """An expected, user-actionable tool failure."""


def read_os_release(path: Path = Path("/etc/os-release")) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            if "=" in line and not line.startswith("#"):
                key, value = line.split("=", 1)
                values[key] = value.strip().strip('"')
    except OSError:
        pass
    return values


def doctor_report(profile: str) -> dict[str, Any]:
    architecture = platform.machine()
    operating_system = read_os_release()
    checks: list[dict[str, Any]] = []

    def add(name: str, ok: bool, detail: str, required: bool = True) -> None:
        checks.append({"name": name, "ok": ok, "required": required, "detail": detail})

    add("architecture", architecture in {"x86_64", "aarch64", "arm64"}, architecture)
    ubuntu_24 = operating_system.get("ID") == "ubuntu" and operating_system.get("VERSION_ID") == "24.04"
    add("operating_system", ubuntu_24,
        operating_system.get("PRETTY_NAME", platform.platform()), required=profile == "ros")
    tools = ["python3", "git", "cmake", "ctest"]
    if profile == "ros":
        tools.extend(["colcon", "ros2"])
    for tool in tools:
        executable = shutil.which(tool)
        add(tool, executable is not None, executable or "not found in PATH")
    compiler = next((shutil.which(name) for name in ("c++", "g++", "clang++") if shutil.which(name)), None)
    add("cxx_compiler", compiler is not None, compiler or "no C++ compiler in PATH")
    if profile == "ros":
        distro = os.environ.get("ROS_DISTRO", "")
        add("ros_distro", distro == "jazzy", distro or "ROS environment not sourced; source /opt/ros/jazzy/setup.bash")
    return {
        "profile": profile,
        "ok": all(check["ok"] or not check["required"] for check in checks),
        "checks": checks,
        "scope": "local build prerequisites only; not a robot or ARM64 runtime acceptance test",
    }


def print_doctor(report: dict[str, Any], as_json: bool) -> None:
    if as_json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return
    for check in report["checks"]:
        state = "OK" if check["ok"] else ("MISSING" if check["required"] else "NOTE")
        print(f"[{state}] {check['name']}: {check['detail']}")
    print(report["scope"])


def run_commands(commands: Iterable[list[str]], root: Path,
                 env: dict[str, str] | None = None) -> int:
    for command in commands:
        print("+ " + " ".join(command), flush=True)
        result = subprocess.run(command, cwd=root, check=False, env=env)
        if result.returncode:
            return result.returncode
    return 0


def verify_ros_packages(root: Path, prefix: list[str], env: dict[str, str] | None) -> None:
    command = prefix + ["list", "--base-paths", str(root / "src"), "--names-only"]
    print("+ " + " ".join(command), flush=True)
    result = subprocess.run(command, cwd=root, check=False, env=env,
                            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode:
        raise ToolError(f"ROS package discovery failed ({result.returncode}): {result.stderr.strip()}")
    found = set(result.stdout.splitlines())
    missing = ROS_PACKAGES - found
    if missing:
        raise ToolError("ROS package discovery is incomplete; missing: " + ", ".join(sorted(missing)))


def checked_ros_report(root: Path, report: Path) -> Path:
    """Never clean or read result files through links outside fixed build/ros."""
    build_base = root / "build/ros"
    current = root
    for component in report.relative_to(root).parts:
        current = current / component
        if current.is_symlink():
            raise ToolError(f"ROS test result paths must not use symbolic links: {current}")
    if not report.is_relative_to(build_base):
        raise ToolError(f"ROS test result is outside build/ros: {report}")
    return report


def clear_ros_test_results(root: Path) -> None:
    """Remove only standard generated reports, immediately before this test run."""
    build_base = root / "build/ros"
    patterns = ("*/Testing/*/Test.xml", "*/test_results/**/*.xml", "*/pytest.xml")
    reports = {path for pattern in patterns for path in build_base.glob(pattern)}
    for report in sorted(reports):
        checked_ros_report(root, report).unlink()


def verify_ros_test_results(root: Path) -> None:
    counts: dict[str, int] = {}
    for package, pattern in ROS_REQUIRED_REPORTS.items():
        reports = sorted((root / "build/ros").glob(pattern))
        if not reports:
            raise ToolError(f"Missing test results from this run for {package}")
        executed = 0
        for report in reports:
            checked_ros_report(root, report)
            try:
                document = ET.parse(report).getroot()
            except ET.ParseError as error:
                raise ToolError(f"Invalid test report {report}: {error}") from error
            if package == "h2_core":
                tests = document.findall("./Testing/Test")
                if any(test.get("Status") != "passed" for test in tests):
                    raise ToolError(f"Test report contains unsuccessful tests: {report}")
                executed += len(tests)
            else:
                tests = list(document.iter("testcase"))
                if any(test.find("failure") is not None or test.find("error") is not None for test in tests):
                    raise ToolError(f"Test report contains unsuccessful tests: {report}")
                executed += sum(test.find("skipped") is None for test in tests)
        if not executed:
            raise ToolError(f"No executed tests in this run for {package} (empty or all skipped)")
        counts[package] = executed
    print("ROS test evidence verified: " + ", ".join(f"{name}={count}" for name, count in counts.items()))


def check_workspace(root: Path, profile: str, jobs: int | None = None) -> int:
    report = doctor_report(profile)
    if not report["ok"]:
        print_doctor(report, False)
        return 2
    env = None
    if profile == "core":
        if not (root / "CMakeLists.txt").is_file():
            raise ToolError(f"No root CMakeLists.txt at {root}")
        commands = [
            ["cmake", "-S", str(root), "-B", str(root / "build/core"), "-DBUILD_TESTING=ON"],
            ["cmake", "--build", str(root / "build/core")],
            ["ctest", "--test-dir", str(root / "build/core"), "--output-on-failure", "--no-tests=error"],
        ]
        if jobs is not None:
            commands[1].extend(["--parallel", str(jobs)])
        result = run_commands(commands, root)
    else:
        if not (root / "src").is_dir():
            raise ToolError(f"No ROS source directory at {root / 'src'}")
        prefix = ["colcon", "--log-base", str(root / "log/ros")]
        shared = ["--base-paths", str(root / "src"), "--build-base", str(root / "build/ros")]
        commands = [
            prefix + ["build"] + shared + ["--install-base", str(root / "install/ros"), "--cmake-args", "-DBUILD_TESTING=ON"],
            prefix + ["test"] + shared + ["--install-base", str(root / "install/ros"), "--return-code-on-test-failure"],
            prefix + ["test-result", "--test-result-base", str(root / "build/ros"), "--verbose"],
        ]
        if jobs is not None:
            position = commands[0].index("--cmake-args")
            commands[0][position:position] = ["--parallel-workers", str(jobs)]
            commands[1].extend(["--parallel-workers", str(jobs)])
            env = os.environ.copy()
            env["CMAKE_BUILD_PARALLEL_LEVEL"] = str(jobs)
            # colcon-cmake otherwise appends its own -j<cpu_count>, overriding
            # CMake's environment limit. Explicit MAKEFLAGS also covers Make.
            env["MAKEFLAGS"] = f"-j{jobs} -l{jobs}"
        verify_ros_packages(root, prefix, env)
        result = run_commands(commands[:1], root, env)
        if result:
            return result
        clear_ros_test_results(root)
        result = run_commands(commands[1:2], root, env)
        if result:
            return result
        verify_ros_test_results(root)
        result = run_commands(commands[2:], root, env)
    if result == 0:
        print(f"{profile} checks passed on {platform.machine()}; no H2 hardware behavior was validated.")
    return result


def excluded(relative: Path) -> bool:
    if any(part in EXCLUDED_PARTS for part in relative.parts):
        return True
    name = relative.name
    return (
        relative.suffix in EXCLUDED_SUFFIXES
        or ".so." in name
        or name == "rtabmap.db"
        or name == ".env"
        or (name.startswith(".env.") and name != ".env.example")
        or name.endswith((".bag.zstd", ".db3.zstd", ".mcap.zstd", ".bag.gz", ".db3.gz", ".mcap.gz"))
        or name.endswith((".local.yaml", ".local.yml", ".local.json", ".local.toml", ".swp", ".swo", "~"))
    )


def source_entries(root: Path) -> list[Path]:
    """Select only explicit source roots, never follow links or export runtime data."""
    selected: list[Path] = []

    def visit(path: Path) -> None:
        relative = path.relative_to(root)
        if excluded(relative):
            return
        mode = path.lstat().st_mode
        if stat.S_ISLNK(mode):
            raise ToolError(f"Source bundles do not accept symbolic links: {relative}")
        if stat.S_ISDIR(mode):
            for child in sorted(path.iterdir()):
                visit(child)
        elif stat.S_ISREG(mode):
            selected.append(path)
        else:
            raise ToolError(f"Non-regular source entry: {relative}")

    for path in sorted(root.iterdir()):
        if path.name in SOURCE_DIRECTORIES or path.name in SOURCE_FILES:
            visit(path)
    if not selected:
        raise ToolError(f"No source files selected under {root}")
    return selected


def git_revision(root: Path) -> str | None:
    if not shutil.which("git"):
        return None
    try:
        result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root,
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                timeout=5, check=False)
        return result.stdout.strip() if result.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


def read_source_file(root: Path, relative: Path) -> tuple[bytes, os.stat_result]:
    """Open every path component without following a replaced directory link."""
    directory = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        for component in relative.parts[:-1]:
            child = os.open(component, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory)
            os.close(directory)
            directory = child
        descriptor = os.open(relative.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory)
        with os.fdopen(descriptor, "rb") as source:
            before = os.fstat(source.fileno())
            if not stat.S_ISREG(before.st_mode):
                raise ToolError(f"Source is no longer a regular file: {relative}")
            if before.st_size > MAX_SOURCE_FILE_BYTES:
                raise ToolError(f"Source file exceeds 64 MiB; keep datasets outside the source bundle: {relative}")
            data = source.read(MAX_SOURCE_FILE_BYTES + 1)
            after = os.fstat(source.fileno())
            if len(data) > MAX_SOURCE_FILE_BYTES or (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise ToolError(f"Source changed while bundling: {relative}; retry after edits finish")
            return data, after
    finally:
        os.close(directory)


def create_bundle(root: Path, output: Path) -> dict[str, Any]:
    root = root.resolve(strict=True)
    if not root.is_dir():
        raise ToolError(f"Workspace is not a directory: {root}")
    entries = source_entries(root)
    output = output.absolute()
    if output in entries:
        raise ToolError("Bundle destination cannot be a selected source file")
    try:
        output_relative = output.relative_to(root)
    except ValueError:
        output_relative = None
    if output_relative and output_relative.parts[0] in SOURCE_DIRECTORIES:
        raise ToolError("Write bundles outside source directories, for example artifacts/source.tar.gz")
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest: dict[str, Any] = {
        "format_version": 1,
        "kind": "source_snapshot",
        "git_revision": git_revision(root),
        "git_revision_is_not_snapshot_identity": True,
        "architecture_independent": True,
        "files": [],
    }
    created = False
    try:
        with output.open("xb") as archive_file:
            created = True
            with tarfile.open(fileobj=archive_file, mode="w:gz") as archive:
                for path in entries:
                    local_path = path.relative_to(root)
                    data, info = read_source_file(root, local_path)
                    relative = local_path.as_posix()
                    entry = tarfile.TarInfo(relative)
                    entry.size = len(data)
                    entry.mode = 0o755 if info.st_mode & 0o111 else 0o644
                    entry.mtime = 0
                    archive.addfile(entry, io.BytesIO(data))
                    manifest["files"].append({"path": relative, "size": len(data), "mode": entry.mode,
                                               "sha256": hashlib.sha256(data).hexdigest()})
                identity_data = json.dumps(manifest["files"], sort_keys=True, separators=(",", ":")).encode()
                manifest["snapshot_sha256"] = hashlib.sha256(identity_data).hexdigest()
                manifest_data = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode()
                entry = tarfile.TarInfo("MANIFEST.json")
                entry.size = len(manifest_data)
                entry.mode = 0o644
                entry.mtime = 0
                archive.addfile(entry, io.BytesIO(manifest_data))
    except BaseException:
        if created:
            output.unlink(missing_ok=True)
        raise
    return {"output": str(output), "file_count": len(entries), "snapshot_sha256": manifest["snapshot_sha256"]}


def deployment_preview(config_path: Path) -> dict[str, Any]:
    try:
        config = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ToolError(f"Cannot read target configuration: {error}") from error
    if not isinstance(config, dict):
        raise ToolError("Target configuration must be a JSON object")
    host = config.get("host", "")
    user = config.get("user", "")
    remote_root = config.get("remote_root", "")
    if not isinstance(host, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", host):
        raise ToolError("Target host must be an SSH alias, hostname, or IPv4 address")
    if not isinstance(user, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]*", user):
        raise ToolError("Target user is missing or invalid")
    if not isinstance(remote_root, str) or not re.fullmatch(r"/[A-Za-z0-9_./-]+", remote_root):
        raise ToolError("remote_root must be a dedicated absolute POSIX path without shell expansions")
    root_path = PurePosixPath(remote_root)
    if ".." in root_path.parts or root_path in {PurePosixPath("/"), PurePosixPath("/home"), PurePosixPath("/tmp")}:
        raise ToolError("remote_root must identify a dedicated project directory")
    return {
        "preview_only": True,
        "remote_actions_executed": False,
        "target": f"{user}@{host}",
        "remote_root": str(root_path),
        "expected_architecture": "aarch64",
        "ros_distro": "jazzy",
        "slots": [str(root_path / "slots/A"), str(root_path / "slots/B")],
        "activation_mode": "hardware_readonly",
        "steps": [
            "Check target environment, motion state, current slot and version compatibility through SSH",
            "Synchronize a source snapshot into the inactive fixed-path slot only",
            "Build on ARM64 with a clean ROS underlay; do not copy x86 build/install/log",
            "Validate candidate; stop and verify the project motion path before switching",
            "Switch current only after checks; start without a real motion writer",
            "Keep the previous compatible slot and collect a versioned verification report",
        ],
        "notice": "Remote execution and activation are not implemented by this command.",
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--root", type=Path, default=WORKSPACE_ROOT, help="workspace root")
    sub = result.add_subparsers(dest="command", required=True)
    doctor = sub.add_parser("doctor", help="check local build prerequisites without changing the system")
    doctor.add_argument("--profile", choices=("core", "ros"), default="core")
    doctor.add_argument("--json", action="store_true")
    check = sub.add_parser("check", help="run real local builds and tests")
    check.add_argument("--profile", choices=("core", "ros"), default="core")
    check.add_argument("--jobs", type=int,
                       help="core: build workers; ROS: maximum parallel packages and build workers per package")
    bundle = sub.add_parser("bundle", help="package allowlisted source files with a SHA-256 manifest")
    bundle.add_argument("--output", type=Path, required=True)
    deploy = sub.add_parser("deploy", help="preview the target deployment plan; never executes remote actions")
    deploy.add_argument("--target-config", type=Path, required=True)
    deploy.add_argument("--preview", action="store_true", help="required: remote execution is not implemented")
    return result


def main(argv: list[str] | None = None) -> int:
    arguments = parser().parse_args(argv)
    try:
        root = arguments.root.resolve(strict=True)
        if arguments.command == "doctor":
            report = doctor_report(arguments.profile)
            print_doctor(report, arguments.json)
            return 0 if report["ok"] else 2
        if arguments.command == "check":
            if arguments.jobs is not None and arguments.jobs < 1:
                raise ToolError("--jobs must be positive")
            return check_workspace(root, arguments.profile, arguments.jobs)
        if arguments.command == "bundle":
            print(json.dumps(create_bundle(root, arguments.output), ensure_ascii=False, indent=2))
            return 0
        if arguments.command == "deploy":
            if not arguments.preview:
                raise ToolError("Remote deployment is not implemented; pass --preview to inspect the plan")
            print(json.dumps(deployment_preview(arguments.target_config), ensure_ascii=False, indent=2))
            return 0
    except (ToolError, OSError, ValueError) as error:
        print(f"h2ctl: {error}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
