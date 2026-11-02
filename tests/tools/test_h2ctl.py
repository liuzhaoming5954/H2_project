"""Offline regression tests for source packaging and honest tool failures."""

from __future__ import annotations

import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch


MODULE_PATH = Path(__file__).resolve().parents[2] / "tools/h2ctl.py"
SPEC = importlib.util.spec_from_file_location("h2ctl", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
h2ctl = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(h2ctl)


class BundleTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        (self.root / "src/example").mkdir(parents=True)
        (self.root / "src/example/node.cpp").write_text("int main() { return 0; }\n")
        (self.root / "README.md").write_text("# Test\n")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_bundle_excludes_runtime_data_and_contains_verified_manifest(self) -> None:
        for relative in [".git/config", ".aws/credentials", "data/secret.mcap", "src/example/build/cache.txt", "config/local/robot.yaml", "src/example/.env", "src/example/test.local.yaml", "src/example/.tools/cache", "src/example/libtest.so.1", "src/example/trace.mcap.zstd"]:
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("not source")
        output = self.root / "artifacts/source.tar.gz"
        report = h2ctl.create_bundle(self.root, output)
        with tarfile.open(output) as archive:
            self.assertEqual(set(archive.getnames()), {"README.md", "src/example/node.cpp", "MANIFEST.json"})
            manifest = json.load(archive.extractfile("MANIFEST.json"))
            for entry in manifest["files"]:
                data = archive.extractfile(entry["path"]).read()
                self.assertEqual(entry["size"], len(data))
                self.assertEqual(entry["sha256"], hashlib.sha256(data).hexdigest())
        self.assertEqual(report["file_count"], 2)
        self.assertEqual(report["snapshot_sha256"], manifest["snapshot_sha256"])

    def test_source_change_changes_snapshot_identity(self) -> None:
        first = h2ctl.create_bundle(self.root, self.root / "artifacts/one.tar.gz")
        (self.root / "src/example/node.cpp").write_text("changed\n")
        second = h2ctl.create_bundle(self.root, self.root / "artifacts/two.tar.gz")
        self.assertNotEqual(first["snapshot_sha256"], second["snapshot_sha256"])

    def test_bundle_rejects_external_symlink_and_does_not_write_archive(self) -> None:
        (self.root / "src/example/linked").symlink_to("/etc/passwd")
        output = self.root / "artifacts/source.tar.gz"
        with self.assertRaisesRegex(h2ctl.ToolError, "symbolic links"):
            h2ctl.create_bundle(self.root, output)
        self.assertFalse(output.exists())

    def test_linked_source_directory_is_rejected(self) -> None:
        (self.root / "src/external").symlink_to("/etc", target_is_directory=True)
        with self.assertRaisesRegex(h2ctl.ToolError, "symbolic links"):
            h2ctl.create_bundle(self.root, self.root / "artifacts/source.tar.gz")

    def test_bundle_destination_inside_source_is_rejected(self) -> None:
        with self.assertRaisesRegex(h2ctl.ToolError, "outside source directories"):
            h2ctl.create_bundle(self.root, self.root / "src/snapshot.tar.gz")

    def test_executable_mode_is_part_of_snapshot_identity(self) -> None:
        script = self.root / "src/example/node.cpp"
        first = h2ctl.create_bundle(self.root, self.root / "artifacts/one.tar.gz")
        script.chmod(0o755)
        second = h2ctl.create_bundle(self.root, self.root / "artifacts/two.tar.gz")
        self.assertNotEqual(first["snapshot_sha256"], second["snapshot_sha256"])

    def test_file_size_limit_does_not_leave_partial_archive(self) -> None:
        output = self.root / "artifacts/source.tar.gz"
        with patch.object(h2ctl, "MAX_SOURCE_FILE_BYTES", 4):
            with self.assertRaisesRegex(h2ctl.ToolError, "exceeds 64 MiB"):
                h2ctl.create_bundle(self.root, output)
        self.assertFalse(output.exists())

    def test_existing_output_is_preserved(self) -> None:
        output = self.root / "source.tar.gz"
        output.write_bytes(b"existing")
        with self.assertRaises(FileExistsError):
            h2ctl.create_bundle(self.root, output)
        self.assertEqual(output.read_bytes(), b"existing")


class DoctorAndCheckTests(unittest.TestCase):
    def test_ros_doctor_rejects_unsourced_or_wrong_distro(self) -> None:
        with patch.object(h2ctl.shutil, "which", return_value="/usr/bin/tool"), \
             patch.object(h2ctl.platform, "machine", return_value="x86_64"), \
             patch.object(h2ctl, "read_os_release", return_value={"ID": "ubuntu", "VERSION_ID": "24.04"}), \
             patch.dict(h2ctl.os.environ, {"ROS_DISTRO": "humble"}):
            report = h2ctl.doctor_report("ros")
        self.assertFalse(report["ok"])
        self.assertFalse(next(c for c in report["checks"] if c["name"] == "ros_distro")["ok"])

    def test_missing_tool_does_not_claim_checks_passed(self) -> None:
        with patch.object(h2ctl, "doctor_report", return_value={"ok": False, "checks": [], "scope": "missing tools"}), \
             patch.object(h2ctl.subprocess, "run") as run, patch("sys.stdout", new=io.StringIO()):
            self.assertEqual(h2ctl.check_workspace(Path("."), "core"), 2)
        run.assert_not_called()

    def test_build_failure_stops_before_test_commands(self) -> None:
        completed = h2ctl.subprocess.CompletedProcess(["build"], 7)
        with patch.object(h2ctl.subprocess, "run", return_value=completed) as run, \
             patch("sys.stdout", new=io.StringIO()):
            code = h2ctl.run_commands([["build"], ["test"]], Path("."))
        self.assertEqual(code, 7)
        self.assertEqual(run.call_count, 1)


class RosCheckTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "src").mkdir()
        self.packages = "\n".join(sorted(h2ctl.ROS_PACKAGES)) + "\n"
        self.calls = []
        doctor = patch.object(h2ctl, "doctor_report", return_value={"ok": True})
        doctor.start()
        self.addCleanup(doctor.stop)
        output = patch("sys.stdout", new=io.StringIO())
        self.output = output.start()
        self.addCleanup(output.stop)

    def write_reports(self, core_tag="fresh") -> dict[str, Path]:
        reports = {
            "h2_core": self.root / f"build/ros/h2_core/Testing/{core_tag}/Test.xml",
            "h2_runtime": self.root / "build/ros/h2_runtime/test_results/h2_runtime/runtime_contract.xunit.xml",
            "h2_supervisor": self.root / "build/ros/h2_supervisor/pytest.xml",
        }
        for package, path in reports.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            if package == "h2_core":
                path.write_text('<Site><Testing><Test Status="passed"><Name>core</Name></Test></Testing></Site>')
            else:
                path.write_text('<testsuites><testsuite tests="1" failures="0" errors="0">'
                                '<testcase name="contract"/></testsuite></testsuites>')
        return reports

    def fake_colcon(self, command, **kwargs):
        self.calls.append((command, kwargs))
        verb = command[3]
        if verb == "list":
            return h2ctl.subprocess.CompletedProcess(command, 0, stdout=self.packages, stderr="")
        if verb == "test":
            self.write_reports()
        return h2ctl.subprocess.CompletedProcess(command, 0)

    def test_missing_or_empty_package_discovery_stops_before_build(self) -> None:
        for packages in ("", "h2_core\nh2_runtime\nh2_supervisor\nh2_interfaces\n"):
            with self.subTest(packages=packages):
                self.packages = packages
                self.calls.clear()
                with patch.object(h2ctl.subprocess, "run", side_effect=self.fake_colcon):
                    with self.assertRaisesRegex(h2ctl.ToolError, "missing:"):
                        h2ctl.check_workspace(self.root, "ros")
                self.assertEqual([call[0][3] for call in self.calls], ["list"])
                self.assertNotIn("checks passed", self.output.getvalue())

    def test_package_discovery_failure_is_not_a_successful_empty_workspace(self) -> None:
        failure = h2ctl.subprocess.CompletedProcess(["colcon"], 4, stdout="", stderr="invalid manifest")
        with patch.object(h2ctl.subprocess, "run", return_value=failure) as run:
            with self.assertRaisesRegex(h2ctl.ToolError, "discovery failed.*invalid manifest"):
                h2ctl.check_workspace(self.root, "ros")
        self.assertEqual(run.call_count, 1)

    def test_ros_success_requires_fresh_reports_and_limits_build_workers(self) -> None:
        old_reports = self.write_reports(core_tag="old")
        cache = self.root / "build/ros/h2_core/CMakeCache.txt"
        cache.write_text("keep incremental build cache")

        def run(command, **kwargs):
            if command[3] == "test":
                self.assertTrue(all(not path.exists() for path in old_reports.values()))
            return self.fake_colcon(command, **kwargs)

        with patch.object(h2ctl.subprocess, "run", side_effect=run), \
             patch.dict(h2ctl.os.environ, {"CMAKE_BUILD_PARALLEL_LEVEL": "99", "MAKEFLAGS": "-j99"}):
            self.assertEqual(h2ctl.check_workspace(self.root, "ros", jobs=2), 0)
            self.assertEqual(h2ctl.os.environ["CMAKE_BUILD_PARALLEL_LEVEL"], "99")
            self.assertEqual(h2ctl.os.environ["MAKEFLAGS"], "-j99")
        self.assertEqual([call[0][3] for call in self.calls], ["list", "build", "test", "test-result"])
        for command, kwargs in self.calls:
            self.assertEqual(kwargs["env"]["CMAKE_BUILD_PARALLEL_LEVEL"], "2")
            self.assertEqual(kwargs["env"]["MAKEFLAGS"], "-j2 -l2")
            if command[3] in {"build", "test"}:
                self.assertEqual(command[command.index("--parallel-workers") + 1], "2")
        self.assertTrue(cache.is_file())
        self.assertFalse(old_reports["h2_core"].exists())
        self.assertIn("ROS test evidence verified", self.output.getvalue())
        self.assertIn("ros checks passed", self.output.getvalue())

    def test_previous_success_reports_cannot_replace_this_run(self) -> None:
        old_reports = self.write_reports(core_tag="old")

        def run(command, **kwargs):
            if command[3] == "test":
                self.calls.append((command, kwargs))
                return h2ctl.subprocess.CompletedProcess(command, 0)  # Nothing ran.
            return self.fake_colcon(command, **kwargs)

        with patch.object(h2ctl.subprocess, "run", side_effect=run):
            with self.assertRaisesRegex(h2ctl.ToolError, "Missing test results from this run"):
                h2ctl.check_workspace(self.root, "ros")
        self.assertTrue(all(not path.exists() for path in old_reports.values()))
        self.assertNotIn("checks passed", self.output.getvalue())

    def test_empty_skipped_failed_or_invalid_reports_do_not_pass(self) -> None:
        cases = (
            ("h2_core", "<Site><Testing/></Site>", "No executed tests"),
            ("h2_runtime", '<testsuite><testcase><skipped/></testcase></testsuite>', "No executed tests"),
            ("h2_supervisor", '<testsuite tests="0"/>', "No executed tests"),
            ("h2_runtime", '<testsuite><testcase><failure/></testcase></testsuite>', "unsuccessful tests"),
            ("h2_supervisor", "not xml", "Invalid test report"),
        )
        for package, content, error in cases:
            with self.subTest(package=package, content=content):
                def run(command, **kwargs):
                    result = self.fake_colcon(command, **kwargs)
                    if command[3] == "test":
                        report = next((self.root / "build/ros").glob(h2ctl.ROS_REQUIRED_REPORTS[package]))
                        report.write_text(content)
                    return result

                with patch.object(h2ctl.subprocess, "run", side_effect=run):
                    with self.assertRaisesRegex(h2ctl.ToolError, error):
                        h2ctl.check_workspace(self.root, "ros")
                self.assertNotIn("checks passed", self.output.getvalue())

    def test_failed_ros_build_or_tests_preserve_nonzero_exit(self) -> None:
        for failing_verb in ("build", "test"):
            with self.subTest(verb=failing_verb):
                self.calls.clear()

                def run(command, **kwargs):
                    result = self.fake_colcon(command, **kwargs)
                    return h2ctl.subprocess.CompletedProcess(command, 7) if command[3] == failing_verb else result

                with patch.object(h2ctl.subprocess, "run", side_effect=run):
                    self.assertEqual(h2ctl.check_workspace(self.root, "ros"), 7)
                self.assertNotIn("test-result", [call[0][3] for call in self.calls])
                self.assertNotIn("checks passed", self.output.getvalue())

    def test_report_cleanup_rejects_links_and_preserves_external_file(self) -> None:
        outside = self.root / "external.xml"
        outside.write_text("preserve")
        path = self.root / "build/ros/h2_supervisor/pytest.xml"
        path.parent.mkdir(parents=True)
        path.symlink_to(outside)
        with self.assertRaisesRegex(h2ctl.ToolError, "symbolic links"):
            h2ctl.clear_ros_test_results(self.root)
        self.assertEqual(outside.read_text(), "preserve")


class DeploymentPreviewTests(unittest.TestCase):
    def write_config(self, root: Path, **overrides: object) -> Path:
        config = {"host": "h2-thor", "user": "unitree", "remote_root": "/home/unitree/h2_nav"}
        config.update(overrides)
        path = root / "target.json"
        path.write_text(json.dumps(config))
        return path

    def test_preview_is_explicitly_readonly_and_never_runs_ssh(self) -> None:
        with tempfile.TemporaryDirectory() as temporary, patch.object(h2ctl.subprocess, "run") as run:
            config = self.write_config(Path(temporary))
            report = h2ctl.deployment_preview(config)
        self.assertTrue(report["preview_only"])
        self.assertFalse(report["remote_actions_executed"])
        self.assertEqual(report["activation_mode"], "hardware_readonly")
        self.assertEqual(len(report["slots"]), 2)
        run.assert_not_called()

    def test_unsafe_roots_and_shell_hosts_are_rejected(self) -> None:
        for overrides in [{"remote_root": "/"}, {"remote_root": "/tmp"}, {"remote_root": "/home/unitree/../other"}, {"host": "h2; touch /tmp/test"}]:
            with self.subTest(overrides=overrides), tempfile.TemporaryDirectory() as temporary:
                with self.assertRaises(h2ctl.ToolError):
                    h2ctl.deployment_preview(self.write_config(Path(temporary), **overrides))

    def test_deploy_without_preview_refuses_to_run(self) -> None:
        with patch("sys.stderr", new=io.StringIO()):
            code = h2ctl.main(["deploy", "--target-config", "does-not-exist.json"])
        self.assertEqual(code, 2)


if __name__ == "__main__":
    unittest.main()
