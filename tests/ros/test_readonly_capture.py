"""Exercise the capture CLI with synthetic ROS samples, never robot hardware.

Only CustomMsg/Imu fixture publishers are created in an isolated localhost domain.
This verifies tool behavior and recorded bytes, not MID-360/H2 acquisition quality.
"""

import json
import math
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import sys
import time
import unittest

from livox_ros_driver2.msg import CustomMsg, CustomPoint
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


ROOT = Path(__file__).resolve().parents[2]
RECORDER_COLDSTART_DELAY_S = 1.5


def process_tree(root_pid):
    """Return descendant PID/group pairs, including recorder's separate group."""
    snapshot = subprocess.run(
        ["ps", "-eo", "pid=,ppid=,pgid="], text=True, capture_output=True, check=True)
    rows = [tuple(map(int, line.split())) for line in snapshot.stdout.splitlines()]
    parents = {root_pid}
    children = {}
    while True:
        discovered = {pid: group for pid, parent, group in rows
                      if parent in parents and pid not in parents}
        if not discovered:
            return children
        children.update(discovered)
        parents.update(discovered)


class SyntheticReadonlyCapture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = cls.node = cls.executor = None
        cls.processes = []
        cls.log_files = []
        cls.observed_descendants = {}
        cls.old_environment = {}
        cls.addClassCleanup(cls.cleanup)
        run_id = time.strftime("%Y%m%d-%H%M%S") + f"-{os.getpid()}"
        cls.evidence = ROOT / "artifacts/readonly-capture-smoke" / run_id
        cls.evidence.mkdir(parents=True, exist_ok=False)
        cls.namespace = f"/synthetic_readonly_capture_{os.getpid()}"
        cls.lidar_topic = cls.namespace + "/lidar"
        cls.imu_topic = cls.namespace + "/imu"
        cls.frame = "synthetic_mid360_fixture_only"
        cls.environment = os.environ.copy()
        updates = {
            "ROS_DOMAIN_ID": str(100 + os.getpid() % 20),
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_STATIC_PEERS": "",
            "ROS_HOME": str(cls.evidence / "ros-home"),
            "ROS_LOG_DIR": str(cls.evidence / "ros-logs"),
        }
        for key, value in updates.items():
            cls.old_environment[key] = os.environ.get(key)
            os.environ[key] = value
            cls.environment[key] = value
        cls.config = cls.evidence / "synthetic-config.json"
        cls.config.write_text(json.dumps({
            "synthetic_fixture_only": True,
            "purpose": "capture CLI integration test; no sensor SDK is initialized",
            "synthetic_recorder_coldstart_delay_s": RECORDER_COLDSTART_DELAY_S,
        }) + "\n")
        (cls.evidence / "fixture.json").write_text(json.dumps({
            "source": "synthetic ROS fixture; no MID-360 or H2 hardware",
            "frame": cls.frame,
            "topics": [cls.lidar_topic, cls.imu_topic],
            "domain": updates["ROS_DOMAIN_ID"],
            "publishers": "CustomMsg and Imu only; no TF/motion/permit publishers",
            "recorder_coldstart_delay_s": RECORDER_COLDSTART_DELAY_S,
        }, indent=2) + "\n")
        cls.context = Context()
        rclpy.init(args=[], context=cls.context)
        cls.node = rclpy.create_node("synthetic_readonly_source", context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)
        cls.lidar_publisher = cls.node.create_publisher(CustomMsg, cls.lidar_topic, 10)
        cls.imu_publisher = cls.node.create_publisher(Imu, cls.imu_topic, 10)
        cls.tick = 0
        cls.node.create_timer(0.01, cls.publish_fixture)
        print(f"Synthetic capture tool evidence: {cls.evidence}", flush=True)

    @classmethod
    def publish_fixture(cls):
        stamp = cls.node.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = cls.frame
        imu.linear_acceleration.z = 1.0  # Synthetic raw-g sample, not calibration.
        cls.imu_publisher.publish(imu)
        cls.tick += 1
        if cls.tick % 5:
            return
        cloud = CustomMsg()
        cloud.header.stamp = stamp
        cloud.header.frame_id = cls.frame
        cloud.timebase = stamp.sec * 1_000_000_000 + stamp.nanosec
        cloud.point_num = 2
        for index in range(2):
            point = CustomPoint()
            point.x = 1.0 + index * 0.01
            point.y = 0.0
            point.z = 0.0
            point.offset_time = index * 1_000_000
            point.reflectivity = 7
            point.line = index
            point.tag = 0
            cloud.points.append(point)
        cls.lidar_publisher.publish(cloud)

    def delayed_recorder_environment(self):
        """Delay only ros2 bag record, then exec its real absolute executable."""
        real_ros2 = shutil.which("ros2", path=self.environment["PATH"])
        self.assertIsNotNone(real_ros2, "ros2 is required for the recording integration test")
        real_ros2 = str(Path(real_ros2).resolve(strict=True))
        wrapper_directory = self.evidence / "synthetic-recorder-wrapper"
        wrapper_directory.mkdir()
        wrapper = wrapper_directory / "ros2"
        wrapper.write_text(
            f"#!{sys.executable}\n"
            '"""Synthetic test-only recorder cold-start delay; no shell or fake recorder."""\n'
            "import os\nimport sys\nimport time\n"
            "if sys.argv[1:3] == ['bag', 'record']:\n"
            f"    time.sleep({RECORDER_COLDSTART_DELAY_S!r})\n"
            f"os.execv({real_ros2!r}, [{real_ros2!r}, *sys.argv[1:]])\n"
        )
        wrapper.chmod(0o700)
        environment = self.environment.copy()
        environment["PATH"] = str(wrapper_directory) + os.pathsep + environment["PATH"]
        return environment

    def run_cli(self, name, command, expected_code, *, imu_topic=None, environment=None):
        log = (self.evidence / f"{name}.log").open("x")
        self.log_files.append(log)
        arguments = [sys.executable, str(ROOT / "tools/readonly_capture.py"), *command,
                     "--lidar-topic", self.lidar_topic,
                     "--imu-topic", imu_topic or self.imu_topic]
        process = subprocess.Popen(
            arguments, env=self.environment if environment is None else environment,
            cwd=ROOT, stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        deadline = time.monotonic() + 16.0
        next_tree_snapshot = 0.0
        while process.poll() is None and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
            if time.monotonic() >= next_tree_snapshot:
                self.observed_descendants.update(process_tree(process.pid))
                next_tree_snapshot = time.monotonic() + 0.2
        log.flush()
        self.assertIsNotNone(process.poll(), f"capture CLI timed out; see {name}.log")
        self.assertEqual(process.returncode, expected_code,
                         f"capture exit {process.returncode}; see {self.evidence / (name + '.log')}")
        # The record CLI must reap its recorder before returning, even though
        # rosbag runs in a separate process group.
        remaining = [pid for pid in self.observed_descendants if Path(f"/proc/{pid}").exists()]
        self.assertEqual(remaining, [], f"capture left descendant processes: {remaining}")
        return process.pid

    def test_inspect_and_record_synthetic_streams(self):
        report_path = self.evidence / "inspect.json"
        self.run_cli("inspect", ["inspect", "--duration", "1.2", "--report", str(report_path)], 0)
        report = json.loads(report_path.read_text())
        self.assertTrue(report["ok"], report)
        for stream in ("lidar", "imu"):
            self.assertGreater(report[stream]["message_count"], 2)
            self.assertEqual(report[stream]["frames_observed"], [self.frame])

        session = self.evidence / "record-session"
        record_pid = self.run_cli("record", [
            "record", "--duration", "1.8", "--startup-timeout", "6",
            "--output", str(session), "--config", str(self.config),
            "--min-free-mib", "1", "--max-size-mib", "16",
        ], 0, environment=self.delayed_recorder_environment())
        record = json.loads((session / "report.json").read_text())
        self.assertTrue(record["ok"], record)
        self.assertIs(record["recorder_ready"], True)
        self.assertEqual(record["recorder_node_name"], f"h2_raw_recorder_{record_pid}")
        self.assertTrue(math.isfinite(record["recorder_ready_wait_s"]))
        self.assertGreaterEqual(record["recorder_ready_wait_s"], RECORDER_COLDSTART_DELAY_S)
        self.assertTrue(record["bag"]["metadata_checks_passed"])
        self.assertEqual(record["recorder_exit_code"], 0)
        self.assertFalse(record["recorder_forced_cleanup"])
        self.assertEqual(json.loads((session / "mid360-config.json").read_text()),
                         json.loads(self.config.read_text()))

        expected = {self.lidar_topic: ("livox_ros_driver2/msg/CustomMsg", CustomMsg),
                    self.imu_topic: ("sensor_msgs/msg/Imu", Imu)}
        counts = {name: 0 for name in expected}
        databases = list((session / "bag").glob("*.db3"))
        self.assertTrue(databases, "no SQLite bag files were finalized")
        for database in databases:
            with sqlite3.connect(f"file:{database}?mode=ro", uri=True) as connection:
                rows = connection.execute("SELECT id, name, type FROM topics").fetchall()
                self.assertEqual({row[1] for row in rows}, set(expected))
                for topic_id, name, message_type in rows:
                    self.assertEqual(message_type, expected[name][0])
                    count = connection.execute(
                        "SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic_id,)).fetchone()[0]
                    counts[name] += count
                    sample = connection.execute(
                        "SELECT data FROM messages WHERE topic_id=? LIMIT 1", (topic_id,)).fetchone()
                    self.assertIsNotNone(sample, name)
                    message = deserialize_message(sample[0], expected[name][1])
                    self.assertEqual(message.header.frame_id, self.frame)
                    if name == self.lidar_topic:
                        self.assertEqual(message.point_num, 2)
                        self.assertEqual([point.offset_time for point in message.points], [0, 1_000_000])
        self.assertTrue(all(count >= 2 for count in counts.values()), counts)
        (self.evidence / "bag-byte-check.json").write_text(json.dumps({
            "source": "synthetic fixture",
            "actual_sqlite_message_counts": counts,
            "types_and_samples_deserialized": True,
        }, indent=2) + "\n")

    def test_missing_imu_stream_fails(self):
        report_path = self.evidence / "missing-imu.json"
        self.run_cli("missing-imu", ["inspect", "--duration", "1.0", "--report", str(report_path)],
                     2, imu_topic=self.namespace + "/not_published_imu")
        report = json.loads(report_path.read_text())
        self.assertFalse(report["ok"])
        self.assertFalse(report["data_checks_passed"])
        self.assertGreater(report["lidar"]["message_count"], 2)
        self.assertEqual(report["imu"]["message_count"], 0)
        self.assertIn("stream_missing_or_stale", report["imu"]["errors"])

    @classmethod
    def cleanup(cls):
        errors = []
        for process in reversed(cls.processes):
            if process.poll() is None:
                cls.observed_descendants.update(process_tree(process.pid))
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
                    errors.append("capture CLI needed forced termination")
                except ProcessLookupError:
                    process.wait(timeout=2)
        for pid, group in cls.observed_descendants.items():
            if Path(f"/proc/{pid}").exists():
                try:
                    os.killpg(group, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                errors.append(f"capture descendant {pid} required cleanup")
        if cls.executor is not None:
            if cls.node is not None:
                cls.executor.remove_node(cls.node)
            cls.executor.shutdown(timeout_sec=1)
        if cls.node is not None:
            cls.node.destroy_node()
        if cls.context is not None and cls.context.ok():
            cls.context.shutdown()
        for log in cls.log_files:
            log.close()
        for key, value in cls.old_environment.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        if errors:
            raise AssertionError("; ".join(errors))


if __name__ == "__main__":
    unittest.main(verbosity=2)
