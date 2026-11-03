"""Raw-topic diagnostics and recording boundaries; fixtures are NOT sensor validation."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest


MODULE_PATH = Path(__file__).resolve().parents[2] / "tools/readonly_capture.py"
SPEC = importlib.util.spec_from_file_location("readonly_capture", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
capture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture)


def header(ns, frame="livox_frame"):
    return NS(frame_id=frame, stamp=NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000))


def cloud(ns, offsets=(0, 10_000_000), x=1.0):
    points = [NS(offset_time=offset, x=x, y=0.0, z=0.0, line=1, tag=0) for offset in offsets]
    return NS(header=header(ns), timebase=ns, point_num=len(points), points=points)


def imu(ns, acc=1.0):
    return NS(header=header(ns), linear_acceleration=NS(x=0.0, y=0.0, z=acc),
              angular_velocity=NS(x=0.0, y=0.0, z=0.0))


class RawDiagnosticsTests(unittest.TestCase):
    def nominal(self, source=1_000_000_000):
        checker = capture.RawDiagnostics()
        for index in range(11):
            receipt = index / 10
            stamp = source + index * 100_000_000
            checker.observe_lidar(cloud(stamp), receipt, stamp + 1_000_000)
            checker.observe_imu(imu(stamp), receipt, stamp + 1_000_000)
        return checker

    def report(self, checker):
        return checker.report(1.01, min_lidar_hz=1, min_imu_hz=5, max_silence=0.5)

    def test_missing_streams_fail(self):
        report = self.report(capture.RawDiagnostics())
        self.assertFalse(report["data_checks_passed"])
        self.assertIn("too_few_samples", report["lidar"]["errors"])
        self.assertIn("stream_missing_or_stale", report["imu"]["errors"])

    def test_nominal_fixture_passes_structure_but_does_not_verify_sync_or_orientation(self):
        report = self.report(self.nominal())
        self.assertTrue(report["data_checks_passed"])
        self.assertEqual(report["lidar"]["message_count"], 11)
        self.assertAlmostEqual(report["lidar"]["receipt_hz"], 10)
        self.assertEqual(report["lidar"]["scan_span_max_ns"], 10_000_000)
        self.assertTrue(report["time_synchronization"].startswith("unverified"))
        self.assertIn("g (raw", report["imu"]["linear_acceleration_unit"])
        self.assertEqual(report["imu"]["angular_velocity_unit"], "rad/s")
        self.assertEqual(report["imu"]["orientation_validity"], "not established")

    def test_unaligned_clock_is_reported_without_claiming_wall_clock_failure(self):
        checker = self.nominal(source=9_000_000_000_000)
        checker.observe_lidar(cloud(9_001_100_000_000), 1.1, 1)
        checker.observe_imu(imu(9_001_100_000_000), 1.1, 1)
        report = checker.report(1.11, 1, 5, 0.5)
        self.assertTrue(report["data_checks_passed"])
        self.assertLess(report["lidar"]["host_minus_source_ms"]["min"], 0)
        self.assertEqual(report["lidar"]["host_minus_source_ms"]["clock_alignment"], "unverified")

    def test_nonincreasing_headers_fail(self):
        checker = self.nominal()
        checker.observe_lidar(cloud(2_000_000_000), 1.01, 2_000_000_000)
        checker.observe_imu(imu(1_900_000_000), 1.01, 2_000_000_000)
        report = self.report(checker)
        self.assertFalse(report["data_checks_passed"])
        self.assertEqual(report["lidar"]["errors"]["nonincreasing_stamp"], 1)
        self.assertEqual(report["imu"]["errors"]["nonincreasing_stamp"], 1)

    def test_count_and_timebase_corruption_fail(self):
        checker = self.nominal()
        msg = cloud(2_100_000_000, offsets=(10, 0))
        msg.point_num += 1
        msg.timebase += 1
        checker.observe_lidar(msg, 1.01, 2_100_000_000)
        report = self.report(checker)
        self.assertFalse(report["data_checks_passed"])
        for name in ("point_num_mismatch", "header_timebase_mismatch"):
            self.assertIn(name, report["lidar"]["errors"])

    def test_in_range_point_offsets_may_arrive_out_of_order(self):
        checker = self.nominal()
        checker.observe_lidar(cloud(2_100_000_000, offsets=(0, 30_000_000, 20_000_000)),
                              1.01, 2_100_000_000)
        report = self.report(checker)
        self.assertTrue(report["data_checks_passed"])
        self.assertEqual(report["lidar"]["warnings"], {"nonmonotonic_point_offsets": 1})
        self.assertEqual(report["lidar"]["scan_span_max_ns"], 30_000_000)

    def test_nonfinite_observations_fail(self):
        checker = self.nominal()
        checker.observe_lidar(cloud(2_100_000_000, x=float("nan")), 1.01, 2_100_000_000)
        checker.observe_imu(imu(2_100_000_000, acc=float("inf")), 1.01, 2_100_000_000)
        report = self.report(checker)
        self.assertFalse(report["data_checks_passed"])
        self.assertIn("nonfinite_point", report["lidar"]["errors"])
        self.assertIn("nonfinite_raw_imu", report["imu"]["errors"])

    def test_empty_scan_and_large_offsets_fail(self):
        checker = self.nominal()
        checker.observe_lidar(cloud(2_100_000_000, offsets=()), 1.01, 2_100_000_000)
        checker.observe_lidar(cloud(2_200_000_000, offsets=(0, 1_000_000_001)), 1.02, 2_200_000_000)
        report = self.report(checker)
        self.assertIn("empty_scan", report["lidar"]["errors"])
        self.assertIn("point_offset_exceeds_scan_limit", report["lidar"]["errors"])

    def test_missing_imu_cannot_pass_even_with_lidar_present(self):
        checker = capture.RawDiagnostics()
        checker.observe_lidar(cloud(1_900_000_000), 0.9, 1_900_000_000)
        checker.observe_lidar(cloud(2_000_000_000), 1, 2_000_000_000)
        report = self.report(checker)
        self.assertTrue(report["lidar"]["data_checks_passed"])
        self.assertFalse(report["data_checks_passed"])

    def test_stale_stream_and_low_rate_fail(self):
        checker = self.nominal()
        report = checker.report(3, min_lidar_hz=20, min_imu_hz=20, max_silence=0.5)
        self.assertFalse(report["data_checks_passed"])
        self.assertIn("stream_missing_or_stale", report["lidar"]["errors"])
        self.assertIn("receipt_rate_below_minimum", report["imu"]["errors"])

    def test_mid_capture_gap_cannot_pass_after_late_recovery(self):
        checker = self.nominal()
        for index in range(11):
            stamp = 4_000_000_000 + index * 10_000_000
            receipt = 3.0 + index / 100
            checker.observe_lidar(cloud(stamp), receipt, stamp)
            checker.observe_imu(imu(stamp), receipt, stamp)
        report = checker.report(3.11, min_lidar_hz=1, min_imu_hz=5, max_silence=0.5)
        self.assertFalse(report["data_checks_passed"])
        for name in ("lidar", "imu"):
            stream = report[name]
            self.assertGreaterEqual(stream["receipt_hz"], stream["required_min_hz"])
            self.assertLess(stream["last_receipt_age_s"], 0.5)
            self.assertEqual(stream["max_receipt_gap_s"], 2.0)
            self.assertIn("receipt_gap_exceeds_limit", stream["errors"])

    def test_frames_must_be_nonempty_and_consistent(self):
        checker = self.nominal()
        msg = imu(2_100_000_000)
        msg.header.frame_id = ""
        checker.observe_imu(msg, 1.01, 2_100_000_000)
        errors = self.report(checker)["imu"]["errors"]
        self.assertIn("empty_frame_id", errors)
        self.assertIn("inconsistent_frame_id", errors)

    def test_stamp_fields_are_not_silently_normalized(self):
        for stamp in (NS(sec=0, nanosec=0), NS(sec=1, nanosec=-1),
                      NS(sec=1, nanosec=1_000_000_000), NS(sec=-1, nanosec=0)):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                capture.stamp_ns(stamp)


class RecordingBoundariesTests(unittest.TestCase):
    expected = {"/livox/lidar": "livox_ros_driver2/msg/CustomMsg", "/livox/imu": "sensor_msgs/msg/Imu"}

    def metadata(self):
        return {"rosbag2_bagfile_information": {"topics_with_message_count": [
            {"topic_metadata": {"name": name, "type": kind}, "message_count": 5}
            for name, kind in self.expected.items()]}}

    def test_bag_success_requires_both_types_with_nonzero_messages(self):
        self.assertTrue(capture.validate_bag_metadata(self.metadata(), self.expected)["metadata_checks_passed"])
        for missing in (True, False):
            data = self.metadata()
            entries = data["rosbag2_bagfile_information"]["topics_with_message_count"]
            if missing:
                entries.pop()
            else:
                entries[0]["message_count"] = 0
            self.assertFalse(capture.validate_bag_metadata(data, self.expected)["metadata_checks_passed"])

    def test_wrong_type_or_extra_topic_cannot_pass_recording(self):
        data = self.metadata()
        entries = data["rosbag2_bagfile_information"]["topics_with_message_count"]
        entries[0]["topic_metadata"]["type"] = "sensor_msgs/msg/PointCloud2"
        self.assertFalse(capture.validate_bag_metadata(data, self.expected)["metadata_checks_passed"])
        entries[0]["topic_metadata"]["type"] = self.expected["/livox/lidar"]
        entries.append({"topic_metadata": {"name": "/motion/command", "type": "unwanted"}, "message_count": 1})
        self.assertFalse(capture.validate_bag_metadata(data, self.expected)["metadata_checks_passed"])

    def test_malformed_or_duplicate_metadata_is_not_treated_as_complete(self):
        for data in (None, [], {}, {"rosbag2_bagfile_information": []}):
            with self.subTest(data=data):
                self.assertFalse(capture.validate_bag_metadata(data, self.expected)["metadata_checks_passed"])
        data = self.metadata()
        entries = data["rosbag2_bagfile_information"]["topics_with_message_count"]
        entries.append(dict(entries[0]))
        self.assertFalse(capture.validate_bag_metadata(data, self.expected)["metadata_checks_passed"])
        entries.pop()
        entries[0]["message_count"] = "invalid"
        self.assertFalse(capture.validate_bag_metadata(data, self.expected)["metadata_checks_passed"])

    def test_recorder_selects_only_explicit_topics_and_uses_argv(self):
        command = capture.recorder_command(Path("/tmp/capture with spaces"), "/livox/lidar", "/livox/imu", "h2_raw_recorder_123")
        self.assertEqual(command[-3:], ["--topics", "/livox/lidar", "/livox/imu"])
        self.assertIn("/tmp/capture with spaces", command)
        self.assertNotIn("--all", command)
        self.assertNotIn("--use-sim-time", command)
        self.assertEqual(command[command.index("--node-name") + 1], "h2_raw_recorder_123")

    def test_recorder_readiness_requires_its_own_two_typed_endpoints(self):
        name = "h2_raw_recorder_123"
        endpoints = {topic: [NS(node_name=name, topic_type=kind)]
                     for topic, kind in self.expected.items()}
        self.assertTrue(capture.recorder_endpoints_ready(endpoints, name, self.expected))

    def test_observer_or_another_recorder_cannot_satisfy_readiness(self):
        endpoints = {topic: [NS(node_name="h2_raw_readonly_observer_123", topic_type=kind),
                             NS(node_name="h2_raw_recorder_456", topic_type=kind)]
                     for topic, kind in self.expected.items()}
        self.assertFalse(capture.recorder_endpoints_ready(endpoints, "h2_raw_recorder_123", self.expected))

    def test_missing_or_wrong_type_recorder_subscription_is_not_ready(self):
        name = "h2_raw_recorder_123"
        endpoints = {topic: [NS(node_name=name, topic_type=kind)]
                     for topic, kind in self.expected.items()}
        endpoints["/livox/imu"] = [NS(node_name=name, topic_type="wrong/type")]
        self.assertFalse(capture.recorder_endpoints_ready(endpoints, name, self.expected))
        del endpoints["/livox/imu"]
        self.assertFalse(capture.recorder_endpoints_ready(endpoints, name, self.expected))

    def test_disk_budget_boundaries_are_fail_closed(self):
        self.assertIsNone(capture.budget_reason(99, 100, 100, 100))
        self.assertEqual(capture.budget_reason(100, 100, 100, 100), "recording_size_budget_reached")
        self.assertEqual(capture.budget_reason(99, 99, 100, 100), "minimum_free_space_reached")

    def test_existing_report_is_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "report.json"
            path.write_text("previous data")
            with self.assertRaises(FileExistsError):
                capture.write_report(path, {"ok": True})
            self.assertEqual(path.read_text(), "previous data")

    def test_report_serializes_without_nan_and_rejects_relative_path(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "report.json"
            capture.write_report(path, self.metadata())
            self.assertEqual(json.loads(path.read_text()), self.metadata())
        with self.assertRaises(capture.CaptureError):
            capture.write_report(Path("relative.json"), {})

    def test_cli_rejects_unsafe_topic_and_nonfinite_duration(self):
        self.assertEqual(capture.topic_name("/livox/lidar"), "/livox/lidar")
        for value in ("/livox/lidar:=/motion/command", "relative", "/", "/a//b"):
            with self.subTest(value=value), self.assertRaises(Exception):
                capture.topic_name(value)
        for value in ("nan", "inf", "-1", "0"):
            with self.subTest(value=value), self.assertRaises(Exception):
                capture.positive(value)


if __name__ == "__main__":
    unittest.main()
