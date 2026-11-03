#!/usr/bin/env python3
"""Inspect/record raw MID-360 topics without publishing robot commands.

Success means this tool observed the declared streams and checked their structure.
It does not establish time synchronization, calibration or localization quality.
ROS imports are lazy so the validation logic can be tested without ROS.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
MIB = 1024 * 1024


class CaptureError(RuntimeError):
    pass


def positive(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be positive and finite")
    return number


def topic_name(value: str) -> str:
    if not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*", value):
        raise argparse.ArgumentTypeError("use an absolute ROS topic name")
    return value


def stamp_ns(stamp) -> int:
    if not isinstance(stamp.sec, int) or not isinstance(stamp.nanosec, int):
        raise ValueError("stamp fields must be integers")
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("stamp is outside the supported nonnegative range")
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    if value <= 0:
        raise ValueError("zero timestamp")
    return value


class StreamStats:
    """Bounded-memory receipt and source-time checks, with no restamping."""

    def __init__(self):
        self.count = 0
        self.first_receipt = self.last_receipt = None
        self.max_receipt_gap_s = None
        self.first_stamp = self.last_stamp = None
        self.min_stamp = self.max_stamp = None
        self.age_min_ms = self.age_max_ms = None
        self.frames = set()
        self.errors = Counter()

    def observe(self, header, receipt: float, wall_ns: int) -> int | None:
        self.count += 1
        if self.first_receipt is None:
            self.first_receipt = receipt
        if self.last_receipt is not None:
            gap = max(0.0, receipt - self.last_receipt)
            self.max_receipt_gap_s = gap if self.max_receipt_gap_s is None else max(self.max_receipt_gap_s, gap)
        self.last_receipt = receipt
        frame = str(header.frame_id)
        if len(self.frames) < 16:
            self.frames.add(frame)
        if not frame:
            self.errors["empty_frame_id"] += 1
        try:
            source = stamp_ns(header.stamp)
        except (ValueError, TypeError, AttributeError):
            self.errors["invalid_stamp"] += 1
            return None
        if self.last_stamp is not None and source <= self.last_stamp:
            self.errors["nonincreasing_stamp"] += 1
        if self.first_stamp is None:
            self.first_stamp = source
        self.last_stamp = source
        self.min_stamp = source if self.min_stamp is None else min(self.min_stamp, source)
        self.max_stamp = source if self.max_stamp is None else max(self.max_stamp, source)
        age = (wall_ns - source) / 1_000_000
        self.age_min_ms = age if self.age_min_ms is None else min(self.age_min_ms, age)
        self.age_max_ms = age if self.age_max_ms is None else max(self.age_max_ms, age)
        return source

    def report(self, now: float, min_hz: float, max_silence: float) -> dict:
        span = 0.0 if self.first_receipt is None else self.last_receipt - self.first_receipt
        hz = (self.count - 1) / span if self.count >= 2 and span > 0 else 0.0
        silence = None if self.last_receipt is None else max(0.0, now - self.last_receipt)
        errors = dict(self.errors)
        if self.count < 2:
            errors["too_few_samples"] = 1
        if hz < min_hz:
            errors["receipt_rate_below_minimum"] = 1
        if silence is None or silence > max_silence:
            errors["stream_missing_or_stale"] = 1
        if self.max_receipt_gap_s is not None and self.max_receipt_gap_s > max_silence:
            errors["receipt_gap_exceeds_limit"] = 1
        if len(self.frames) != 1:
            errors["inconsistent_frame_id"] = 1
        return {
            "message_count": self.count,
            "receipt_hz": hz,
            "required_min_hz": min_hz,
            "last_receipt_age_s": silence,
            "max_receipt_gap_s": self.max_receipt_gap_s,
            "source_first_ns": self.first_stamp,
            "source_last_ns": self.last_stamp,
            "frames_observed": sorted(self.frames),
            "host_minus_source_ms": {"min": self.age_min_ms, "max": self.age_max_ms,
                                     "clock_alignment": "unverified"},
            "errors": errors,
            "data_checks_passed": not errors,
        }


class RawDiagnostics:
    def __init__(self, max_scan_span_ms: float = 1000.0):
        self.lidar = StreamStats()
        self.imu = StreamStats()
        self.max_scan_span_ns = int(max_scan_span_ms * 1_000_000)
        self.scan_span_min_ns = self.scan_span_max_ns = None
        self.lidar_end_max_ns = None
        self.points_seen = 0
        self.lines = set()
        self.tags = set()

    def observe_lidar(self, msg, receipt: float, wall_ns: int) -> None:
        source = self.lidar.observe(msg.header, receipt, wall_ns)
        if source is not None and source != msg.timebase:
            self.lidar.errors["header_timebase_mismatch"] += 1
        points = msg.points
        if msg.point_num != len(points):
            self.lidar.errors["point_num_mismatch"] += 1
        if not points:
            self.lidar.errors["empty_scan"] += 1
            return
        self.points_seen += len(points)
        offsets = []
        previous = None
        for point in points:
            offset = point.offset_time
            if not isinstance(offset, int) or not 0 <= offset <= 0xFFFFFFFF:
                self.lidar.errors["invalid_point_offset"] += 1
                continue
            offsets.append(offset)
            if previous is not None and offset < previous:
                self.lidar.errors["nonmonotonic_point_offsets"] += 1
            previous = offset
            if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
                self.lidar.errors["nonfinite_point"] += 1
            self.lines.add(int(point.line))
            self.tags.add(int(point.tag))
        if not offsets:
            return
        span = max(offsets) - min(offsets)
        if max(offsets) > self.max_scan_span_ns:
            self.lidar.errors["point_offset_exceeds_scan_limit"] += 1
        self.scan_span_min_ns = span if self.scan_span_min_ns is None else min(self.scan_span_min_ns, span)
        self.scan_span_max_ns = span if self.scan_span_max_ns is None else max(self.scan_span_max_ns, span)
        end = int(msg.timebase) + max(offsets)
        self.lidar_end_max_ns = end if self.lidar_end_max_ns is None else max(self.lidar_end_max_ns, end)

    def observe_imu(self, msg, receipt: float, wall_ns: int) -> None:
        self.imu.observe(msg.header, receipt, wall_ns)
        vectors = (msg.angular_velocity, msg.linear_acceleration)
        if not all(math.isfinite(value) for vector in vectors for value in (vector.x, vector.y, vector.z)):
            self.imu.errors["nonfinite_raw_imu"] += 1

    def report(self, now: float, min_lidar_hz: float, min_imu_hz: float,
               max_silence: float) -> dict:
        lidar = self.lidar.report(now, min_lidar_hz, max_silence)
        imu = self.imu.report(now, min_imu_hz, max_silence)
        lidar.update({"points_seen": self.points_seen, "scan_span_min_ns": self.scan_span_min_ns,
                      "scan_span_max_ns": self.scan_span_max_ns,
                      "line_values": sorted(self.lines), "tag_values": sorted(self.tags)})
        imu.update({"linear_acceleration_unit": "g (raw pinned Livox driver)",
                    "angular_velocity_unit": "rad/s", "orientation_validity": "not established"})
        overlap = None
        if self.lidar.min_stamp is not None and self.imu.min_stamp is not None:
            overlap = max(0, min(self.lidar_end_max_ns or self.lidar.max_stamp,
                                 self.imu.max_stamp) - max(self.lidar.min_stamp, self.imu.min_stamp))
        return {
            "scope": "raw stream presence, structure and receipt/source-time diagnostics only",
            "data_checks_passed": lidar["data_checks_passed"] and imu["data_checks_passed"],
            "time_synchronization": "unverified; source clock/PTP mode and alignment require independent evidence",
            "calibration_and_tf": "unverified; no TF is published by this tool",
            "source_range_overlap_ns": overlap,
            "overlap_limit": "numeric timestamp overlap only; capture-window edges and clock bases are unverified",
            "lidar": lidar, "imu": imu,
        }


def recorder_command(output: Path, lidar_topic: str, imu_topic: str) -> list[str]:
    return ["ros2", "bag", "record", "--storage", "sqlite3", "--output", str(output),
            "--disable-keyboard-controls", "--topics", lidar_topic, imu_topic]


def validate_bag_metadata(metadata: dict, expected: dict[str, str]) -> dict:
    if not isinstance(metadata, dict):
        return {"metadata_checks_passed": False, "topics": {}, "error": "metadata is not a map"}
    info = metadata.get("rosbag2_bagfile_information", {})
    if not isinstance(info, dict) or not isinstance(info.get("topics_with_message_count"), list):
        return {"metadata_checks_passed": False, "topics": {}, "error": "missing topic counts"}
    observed = {}
    valid = True
    for entry in info["topics_with_message_count"]:
        if not isinstance(entry, dict) or not isinstance(entry.get("topic_metadata"), dict):
            valid = False
            continue
        topic = entry.get("topic_metadata", {})
        name = topic.get("name")
        count = entry.get("message_count", 0)
        if not isinstance(name, str) or name in observed or not isinstance(count, int) or isinstance(count, bool):
            valid = False
            continue
        observed[name] = {"type": topic.get("type"), "count": count}
    valid = valid and set(observed) == set(expected) and all(
        observed[name]["type"] == message_type and observed[name]["count"] > 0
        for name, message_type in expected.items() if name in observed)
    return {"metadata_checks_passed": valid, "topics": observed}


def directory_size(path: Path) -> int:
    return sum(file.stat().st_size for file in path.rglob("*") if file.is_file() and not file.is_symlink())


def budget_reason(total: int, free: int, max_bytes: int, min_free: int) -> str | None:
    if free < min_free:
        return "minimum_free_space_reached"
    if total >= max_bytes:
        return "recording_size_budget_reached"
    return None


def snapshot_environment(config: Path, session: Path) -> dict:
    data = config.read_bytes()
    if len(data) > MIB:
        raise CaptureError("config exceeds 1 MiB")
    json.loads(data)
    (session / "mid360-config.json").write_bytes(data)
    result = {"architecture": platform.machine(), "platform": platform.platform(),
              "ROS_DISTRO": os.environ.get("ROS_DISTRO"),
              "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
              "observer_tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "config_sha256": hashlib.sha256(data).hexdigest()}
    for key, command in (
        ("git_head", ["git", "-C", str(ROOT), "rev-parse", "HEAD"]),
        ("rtabmap_packages", ["dpkg-query", "-W", "-f=${Package}=${Version}\n", "ros-jazzy-rtabmap*"]),
    ):
        try:
            completed = subprocess.run(command, capture_output=True, text=True, timeout=5, check=False)
            result[key] = completed.stdout.strip() if completed.returncode == 0 else "unavailable"
        except (OSError, subprocess.TimeoutExpired):
            result[key] = "unavailable"
    for name, source in (("navigation.repos", ROOT / "dependencies/navigation.repos"),
                         ("build-summary.txt", ROOT / "log/mapping/build-summary.txt")):
        if source.is_file():
            content = source.read_bytes()
            (session / name).write_bytes(content)
            result[name + ".sha256"] = hashlib.sha256(content).hexdigest()
    return result


def stop_recorder(process: subprocess.Popen) -> tuple[int, bool]:
    forced = False
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        return process.wait(timeout=15), forced
    except subprocess.TimeoutExpired:
        forced = True
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            return process.wait(timeout=3), forced
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            return process.wait(timeout=3), forced


def run_observation(args) -> tuple[dict, int]:
    # Only subscriptions are created; no robot SDK, TF or motion publishers.
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from livox_ros_driver2.msg import CustomMsg
        from sensor_msgs.msg import Imu
    except ImportError as error:
        raise CaptureError("source Jazzy and install/mapping/setup.bash first: " + str(error)) from error
    if os.environ.get("ROS_DISTRO") != "jazzy":
        raise CaptureError("ROS_DISTRO must be jazzy")
    if args.lidar_topic == args.imu_topic:
        raise CaptureError("lidar and IMU topics must be distinct")
    if args.duration > 3600:
        raise CaptureError("duration must not exceed 3600 seconds")
    session = args.output if args.command == "record" else None
    report = {"completed_duration": False, "failure": None, "robot_motion": "no command/permit/TF publishers"}
    process = log = None
    context = Context()
    node = executor = None
    diagnostics = RawDiagnostics(args.max_scan_span_ms)
    if session is not None:
        if not session.is_absolute() or session.exists() or session.is_symlink():
            raise CaptureError("--output must be an absolute, new session directory")
        if not args.config.is_file() or not args.config.is_absolute():
            raise CaptureError("--config must be an existing absolute JSON file")
        if shutil.which("ros2") is None:
            raise CaptureError("ros2 command is missing")
        session.parent.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(session.parent).free < args.min_free_mib * MIB:
            raise CaptureError("insufficient free space before recording")
        # Reserve the session path atomically; never overwrite a previous run.
        session.mkdir(mode=0o700)
        report["environment"] = snapshot_environment(args.config, session)
    try:
        rclpy.init(args=[], context=context)
        node = rclpy.create_node("h2_raw_readonly_observer_" + str(os.getpid()), context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        node.create_subscription(CustomMsg, args.lidar_topic,
                                 lambda msg: diagnostics.observe_lidar(msg, time.monotonic(), time.time_ns()),
                                 qos_profile_sensor_data)
        node.create_subscription(Imu, args.imu_topic,
                                 lambda msg: diagnostics.observe_imu(msg, time.monotonic(), time.time_ns()),
                                 qos_profile_sensor_data)
        if session is not None:
            deadline = time.monotonic() + args.startup_timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
                if diagnostics.lidar.count >= 2 and diagnostics.imu.count >= 2:
                    break
            else:
                raise CaptureError("both raw streams were not observed before startup timeout")
            # Inspect exactly the recording window, not just the startup samples.
            diagnostics = RawDiagnostics(args.max_scan_span_ms)
            command = recorder_command(session / "bag", args.lidar_topic, args.imu_topic)
            report["recorder_command"] = command
            log = (session / "recorder.log").open("x")
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        started = time.monotonic()
        next_budget_check = started
        while time.monotonic() - started < args.duration:
            executor.spin_once(timeout_sec=0.1)
            if process is not None and process.poll() is not None:
                raise CaptureError(f"recorder exited before duration (code {process.returncode})")
            now = time.monotonic()
            if session is not None and now >= next_budget_check:
                reason = budget_reason(directory_size(session), shutil.disk_usage(session).free,
                                       int(args.max_size_mib * MIB), int(args.min_free_mib * MIB))
                if reason:
                    raise CaptureError(reason + "; partial data retained")
                next_budget_check = now + 0.25
        report["completed_duration"] = True
    except KeyboardInterrupt:
        report["failure"] = "interrupted; partial data retained"
    except Exception as error:
        report["failure"] = str(error) or type(error).__name__
    finally:
        observed_at = time.monotonic()
        if process is not None:
            code, forced = stop_recorder(process)
            report["recorder_exit_code"] = code
            report["recorder_forced_cleanup"] = forced
            if code != 0 or forced:
                report["failure"] = report["failure"] or "recorder did not finalize normally"
        if log is not None:
            log.close()
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown(timeout_sec=1)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        report.update(diagnostics.report(observed_at, args.min_lidar_hz,
                                         args.min_imu_hz, args.max_silence))
    report["topics"] = {"lidar": args.lidar_topic, "imu": args.imu_topic}
    if process is not None:
        try:
            import yaml
            metadata = yaml.safe_load((session / "bag/metadata.yaml").read_text())
            report["bag"] = validate_bag_metadata(metadata, {
                args.lidar_topic: "livox_ros_driver2/msg/CustomMsg", args.imu_topic: "sensor_msgs/msg/Imu"})
        except Exception as error:
            report["bag"] = {"metadata_checks_passed": False, "error": str(error)}
        if not report["bag"]["metadata_checks_passed"]:
            report["failure"] = report["failure"] or "bag metadata did not confirm both raw streams"
    report["ok"] = report["completed_duration"] and report["data_checks_passed"] and report["failure"] is None
    if session is not None:
        report["session_bytes"] = directory_size(session)
        report["budget_limit"] = "polled every 0.25 s; in-flight writes/finalization may exceed the size threshold"
        write_report(session / "report.json", report)
    return report, 0 if report["ok"] else 2


def write_report(path: Path, report: dict) -> None:
    if not path.is_absolute():
        raise CaptureError("report path must be absolute")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    sub = result.add_subparsers(dest="command", required=True)
    for name in ("inspect", "record"):
        command = sub.add_parser(name)
        command.add_argument("--duration", type=positive, default=10.0 if name == "inspect" else 60.0)
        command.add_argument("--lidar-topic", type=topic_name, default="/livox/lidar")
        command.add_argument("--imu-topic", type=topic_name, default="/livox/imu")
        command.add_argument("--min-lidar-hz", type=positive, default=1.0)
        command.add_argument("--min-imu-hz", type=positive, default=10.0)
        command.add_argument("--max-silence", type=positive, default=1.0)
        command.add_argument("--max-scan-span-ms", type=positive, default=1000.0)
        command.add_argument("--report", type=Path)
        if name == "record":
            command.add_argument("--output", type=Path, required=True,
                                 help="absolute new session directory; bag is written beneath it")
            command.add_argument("--config", type=Path, required=True, help="absolute actual MID-360 JSON config")
            command.add_argument("--startup-timeout", type=positive, default=10.0)
            command.add_argument("--max-size-mib", type=positive, default=2048.0,
                                 help="polled session-size threshold, not an exact hard cap")
            command.add_argument("--min-free-mib", type=positive, default=2048.0)
    return result


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.report is not None and (not args.report.is_absolute() or args.report.exists() or args.report.is_symlink()):
            raise CaptureError("--report must be an absolute, new file")
        report, code = run_observation(args)
        if args.report is not None:
            write_report(args.report, report)
        print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
        return code
    except (CaptureError, OSError, ValueError) as error:
        print("readonly capture: " + str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
