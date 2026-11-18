"""Run the project's isolated MID-360S FAST-LIO replay and deliver native output messages."""

from contextlib import ExitStack, contextmanager
import fcntl
from functools import partial
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml

from readonly_capture import stamp_ns, validate_bag_metadata


ROOT = Path(__file__).resolve().parents[1]
NAMESPACE = "/h2_lio_replay"
TYPES = {"/livox/lidar": "livox_ros_driver2/msg/CustomMsg", "/livox/imu": "sensor_msgs/msg/Imu"}
INPUTS = {"/livox/lidar": NAMESPACE + "/raw_lidar", "/livox/imu": NAMESPACE + "/raw_imu"}
NOMINAL_HZ = {"/livox/lidar": 10.0, "/livox/imu": 200.0}
CONFIG = ROOT / "src/h2_bringup/config/fastlio_mid360s_nominal.yaml"
NS_PER_S = 1_000_000_000


def _bag_info(bag):
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())
    checked = validate_bag_metadata(metadata, TYPES)
    if not checked["metadata_checks_passed"]:
        raise ValueError("Expected a completed bag with both project raw topics and their recorded types")
    info = metadata["rosbag2_bagfile_information"]
    if any(item["topic_metadata"].get("serialization_format") != "cdr"
           for item in info["topics_with_message_count"]):
        raise ValueError("Expected recorded CDR topics")
    paths = info.get("relative_file_paths")
    if info.get("storage_identifier") != "sqlite3" or not isinstance(paths, list) or not paths:
        raise ValueError("Expected SQLite bag with a nonempty relative_file_paths list")
    for name in paths:
        if (not isinstance(name, str) or Path(name).name != name or not name.endswith(".db3")
                or not (bag / name).is_file() or (bag / name).stat().st_size == 0):
            raise ValueError(f"Missing or invalid bag database: {name!r}")
    if len(set(paths)) != len(paths):
        raise ValueError("Duplicate bag file path")
    duration_ns = info.get("duration", {}).get("nanoseconds")
    expected = {topic: checked["topics"][topic]["count"] for topic in TYPES}
    if type(duration_ns) is not int or duration_ns <= 0 or info.get("message_count") != sum(expected.values()):
        raise ValueError("Invalid bag duration or inconsistent metadata message totals")
    return duration_ns / NS_PER_S, expected


@contextmanager
def _local_environment(values):
    previous = {key: os.environ.get(key) for key in values}
    try:
        for key, value in values.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        yield
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def _source_timing(frequency):
    return {"first_ns": None, "last_ns": None, "mean_hz": None, "max_gap_s": None,
            "nonincreasing_count": 0, "gap_threshold_s": 2.5 / frequency,
            "gaps_over_threshold": 0, "nominal_hz": frequency}


def _observe_source_time(series, stamp, count, topic):
    if series["last_ns"] is None:
        series["first_ns"] = series["last_ns"] = stamp
        return
    gap = (stamp - series["last_ns"]) / NS_PER_S
    if gap <= 0:
        series["nonincreasing_count"] += 1
        raise RuntimeError(f"Non-increasing raw source timestamp on {topic}: {series['last_ns']} -> {stamp}")
    series["last_ns"] = stamp
    series["max_gap_s"] = max(series["max_gap_s"] or 0.0, gap)
    if gap > series["gap_threshold_s"]:
        series["gaps_over_threshold"] += 1
    series["mean_hz"] = (count - 1) * NS_PER_S / (stamp - series["first_ns"])


def _stop(process, label, report):
    if process.poll() is None:
        # Launch forwards this signal to FAST-LIO. Signalling the whole group here
        # would interrupt the child twice; only use the group for fallback cleanup.
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            report[label + "_forced_stop"] = True
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
    report[label + "_exit_code"] = process.returncode


def _run(bag, output, on_odom, on_cloud, domain, rate, report, stack):
    duration, expected = _bag_info(bag)
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.serialization import deserialize_message
        from livox_ros_driver2.msg import CustomMsg
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2
        from std_msgs.msg import Header
    except ImportError as error:
        raise RuntimeError("Source ROS Jazzy, install/mapping/local_setup.bash and install/ros/local_setup.bash") from error

    counts = dict.fromkeys(TYPES, 0)
    source_timing = {topic: _source_timing(hz) for topic, hz in NOMINAL_HZ.items()}
    outputs = {"odometry": 0, "world_cloud": 0}
    timing = {key: {"first_ns": None, "last_ns": None, "max_gap_s": 0.0} for key in outputs}
    digests = {key: hashlib.sha256() for key in outputs}
    report.update(bag_duration_s=duration, expected_inputs=expected, input_counts=counts, source_timing=source_timing,
                  output_counts=outputs, output_timing=timing)
    # Concurrent runs in different output directories still share the launch's fixed ROS domain.
    uid = os.getuid()
    lock = stack.enter_context(open(f"/tmp/h2-lio-replay-{uid}-{domain}.lock", "a"))
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        raise RuntimeError("Another offline replay already owns domain 167") from error

    env = {"ROS_DOMAIN_ID": str(domain), "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
           "ROS_LOCALHOST_ONLY": None, "ROS_STATIC_PEERS": "", "ROS_LOG_DIR": str(output / "ros-logs"),
           "ROS_HOME": str(output / "ros-home"), "OMP_NUM_THREADS": "2"}
    stack.enter_context(_local_environment(env))
    context = Context()
    context.init(args=[], domain_id=domain)
    stack.callback(context.try_shutdown)
    node = rclpy.create_node(f"field_replay_observer_{os.getpid()}", namespace=NAMESPACE, context=context)
    stack.callback(node.destroy_node)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    stack.callback(executor.shutdown)

    # Accept neither prerecorded TF nor an existing player/estimator in this namespace.
    discovery_until = time.monotonic() + 1.5
    while time.monotonic() < discovery_until:
        executor.spin_once(timeout_sec=0.05)
    if (any(ns == NAMESPACE and name != node.get_name() for name, ns in node.get_node_names_and_namespaces())
            or any(node.count_publishers(topic) for topic in INPUTS.values())):
        raise RuntimeError("Existing replay participants in domain 167; stop the previous replay first")

    def raw_input(topic, raw):
        counts[topic] += 1
        # Header is the first CDR field in both pinned message definitions.
        # Decode just that prefix, avoiding allocation of every CustomPoint.
        header = deserialize_message(raw, Header)
        _observe_source_time(source_timing[topic], stamp_ns(header.stamp), counts[topic], topic)

    def receive(key, callback, msg):
        if msg.header.frame_id != "camera_init" or (key == "odometry" and msg.child_frame_id != "body"):
            raise RuntimeError(f"Unexpected native {key} frame")
        stamp = stamp_ns(msg.header.stamp)
        series = timing[key]
        if series["last_ns"] is None:
            series["first_ns"] = stamp
        else:
            gap = (stamp - series["last_ns"]) / NS_PER_S
            if gap <= 0:
                raise RuntimeError(f"Non-increasing {key} timestamp: {stamp}")
            series["max_gap_s"] = max(series["max_gap_s"], gap)
        series["last_ns"] = stamp
        digests[key].update(stamp.to_bytes(8, "little", signed=True))
        outputs[key] += 1
        callback(msg)

    node.create_subscription(CustomMsg, INPUTS["/livox/lidar"], partial(raw_input, "/livox/lidar"), 64, raw=True)
    node.create_subscription(Imu, INPUTS["/livox/imu"], partial(raw_input, "/livox/imu"), 512, raw=True)
    node.create_subscription(Odometry, NAMESPACE + "/odometry",
                             lambda msg: receive("odometry", on_odom, msg), 128)
    node.create_subscription(PointCloud2, NAMESPACE + "/world_cloud",
                             lambda msg: receive("world_cloud", on_cloud, msg), 64)

    config = output / "parameters.yaml"
    config.write_bytes(CONFIG.read_bytes())
    report["configuration_sha256"] = hashlib.sha256(config.read_bytes()).hexdigest()

    def launch(label, command):
        report.setdefault("commands", {})[label] = command
        log = stack.enter_context((output / (label + ".log")).open("x"))
        process = subprocess.Popen(command, cwd=output, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        stack.callback(_stop, process, label, report)
        return process

    fast = launch("fastlio", ["ros2", "launch", "h2_bringup", "lio_static_replay.launch.py",
                              "params_file:=" + str(config)])

    def spin():
        executor.spin_once(timeout_sec=0.02)
        if fast.poll() is not None:
            raise RuntimeError(f"FAST-LIO launch exited early ({fast.returncode}); inspect fastlio.log")

    def connected(topic):
        return any(ep.node_name == "fastlio_static" and ep.node_namespace == NAMESPACE
                   for ep in node.get_subscriptions_info_by_topic(topic))

    deadline = time.monotonic() + 30
    while not (all(connected(topic) for topic in INPUTS.values()) and
               all(node.count_publishers(NAMESPACE + "/" + key) == 1 for key in outputs)):
        spin()
        if time.monotonic() > deadline:
            raise RuntimeError("FAST-LIO input/output endpoints not ready; inspect fastlio.log")

    play = launch("player", ["ros2", "bag", "play", str(bag), "--topics", *TYPES,
                             "--rate", str(rate), "--delay", "2", "--read-ahead-queue-size", "512",
                             "--wait-for-all-acked", "1000", "--disable-keyboard-controls",
                             "--remap", *[key + ":=" + value for key, value in INPUTS.items()]])
    started = time.monotonic()
    deadline = started + 45 + 1.5 * duration / rate
    progress_at = started + 15
    while play.poll() is None:
        spin()
        now = time.monotonic()
        if now > deadline:
            raise RuntimeError("Bag playback timed out; inspect player.log")
        if now >= progress_at:
            if not all(connected(topic) for topic in INPUTS.values()):
                raise RuntimeError("FAST-LIO disappeared during playback; inspect fastlio.log")
            print(f"回放中：已接收点云 {counts['/livox/lidar']}/{expected['/livox/lidar']}，"
                  f"已收到配准点云 {outputs['world_cloud']} 帧", flush=True)
            progress_at = now + 15
    if play.returncode:
        raise RuntimeError(f"Bag player exited with code {play.returncode}; inspect player.log")

    # Continue spinning after EOF: the node's wall timer processes scans independently
    # of playback. Wait for silence on both streams, not just player termination.
    deadline = time.monotonic() + 60
    quiet_since = time.monotonic()
    last_progress = None
    while time.monotonic() < deadline:
        spin()
        progress = (*counts.values(), *outputs.values())
        if progress != last_progress:
            quiet_since = time.monotonic()
            last_progress = progress
        if time.monotonic() - quiet_since >= 2:
            break
    else:
        raise RuntimeError("FAST-LIO output did not drain after playback")
    if counts != expected:
        raise RuntimeError(f"Replayed input counts differ from bag: observed {counts}, expected {expected}")
    if outputs["odometry"] < 2 or digests["odometry"].digest() != digests["world_cloud"].digest():
        raise RuntimeError(f"Missing or unmatched odometry/world-cloud timestamps: {outputs}")

    lidar_timing = source_timing["/livox/lidar"]
    edges = {"first": lidar_timing["first_ns"], "last": lidar_timing["last_ns"]}
    report["lidar_header_edges_ns"] = edges
    odom = timing["odometry"]
    report["first_output_minus_first_scan_s"] = (odom["first_ns"] - edges["first"]) / NS_PER_S
    report["last_output_minus_last_scan_s"] = (odom["last_ns"] - edges["last"]) / NS_PER_S
    # The pinned pipeline uses 10 Hz lidar and needs several scans to initialize.
    # Timestamp edges use scan START vs odometry at scan END, and allow for that offset.
    report["coverage_warning"] = max(abs(report["first_output_minus_first_scan_s"]),
                                     abs(report["last_output_minus_last_scan_s"]),
                                     odom["max_gap_s"]) > 1.0


def run_replay(bag: Path, output: Path, on_odom, on_cloud, *, domain: int, rate: float) -> dict:
    """Play a finalized field bag, preserving native frames/stamps in the two callbacks.

    The caller reserves a new output directory. On failure diagnostics/logs are retained
    and an exception is raised; no successfully completed report is returned.
    """
    bag, output = Path(bag).resolve(), Path(output).resolve(strict=True)
    if domain != 167 or not math.isfinite(rate) or rate <= 0:
        raise ValueError("Replay requires fixed domain 167 and a finite positive playback rate")
    report = {"ok": False, "failure": None, "input_bag": str(bag), "playback_rate": rate,
              "domain": domain, "namespace": NAMESPACE}
    try:
        with ExitStack() as stack:
            _run(bag, output, on_odom, on_cloud, domain, rate, report, stack)
        if report["fastlio_exit_code"] != 0 or report.get("fastlio_forced_stop"):
            raise RuntimeError("FAST-LIO did not shut down cleanly; inspect fastlio.log")
        report["ok"] = True
    except BaseException as error:
        report["failure"] = str(error) or type(error).__name__
        raise
    finally:
        (output / "replay-diagnostics.json").write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return report
