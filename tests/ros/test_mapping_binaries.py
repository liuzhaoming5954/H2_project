"""Start real mapping binaries, without a driver, sensor samples or motion.

Run after sourcing /opt/ros/jazzy/setup.bash and install/mapping/setup.bash.
This checks startup, actual DDS subscription types and effective parameters only.
It supplies NO observations and makes NO calibration or localization claim.
"""

import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import NodeNameNonExistentError
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.msg import ParameterType


ROOT = Path(__file__).resolve().parents[2]


class MappingBinarySmoke(unittest.TestCase):
    def setUp(self):
        self.processes = []
        self.log_files = []
        self.node = None
        self.context = None
        self.executor = None
        self.temp = None
        self.old_environment = {}
        self.report = {"checks_passed": False, "exit_codes": {}, "checks": {}}
        self.addCleanup(self.cleanup)

        run_id = time.strftime("%Y%m%d-%H%M%S") + f"-{os.getpid()}"
        self.evidence = ROOT / "artifacts" / "mapping-smoke" / run_id
        self.evidence.mkdir(parents=True, exist_ok=False)
        self.temp = tempfile.TemporaryDirectory(prefix="h2-mapping-smoke-")
        self.sandbox = Path(self.temp.name)
        self.namespace = f"/mapping_binary_smoke_{os.getpid()}"
        self.domain = 80 + os.getpid() % 20  # Separate from 72/73/130..189.
        self.topics = {
            "raw_points": self.namespace + "/raw_points",
            "raw_imu": self.namespace + "/raw_imu",
            "odom": self.namespace + "/external_odom",
            "scan_cloud": self.namespace + "/deskewed_imu_cloud",
        }
        self.frames = {
            "base": "mapping_binary_smoke_body",
            "map": "mapping_binary_smoke_map",
        }
        self.database = self.sandbox / "rtabmap-smoke.db"

        for directory in ("ros-home", "ros-logs", "cache", "rtab-work"):
            (self.sandbox / directory).mkdir()
        env_updates = {
            "ROS_DOMAIN_ID": str(self.domain),
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_STATIC_PEERS": "",
            "ROS_HOME": str(self.sandbox / "ros-home"),
            "ROS_LOG_DIR": str(self.sandbox / "ros-logs"),
            "XDG_CACHE_HOME": str(self.sandbox / "cache"),
            "MPLCONFIGDIR": str(self.sandbox / "cache" / "matplotlib"),
            "QT_QPA_PLATFORM": "offscreen",
        }
        for key, value in env_updates.items():
            self.old_environment[key] = os.environ.get(key)
            os.environ[key] = value
        self.environment = os.environ.copy()
        self.report.update({
            "domain": self.domain,
            "namespace": self.namespace,
            "topics": self.topics,
            "frames": self.frames,
            "temporary_database": str(self.database),
            "scope": "binary startup, endpoint types and parameters; no sensor data",
            "limitations": [
                "No Livox driver, H2 SDK writer or fabricated sensor data is started.",
                "Identity extrinsics below are constructor inputs, not H2 calibration.",
                "FAST-LIO native camera_init/body frames still require project adaptation.",
                "No LIO accuracy, deskewing, TF correctness or map quality is demonstrated.",
            ],
        })

        # The pinned FAST-LIO constructor dereferences these arrays immediately.
        # Identity is safe for an observation-free startup test, not calibration.
        fastlio_params = {
            "common.lid_topic": self.topics["raw_points"],
            "common.imu_topic": self.topics["raw_imu"],
            "common.time_sync_en": False,
            "common.time_offset_lidar_to_imu": 0.0,
            "preprocess.lidar_type": 1,
            "preprocess.scan_line": 4,
            "preprocess.timestamp_unit": 3,
            "preprocess.scan_rate": 10,
            "feature_extract_enable": False,
            "point_filter_num": 3,
            "cube_side_length": 1000.0,
            "mapping.det_range": 100.0,
            "mapping.extrinsic_est_en": False,
            "mapping.extrinsic_T": [0.0, 0.0, 0.0],
            "mapping.extrinsic_R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "runtime_pos_log_enable": False,
            "pcd_save.pcd_save_en": False,
            "publish.path_en": False,
            "publish.map_en": False,
            "publish.effect_map_en": False,
            "publish.scan_bodyframe_pub_en": True,
            "map_file_path": str(self.sandbox / "unused-map.pcd"),
        }
        rtabmap_params = {
            "frame_id": self.frames["base"],
            "map_frame_id": self.frames["map"],
            "odom_frame_id": "",  # Empty means consume external odom topic.
            "publish_tf": False,
            "subscribe_rgb": False,
            "subscribe_depth": False,
            "subscribe_rgbd": False,
            "subscribe_stereo": False,
            "subscribe_scan": False,
            "subscribe_scan_cloud": True,
            "subscribe_odom": True,
            "subscribe_odom_info": False,
            "database_path": str(self.database),
            "approx_sync": True,
            "wait_for_transform": 0.05,
            # RTAB-Map core parameters are STRINGS, ROS wrapper parameters above
            # retain their native types. In particular, integer 1 would be wrong.
            "Reg/Strategy": "1",
            "Reg/Force3DoF": "false",
            "Optimizer/Slam2D": "false",
            "Mem/IncrementalMemory": "true",
            "RGBD/CreateOccupancyGrid": "false",
            "Rtabmap/DetectionRate": "1",
            "Rtabmap/WorkingDirectory": str(self.sandbox / "rtab-work"),
        }
        fastlio_file = self.write_parameters("fastlio", fastlio_params)
        rtabmap_file = self.write_parameters("rtabmap", rtabmap_params)

        self.start_binary(
            "fast_lio", "fastlio_mapping", "fastlio", fastlio_file,
            {
                "/Odometry": self.topics["odom"],
                "/cloud_registered_body": self.topics["scan_cloud"],
                "/cloud_registered": self.namespace + "/world_cloud",
                "/cloud_effected": self.namespace + "/effect_cloud",
                "/Laser_map": self.namespace + "/lio_map",
                "/path": self.namespace + "/lio_path",
                "/tf": self.namespace + "/lio_tf",
                "/tf_static": self.namespace + "/lio_tf_static",
            },
        )
        self.start_binary(
            "rtabmap_slam", "rtabmap", "rtabmap", rtabmap_file,
            {
                "odom": self.topics["odom"],
                "scan_cloud": self.topics["scan_cloud"],
                "/tf": self.namespace + "/backend_tf",
                "/tf_static": self.namespace + "/backend_tf_static",
            },
        )
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node(
            "mapping_binary_observer", namespace=self.namespace,
            context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        print(f"Mapping binary smoke evidence: {self.evidence}", flush=True)

    def write_parameters(self, name, parameters):
        path = self.sandbox / f"{name}.yaml"
        # JSON scalars/arrays are valid YAML while preserving strings like '1'.
        lines = ["/**:", "  ros__parameters:"]
        lines.extend(f"    {json.dumps(key)}: {json.dumps(value)}"
                     for key, value in parameters.items())
        path.write_text("\n".join(lines) + "\n")
        shutil.copy2(path, self.evidence / path.name)
        return path

    def start_binary(self, package, executable, name, parameter_file, remaps):
        binary = Path(get_package_prefix(package)) / "lib" / package / executable
        self.assertTrue(binary.is_file(),
                        f"Missing {binary}; build/source install/mapping/setup.bash first")
        self.assertTrue(os.access(binary, os.X_OK), f"Not executable: {binary}")
        command = [str(binary), "--ros-args", "--params-file", str(parameter_file),
                   "-r", f"__node:={name}", "-r", f"__ns:={self.namespace}"]
        for source, destination in remaps.items():
            command.extend(["-r", f"{source}:={destination}"])
        log_path = self.sandbox / f"{name}.log"
        log = log_path.open("w")
        self.log_files.append((name, log_path, log))
        process = subprocess.Popen(
            command, cwd=self.sandbox, env=self.environment,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append((name, process))
        self.report.setdefault("commands", {})[name] = command

    def assert_alive(self):
        for name, process in self.processes:
            code = process.poll()
            if code is not None:
                self.fail(f"{name} exited before completion with {code}; logs: {self.evidence}")

    def wait_for(self, predicate, explanation, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assert_alive()
            self.executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        self.fail(f"Timed out: {explanation}; logs: {self.evidence}")

    def subscriptions(self, name):
        try:
            return dict(self.node.get_subscriber_names_and_types_by_node(name, self.namespace))
        except NodeNameNonExistentError:
            return {}  # Discovery is asynchronous; wait_for also checks liveness.

    def effective_parameters(self, name, expected):
        client = AsyncParameterClient(self.node, f"{self.namespace}/{name}")
        self.wait_for(lambda: client.wait_for_services(timeout_sec=0.05),
                      f"{name} parameter services")
        keys = list(expected)
        future = client.get_parameters(keys)
        self.wait_for(future.done, f"{name} get_parameters reply")
        self.assertIsNone(future.exception(), f"{name} parameter request failed")
        result = future.result()
        self.assertIsNotNone(result)
        self.assertEqual(len(result.values), len(keys))
        actual = {}
        for key, value in zip(keys, result.values):
            wanted = expected[key]
            if isinstance(wanted, bool):
                self.assertEqual(value.type, ParameterType.PARAMETER_BOOL, key)
                actual[key] = value.bool_value
            elif isinstance(wanted, int):
                self.assertEqual(value.type, ParameterType.PARAMETER_INTEGER, key)
                actual[key] = value.integer_value
            elif isinstance(wanted, float):
                self.assertEqual(value.type, ParameterType.PARAMETER_DOUBLE, key)
                actual[key] = value.double_value
            elif isinstance(wanted, str):
                self.assertEqual(value.type, ParameterType.PARAMETER_STRING, key)
                actual[key] = value.string_value
            else:
                self.fail(f"Unsupported assertion type for {key}")
            self.assertEqual(actual[key], wanted, f"{name}: {key}")
        self.report["checks"][f"{name}_effective_parameters"] = actual
        # AsyncParameterClient has no destroy() in Jazzy; the observer node
        # owns these underlying clients and destroys them during cleanup.

    def test_real_binaries_start_with_lidar_and_external_odometry_contract(self):
        wanted_lio = {
            self.topics["raw_points"]: "livox_ros_driver2/msg/CustomMsg",
            self.topics["raw_imu"]: "sensor_msgs/msg/Imu",
        }
        wanted_backend = {
            self.topics["scan_cloud"]: "sensor_msgs/msg/PointCloud2",
            self.topics["odom"]: "nav_msgs/msg/Odometry",
        }
        for name, wanted in (("fastlio", wanted_lio), ("rtabmap", wanted_backend)):
            self.wait_for(
                lambda name=name, wanted=wanted: all(
                    message_type in self.subscriptions(name).get(topic, [])
                    for topic, message_type in wanted.items()),
                f"{name} actual subscription types")
            subscriptions = self.subscriptions(name)
            self.report["checks"][f"{name}_subscriptions"] = subscriptions
            self.assertFalse(any(
                kind in {"sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo",
                         "rtabmap_msgs/msg/RGBDImage", "rtabmap_msgs/msg/OdomInfo"}
                for kinds in subscriptions.values() for kind in kinds),
                f"{name} unexpectedly subscribed to visual/odometry-info input")

        self.effective_parameters("fastlio", {
            "preprocess.lidar_type": 1,
            "cube_side_length": 1000.0,
            "mapping.det_range": 100.0,
            "common.lid_topic": self.topics["raw_points"],
            "common.imu_topic": self.topics["raw_imu"],
            "runtime_pos_log_enable": False,
            "pcd_save.pcd_save_en": False,
        })
        self.effective_parameters("rtabmap", {
            "publish_tf": False,
            "subscribe_scan_cloud": True,
            "subscribe_odom": True,
            "subscribe_odom_info": False,
            "subscribe_rgb": False,
            "subscribe_depth": False,
            "subscribe_rgbd": False,
            "subscribe_stereo": False,
            "odom_frame_id": "",
            "frame_id": self.frames["base"],
            "map_frame_id": self.frames["map"],
            "database_path": str(self.database),
            "Reg/Strategy": "1",
        })
        self.assertEqual(self.node.count_publishers(self.topics["raw_points"]), 0,
                         "A point-cloud driver is active in this supposedly empty graph")
        self.assertEqual(self.node.count_publishers(self.topics["raw_imu"]), 0,
                         "An IMU source is active in this supposedly empty graph")
        for topic in ("/motion/command", "/motion/safe_cmd", "/motion/permit"):
            self.assertEqual(self.node.count_publishers(topic), 0,
                             f"Unexpected motion publisher: {topic}")

        # Startup discovery alone is not enough: both real binaries must remain
        # alive after initialization, even though no observation ever arrives.
        until = time.monotonic() + 3.0
        while time.monotonic() < until:
            self.assert_alive()
            self.executor.spin_once(timeout_sec=0.05)
        self.report["checks"]["no_sensor_or_motion_publishers"] = True
        self.report["checks"]["post_initialization_alive_seconds"] = 3.0
        self.report["checks_passed"] = True

    def cleanup(self):
        errors = []
        if self.executor is not None:
            try:
                if self.node is not None:
                    self.executor.remove_node(self.node)
                self.executor.shutdown(timeout_sec=1.0)
            except Exception as error:
                errors.append(f"observer executor shutdown failed: {error}")
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception as error:
                errors.append(f"observer destruction failed: {error}")
        if self.context is not None and self.context.ok():
            try:
                self.context.shutdown()
            except Exception as error:
                errors.append(f"ROS context shutdown failed: {error}")
        for name, process in reversed(self.processes):
            code = process.poll()
            if code is not None:
                errors.append(f"{name} exited prematurely with {code}")
            else:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass  # A concurrent crash is still caught by its exit code.
                try:
                    code = process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        code = process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        code = process.wait(timeout=2)
                    errors.append(f"{name} did not stop on SIGINT; forced cleanup")
                if code != 0:
                    errors.append(f"{name} shutdown returned {code}")
            self.report["exit_codes"][name] = code
        for name, path, log in self.log_files:
            log.close()
            if path.exists() and hasattr(self, "evidence"):
                shutil.copy2(path, self.evidence / f"{name}.log")
        if self.temp is not None and hasattr(self, "evidence"):
            ros_logs = self.sandbox / "ros-logs"
            if ros_logs.exists():
                shutil.copytree(ros_logs, self.evidence / "ros-logs", dirs_exist_ok=True)
        if hasattr(self, "evidence"):
            self.report["cleanup_errors"] = errors
            self.report["checks_passed"] = self.report["checks_passed"] and not errors
            (self.evidence / "report.json").write_text(
                json.dumps(self.report, indent=2, sort_keys=True) + "\n")
        for key, old in self.old_environment.items():
            if old is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = old
        if self.temp is not None:
            self.temp.cleanup()
        if errors:
            self.fail("; ".join(errors) + f"; logs: {getattr(self, 'evidence', '')}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
