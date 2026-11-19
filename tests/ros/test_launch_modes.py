"""Verify installed launch boundaries and the actual default mock graph."""

import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

import rclpy
from h2_interfaces.msg import MotionStatus
from nav_msgs.msg import Odometry


class LaunchModes(unittest.TestCase):
    def environment(self):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "73"
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        env["ROS_LOG_DIR"] = str(Path("artifacts/launch-check/ros-logs").resolve())
        return env

    def test_readonly_boundaries_do_not_fake_hardware_support(self):
        replay = subprocess.run(
            ["ros2", "launch", "h2_bringup", "system.launch.py", "mode:=replay"],
            env=self.environment(), capture_output=True, text=True, timeout=15)
        self.assertEqual(replay.returncode, 0, replay.stdout + replay.stderr)
        self.assertIn("no motion writer", replay.stdout)
        for mode in ("hardware", "invalid"):
            result = subprocess.run(
                ["ros2", "launch", "h2_bringup", "system.launch.py", f"mode:={mode}"],
                env=self.environment(), capture_output=True, text=True, timeout=15)
            self.assertNotEqual(result.returncode, 0, mode)
            if mode == "hardware":
                self.assertIn("not implemented", result.stdout + result.stderr)

    def test_readonly_requires_explicit_sensor_configuration(self):
        result = subprocess.run(
            ["ros2", "launch", "h2_bringup", "system.launch.py",
             "mode:=hardware_readonly"],
            env=self.environment(), capture_output=True, text=True, timeout=15)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("lidar_config", result.stdout + result.stderr)

    def test_sim_launch_produces_stationary_disabled_state(self):
        env = self.environment()
        os.environ.update({key: env[key] for key in (
            "ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_LOG_DIR")})
        evidence = Path("artifacts/launch-check")
        evidence.mkdir(parents=True, exist_ok=True)
        with (evidence / "sim.log").open("w") as log:
            process = subprocess.Popen(
                ["ros2", "launch", "h2_bringup", "system.launch.py",
                 "mode:=sim", "ros_domain_id:=73"],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            node = None
            try:
                rclpy.init()
                node = rclpy.create_node("launch_boundary_test")
                samples = {"status": None, "odom": None}
                node.create_subscription(MotionStatus, "/motion/status",
                                         lambda msg: samples.update(status=msg), 10)
                node.create_subscription(Odometry, "/state/odom",
                                         lambda msg: samples.update(odom=msg), 10)
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    self.assertIsNone(process.poll(), "sim launch exited unexpectedly")
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if all(value is not None for value in samples.values()):
                        break
                self.assertIsNotNone(samples["status"])
                self.assertIsNotNone(samples["odom"])
                self.assertFalse(samples["status"].enabled)
                self.assertEqual(samples["status"].output.linear.x, 0.0)
                self.assertEqual(samples["odom"].pose.pose.position.x, 0.0)
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
