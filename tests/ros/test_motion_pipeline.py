"""Real ROS process smoke test; uses only mock hardware on localhost DDS."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

import rclpy
from h2_interfaces.msg import MotionCommand, MotionStatus
from h2_interfaces.srv import SetMotionEnabled
from nav_msgs.msg import Odometry


class MotionPipelineTest(unittest.TestCase):
    def setUp(self):
        self.processes = []
        self.logs = []
        self.addCleanup(self.cleanup_runtime)
        self.status = None
        self.odom = None
        self.counter = 0
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "72"
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        os.environ.update({key: env[key] for key in (
            "ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE")})
        self.evidence = Path("artifacts/ros-smoke")
        self.evidence.mkdir(parents=True, exist_ok=True)
        env["ROS_LOG_DIR"] = str((self.evidence / "ros-logs").resolve())
        os.environ["ROS_LOG_DIR"] = env["ROS_LOG_DIR"]
        for package, executable in (
                ("h2_runtime", "motion_guard"),
                ("h2_runtime", "mock_platform"),
                ("h2_supervisor", "supervisor")):
            log = (self.evidence / f"{executable}.log").open("w")
            self.logs.append(log)
            process = subprocess.Popen(
                ["ros2", "run", package, executable], stdout=log,
                stderr=subprocess.STDOUT, env=env, start_new_session=True)
            self.processes.append(process)
        rclpy.init()
        self.node = rclpy.create_node("foundation_smoke")
        self.node.create_subscription(MotionStatus, "/motion/status", self.on_status, 10)
        self.node.create_subscription(Odometry, "/state/odom", self.on_odom, 10)
        self.publisher = self.node.create_publisher(MotionCommand, "/motion/command", 10)
        self.client = self.node.create_client(SetMotionEnabled, "/motion/set_enabled")

    def cleanup_runtime(self):
        if hasattr(self, "node"):
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in self.processes:
            self.stop_process(process)
        for log in self.logs:
            log.close()

    @staticmethod
    def stop_process(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)

    def on_status(self, msg):
        self.status = msg

    def on_odom(self, msg):
        self.odom = msg

    def wait_for(self, predicate, timeout=8, publish=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish is not None:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if predicate():
                return
        reason = self.status.reason if self.status is not None else "no status"
        self.fail(f"condition timed out; motion reason={reason}; logs={self.evidence}")

    def enable(self):
        self.assertTrue(self.client.wait_for_service(timeout_sec=10))
        self.wait_for(lambda: self.node.count_publishers("/motion/permit") > 0)
        request = SetMotionEnabled.Request()
        request.enable = True
        future = self.client.call_async(request)
        self.wait_for(future.done)
        self.assertTrue(future.result().success, future.result().message)
        session = future.result().session_id
        self.assertTrue(session)
        self.wait_for(lambda: self.status is not None
                      and self.status.session_id == session and self.status.enabled)
        return session

    def command(self, session):
        self.counter += 1
        msg = MotionCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.source = "navigation"
        msg.session_id = session
        msg.sequence = self.counter
        msg.velocity.linear.x = 0.1
        msg.validity.nanosec = 300_000_000
        self.publisher.publish(msg)
        self.last_command = msg

    def test_motion_requires_fresh_commands_and_independent_permission(self):
        self.wait_for(lambda: self.status is not None and self.odom is not None)
        self.assertFalse(self.status.enabled)
        self.assertEqual(self.status.output.linear.x, 0.0)

        session = self.enable()
        initial_x = self.odom.pose.pose.position.x
        self.wait_for(lambda: self.odom.pose.pose.position.x > initial_x + 0.035,
                      publish=lambda: self.command(session))
        self.wait_for(lambda: not self.status.enabled and not self.status.command_valid)
        self.assertEqual(self.status.output.linear.x, 0.0)
        stopped_x = self.odom.pose.pose.position.x

        # Re-publishing identical stale data cannot revive an expired session.
        old = self.last_command
        until = time.monotonic() + 0.4
        while time.monotonic() < until:
            self.publisher.publish(old)
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertFalse(self.status.enabled)
        self.assertLess(abs(self.odom.pose.pose.position.x - stopped_x), 0.02)

        second_session = self.enable()
        self.assertNotEqual(session, second_session)
        restart_x = self.odom.pose.pose.position.x
        self.wait_for(lambda: self.odom.pose.pose.position.x > restart_x + 0.025,
                      publish=lambda: self.command(second_session))
        self.stop_process(self.processes[2])  # Independent supervisor disappears.
        self.wait_for(lambda: not self.status.enabled,
                      publish=lambda: self.command(second_session))
        self.assertEqual(self.status.output.linear.x, 0.0)
        self.assertTrue(math.isfinite(self.odom.pose.pose.position.x))


if __name__ == "__main__":
    unittest.main(verbosity=2)
