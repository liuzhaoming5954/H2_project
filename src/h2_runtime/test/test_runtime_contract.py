"""Exercise the actual ROS boundary and mock, without any Unitree SDK writer."""

import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

# Keep this integration test separate from any developer's running ROS graph.
os.environ["ROS_DOMAIN_ID"] = str(130 + os.getpid() % 60)
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["ROS_LOG_DIR"] = str(Path(tempfile.gettempdir()) / f"h2-runtime-test-{os.getpid()}")

from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Duration
from h2_interfaces.msg import MotionCommand, MotionPermit, MotionStatus
from h2_interfaces.srv import SetMotionEnabled
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration as RosDuration


class RuntimeContract(unittest.TestCase):
    def setUp(self):
        self.processes = []
        self.node = None
        self.logs = None
        self.addCleanup(self.cleanup_runtime)
        rclpy.init()
        self.node = rclpy.create_node("test_motion_runtime")
        self.status = None
        self.safe = None
        self.odom = None
        self.status_sub = self.node.create_subscription(
            MotionStatus, "/motion/status", lambda msg: setattr(self, "status", msg), 10
        )
        self.safe_sub = self.node.create_subscription(
            MotionCommand, "/motion/safe_cmd", lambda msg: setattr(self, "safe", msg), 10
        )
        self.odom_sub = self.node.create_subscription(
            Odometry, "/state/odom", lambda msg: setattr(self, "odom", msg), 10
        )
        self.commands = self.node.create_publisher(MotionCommand, "/motion/command", 10)
        self.permits = self.node.create_publisher(MotionPermit, "/motion/permit", 10)
        self.enable = self.node.create_client(SetMotionEnabled, "/motion/set_enabled")
        executable_dir = Path(get_package_prefix("h2_runtime")) / "lib" / "h2_runtime"
        self.logs = tempfile.TemporaryFile(mode="w+")
        for name, extra in (
            ("motion_guard", ["-p", "max_command_validity_s:=1.0", "-p", "max_permit_validity_s:=2.0"]),
            ("mock_platform", []),
        ):
            self.processes.append(
                subprocess.Popen(
                    [str(executable_dir / name), "--ros-args", *extra],
                    stdout=self.logs,
                    stderr=subprocess.STDOUT,
                    env=os.environ.copy(),
                )
            )
        self.assertTrue(self.enable.wait_for_service(timeout_sec=8.0))
        self.wait_for(lambda: self.status is not None and self.odom is not None)

    def cleanup_runtime(self):
        for process in self.processes:
            process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        if self.node is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self.logs is not None:
            self.logs.close()

    def wait_for(self, predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            for process in self.processes:
                if process.poll() is not None:
                    self.logs.seek(0)
                    self.fail(f"runtime exited: {self.logs.read()}")
            if predicate():
                return
        self.fail("timed out waiting for ROS runtime contract")

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def set_enabled(self, enabled=True):
        request = SetMotionEnabled.Request()
        request.enable = enabled
        result = self.enable.call_async(request)
        self.wait_for(result.done)
        self.assertTrue(result.result().success)
        return result.result().session_id

    def permit(self, session, sequence=1):
        msg = MotionPermit()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.source = "supervisor"
        msg.session_id = session
        msg.sequence = sequence
        msg.enabled = True
        msg.validity = Duration(sec=1, nanosec=200_000_000)
        self.permits.publish(msg)
        return msg

    def command(self, session, sequence=1):
        msg = MotionCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.source = "navigation"
        msg.session_id = session
        msg.sequence = sequence
        msg.velocity.linear.x = 1.0
        msg.velocity.angular.z = 1.0
        msg.validity = Duration(nanosec=400_000_000)
        self.commands.publish(msg)
        return msg

    def test_pending_permit_ttl_latch_and_rearm(self):
        self.assertFalse(self.status.enabled)
        self.assertEqual(self.odom.twist.twist.linear.x, 0.0)
        session = self.set_enabled()
        self.wait_for(lambda: self.status.session_id == session)
        self.assertFalse(self.status.enabled, "enable must not invent a health permit")
        self.assertEqual(self.status.reason, "waiting_permission")
        self.command(session)
        self.spin_for(0.08)
        self.assertFalse(self.status.command_valid)

        self.wait_for(lambda: self.status.reason == "pending_permission_timeout")
        self.permit(session)
        self.spin_for(0.08)
        self.assertFalse(self.status.enabled, "late first permit must not revive an expired enable")
        late_revoke = self.permit(session, 2)
        late_revoke.enabled = False
        self.permits.publish(late_revoke)
        self.spin_for(0.08)
        self.assertEqual(self.status.reason, "pending_permission_timeout")
        session = self.set_enabled()

        self.permit(session)
        self.wait_for(lambda: self.status.enabled)
        original = self.command(session)
        self.wait_for(lambda: self.status.command_valid and self.safe.validity.nanosec != 0)
        self.assertAlmostEqual(self.status.output.linear.x, 0.2)
        self.assertAlmostEqual(self.status.output.angular.z, 0.3)
        self.assertEqual(self.safe.header.stamp, original.header.stamp)
        self.assertEqual(self.safe.validity, original.validity)
        self.wait_for(lambda: self.odom.twist.twist.linear.x > 0.0)

        self.spin_for(0.20)
        self.commands.publish(original)  # Repetition must not restart the 400 ms lifetime.
        self.wait_for(lambda: not self.status.enabled and self.status.reason == "command_expired")
        self.wait_for(lambda: self.odom.twist.twist.linear.x == 0.0)
        stopped_x = self.odom.pose.pose.position.x
        self.permit(session, 2)
        self.command(session, 2)
        self.spin_for(0.08)
        self.assertFalse(self.status.enabled, "permit alone must not rearm an expired session")
        self.assertAlmostEqual(self.odom.pose.pose.position.x, stopped_x)
        late_revoke = self.permit(session, 3)
        late_revoke.enabled = False
        self.permits.publish(late_revoke)
        expired = self.command(session, 3)
        expired.sequence = 4
        expired.header.stamp = (self.node.get_clock().now() - RosDuration(seconds=5)).to_msg()
        self.commands.publish(expired)
        self.spin_for(0.08)
        self.assertEqual(self.status.reason, "command_expired",
                         "in-flight revocation/stale traffic must preserve the original stop cause")

        fresh_session = self.set_enabled()
        self.assertNotEqual(fresh_session, session)
        self.permit(fresh_session)
        self.wait_for(lambda: self.status.enabled)
        stale = self.command(fresh_session)
        stale.sequence = 2
        stale.header.stamp = (self.node.get_clock().now() - RosDuration(seconds=1)).to_msg()
        self.commands.publish(stale)
        self.wait_for(lambda: not self.status.enabled)
        self.assertEqual(self.status.reason, "invalid_command_timestamp_or_validity")
        self.wait_for(lambda: self.odom.twist.twist.linear.x == 0.0)


if __name__ == "__main__":
    unittest.main()
