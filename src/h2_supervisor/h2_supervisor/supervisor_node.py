import math
import time

import rclpy
from rclpy.node import Node
from h2_interfaces.msg import MotionPermit, MotionStatus
from nav_msgs.msg import Odometry

from .health import HealthMonitor, decode_stamp


def stamp_seconds(stamp):
    return decode_stamp(stamp.sec, stamp.nanosec)


class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")
        self.declare_parameter("mode", "sim")
        self.declare_parameter("max_observation_age_s", 0.3)
        self.declare_parameter("permit_validity_s", 0.25)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.monitor = HealthMonitor(
            mode=self.get_parameter("mode").value,
            max_age=self.get_parameter("max_observation_age_s").value,
        )
        self.validity = float(self.get_parameter("permit_validity_s").value)
        rate = float(self.get_parameter("rate_hz").value)
        if not (math.isfinite(rate) and rate > 0
                and math.isfinite(self.validity) and 0 < self.validity <= 0.5):
            raise ValueError("invalid supervisor rate or permit validity")
        self.sequence = 0
        self.session = ""
        self.permission_was_granted = False
        self.last_decision = None
        self.publisher = self.create_publisher(MotionPermit, "/motion/permit", 10)
        self.create_subscription(Odometry, "/state/odom", self.on_odom, 10)
        self.create_subscription(MotionStatus, "/motion/status", self.on_guard, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().warning(
            "Simulation health policy only; no hardware motion permission is implemented")

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        twist = msg.twist.twist
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        valid = (all(math.isfinite(v) for v in values) and abs(norm - 1.0) < 0.05
                 and msg.header.frame_id == self.get_parameter("odom_frame").value
                 and msg.child_frame_id == self.get_parameter("base_frame").value)
        self.monitor.observe_odom(received=time.monotonic(),
                                  stamp=stamp_seconds(msg.header.stamp), valid=valid)

    def on_guard(self, msg):
        frame_valid = msg.header.frame_id == self.get_parameter("base_frame").value
        self.monitor.observe_guard(
            received=time.monotonic(), stamp=stamp_seconds(msg.header.stamp),
            session_id=msg.session_id if frame_valid else "",
            enabled=msg.enabled if frame_valid else False,
            reason=msg.reason if frame_valid else "invalid_guard_frame")

    def tick(self):
        decision = self.monitor.evaluate(
            steady_now=time.monotonic(), ros_now=self.get_clock().now().nanoseconds*1e-9)
        if not decision.session_id:
            return
        if self.session != decision.session_id:
            self.session = decision.session_id
            self.sequence = 0
            self.permission_was_granted = False
        key = (self.session, decision.allowed, decision.reason)
        if key != self.last_decision:
            self.get_logger().info(f"permission allowed={decision.allowed}: {decision.reason}")
            self.last_decision = key
        if not self.monitor.guard_requests_permission:
            return  # An already stopped guard must retain its original fault reason.
        if not decision.allowed and not self.permission_was_granted:
            # Initial discovery/odometry may not be ready when enable is requested.
            # Withhold the FIRST lease; the guard's pending timeout remains active.
            # Sending a revocation here would latch an otherwise valid startup.
            return
        if decision.allowed:
            self.permission_was_granted = True
        self.sequence += 1
        msg = MotionPermit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("base_frame").value
        msg.source = "supervisor"
        msg.session_id = self.session
        msg.sequence = self.sequence
        msg.enabled = decision.allowed
        nanos = int(self.validity*1e9)
        msg.validity.sec, msg.validity.nanosec = divmod(nanos, 1_000_000_000)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Supervisor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
