#include <h2_interfaces/msg/motion_command.hpp>
#include <h2_interfaces/msg/motion_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

class MockPlatform final : public rclcpp::Node {
 public:
  MockPlatform() : Node("mock_platform") {
    odom_frame_ = declare_parameter("odom_frame", "odom");
    footprint_frame_ = declare_parameter("base_frame", "base_footprint");
    body_frame_ = declare_parameter("body_frame", "base_link");
    source_ = declare_parameter("command_source", "navigation");
    body_height_ = declare_parameter("body_height_m", 0.9);
    const auto rate = declare_parameter("update_rate_hz", 50.0);
    max_validity_s_ = declare_parameter("max_command_validity_s", 0.5);
    status_timeout_s_ = declare_parameter("status_timeout_s", 0.15);
    max_forward_mps_ = declare_parameter("max_forward_mps", 0.2);
    max_yaw_rps_ = declare_parameter("max_yaw_rps", 0.3);
    if (odom_frame_.empty() || footprint_frame_.empty() || body_frame_.empty() ||
        source_.empty() || !std::isfinite(rate) || rate < 1.0 || rate > 1000.0 ||
        !std::isfinite(body_height_) || body_height_ < 0.0 ||
        !std::isfinite(max_validity_s_) || max_validity_s_ <= 0.0 || max_validity_s_ > 1.0 ||
        !std::isfinite(status_timeout_s_) || status_timeout_s_ <= 0.0 || status_timeout_s_ > 1.0 ||
        !std::isfinite(max_forward_mps_) || max_forward_mps_ <= 0.0 ||
        !std::isfinite(max_yaw_rps_) || max_yaw_rps_ < 0.0) {
      throw std::invalid_argument("invalid mock platform parameters");
    }
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/state/odom", 10);
    body_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/state/odom_body_3d", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    command_sub_ = create_subscription<h2_interfaces::msg::MotionCommand>(
        "/motion/safe_cmd", rclcpp::QoS(1).reliable(),
        [this](h2_interfaces::msg::MotionCommand::ConstSharedPtr msg) { command(*msg); });
    status_sub_ = create_subscription<h2_interfaces::msg::MotionStatus>(
        "/motion/status", rclcpp::QoS(10).reliable(),
        [this](h2_interfaces::msg::MotionStatus::ConstSharedPtr msg) {
          if (msg->header.frame_id != footprint_frame_ || msg->header.stamp.sec < 0 ||
              msg->header.stamp.nanosec >= 1000000000U) {
            clear();
            status_valid_ = false;
            return;
          }
          const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (stamp <= last_status_stamp_) {
            return;  // Old status must not roll the authorized session backward.
          }
          const auto age = (get_clock()->now() - rclcpp::Time(msg->header.stamp)).seconds();
          if (!std::isfinite(age) || age < -0.02 || age > status_timeout_s_) {
            clear();
            status_valid_ = false;
            return;
          }
          last_status_stamp_ = stamp;
          if (session_ != msg->session_id) {
            clear();
            session_ = msg->session_id;
            sequence_ = 0;
            have_sequence_ = false;
          }
          status_valid_ = msg->enabled && msg->command_valid && !session_.empty();
          authorized_sequence_ = msg->command_sequence;
          last_status_ = Clock::now();
          if (!status_valid_) {
            clear();
          }
        });
    last_update_ros_ = get_clock()->now();
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / rate)),
        [this]() { update(); });
    RCLCPP_WARN(get_logger(), "MOCK platform: ideal kinematics only; no Unitree SDK is loaded");
  }

 private:
  void clear() { vx_ = 0.0; wz_ = 0.0; deadline_ = Clock::time_point::min(); }

  void command(const h2_interfaces::msg::MotionCommand& msg) {
    // A stop is always conservative and does not renew any command lease.
    if (msg.validity.sec == 0 && msg.validity.nanosec == 0) {
      clear();
      return;
    }
    if (!status_valid_ || msg.session_id != session_ || msg.source != source_ ||
        msg.header.frame_id != footprint_frame_ ||
        msg.sequence != authorized_sequence_ ||
        (have_sequence_ && msg.sequence <= sequence_)) {
      return;
    }
    const auto& v = msg.velocity;
    if (!std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) || !std::isfinite(v.linear.z) ||
        !std::isfinite(v.angular.x) || !std::isfinite(v.angular.y) || !std::isfinite(v.angular.z) ||
        v.linear.x < 0.0 || v.linear.y != 0.0 || v.linear.z != 0.0 ||
        v.angular.x != 0.0 || v.angular.y != 0.0 ||
        msg.validity.sec < 0 || msg.validity.nanosec >= 1000000000U ||
        msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000U) {
      clear();
      return;
    }
    const double validity = static_cast<double>(msg.validity.sec) + msg.validity.nanosec * 1e-9;
    const double age = (get_clock()->now() - rclcpp::Time(msg.header.stamp)).seconds();
    if (validity <= 0.0 || validity > max_validity_s_ || age < -0.02 || age >= validity) {
      clear();
      return;
    }
    sequence_ = msg.sequence;
    have_sequence_ = true;
    vx_ = std::min(v.linear.x, max_forward_mps_);
    wz_ = std::clamp(v.angular.z, -max_yaw_rps_, max_yaw_rps_);
    deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(validity - std::max(0.0, age)));
  }

  void update() {
    const auto steady = Clock::now();
    if (!status_valid_ || steady >= deadline_ ||
        std::chrono::duration<double>(steady - last_status_).count() > status_timeout_s_) {
      clear();
    }
    const auto now = get_clock()->now();
    const double elapsed = (now - last_update_ros_).seconds();
    last_update_ros_ = now;
    if (elapsed < 0.0 || elapsed > 0.25) {
      clear();  // Time resets and scheduling gaps do not create teleportation.
    } else {
      const double heading_mid = yaw_ + 0.5 * wz_ * elapsed;
      x_ += vx_ * std::cos(heading_mid) * elapsed;
      y_ += vx_ * std::sin(heading_mid) * elapsed;
      yaw_ = std::atan2(std::sin(yaw_ + wz_ * elapsed), std::cos(yaw_ + wz_ * elapsed));
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = footprint_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(0.5 * yaw_);
    odom.pose.pose.orientation.w = std::cos(0.5 * yaw_);
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.angular.z = wz_;
    // This is an explicitly ideal mock state, not an accuracy claim for LIO.
    odom_pub_->publish(odom);
    auto body = odom;
    body.child_frame_id = body_frame_;
    body.pose.pose.position.z = body_height_;
    body_odom_pub_->publish(body);

    geometry_msgs::msg::TransformStamped footprint;
    footprint.header = odom.header;
    footprint.child_frame_id = footprint_frame_;
    footprint.transform.translation.x = x_;
    footprint.transform.translation.y = y_;
    footprint.transform.rotation = odom.pose.pose.orientation;
    geometry_msgs::msg::TransformStamped base;
    base.header.stamp = now;
    base.header.frame_id = footprint_frame_;
    base.child_frame_id = body_frame_;
    base.transform.translation.z = body_height_;
    base.transform.rotation.w = 1.0;
    tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{footprint, base});
  }

  std::string odom_frame_, footprint_frame_, body_frame_, source_, session_;
  double body_height_{0.9}, max_validity_s_{0.5}, status_timeout_s_{0.15};
  double max_forward_mps_{0.2}, max_yaw_rps_{0.3};
  double x_{0.0}, y_{0.0}, yaw_{0.0}, vx_{0.0}, wz_{0.0};
  std::uint64_t sequence_{0};
  std::uint64_t authorized_sequence_{0};
  std::int64_t last_status_stamp_{-1};
  bool have_sequence_{false}, status_valid_{false};
  Clock::time_point deadline_{Clock::time_point::min()}, last_status_{Clock::time_point::min()};
  rclcpp::Time last_update_ros_{0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, body_odom_pub_;
  rclcpp::Subscription<h2_interfaces::msg::MotionCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<h2_interfaces::msg::MotionStatus>::SharedPtr status_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MockPlatform>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("mock_platform"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
