#include <h2_core/motion_guard.hpp>
#include <h2_interfaces/msg/motion_command.hpp>
#include <h2_interfaces/msg/motion_permit.hpp>
#include <h2_interfaces/msg/motion_status.hpp>
#include <h2_interfaces/srv/set_motion_enabled.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
using h2::core::Time;

Time monotonic_now() {
  return std::chrono::duration_cast<Time>(
      std::chrono::steady_clock::now().time_since_epoch());
}

Time seconds(double value) {
  if (!std::isfinite(value) || value <= 0.0 || value > 60.0) {
    throw std::invalid_argument("duration parameters must be finite in (0, 60]");
  }
  return std::chrono::duration_cast<Time>(std::chrono::duration<double>(value));
}

std::string new_session() {
  std::random_device random;
  std::ostringstream out;
  out << "session-" << std::hex << std::setfill('0');
  for (int index = 0; index < 4; ++index) {
    out << std::setw(8) << static_cast<std::uint32_t>(random());
  }
  return out.str();
}

bool finite_twist(const geometry_msgs::msg::Twist& value) {
  return std::isfinite(value.linear.x) && std::isfinite(value.linear.y) &&
         std::isfinite(value.linear.z) && std::isfinite(value.angular.x) &&
         std::isfinite(value.angular.y) && std::isfinite(value.angular.z);
}

geometry_msgs::msg::Twist to_twist(const h2::core::Velocity& value) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = value.vx;
  twist.linear.y = value.vy;
  twist.angular.z = value.wz;
  return twist;
}

class MotionGuardNode final : public rclcpp::Node {
 public:
  MotionGuardNode() : Node("motion_guard"), guard_(read_limits()) {
    command_source_ = declare_parameter("command_source", "navigation");
    permit_source_ = declare_parameter("permit_source", "supervisor");
    base_frame_ = declare_parameter("base_frame", "base_footprint");
    future_tolerance_ = seconds(declare_parameter("future_tolerance_s", 0.02));
    pending_timeout_ = seconds(declare_parameter("pending_timeout_s", 1.0));
    const double rate = declare_parameter("update_rate_hz", 50.0);
    if (command_source_.empty() || permit_source_.empty() || base_frame_.empty() ||
        !std::isfinite(rate) || rate < 1.0 || rate > 1000.0) {
      throw std::invalid_argument("invalid motion guard identity, frame, or rate");
    }

    command_pub_ = create_publisher<h2_interfaces::msg::MotionCommand>(
        "/motion/safe_cmd", rclcpp::QoS(1).reliable());
    status_pub_ = create_publisher<h2_interfaces::msg::MotionStatus>(
        "/motion/status", rclcpp::QoS(10).reliable());
    command_sub_ = create_subscription<h2_interfaces::msg::MotionCommand>(
        "/motion/command", rclcpp::QoS(1).reliable(),
        [this](h2_interfaces::msg::MotionCommand::ConstSharedPtr msg) { on_command(*msg); });
    permit_sub_ = create_subscription<h2_interfaces::msg::MotionPermit>(
        "/motion/permit", rclcpp::QoS(1).reliable(),
        [this](h2_interfaces::msg::MotionPermit::ConstSharedPtr msg) { on_permit(*msg); });
    enable_service_ = create_service<h2_interfaces::srv::SetMotionEnabled>(
        "/motion/set_enabled",
        [this](const std::shared_ptr<h2_interfaces::srv::SetMotionEnabled::Request> request,
               std::shared_ptr<h2_interfaces::srv::SetMotionEnabled::Response> response) {
          const auto now = monotonic_now();
          guard_.revoke(h2::core::Reason::Revoked, now);
          have_command_ = false;
          have_permission_sequence_ = false;
          boundary_reason_.clear();
          pending_ = request->enable;
          pending_deadline_ = pending_ ? now + pending_timeout_ : Time{0};
          if (pending_) {
            session_ = new_session();
          }
          response->success = true;
          response->session_id = session_;
          response->message = pending_ ? "waiting for a fresh supervisor permit" : "disabled";
          publish();
        });
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / rate)),
        [this]() { publish(); });
    RCLCPP_INFO(get_logger(), "Motion disabled; enable creates a new session and still requires a permit");
  }

 private:
  h2::core::Limits read_limits() {
    h2::core::Limits limits;
    limits.max_forward_mps = declare_parameter("max_forward_mps", 0.2);
    limits.max_yaw_rps = declare_parameter("max_yaw_rps", 0.3);
    limits.max_command_validity = seconds(declare_parameter("max_command_validity_s", 0.5));
    limits.max_permission_validity = seconds(declare_parameter("max_permit_validity_s", 0.5));
    return limits;
  }

  // Convert a checked ROS acquisition age to the SAME monotonic timeline used
  // by the guard. Subtracting age, rather than stamping receipt, preserves TTL.
  bool metadata(const std_msgs::msg::Header& header,
                const builtin_interfaces::msg::Duration& validity,
                Time maximum, Time now, Time& issued, Time& duration) const {
    if (header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000U ||
        validity.sec < 0 || validity.nanosec >= 1000000000U) {
      return false;
    }
    duration = Time{static_cast<std::int64_t>(validity.sec) * 1000000000LL + validity.nanosec};
    if (duration <= Time{0} || duration > maximum) {
      return false;
    }
    const auto stamp = static_cast<std::int64_t>(header.stamp.sec) * 1000000000LL +
                       header.stamp.nanosec;
    auto age = Time{get_clock()->now().nanoseconds() - stamp};
    if (age < -future_tolerance_ || age >= duration) {
      return false;
    }
    age = std::max(age, Time{0});
    issued = now - age;
    return true;
  }

  void boundary_stop(const std::string& reason, Time now) {
    pending_ = false;
    have_command_ = false;
    boundary_reason_ = reason;
    guard_.revoke(h2::core::Reason::Fault, now);
  }

  void expire_pending(Time now) {
    if (pending_ && now >= pending_deadline_) {
      pending_ = false;
      have_command_ = false;
      boundary_reason_ = "pending_permission_timeout";
      guard_.revoke(h2::core::Reason::Revoked, now);
    }
  }

  void on_permit(const h2_interfaces::msg::MotionPermit& msg) {
    if (msg.source != permit_source_ || session_.empty() || msg.session_id != session_) {
      return;
    }
    const Time now = monotonic_now();
    expire_pending(now);
    if (msg.header.frame_id != base_frame_) {
      boundary_stop("invalid_permit_frame", now);
      return;
    }
    if (!msg.enabled) {
      pending_ = false;
      have_command_ = false;
      boundary_reason_ = "permission_revoked";
      guard_.revoke(h2::core::Reason::Revoked, now);
      publish();
      return;
    }
    if (have_permission_sequence_ && msg.sequence <= permission_sequence_) {
      return;
    }
    Time issued, duration;
    if (!metadata(msg.header, msg.validity, guard_.limits().max_permission_validity,
                  now, issued, duration)) {
      boundary_stop("invalid_permit_timestamp_or_validity", now);
      return;
    }
    // Permit issuer and authorized command source are deliberately distinct.
    const h2::core::Lease lease{command_source_, session_, msg.sequence, issued, duration};
    if (pending_) {
      const auto result = guard_.arm(lease, now);
      pending_ = false;
      boundary_reason_ = result ? "" : h2::core::to_string(result.reason);
      if (result) {
        permission_sequence_ = msg.sequence;
        have_permission_sequence_ = true;
      }
    } else {
      if (guard_.renew_permission(lease, now)) {
        permission_sequence_ = msg.sequence;
        have_permission_sequence_ = true;
      }
    }
  }

  void on_command(const h2_interfaces::msg::MotionCommand& msg) {
    if (pending_ || session_.empty() || msg.session_id != session_ ||
        msg.source != command_source_) {
      return;
    }
    if (have_command_ && msg.sequence <= last_command_.sequence) {
      return;  // A delayed/replayed old packet cannot replace a newer command.
    }
    const Time now = monotonic_now();
    if (msg.header.frame_id != base_frame_ || !finite_twist(msg.velocity) ||
        msg.velocity.linear.z != 0.0 || msg.velocity.angular.x != 0.0 ||
        msg.velocity.angular.y != 0.0) {
      boundary_stop("invalid_command_frame_or_motion", now);
      return;
    }
    const bool zero = msg.velocity.linear.x == 0.0 && msg.velocity.linear.y == 0.0 &&
                      msg.velocity.angular.z == 0.0;
    if (zero && msg.validity.sec == 0 && msg.validity.nanosec == 0) {
      pending_ = false;
      have_command_ = false;
      boundary_reason_ = "explicit_stop";
      guard_.revoke(h2::core::Reason::Revoked, now);
      return;
    }
    Time issued, duration;
    if (!metadata(msg.header, msg.validity, guard_.limits().max_command_validity,
                  now, issued, duration)) {
      boundary_stop("invalid_command_timestamp_or_validity", now);
      return;
    }
    const h2::core::Command command{
        msg.source, msg.session_id, msg.sequence, issued, duration,
        {msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.angular.z}};
    if (guard_.accept(command, now)) {
      last_command_ = msg;
      have_command_ = true;
      boundary_reason_.clear();
    }
  }

  void publish() {
    const auto now = monotonic_now();
    expire_pending(now);
    const auto output = guard_.tick(now);
    h2_interfaces::msg::MotionStatus status;
    status.header.stamp = get_clock()->now();
    status.header.frame_id = base_frame_;
    status.session_id = session_;
    status.command_sequence = have_command_ ? last_command_.sequence : 0;
    status.enabled = output.armed;
    status.command_valid = output.command_valid && have_command_;
    status.output = to_twist(output.velocity);
    status.reason = pending_ ? "waiting_permission" :
        (!output.armed && !boundary_reason_.empty() ? boundary_reason_ :
         h2::core::to_string(output.reason));

    h2_interfaces::msg::MotionCommand safe;
    if (status.command_valid) {
      safe = last_command_;  // Preserve ORIGINAL stamp/sequence/validity.
      safe.velocity = status.output;
    } else {
      safe.header = status.header;
      safe.source = command_source_;
      safe.session_id = session_;
      safe.sequence = status.command_sequence;
      // Zero validity and zero velocity is an immediate terminal stop.
    }
    status_pub_->publish(status);
    command_pub_->publish(safe);
  }

  h2::core::MotionGuard guard_;
  std::string command_source_, permit_source_, base_frame_, session_, boundary_reason_;
  Time future_tolerance_{0};
  Time pending_timeout_{0}, pending_deadline_{0};
  bool pending_{false};
  bool have_command_{false};
  bool have_permission_sequence_{false};
  std::uint64_t permission_sequence_{0};
  h2_interfaces::msg::MotionCommand last_command_;
  rclcpp::Publisher<h2_interfaces::msg::MotionCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<h2_interfaces::msg::MotionStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<h2_interfaces::msg::MotionCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<h2_interfaces::msg::MotionPermit>::SharedPtr permit_sub_;
  rclcpp::Service<h2_interfaces::srv::SetMotionEnabled>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotionGuardNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("motion_guard"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
