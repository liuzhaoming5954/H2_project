#include "h2_core/motion_guard.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace h2::core {

const char* to_string(Reason reason) noexcept {
  switch (reason) {
    case Reason::NotArmed: return "not_armed";
    case Reason::WaitingForCommand: return "waiting_for_command";
    case Reason::Ready: return "ready";
    case Reason::AlreadyArmed: return "already_armed";
    case Reason::InvalidIdentity: return "invalid_identity";
    case Reason::SessionReused: return "session_reused";
    case Reason::SourceMismatch: return "source_mismatch";
    case Reason::SessionMismatch: return "session_mismatch";
    case Reason::SequenceNotIncreasing: return "sequence_not_increasing";
    case Reason::InvalidTimestamp: return "invalid_timestamp";
    case Reason::InvalidValidity: return "invalid_validity";
    case Reason::PermissionExpired: return "permission_expired";
    case Reason::CommandExpired: return "command_expired";
    case Reason::NonFiniteVelocity: return "non_finite_velocity";
    case Reason::UnsupportedMotion: return "unsupported_motion";
    case Reason::ClockError: return "clock_error";
    case Reason::Revoked: return "revoked";
    case Reason::ManualTakeover: return "manual_takeover";
    case Reason::Fault: return "fault";
    case Reason::EmergencyStop: return "emergency_stop";
  }
  return "unknown";
}

MotionGuard::MotionGuard(Limits limits) : limits_(std::move(limits)) {
  if (!std::isfinite(limits_.max_forward_mps) ||
      !std::isfinite(limits_.max_yaw_rps) ||
      limits_.max_forward_mps <= 0.0 || limits_.max_yaw_rps < 0.0 ||
      limits_.max_command_validity <= Time::zero() ||
      limits_.max_permission_validity <= Time::zero()) {
    throw std::invalid_argument("motion limits must be finite and validities positive");
  }
}

bool MotionGuard::valid_identity(const std::string& value) {
  if (value.empty() || value.size() > 128) return false;
  for (const unsigned char byte : value) {
    if (byte <= 0x20 || byte == 0x7f) return false;
  }
  return true;
}

Result MotionGuard::validate_time(Time issued_at, Time valid_for, Time maximum,
                                  Time now, Reason expiry) {
  if (now < Time::zero() || issued_at < Time::zero() || issued_at > now) {
    return {false, Reason::InvalidTimestamp};
  }
  if (valid_for <= Time::zero() || valid_for > maximum) {
    return {false, Reason::InvalidValidity};
  }
  // Subtraction avoids overflowing an issued_at + valid_for deadline.
  if (now - issued_at >= valid_for) return {false, expiry};
  return {true, Reason::Ready};
}

Result MotionGuard::validate_lease(const Lease& permission, Time now) const {
  if (!valid_identity(permission.source) || !valid_identity(permission.session)) {
    return {false, Reason::InvalidIdentity};
  }
  return validate_time(permission.issued_at, permission.valid_for,
                       limits_.max_permission_validity, now,
                       Reason::PermissionExpired);
}

void MotionGuard::stop(Reason reason) {
  armed_ = false;
  has_command_ = false;
  command_.velocity = {};
  reason_ = reason;
}

bool MotionGuard::advance(Time now) {
  if (now < Time::zero() || (observed_time_ && now < last_time_)) {
    stop(Reason::ClockError);
    return false;
  }
  observed_time_ = true;
  last_time_ = now;
  if (armed_ && now - permission_.issued_at >= permission_.valid_for) {
    stop(Reason::PermissionExpired);
  }
  if (armed_ && has_command_ &&
      now - command_.issued_at >= command_.valid_for) {
    stop(Reason::CommandExpired);
  }
  return true;
}

Result MotionGuard::arm(const Lease& permission, Time now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!advance(now)) return {false, reason_};
  if (armed_) return {false, Reason::AlreadyArmed};
  const Result valid = validate_lease(permission, now);
  if (!valid) return valid;
  if (used_sessions_.count(permission.session) != 0) {
    return {false, Reason::SessionReused};
  }
  used_sessions_.insert(permission.session);
  permission_ = permission;
  command_ = {};
  has_command_ = false;
  armed_ = true;
  reason_ = Reason::WaitingForCommand;
  return {true, reason_};
}

Result MotionGuard::renew_permission(const Lease& permission, Time now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!advance(now) || !armed_) return {false, reason_};
  if (permission.source != permission_.source) {
    return {false, Reason::SourceMismatch};
  }
  if (permission.session != permission_.session) {
    return {false, Reason::SessionMismatch};
  }
  if (permission.sequence <= permission_.sequence) {
    return {false, Reason::SequenceNotIncreasing};
  }
  Result valid = validate_lease(permission, now);
  if (permission.issued_at < permission_.issued_at) {
    valid = {false, Reason::InvalidTimestamp};
  }
  if (!valid) {
    stop(valid.reason);
    return valid;
  }
  permission_ = permission;
  return {true, has_command_ ? Reason::Ready : Reason::WaitingForCommand};
}

Result MotionGuard::accept(const Command& command, Time now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!advance(now) || !armed_) return {false, reason_};
  if (command.source != permission_.source) {
    return {false, Reason::SourceMismatch};
  }
  if (command.session != permission_.session) {
    return {false, Reason::SessionMismatch};
  }
  if (has_command_ && command.sequence <= command_.sequence) {
    return {false, Reason::SequenceNotIncreasing};
  }
  Result valid = validate_time(command.issued_at, command.valid_for,
                               limits_.max_command_validity, now,
                               Reason::CommandExpired);
  if (has_command_ && command.issued_at < command_.issued_at) {
    valid = {false, Reason::InvalidTimestamp};
  }
  if (!valid) {
    stop(valid.reason);
    return valid;
  }
  if (!std::isfinite(command.velocity.vx) ||
      !std::isfinite(command.velocity.vy) ||
      !std::isfinite(command.velocity.wz)) {
    stop(Reason::NonFiniteVelocity);
    return {false, reason_};
  }
  if (command.velocity.vx < 0.0 || command.velocity.vy != 0.0) {
    stop(Reason::UnsupportedMotion);
    return {false, reason_};
  }
  command_ = command;
  command_.velocity.vx = std::min(command.velocity.vx, limits_.max_forward_mps);
  command_.velocity.wz = std::clamp(command.velocity.wz,
                                    -limits_.max_yaw_rps, limits_.max_yaw_rps);
  has_command_ = true;
  reason_ = Reason::Ready;
  return {true, reason_};
}

void MotionGuard::revoke(Reason reason, Time now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!advance(now)) return;
  if (reason == Reason::Ready || reason == Reason::WaitingForCommand ||
      reason == Reason::AlreadyArmed) {
    reason = Reason::Revoked;
  }
  stop(reason);
}

Output MotionGuard::tick(Time now) {
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now);
  Output output;
  output.reason = reason_;
  output.armed = armed_;
  output.command_valid = armed_ && has_command_;
  if (output.command_valid) output.velocity = command_.velocity;
  return output;
}

}  // namespace h2::core
