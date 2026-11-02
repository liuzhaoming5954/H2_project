#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_set>

namespace h2::core {

// All times use the same monotonic clock. A ROS/wall-clock stamp must be
// validated and converted by the boundary adapter before reaching this API.
using Time = std::chrono::nanoseconds;

struct Velocity {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct Limits {
  double max_forward_mps{0.2};
  double max_yaw_rps{0.4};
  Time max_command_validity{std::chrono::milliseconds{250}};
  Time max_permission_validity{std::chrono::milliseconds{500}};
};

struct Lease {
  std::string source;
  std::string session;
  std::uint64_t sequence{0};
  Time issued_at{0};
  Time valid_for{0};
};

struct Command {
  std::string source;
  std::string session;
  std::uint64_t sequence{0};
  Time issued_at{0};
  Time valid_for{0};
  Velocity velocity;
};

enum class Reason {
  NotArmed,
  WaitingForCommand,
  Ready,
  AlreadyArmed,
  InvalidIdentity,
  SessionReused,
  SourceMismatch,
  SessionMismatch,
  SequenceNotIncreasing,
  InvalidTimestamp,
  InvalidValidity,
  PermissionExpired,
  CommandExpired,
  NonFiniteVelocity,
  UnsupportedMotion,
  ClockError,
  Revoked,
  ManualTakeover,
  Fault,
  EmergencyStop,
};

const char* to_string(Reason reason) noexcept;

struct Result {
  bool accepted{false};
  Reason reason{Reason::NotArmed};
  explicit operator bool() const noexcept { return accepted; }
};

struct Output {
  Velocity velocity;
  Reason reason{Reason::NotArmed};
  bool armed{false};
  bool command_valid{false};
};

// The guard is a final authority/expiry/limit check, not an obstacle detector,
// gait controller or hardware emergency stop. All public methods are serialized.
// Expired permission/commands latch a zero-output state until an explicit arm
// with a previously unused session. Duplicate messages never refresh a deadline.
class MotionGuard {
 public:
  explicit MotionGuard(Limits limits = {});

  Result arm(const Lease& permission, Time now);
  Result renew_permission(const Lease& permission, Time now);
  Result accept(const Command& command, Time now);
  void revoke(Reason reason, Time now);
  Output tick(Time now);
  Limits limits() const noexcept { return limits_; }

 private:
  bool advance(Time now);
  void stop(Reason reason);
  Result validate_lease(const Lease& permission, Time now) const;
  static bool valid_identity(const std::string& value);
  static Result validate_time(Time issued_at, Time valid_for, Time maximum,
                              Time now, Reason expiry);

  const Limits limits_;
  std::mutex mutex_;
  bool observed_time_{false};
  Time last_time_{0};
  bool armed_{false};
  bool has_command_{false};
  Lease permission_;
  Command command_;
  Reason reason_{Reason::NotArmed};
  std::unordered_set<std::string> used_sessions_;
};

}  // namespace h2::core
