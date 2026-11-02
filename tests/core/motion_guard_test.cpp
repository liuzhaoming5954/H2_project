#include "h2_core/motion_guard.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace h2::core;
using namespace std::chrono_literals;

void expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void zero(const Output& output, const std::string& context) {
  expect(output.velocity.vx == 0.0 && output.velocity.vy == 0.0 &&
         output.velocity.wz == 0.0, context + ": output must be zero");
}

Lease lease(std::string session = "session-1", Time issued = 0ns,
            Time validity = 500ms, std::uint64_t sequence = 1) {
  return {"navigation", std::move(session), sequence, issued, validity};
}

Command command(std::string session = "session-1", Time issued = 0ns,
                Time validity = 200ms, std::uint64_t sequence = 1,
                Velocity velocity = {0.1, 0.0, 0.2}) {
  return {"navigation", std::move(session), sequence, issued, validity, velocity};
}

void default_is_stopped() {
  MotionGuard guard;
  const auto output = guard.tick(0ns);
  zero(output, "startup");
  expect(!output.armed && !output.command_valid &&
         output.reason == Reason::NotArmed, "startup must be unarmed");
  expect(!guard.accept(command(), 0ns).accepted, "unarmed command rejected");
}

void invalid_limits_are_rejected() {
  for (const double invalid : {0.0, -1.0,
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()}) {
    Limits limits;
    limits.max_forward_mps = invalid;
    bool threw = false;
    try { MotionGuard guard(limits); }
    catch (const std::invalid_argument&) { threw = true; }
    expect(threw, "invalid forward limit must throw");
  }
  Limits limits;
  limits.max_permission_validity = 0ns;
  bool threw = false;
  try { MotionGuard guard(limits); }
  catch (const std::invalid_argument&) { threw = true; }
  expect(threw, "zero permission validity limit must throw");
}

void arming_requires_fresh_valid_permission() {
  MotionGuard guard;
  auto permission = lease("stale", 0ns, 100ms);
  expect(guard.arm(permission, 100ms).reason == Reason::PermissionExpired,
         "expiry boundary is exclusive");
  permission = lease("future", 101ms);
  expect(guard.arm(permission, 100ms).reason == Reason::InvalidTimestamp,
         "future lease rejected");
  permission = lease("invalid-duration", 100ms, 501ms);
  expect(guard.arm(permission, 100ms).reason == Reason::InvalidValidity,
         "oversized lease rejected");
  permission = lease("", 100ms);
  expect(guard.arm(permission, 100ms).reason == Reason::InvalidIdentity,
         "empty session rejected");
  expect(guard.arm(lease("valid", 100ms), 100ms).accepted,
         "valid permission arms");
  auto output = guard.tick(100ms);
  zero(output, "armed without command");
  expect(output.armed && !output.command_valid &&
         output.reason == Reason::WaitingForCommand,
         "arming does not itself produce motion");
}

void velocity_limits_preserve_allowed_motion() {
  MotionGuard guard;
  expect(guard.arm(lease(), 0ns).accepted, "arm");
  expect(guard.accept(command("session-1", 1ms, 200ms, 1,
                              {5.0, 0.0, -5.0}), 1ms).accepted,
         "finite forward/yaw command accepted and limited");
  auto output = guard.tick(10ms);
  expect(output.armed && output.command_valid && output.reason == Reason::Ready,
         "valid command usable");
  expect(output.velocity.vx == 0.2 && output.velocity.vy == 0.0 &&
         output.velocity.wz == -0.4, "both limits enforced");
}

void duplicate_command_does_not_extend_original_expiry() {
  MotionGuard guard;
  expect(guard.arm(lease(), 0ns).accepted, "arm");
  expect(guard.accept(command("session-1", 0ns, 100ms, 7), 0ns).accepted,
         "initial command");
  auto duplicate = command("session-1", 80ms, 100ms, 7);
  expect(guard.accept(duplicate, 80ms).reason == Reason::SequenceNotIncreasing,
         "re-stamped duplicate rejected");
  auto output = guard.tick(100ms);
  zero(output, "original command expired");
  expect(!output.armed && output.reason == Reason::CommandExpired,
         "expiry latches stop");
  expect(!guard.accept(command("session-1", 101ms, 100ms, 8), 101ms).accepted,
         "new command alone cannot revive expired authority");
  expect(!guard.renew_permission(lease("session-1", 102ms, 400ms, 2),
                                 102ms).accepted,
         "new permission alone cannot revive expired authority");
  expect(guard.arm(lease("session-1", 103ms), 103ms).reason ==
         Reason::SessionReused, "explicit rearm needs a new session");
  expect(guard.arm(lease("session-2", 104ms), 104ms).accepted,
         "explicit fresh session can rearm");
  zero(guard.tick(104ms), "rearm must not replay previous command");
}

void permission_expiry_is_independent_of_command_updates() {
  MotionGuard guard;
  expect(guard.arm(lease("session-1", 0ns, 100ms, 5), 0ns).accepted, "arm");
  expect(guard.accept(command("session-1", 50ms), 50ms).accepted, "command");
  expect(guard.renew_permission(lease("session-1", 80ms, 100ms, 5),
                                80ms).reason == Reason::SequenceNotIncreasing,
         "re-stamped duplicate permission rejected");
  const auto output = guard.tick(100ms);
  zero(output, "permission expired while command fresh");
  expect(!output.armed && output.reason == Reason::PermissionExpired,
         "permission expiry takes precedence");
}

void fresh_permission_renews_but_not_command() {
  MotionGuard guard;
  expect(guard.arm(lease("session-1", 0ns, 100ms), 0ns).accepted, "arm");
  expect(guard.accept(command("session-1", 0ns, 200ms), 0ns).accepted, "command");
  expect(guard.renew_permission(lease("session-1", 90ms, 300ms, 2),
                                90ms).accepted, "fresh permission renewed");
  expect(guard.tick(100ms).command_valid, "new permission remains live");
  const auto output = guard.tick(200ms);
  zero(output, "command expired despite renewed permission");
  expect(output.reason == Reason::CommandExpired, "command expires independently");
}

void wrong_owner_and_sequence_do_not_replace_current_command() {
  MotionGuard guard;
  expect(guard.arm(lease(), 0ns).accepted, "arm");
  expect(guard.accept(command(), 0ns).accepted, "command");
  auto other = command("session-1", 10ms, 200ms, 2, {0.2, 0.0, 0.0});
  other.source = "teleop";
  expect(guard.accept(other, 10ms).reason == Reason::SourceMismatch,
         "other source rejected");
  other.source = "navigation";
  other.session = "previous-session";
  expect(guard.accept(other, 11ms).reason == Reason::SessionMismatch,
         "other session rejected");
  auto other_permission = lease("previous-session", 12ms, 300ms, 2);
  expect(guard.renew_permission(other_permission, 12ms).reason ==
         Reason::SessionMismatch, "other permission session rejected");
  expect(guard.arm(lease("another", 13ms), 13ms).reason == Reason::AlreadyArmed,
         "arm cannot steal an active authority");
  const auto output = guard.tick(20ms);
  expect(output.command_valid && output.velocity.vx == 0.1,
         "rejected unrelated input did not replace original command");
}

void invalid_owner_commands_latch_zero() {
  const std::vector<std::pair<Velocity, Reason>> cases = {
    {{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, Reason::NonFiniteVelocity},
    {{0.1, 0.0, std::numeric_limits<double>::infinity()}, Reason::NonFiniteVelocity},
    {{-0.1, 0.0, 0.0}, Reason::UnsupportedMotion},
    {{0.1, 0.01, 0.0}, Reason::UnsupportedMotion},
  };
  for (const auto& [velocity, reason] : cases) {
    MotionGuard guard;
    expect(guard.arm(lease(), 0ns).accepted, "arm");
    expect(guard.accept(command(), 0ns).accepted, "initial command");
    expect(guard.accept(command("session-1", 1ms, 100ms, 2, velocity),
                        1ms).reason == reason, "bad owner command rejected");
    const auto output = guard.tick(1ms);
    zero(output, "invalid command must clear previous motion");
    expect(!output.armed && output.reason == reason, "invalid input latches stop");
  }
}

void invalid_time_or_validity_cannot_replace_current_command() {
  {
    MotionGuard guard;
    expect(guard.arm(lease(), 0ns).accepted, "arm");
    expect(guard.accept(command(), 0ns).accepted, "initial command");
    expect(guard.accept(command("session-1", 11ms, 100ms, 2),
                        10ms).reason == Reason::InvalidTimestamp,
           "future command rejected");
    zero(guard.tick(10ms), "future command stopped previous motion");
  }
  {
    MotionGuard guard;
    expect(guard.arm(lease(), 0ns).accepted, "arm");
    expect(guard.accept(command("session-1", 0ns, 0ns),
                        0ns).reason == Reason::InvalidValidity,
           "zero validity rejected");
    zero(guard.tick(0ns), "zero validity");
  }
  {
    MotionGuard guard;
    expect(guard.arm(lease(), 0ns).accepted, "arm");
    expect(guard.accept(command("session-1", 0ns, 100ms),
                        100ms).reason == Reason::CommandExpired,
           "delayed command already expired at arrival");
    zero(guard.tick(100ms), "delayed command");
  }
}

void revocation_does_not_auto_resume() {
  for (const Reason reason : {Reason::ManualTakeover, Reason::Fault,
                              Reason::EmergencyStop, Reason::Revoked}) {
    MotionGuard guard;
    expect(guard.arm(lease(), 0ns).accepted, "arm");
    expect(guard.accept(command(), 0ns).accepted, "command");
    guard.revoke(reason, 20ms);
    auto output = guard.tick(20ms);
    zero(output, "revocation");
    expect(!output.armed && output.reason == reason, "revocation reason retained");
    expect(!guard.accept(command("session-1", 21ms, 100ms, 2),
                         21ms).accepted, "cannot resume by publishing command");
    expect(!guard.renew_permission(lease("session-1", 22ms, 300ms, 2),
                                    22ms).accepted,
           "cannot resume by publishing permission");
  }
}

void clock_rollback_stops_even_with_fresh_data() {
  MotionGuard guard;
  expect(guard.arm(lease("session-1", 100ms), 100ms).accepted, "arm");
  expect(guard.accept(command("session-1", 100ms), 100ms).accepted, "command");
  auto output = guard.tick(99ms);
  zero(output, "clock rollback");
  expect(!output.armed && output.reason == Reason::ClockError,
         "rollback latches clock error");
  expect(!guard.arm(lease("session-2", 99ms), 99ms).accepted,
         "cannot arm on regressed timeline");
  expect(guard.arm(lease("session-2", 101ms), 101ms).accepted,
         "explicit new session after valid time can rearm");
}

void timestamp_math_is_safe_near_integer_limit() {
  MotionGuard guard;
  const Time near_max{std::numeric_limits<std::int64_t>::max() - 10};
  expect(guard.arm(lease("session-1", near_max, 100ns), near_max).accepted,
         "lease deadline need not be representable as a sum");
  expect(guard.accept(command("session-1", near_max, 50ns), near_max).accepted,
         "command age comparison avoids sum overflow");
  expect(guard.tick(near_max + 5ns).command_valid, "near-limit time remains usable");
}

void expiry_is_enforced_at_message_arrival_without_a_timer_tick() {
  {
    MotionGuard guard;
    expect(guard.arm(lease(), 0ns).accepted, "arm");
    expect(guard.accept(command("session-1", 0ns, 100ms), 0ns).accepted,
           "initial command");
    // No tick has observed the expiry yet. A newly arrived command still must
    // not make an expired authority live again, including at the boundary.
    const auto result = guard.accept(command("session-1", 100ms, 100ms, 2), 100ms);
    expect(!result.accepted && result.reason == Reason::CommandExpired,
           "command callback must enforce expiry before accepting replacement");
    zero(guard.tick(100ms), "expired command replaced before timer");
  }
  {
    MotionGuard guard;
    expect(guard.arm(lease("session-1", 0ns, 100ms), 0ns).accepted, "arm");
    expect(guard.accept(command("session-1", 0ns, 200ms), 0ns).accepted,
           "initial command outlives permission");
    const auto result = guard.renew_permission(lease("session-1", 100ms, 300ms, 2),
                                                100ms);
    expect(!result.accepted && result.reason == Reason::PermissionExpired,
           "permission callback must not renew an already expired lease");
    zero(guard.tick(100ms), "expired permission renewed before timer");
  }
  {
    MotionGuard guard;
    expect(guard.arm(lease(), 0ns).accepted, "arm");
    expect(guard.accept(command("session-1", 0ns, 100ms), 0ns).accepted,
           "initial command");
    const auto result = guard.renew_permission(lease("session-1", 100ms, 300ms, 2),
                                                100ms);
    expect(!result.accepted && result.reason == Reason::CommandExpired,
           "permit renewal must detect command expiry too");
    zero(guard.tick(100ms), "fresh permit after command expiration");
  }
}

void rearm_never_authorizes_packets_from_a_previous_session() {
  MotionGuard guard;
  expect(guard.arm(lease(), 0ns).accepted, "first session arm");
  expect(guard.accept(command(), 0ns).accepted, "first session command");
  guard.revoke(Reason::ManualTakeover, 10ms);
  expect(guard.arm(lease("session-2", 20ms, 300ms, 0), 20ms).accepted,
         "new session can reset its own sequence space");
  expect(guard.accept(command("session-1", 21ms, 100ms, 100),
                      21ms).reason == Reason::SessionMismatch,
         "higher sequence from old session cannot authorize motion");
  expect(guard.renew_permission(lease("session-1", 22ms, 300ms, 100),
                                22ms).reason == Reason::SessionMismatch,
         "old permission cannot replace a new session lease");
  auto output = guard.tick(22ms);
  zero(output, "new session still has no authorized command");
  expect(output.armed && !output.command_valid,
         "unrelated stale traffic does not destroy new authority");
  expect(guard.accept(command("session-2", 23ms, 100ms, 0), 23ms).accepted,
         "new session accepts its own first sequence");
  expect(guard.tick(23ms).command_valid, "new session motion becomes valid");
}

void sequence_wraparound_cannot_refresh_command_or_permission() {
  const auto maximum = std::numeric_limits<std::uint64_t>::max();
  {
    MotionGuard guard;
    expect(guard.arm(lease(), 0ns).accepted, "arm");
    expect(guard.accept(command("session-1", 0ns, 100ms, maximum), 0ns).accepted,
           "last representable command sequence accepted");
    expect(guard.accept(command("session-1", 50ms, 200ms, 0),
                        50ms).reason == Reason::SequenceNotIncreasing,
           "wrapped command sequence rejected");
    const auto output = guard.tick(100ms);
    zero(output, "wrapped command did not extend original deadline");
    expect(output.reason == Reason::CommandExpired, "original command expires");
  }
  {
    MotionGuard guard;
    expect(guard.arm(lease("session-1", 0ns, 100ms, maximum), 0ns).accepted,
           "last representable lease sequence accepted");
    expect(guard.accept(command("session-1", 0ns, 200ms), 0ns).accepted,
           "command outlives lease");
    expect(guard.renew_permission(lease("session-1", 50ms, 300ms, 0),
                                  50ms).reason == Reason::SequenceNotIncreasing,
           "wrapped permission sequence rejected");
    const auto output = guard.tick(100ms);
    zero(output, "wrapped lease did not extend original deadline");
    expect(output.reason == Reason::PermissionExpired, "original lease expires");
  }
}

void increasing_sequence_cannot_hide_regressing_issue_time() {
  {
    MotionGuard guard;
    expect(guard.arm(lease(), 0ns).accepted, "arm");
    expect(guard.accept(command("session-1", 20ms, 100ms, 1), 20ms).accepted,
           "initial command");
    expect(guard.accept(command("session-1", 19ms, 100ms, 2),
                        21ms).reason == Reason::InvalidTimestamp,
           "higher sequence does not authorize regressing command time");
    auto output = guard.tick(21ms);
    zero(output, "regressing command timestamp latches stop");
    expect(!output.armed, "invalid owner timestamp cannot leave motion armed");
  }
  {
    MotionGuard guard;
    expect(guard.arm(lease("session-1", 20ms), 20ms).accepted, "arm");
    expect(guard.accept(command("session-1", 20ms), 20ms).accepted, "command");
    expect(guard.renew_permission(lease("session-1", 19ms, 300ms, 2),
                                  21ms).reason == Reason::InvalidTimestamp,
           "higher sequence does not authorize regressing permission time");
    auto output = guard.tick(21ms);
    zero(output, "regressing permission timestamp latches stop");
    expect(!output.armed, "invalid permission cannot leave motion armed");
  }
}

void waiting_for_first_command_does_not_suspend_permission_expiry() {
  MotionGuard guard;
  expect(guard.arm(lease("session-1", 0ns, 100ms), 0ns).accepted, "arm");
  const auto waiting = guard.tick(99ms);
  zero(waiting, "waiting for first command");
  expect(waiting.armed && waiting.reason == Reason::WaitingForCommand,
         "fresh permission permits waiting without producing motion");
  const auto expired = guard.tick(100ms);
  zero(expired, "unused permission expired");
  expect(!expired.armed && expired.reason == Reason::PermissionExpired,
         "unused permission still has a finite lifetime");
  expect(!guard.accept(command("session-1", 101ms), 101ms).accepted,
         "first command cannot revive an unused expired session");
}

}  // namespace

int main() {
  const std::vector<std::pair<const char*, std::function<void()>>> tests = {
    {"default_is_stopped", default_is_stopped},
    {"invalid_limits_are_rejected", invalid_limits_are_rejected},
    {"arming_requires_fresh_valid_permission", arming_requires_fresh_valid_permission},
    {"velocity_limits_preserve_allowed_motion", velocity_limits_preserve_allowed_motion},
    {"duplicate_command_does_not_extend_original_expiry", duplicate_command_does_not_extend_original_expiry},
    {"permission_expiry_is_independent_of_command_updates", permission_expiry_is_independent_of_command_updates},
    {"fresh_permission_renews_but_not_command", fresh_permission_renews_but_not_command},
    {"wrong_owner_and_sequence_do_not_replace_current_command", wrong_owner_and_sequence_do_not_replace_current_command},
    {"invalid_owner_commands_latch_zero", invalid_owner_commands_latch_zero},
    {"invalid_time_or_validity_cannot_replace_current_command", invalid_time_or_validity_cannot_replace_current_command},
    {"revocation_does_not_auto_resume", revocation_does_not_auto_resume},
    {"clock_rollback_stops_even_with_fresh_data", clock_rollback_stops_even_with_fresh_data},
    {"timestamp_math_is_safe_near_integer_limit", timestamp_math_is_safe_near_integer_limit},
    {"expiry_is_enforced_at_message_arrival_without_a_timer_tick", expiry_is_enforced_at_message_arrival_without_a_timer_tick},
    {"rearm_never_authorizes_packets_from_a_previous_session", rearm_never_authorizes_packets_from_a_previous_session},
    {"sequence_wraparound_cannot_refresh_command_or_permission", sequence_wraparound_cannot_refresh_command_or_permission},
    {"increasing_sequence_cannot_hide_regressing_issue_time", increasing_sequence_cannot_hide_regressing_issue_time},
    {"waiting_for_first_command_does_not_suspend_permission_expiry", waiting_for_first_command_does_not_suspend_permission_expiry},
  };
  int failed = 0;
  for (const auto& [name, test] : tests) {
    try {
      test();
      std::cout << "PASS " << name << '\n';
    } catch (const std::exception& error) {
      ++failed;
      std::cerr << "FAIL " << name << ": " << error.what() << '\n';
    }
  }
  std::cout << tests.size() - static_cast<std::size_t>(failed) << '/'
            << tests.size() << " motion guard tests passed\n";
  return failed == 0 ? 0 : 1;
}
