"""ROS-independent freshness policy; all times are caller-supplied seconds."""

from dataclasses import dataclass
import math


def decode_stamp(sec, nanosec):
    """Reject malformed ROS timestamp fields rather than normalizing them."""
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return float("nan")
    return sec + nanosec * 1e-9


@dataclass(frozen=True)
class Decision:
    allowed: bool
    reason: str
    session_id: str = ""


@dataclass(frozen=True)
class Observation:
    received: float
    stamp: float
    valid: bool


class HealthMonitor:
    """Sim-only policy, deliberately unsuitable as a hardware safety certificate."""

    def __init__(self, mode="sim", max_age=0.3, future_tolerance=0.02):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError("max_age must be positive and finite")
        if not math.isfinite(future_tolerance) or future_tolerance < 0:
            raise ValueError("future_tolerance must be non-negative and finite")
        self.mode = mode
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.odom = None
        self.guard = None
        self.session_id = ""
        self.guard_requests_permission = False

    def observe_odom(self, *, received, stamp, valid):
        self.odom = Observation(received, stamp, valid)

    def observe_guard(self, *, received, stamp, session_id, enabled, reason):
        self.guard = Observation(received, stamp, bool(session_id))
        self.session_id = session_id
        self.guard_requests_permission = enabled or reason == "waiting_permission"

    def _fresh(self, observation, steady_now, ros_now):
        if observation is None or not observation.valid:
            return False
        values = (observation.received, observation.stamp, steady_now, ros_now)
        if not all(math.isfinite(value) for value in values):
            return False
        # Receipt age is monotonic; stamp age catches delayed/replayed data.
        receipt_age = steady_now - observation.received
        stamp_age = ros_now - observation.stamp
        return (0 <= receipt_age <= self.max_age
                and observation.stamp > 0
                and -self.future_tolerance <= stamp_age <= self.max_age)

    def evaluate(self, *, steady_now, ros_now):
        if self.mode != "sim":
            return Decision(False, "hardware_policy_not_implemented", self.session_id)
        if not self._fresh(self.odom, steady_now, ros_now):
            return Decision(False, "odom_unhealthy", self.session_id)
        if not self._fresh(self.guard, steady_now, ros_now):
            return Decision(False, "guard_unhealthy", self.session_id)
        if not self.guard_requests_permission:
            return Decision(False, "guard_not_requested", self.session_id)
        return Decision(True, "sim_healthy", self.session_id)
