import math
import unittest
from h2_supervisor.health import HealthMonitor, decode_stamp


class HealthTests(unittest.TestCase):
    def test_malformed_stamp_fields_are_not_normalized(self):
        self.assertTrue(math.isnan(decode_stamp(-1, 1_000_000_001)))
        self.assertTrue(math.isnan(decode_stamp(100, 1_000_000_000)))
        self.assertTrue(math.isnan(decode_stamp(100, -1)))
        self.assertAlmostEqual(decode_stamp(100, 500_000_000), 100.5)

    def ready(self, mode="sim"):
        monitor = HealthMonitor(mode=mode)
        monitor.observe_odom(received=10, stamp=100, valid=True)
        monitor.observe_guard(received=10, stamp=100, session_id="s1",
                              enabled=False, reason="waiting_permission")
        return monitor

    def test_default_has_no_permission(self):
        self.assertFalse(HealthMonitor().evaluate(steady_now=10, ros_now=100).allowed)

    def test_pending_session_with_fresh_inputs_can_receive_lease(self):
        self.assertTrue(self.ready().evaluate(steady_now=10.1, ros_now=100.1).allowed)

    def test_receipt_timeout_cannot_be_hidden_by_ros_clock_pause(self):
        self.assertFalse(self.ready().evaluate(steady_now=10.5, ros_now=100).allowed)

    def test_stale_and_future_headers_are_rejected(self):
        self.assertFalse(self.ready().evaluate(steady_now=10.1, ros_now=101).allowed)
        self.assertFalse(self.ready().evaluate(steady_now=10.1, ros_now=99).allowed)

    def test_non_finite_and_invalid_odom_fail_closed(self):
        monitor = self.ready()
        monitor.observe_odom(received=10, stamp=float("nan"), valid=True)
        self.assertFalse(monitor.evaluate(steady_now=10.1, ros_now=100.1).allowed)
        monitor.observe_odom(received=10, stamp=100, valid=False)
        self.assertFalse(monitor.evaluate(steady_now=10.1, ros_now=100.1).allowed)

    def test_command_expiry_does_not_implicitly_rearm(self):
        monitor = self.ready()
        monitor.observe_guard(received=10, stamp=100, session_id="s1",
                              enabled=False, reason="command_expired")
        self.assertFalse(monitor.evaluate(steady_now=10.1, ros_now=100.1).allowed)

    def test_hardware_never_receives_simulation_permission(self):
        result = self.ready("hardware").evaluate(steady_now=10.1, ros_now=100.1)
        self.assertFalse(result.allowed)
        self.assertEqual(result.reason, "hardware_policy_not_implemented")

    def test_clock_or_monotonic_regression_fails_closed(self):
        self.assertFalse(self.ready().evaluate(steady_now=9, ros_now=100).allowed)

    def test_invalid_policy_parameters_rejected(self):
        for value in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                HealthMonitor(max_age=value)


if __name__ == "__main__":
    unittest.main()
