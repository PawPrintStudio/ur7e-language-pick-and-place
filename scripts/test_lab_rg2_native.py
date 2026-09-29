"""Failure-path checks for the bounded native RG2 bench diagnostic."""
import unittest
from unittest.mock import Mock, patch

import lab_rg2_native as lab


def state(**changes):
    value = dict(width=59.0, status=0, busy=False, safety_failed=False,
                 s1_pushed=False, s1_triggered=False, s2_pushed=False,
                 s2_triggered=False, operation_counter=1)
    value.update(changes)
    return value


class NativeChecks(unittest.TestCase):
    def test_invalid_feedback_rejected(self):
        for changes in ({"width": float("nan")}, {"width": -17.1},
                        {"status": 4}, {"safety_failed": True}):
            with self.subTest(changes=changes), self.assertRaises(RuntimeError):
                lab.validate_state(state(**changes))

    def test_busy_status_is_not_error(self):
        lab.validate_state(state(status=1, busy=True))

    @patch.object(lab, "dashboard_gate")
    def test_excessive_step_never_sent(self, gate):
        rpc = Mock()
        rpc.rg_get_all_variables.return_value = state()
        with self.assertRaises(ValueError):
            lab.move(rpc, "unused", 2, 100, 10)
        rpc.rg_grip.assert_not_called()

    @patch.object(lab, "dashboard_gate")
    def test_gate_failure_never_commands(self, gate):
        gate.side_effect = RuntimeError("robot program running")
        rpc = Mock()
        with self.assertRaises(RuntimeError):
            lab.move(rpc, "unused", 2, 70, 10)
        rpc.rg_grip.assert_not_called()

    @patch.object(lab, "dashboard_gate")
    def test_invalid_feedback_after_command_attempts_stop(self, gate):
        rpc = Mock()
        rpc.rg_grip.return_value = 0
        rpc.rg_stop.return_value = 0
        rpc.rg_get_all_variables.side_effect = [state(), state(width=-17.1), state()]
        with self.assertRaises(RuntimeError):
            lab.move(rpc, "unused", 2, 70, 10)
        rpc.rg_stop.assert_called_once_with(2)

    @patch.object(lab, "dashboard_gate")
    def test_cancel_before_command_confirms_stop(self, gate):
        rpc = Mock()
        rpc.rg_get_all_variables.return_value = state()
        rpc.rg_stop.return_value = 0
        with self.assertRaises(lab.MotionCanceled):
            lab.move(rpc, "unused", 2, 70, 10, canceled=lambda: True)
        rpc.rg_grip.assert_not_called()
        rpc.rg_stop.assert_called_once_with(2)

    @patch.object(lab, "dashboard_gate")
    def test_stop_rejection_is_not_clean_cancel(self, gate):
        rpc = Mock()
        rpc.rg_get_all_variables.return_value = state()
        rpc.rg_stop.return_value = -1
        with self.assertRaisesRegex(RuntimeError, "could not be confirmed"):
            lab.move(rpc, "unused", 2, 70, 10, canceled=lambda: True)

    @patch.object(lab.time, "sleep")
    @patch.object(lab, "dashboard_gate")
    def test_completion_needs_three_settled_samples(self, gate, sleep):
        rpc = Mock()
        rpc.rg_grip.return_value = 0
        rpc.rg_get_all_variables.side_effect = [state(), state(width=65, busy=True, status=1),
                                               state(width=70), state(width=70), state(width=70)]
        self.assertEqual(lab.move(rpc, "unused", 2, 70, 10)["width"], 70)
        self.assertEqual(rpc.rg_get_all_variables.call_count, 5)
        rpc.rg_grip.assert_called_once_with(2, 70.0, 10.0)


if __name__ == "__main__":
    unittest.main()
