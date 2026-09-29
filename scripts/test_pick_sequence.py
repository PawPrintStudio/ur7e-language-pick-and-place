"""The shared pick plan and its split into pendant handoff segments (no ROS)."""
import unittest

from pick_sequence import (APPROACH_ABOVE, CLOSE, DESCEND, GOTO_NAMED, LIFT, OPEN,
                           RETREAT, Gripper, Motion, pick_place_steps, split_for_handoff)

PICK = (0.45, -0.15, 0.08)
PLACE = (0.45, 0.20, 0.08)


class PickSequenceTest(unittest.TestCase):
    def test_gripper_steps_bracket_the_carry(self):
        steps = pick_place_steps(PICK, PLACE)
        grips = [s.action for s in steps if isinstance(s, Gripper)]
        self.assertEqual(grips, [OPEN, CLOSE, OPEN])
        close = steps.index(Gripper(CLOSE))
        self.assertEqual(steps[close - 1].primitive, DESCEND)
        self.assertEqual(steps[close + 1].primitive, LIFT)

    def test_split_matches_pendant_program(self):
        before_close, before_open, after_open = split_for_handoff(pick_place_steps(PICK, PLACE))
        self.assertEqual([m.primitive for m in before_close], [GOTO_NAMED, APPROACH_ABOVE, DESCEND])
        self.assertEqual(before_close[1].xyz, PICK)
        self.assertEqual([m.primitive for m in before_open], [LIFT, APPROACH_ABOVE, DESCEND])
        self.assertEqual(before_open[1].xyz, PLACE)
        self.assertEqual([m.primitive for m in after_open], [RETREAT, GOTO_NAMED, GOTO_NAMED])
        self.assertEqual(after_open[-1].named_pose, "home")

    def test_split_rejects_plans_the_pendant_cannot_run(self):
        home = Motion(GOTO_NAMED, named_pose="home")
        for grips in ([], [CLOSE], [CLOSE, OPEN, CLOSE], [CLOSE, OPEN, OPEN]):
            with self.subTest(grips=grips):
                with self.assertRaises(ValueError):
                    split_for_handoff([home] + [Gripper(g) for g in grips])

    def test_rejects_bad_targets(self):
        with self.assertRaises(ValueError):
            pick_place_steps((0.4, float("nan"), 0.1), PLACE)
        with self.assertRaises(ValueError):
            pick_place_steps(PICK, PLACE, hover=0.0)


if __name__ == "__main__":
    unittest.main()
