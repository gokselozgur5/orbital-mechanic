import math
import unittest
from orbital.motion import MotionGuard


class MotionTests(unittest.TestCase):
    def armed(self):
        g = MotionGuard()
        g.feedback(0, 0, 10, 1)
        self.assertTrue(g.reset(1))
        g.feedback(0, 0, 10.1, 1.1)
        return g

    def test_startup_requires_fresh_stationary_feedback(self):
        g = MotionGuard()
        self.assertFalse(g.command(.1, 0, 1, 1))
        self.assertFalse(g.reset(1))
        g.feedback(.1, 0, 1, 1)
        self.assertFalse(g.reset(1))
        self.assertEqual(g.output(1),(0,0))

    def test_all_motion_directions_and_speed_bounds(self):
        for linear, angular in [(.2,0),(-.2,0),(0,.5),(0,-.5),(.1,.2)]:
            g = self.armed()
            self.assertTrue(g.command(linear,angular,10.1,1.1))
            self.assertEqual(g.output(1.2),(linear,angular))

    def test_timeout_latches_and_late_commands_cannot_restart(self):
        g = self.armed(); g.command(.1,0,10.1,1.1)
        self.assertEqual(g.output(1.44),(.1,0))
        self.assertEqual(g.output(1.451),(0,0))
        self.assertEqual(g.reason,'command_timeout')
        self.assertFalse(g.command(.1,0,10.2,1.46))

    def test_late_command_cannot_hide_timeout_when_timer_callback_is_delayed(self):
        g = self.armed(); g.command(.1,0,10.1,1.1)
        g.feedback(.1,0,10.5,1.5)
        self.assertFalse(g.command(.1,0,10.5,1.5))
        self.assertEqual(g.reason,'command_timeout')
        self.assertEqual(g.output(1.5),(0,0))

    def test_stop_then_reset_does_not_replay_old_velocity(self):
        g = self.armed(); g.command(.1,0,10.1,1.1); g.stop()
        self.assertEqual(g.output(1.11),(0,0))
        g.feedback(0,0,10.2,1.2)
        self.assertTrue(g.reset(1.2))
        self.assertEqual(g.output(1.2),(0,0))
        self.assertFalse(g.command(.1,0,10.1,1.21))
        self.assertTrue(g.latched)

    def test_invalid_values_fail_closed(self):
        for linear, angular in [(math.nan,0),(math.inf,0),(.201,0),(0,.501)]:
            g = self.armed(); g.command(.1,0,10.1,1.1)
            self.assertFalse(g.command(linear,angular,10.15,1.15))
            self.assertEqual(g.output(1.16),(0,0))

    def test_frozen_feedback_cannot_be_refreshed_by_duplicate_samples(self):
        g = self.armed()
        g.feedback(0,0,10.1,1.59)
        self.assertEqual(g.output(1.601),(0,0))
        self.assertEqual(g.reason,'feedback_timeout')

    def test_clock_reset_invalidates_motion(self):
        g = self.armed(); g.command(.1,0,10.1,1.1)
        g.feedback(0,0,.1,1.2)
        self.assertTrue(g.latched)
        self.assertEqual(g.reason,'simulation_reset')

    def test_source_time_and_monotonic_time_reversal(self):
        for stamp in (9,20):
            g = self.armed()
            self.assertFalse(g.command(.1,0,stamp,1.1))
        g = self.armed(); g.command(.1,0,10.1,1.1)
        self.assertEqual(g.output(.9),(0,0))

    def test_nonfinite_feedback_cannot_arm(self):
        g = self.armed(); g.feedback(math.nan,0,10.2,1.2)
        self.assertFalse(g.reset(1.2))


if __name__ == '__main__': unittest.main()
