import unittest
from orbital.navigation import NavigationPolicy, Target


class NavigationPolicyTests(unittest.TestCase):
    def ready(self):
        p=NavigationPolicy(['pump','cooler','power_unit'])
        self.assertTrue(p.observe(sim_time=1.,valid=True,now=10.))
        return p

    def active(self):
        p=self.ready();t=p.request('pump',now=10.01)
        self.assertTrue(p.arm(t,stationary=True,now=10.02))
        return p,t

    def test_startup_allowlist_and_explicit_arm(self):
        p=NavigationPolicy(['pump'])
        self.assertTrue(p.stop_required)
        self.assertIsNone(p.request('pump',now=1.))
        p.observe(sim_time=1.,valid=True,now=1.1)
        self.assertIsNone(p.request('unknown',now=1.11))
        t=p.request('pump',now=1.12)
        self.assertIsNotNone(t);self.assertTrue(p.stop_required)
        self.assertFalse(p.arm(t,stationary=False,now=1.13))
        self.assertTrue(p.arm(t,stationary=True,now=1.14))
        self.assertFalse(p.stop_required)
        self.assertIsNone(p.request('pump',now=1.15))

    def test_late_health_cannot_hide_expired_target(self):
        p,t=self.active()
        p.observe(sim_time=2.,valid=True,now=10.7)
        self.assertTrue(p.stop_required)
        self.assertEqual(p.cancel_pending,t)
        self.assertIsNone(p.target)
        self.assertFalse(p.arm(t,stationary=True,now=10.71))
        self.assertIsNone(p.request('cooler',now=10.72))
        self.assertTrue(p.confirm_terminal(t))
        new=p.request('cooler',now=10.73)
        self.assertGreater(new.epoch,t.epoch)
        self.assertTrue(p.stop_required)
        self.assertTrue(p.arm(new,stationary=True,now=10.74))

    def test_reset_and_initial_pose_invalidate_identity(self):
        for cause in ('reset','initial_pose'):
            with self.subTest(cause=cause):
                p,t=self.active()
                if cause=='reset':p.observe(sim_time=0.,valid=True,now=10.1)
                else:p.replace_initial_pose()
                self.assertTrue(p.stop_required)
                self.assertFalse(p.finish(t,now=10.11))
                self.assertEqual(p.cancel_pending,t)
                p.observe(sim_time=2.,valid=True,now=10.12)
                self.assertIsNone(p.request('pump',now=10.13))
                self.assertTrue(p.confirm_terminal(t))
                new=p.request('pump',now=10.14)
                self.assertNotEqual(new,t)
                self.assertFalse(p.arm(t,stationary=True,now=10.15))

    def test_replayed_health_does_not_extend_lease(self):
        p,t=self.active()
        self.assertFalse(p.observe(sim_time=1.,valid=True,now=10.4))
        p.tick(10.5)
        self.assertTrue(p.stop_required);self.assertEqual(p.cancel_pending,t)

    def test_wrong_or_previous_run_terminal_result_cannot_release_interlock(self):
        p,t=self.active();p.cancel()
        other=Target('old-run',t.epoch,t.sequence,t.waypoint)
        self.assertFalse(p.confirm_terminal(other))
        self.assertFalse(p.confirm_terminal(None))
        self.assertEqual(p.cancel_pending,t)
        self.assertTrue(p.confirm_terminal(t))
        self.assertFalse(p.confirm_terminal(t))
        self.assertTrue(p.stop_required)

    def test_invalid_health_and_backward_wall_clock_stop(self):
        for sample in ({'sim_time':float('nan'),'valid':True,'now':10.1},
                       {'sim_time':2.,'valid':False,'now':10.1},
                       {'sim_time':2.,'valid':True,'now':9.9}):
            with self.subTest(sample=sample):
                p,t=self.active();self.assertFalse(p.observe(**sample))
                self.assertTrue(p.stop_required);self.assertEqual(p.cancel_pending,t)

    def test_finish_does_not_arm_next_goal_or_accept_late_success(self):
        p,t=self.active()
        self.assertTrue(p.finish(t,now=10.1));self.assertTrue(p.stop_required)
        new=p.request('cooler',now=10.11)
        self.assertNotEqual(t,new)
        self.assertFalse(p.finish(t,now=10.12))
        self.assertFalse(p.finish(new,now=10.13))
        self.assertTrue(p.arm(new,stationary=True,now=10.14))
        p.cancel();self.assertFalse(p.finish(new,now=10.15))
        self.assertEqual(p.cancel_pending,new)


if __name__=='__main__':unittest.main()
