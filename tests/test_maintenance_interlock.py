"""Hold observer consumes actual guard and supervisor snapshots."""
import copy
import json
from types import SimpleNamespace as NS
import unittest

from orbital.inspection_protocol import canonical
from orbital.maintenance_interlock import HoldEvidence
from orbital.motion import MotionGuard
import test_maintenance_hold as hold_fixture
import test_supervisor_arm as nav_fixture


class HoldEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.f = nav_fixture.SupervisorArmTests()
        self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.motion = MotionGuard()
        self.ident = hold_fixture.owner()
        self.nav = None
        self.guard = None
        self.available = True
        self.sent = []
        self.f.supervisor.nav_state_pub = NS(publish=lambda msg:setattr(self, 'nav', json.loads(msg.data)))
        def authority():
            if not self.available: raise ValueError('source unavailable')
        def publish(text):
            self.sent.append(json.loads(text))
            self.motion.maintenance_command(text, self.f.now)
        self.reader = HoldEvidence(now=lambda:self.f.now,
            sim_ns=lambda:round(self.f.supervisor.sim_time*1e9), publish=publish, authority=authority)
        self.feed(); self.feed()

    def feed(self, delta=.05):
        self.f.health(dt=delta)
        self.motion.feedback(0., 0., self.f.supervisor.sim_time, self.f.now)
        self.guard = {'latched': self.motion.latched,
                      'maintenance': self.motion.maintenance.snapshot(round(self.f.supervisor.sim_time*1e9))}
        self.f.supervisor.on_guard(nav_fixture.Message(data=canonical(self.guard)))
        self.f.supervisor.tick()
        self.reader.on_guard(canonical(self.guard)); self.reader.on_navigation(canonical(self.nav))

    def acquire(self):
        self.assertTrue(self.reader.available())
        self.reader.begin(self.ident)
        self.assertFalse(self.reader.held())
        for _ in range(5): self.feed()
        self.assertTrue(self.reader.held())

    def test_exact_release_needs_new_guard_and_navigation_snapshots(self):
        self.acquire()
        self.assertFalse(self.reader.release())
        self.assertIsNotNone(self.reader.owner)
        self.feed()
        self.assertTrue(self.reader.release())
        self.assertIsNone(self.reader.owner)
        self.assertTrue(self.motion.latched)
        self.assertEqual([v['op'] for v in self.sent], ['acquire', 'release'])

    def test_gap_requires_new_pair_and_never_expires_guard_ownership(self):
        self.acquire(); self.feed(.201)
        self.assertFalse(self.reader.held())
        self.assertEqual(self.motion.maintenance.active, self.ident)
        self.feed()
        self.assertTrue(self.reader.held())

    def test_duplicate_guard_sequence_permanently_invalidates_observer(self):
        self.acquire()
        self.reader.on_guard(canonical(self.guard)); self.feed()
        with self.assertRaises(ValueError): self.reader.held()
        self.assertEqual(self.motion.maintenance.active, self.ident)

    def test_navigation_restart_and_source_loss_prevent_release(self):
        self.acquire()
        nav = copy.deepcopy(self.nav)
        nav['run_id'] = 'replacement'
        self.reader.on_navigation(canonical(nav))
        self.feed()
        with self.assertRaises(ValueError): self.reader.release()
        self.assertEqual(self.motion.maintenance.active, self.ident)

    def test_lost_publisher_cannot_resume_an_old_hold(self):
        self.acquire(); self.available = False; self.feed()
        self.available = True; self.feed()
        with self.assertRaises(ValueError): self.reader.held()
        self.assertEqual(self.motion.maintenance.active, self.ident)

    def test_initial_zero_clock_waits_but_regression_after_start_is_fatal(self):
        stamp = 0
        reader = HoldEvidence(now=lambda:10., sim_ns=lambda:stamp, publish=lambda _:None, authority=lambda:None)
        with self.assertRaises(ValueError): reader.clocks()
        self.assertIsNone(reader.failure)
        stamp = 1000000000
        self.assertEqual(reader.clocks(), (10., stamp))
        stamp = 0
        with self.assertRaises(ValueError): reader.clocks()
        self.assertIsNotNone(reader.failure)
