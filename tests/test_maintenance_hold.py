from dataclasses import asdict
import json
from types import SimpleNamespace as NS
import unittest
import uuid

from orbital.maintenance_hold import MAX_HOLDS, state
from orbital.motion import MotionGuard
import test_supervisor_arm as nav_fixture

Message = nav_fixture.Message


def owner():
    return {key: uuid.uuid4().hex for key in ('owner_run_id', 'mission_id', 'action_id', 'token')}


def request(guard, ident, op='acquire', **overrides):
    return json.dumps({'schema_version': 1, 'guard_run_id': guard.maintenance.run_id,
                      'command_id': uuid.uuid4().hex, 'op': op, 'identity': ident, **overrides})


class GuardHoldTests(unittest.TestCase):
    def setUp(self):
        self.guard = MotionGuard()
        self.guard.feedback(0., 0., 10., 10.)
        self.owner = owner()

    def send(self, op='acquire', ident=None, **kwargs):
        return self.guard.maintenance_command(request(self.guard, ident or self.owner, op, **kwargs), 10.)

    def test_acquire_stops_and_blocks_even_explicit_reset(self):
        self.assertTrue(self.guard.reset(10.))
        self.guard.feedback(0., 0., 10.1, 10.1)
        self.assertTrue(self.guard.command(.1, 0., 10.1, 10.1))
        self.assertTrue(self.send()['accepted'])
        self.assertEqual(self.guard.output(10.1), (0., 0.))
        self.assertFalse(self.guard.reset(10.1))
        self.assertFalse(self.guard.command(.1, 0., 10.2, 10.1))

    def test_client_loss_and_clock_reset_do_not_expire_hold(self):
        self.send()
        self.guard.feedback(0., 0., 0., 1000.)
        self.assertFalse(self.guard.reset(1000.))
        self.assertEqual(self.guard.maintenance.active, self.owner)
        self.assertEqual(self.guard.output(10000.), (0., 0.))

    def test_release_is_exact_stationary_and_never_arms(self):
        self.send()
        self.assertFalse(self.send('release', ident=owner())['accepted'])
        self.guard.speed = (.1, 0.)
        self.assertEqual(self.send('release')['error_code'], 'stationary_required')
        self.guard.speed = (0., 0.)
        self.assertTrue(self.send('release')['accepted'])
        self.assertIsNone(self.guard.maintenance.active)
        self.assertTrue(self.guard.latched)
        self.assertEqual(self.guard.output(10.), (0., 0.))
        self.assertTrue(self.guard.reset(10.))
        self.assertFalse(self.guard.command(.1, 0., 10., 10.))

    def test_replay_cannot_reacquire_or_release_a_new_owner(self):
        self.send()
        self.assertEqual(self.send()['status'], 'held')
        self.send('release')
        second = owner()
        self.assertTrue(self.send(ident=second)['accepted'])
        self.assertEqual(self.send()['status'], 'released')
        self.assertEqual(self.send('release')['status'], 'released')
        self.assertEqual(self.guard.maintenance.active, second)
        self.assertTrue(self.guard.latched)

    def test_release_before_delayed_acquire_permanently_records_cancellation(self):
        self.assertEqual(self.send('release')['status'], 'released')
        self.assertEqual(self.send()['status'], 'released')
        self.assertIsNone(self.guard.maintenance.active)
        self.assertEqual(self.guard.maintenance.last_released, self.owner)
        self.assertTrue(self.guard.latched)

    def test_conflict_wrong_run_unknown_and_overflow_do_not_mutate(self):
        self.send()
        changed = {**self.owner, 'mission_id': uuid.uuid4().hex}
        self.assertEqual(self.send(ident=changed)['error_code'], 'hold_conflict')
        self.assertEqual(self.send(guard_run_id=uuid.uuid4().hex)['error_code'], 'wrong_run')
        self.assertEqual(self.send(ident=owner())['error_code'], 'busy')
        self.send('release')
        for _ in range(MAX_HOLDS-1):
            ident = owner()
            self.assertTrue(self.send(ident=ident)['accepted'])
            self.assertTrue(self.send('release', ident=ident)['accepted'])
        self.assertEqual(self.send(ident=owner())['error_code'], 'history_full')
        self.assertEqual(self.send('query')['status'], 'released')
        self.assertIsNone(self.guard.maintenance.active)

    def test_malformed_or_duplicate_fields_never_release(self):
        self.send()
        for text in ('{}', '[]', '{"op":"release","op":"query"}', 'x'*4097,
                     request(self.guard, self.owner, 'release', schema_version=True)):
            reply = self.guard.maintenance_command(text, 10.)
            self.assertFalse(reply['accepted'])
            self.assertEqual(self.guard.maintenance.active, self.owner)
        snapshot = state(self.guard.maintenance.snapshot(10_000_000_000))
        snapshot['active']['token'] = '0'*32
        self.assertEqual(self.guard.maintenance.active, self.owner)


class MaintenanceSupervisorTests(unittest.TestCase):
    setUp = nav_fixture.SupervisorArmTests.setUp
    health = nav_fixture.SupervisorArmTests.health
    guard = nav_fixture.SupervisorArmTests.guard
    command = nav_fixture.SupervisorArmTests.command
    reserve = nav_fixture.SupervisorArmTests.reserve
    arm = nav_fixture.SupervisorArmTests.arm

    def hold(self):
        self.motion = MotionGuard()
        self.ident = owner()
        self.motion.feedback(0., 0., self.supervisor.sim_time, self.now)
        self.motion.maintenance_command(request(self.motion, self.ident), self.now)
        self.states = []
        self.supervisor.nav_state_pub = NS(publish=lambda msg: self.states.append(json.loads(msg.data)))
        self.held_guard()

    def held_guard(self):
        self.supervisor.on_guard(Message(data=json.dumps({
            'latched': self.motion.latched,
            'maintenance': self.motion.maintenance.snapshot(round(self.supervisor.sim_time*1e9))})))

    def settle(self):
        for _ in range(5):
            self.health(dt=.05)
            self.held_guard()
            self.supervisor.tick()

    def test_idle_hold_blocks_reservation_until_exact_release(self):
        self.hold()
        self.settle()
        self.assertTrue(self.states[-1]['maintenance_quiescent'])
        self.assertFalse(self.command('request', waypoint='pump'))
        self.motion.feedback(0., 0., self.supervisor.sim_time, self.now)
        self.assertTrue(self.motion.maintenance_command(request(self.motion, self.ident, 'release'), self.now)['accepted'])
        self.held_guard()
        self.supervisor.tick()
        self.assertFalse(self.states[-1]['maintenance_quiescent'])
        self.assertTrue(self.command('request', waypoint='pump'))
        self.assertEqual(self.supervisor.session.client.calls, [])

    def test_pending_arm_cannot_be_quiescent_or_dispatch_after_hold(self):
        target = self.reserve()
        future = self.arm(target)
        self.hold()
        self.settle()
        self.assertFalse(self.states[-1]['maintenance_quiescent'])
        self.assertIs(self.supervisor.arm_future, future)
        future.set_result(NS(success=True))
        self.settle()
        self.assertTrue(self.states[-1]['maintenance_quiescent'])
        self.assertEqual(self.supervisor.session.client.calls, [])
        self.assertFalse(self.command('arm', target=asdict(target)))

    def test_missing_hold_or_restarted_guard_is_sticky(self):
        for mode in ('missing', 'restart'):
            with self.subTest(mode=mode):
                # Each case gets a fresh real supervisor and its own callbacks.
                self.setUp()
                self.hold()
                self.settle()
                packet = {'latched': True}
                if mode == 'restart':
                    packet['maintenance'] = MotionGuard().maintenance.snapshot(round(self.supervisor.sim_time*1e9))
                self.supervisor.on_guard(Message(data=json.dumps(packet)))
                self.settle()
                self.assertTrue(self.supervisor.maintenance_error)
                self.assertFalse(self.states[-1]['maintenance_quiescent'])
                self.assertFalse(self.command('request', waypoint='pump'))


if __name__ == '__main__':
    unittest.main()
