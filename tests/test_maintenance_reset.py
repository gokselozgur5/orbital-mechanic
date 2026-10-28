"""Bounded reset uses public history, exact ownership and the real station."""
import copy
import hashlib
import uuid
from unittest.mock import patch

from orbital.inspection_owner import InspectionOwner
from orbital.inspection_protocol import canonical
from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.maintenance_owner import MaintenanceOwner
from orbital.maintenance_protocol import result, state
from orbital.machines import Fault
from orbital.station_control import EffectRejected
from orbital.isolation_trace import IsolationTrace
from test_machine_registry import RegistryFixture
import test_inspection_owner as capture
import test_maintenance_owner as isolation


class ResetFixture(RegistryFixture):
    def setUp(self):
        super().setUp()
        self.clock, self.live, self.hold = capture.Clock(), capture.Guard(), isolation.Hold()
        self.live.current.update(distance_m=.6, stationarity={'max_linear_m_s': .001})
        self.model = MachineTelemetry(MachineRegistry.load(self.root), Fault.LATCHED)
        self.live.identity = lambda:{'registry_identity': self.model.registry.identity, 'authority_identity': 'b'*64}
        begin = self.hold.begin
        def acquire(token):
            self.hold.acknowledged = self.hold.released = False
            self.hold.release_at = None
            begin(token)
        self.hold.begin = acquire
        self.workers, self.results = [], []
        def worker(payload):
            child = capture.Worker(payload)
            child.reply.update(observation_id=uuid.uuid4().hex)
            self.workers.append(child)
            return child
        self.owner = MaintenanceOwner(self.root, self.model, self.live, self.hold, worker,
            now=lambda:self.clock.wall, sim_ns=lambda:self.clock.sim, on_result=self.results.append,
            inspection_factory=lambda *a, **kw:InspectionOwner(*a, **kw, load=self.load), load=self.load)
        self.request = {'schema_version': 1, 'run_id': self.owner.run_id, 'mission_id': uuid.uuid4().hex,
            'action_id': uuid.uuid4().hex, 'command_id': uuid.uuid4().hex, 'op': 'isolate',
            'arguments': {'machine_id': 'pump', 'station_run_id': self.model.run_id,
                          'control_revision': 0, 'timeout_s': 10.}}

    def sample(self, delta=.5):
        self.clock.advance(delta)
        packet = self.model.sample(self.clock.sim)
        self.packet = packet
        self.owner.telemetry(canonical(packet))
        self.owner.tick()
        return packet

    def load(self, root, reference, action):
        packet = self.packet
        robot = {'within_settled_inspection_pose': True, **copy.deepcopy(self.live.current)}
        return {'machine_id': 'pump', 'observed_monotonic_s': self.clock.wall,
            'sample_stamp_ns': packet['stamp_ns'], 'observed_sim_stamp_ns': self.clock.sim,
            'registry_identity': self.model.registry.identity,
            'camera': {'stamp_ns': self.clock.sim, 'received_age_s': .02, 'info_received_age_s': .02,
                       'robot_state': robot}, 'robot_state': robot, 'telemetry_freshness': {'received_age_s': .03},
            'telemetry_source': {'schema_version': 2, 'run_id': packet['run_id'],
                'control_revision': packet['control']['revision'],
                'sample_sha256': hashlib.sha256(canonical(packet).encode()).hexdigest()},
            'measurements': {'voltage_v': packet['voltage_v'], **packet['machines']['pump']}}

    def send(self, request=None, op=None):
        value = copy.deepcopy(self.request if request is None else request)
        if op is not None: value.update(op=op, arguments={}, command_id=uuid.uuid4().hex)
        return self.owner.handle(canonical(value))

    def start(self, request):
        self.assertTrue(self.send(request)['accepted'])
        self.owner.tick()
        self.hold.acknowledged = True
        for _ in range(3): self.owner.tick()

    def apply(self):
        self.workers[-1].exited = True
        self.owner.tick()

    def finish(self):
        self.hold.released = True
        self.owner.tick()
        self.assertIsNone(self.owner.active)
        state(canonical(self.owner.state()))
        return result(self.results[-1])

    def isolated(self, fault=Fault.LATCHED):
        self.model.station.machines['pump'].fault = fault  # Evaluator scenario setup only.
        self.sample(.05); self.sample()
        self.start(self.request); self.apply()
        self.sample(); self.sample()
        value = self.finish()
        self.assertEqual(value['status'], 'succeeded')
        self.reset_request = {**self.request, 'op': 'reset', 'action_id': uuid.uuid4().hex,
            'command_id': uuid.uuid4().hex, 'arguments': {**self.request['arguments'],
                'control_revision': 1, 'isolation_action_id': self.request['action_id']}}
        return value

    def reset_complete(self):
        self.isolated()
        self.start(self.reset_request); self.apply()
        self.sample(); self.sample()
        return self.finish()


class ResetOwnerTests(ResetFixture):
    def test_same_model_one_reset_new_inspection_and_matching_public_effect(self):
        value = self.reset_complete()
        self.assertEqual(value['operation'], 'reset')
        self.assertEqual(value['schema_version'], 2)
        self.assertEqual(value['status'], 'succeeded')
        self.assertEqual(value['isolation'], self.results[0])
        self.assertNotEqual(value['observation']['observation_id'], self.results[0]['observation']['observation_id'])
        self.assertEqual(len(self.workers), 2)
        self.assertEqual(self.model.station.machines['pump'].reset_count, 1)
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertEqual(value['state_change']['effect'], self.model.last_applied)
        self.assertEqual(value['state_change']['verification']['control_revision'], 2)
        self.assertLess(len(canonical(self.owner.state()).encode()), 16384)

    def test_high_vibration_scenario_cannot_reset(self):
        self.isolated(Fault.JAM)
        self.assertEqual(self.send(self.reset_request)['error_code'], 'reset_not_eligible')
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertEqual(self.model.station.machines['pump'].reset_count, 0)
        self.assertEqual(len(self.workers), 1)

    def test_healthy_measurements_do_not_establish_reset_eligibility(self):
        self.isolated(Fault.NONE)
        self.assertEqual(self.send(self.reset_request)['error_code'], 'reset_not_eligible')

    def test_high_vibration_history_is_sticky_after_later_low_readings(self):
        self.model.station.machines['pump'].fault = Fault.JAM
        self.sample(.05)
        self.isolated(Fault.LATCHED)
        self.assertEqual(self.send(self.reset_request)['error_code'], 'reset_not_eligible')

    def test_missing_powered_history_is_not_repaired_by_new_samples(self):
        self.sample(.05)
        self.clock.advance(.5); self.model.sample(self.clock.sim)  # Deliberately dropped publication.
        self.isolated()
        self.assertEqual(self.send(self.reset_request)['error_code'], 'reset_not_eligible')
        self.assertEqual(self.owner.reset_evidence.failure, 'missing_powered_history')

    def test_malformed_history_after_isolation_refuses_reset(self):
        self.isolated()
        self.owner.telemetry('{"bad":1}')
        self.assertEqual(self.send(self.reset_request)['error_code'], 'reset_not_eligible')

    def test_inspection_must_resolve_to_retained_powered_measurements(self):
        real_load = self.owner.load
        def contradictory(*args):
            value = real_load(*args)
            value['measurements']['current_a'] += 1
            return value
        self.owner.load = contradictory
        self.isolated()
        self.assertEqual(self.send(self.reset_request)['error_code'], 'reset_not_eligible')

    def test_prior_identity_run_revision_and_mission_cannot_be_adopted(self):
        self.isolated()
        variants = [({**self.reset_request, 'mission_id': uuid.uuid4().hex}, 'isolation_required'),
                    ({**self.reset_request, 'run_id': uuid.uuid4().hex}, 'wrong_run')]
        for field, value, error in (('isolation_action_id', uuid.uuid4().hex, 'isolation_required'),
                ('machine_id', 'cooler', 'isolation_required'), ('control_revision', 0, 'wrong_station_state'),
                ('station_run_id', uuid.uuid4().hex, 'wrong_station_state')):
            variants.append(({**self.reset_request, 'arguments': {**self.reset_request['arguments'], field:value}}, error))
        for request, error in variants:
            with self.subTest(error=error): self.assertEqual(self.send(request)['error_code'], error)
        self.assertEqual(len(self.workers), 1)

    def test_cooldown_waits_under_hold_then_takes_new_inspection(self):
        self.model.station.machines['pump'].temperature_c = 41.
        self.isolated()
        self.start(self.reset_request)
        self.assertEqual(self.owner.active.phase, 'cooling')
        self.assertEqual(len(self.workers), 1)
        self.assertTrue(self.hold.held())
        for _ in range(8):
            self.sample()
            if self.owner.active.phase != 'cooling': break
        self.owner.tick()
        self.assertEqual(len(self.workers), 2)
        self.assertLessEqual(self.model.station.machines['pump'].temperature_c, 40.)
        self.apply(); self.sample(); self.sample()
        self.assertEqual(self.finish()['status'], 'succeeded')

    def test_cooldown_deadline_never_spends_attempt_or_releases_before_ack(self):
        self.model.station.machines['pump'].temperature_c = 50.
        self.isolated()
        self.reset_request['arguments']['timeout_s'] = .3
        self.start(self.reset_request)
        self.sample()
        self.assertIsNotNone(self.owner.active)
        self.assertEqual(len(self.workers), 1)
        self.assertEqual(self.finish()['status'], 'timed_out')
        self.assertTrue(self.model.reset_available('pump'))

    def test_cancel_before_capture_reap_keeps_hold_and_never_pulses(self):
        self.isolated(); self.start(self.reset_request)
        self.send(self.reset_request, 'cancel'); self.owner.tick()
        self.assertIsNone(self.hold.release_at)
        self.assertTrue(self.model.reset_available('pump'))
        self.apply()
        value = self.finish()
        self.assertEqual(value['status'], 'cancelled')
        self.assertEqual(value['state_change']['disposition'], 'not_applied')
        self.assertFalse(self.model.station.machines['pump'].powered)

    def test_cancel_after_pulse_requires_new_verified_samples_without_restore(self):
        self.isolated(); self.start(self.reset_request); self.apply()
        self.send(self.reset_request, 'cancel'); self.owner.tick()
        self.assertIsNone(self.hold.release_at)
        self.sample(); self.sample()
        value = self.finish()
        self.assertEqual(value['status'], 'cancelled')
        self.assertEqual(value['state_change']['disposition'], 'applied')
        self.assertEqual(self.model.station.machines['pump'].reset_count, 1)
        self.assertFalse(self.model.station.machines['pump'].powered)

    def test_final_artifact_delay_prevents_reset_without_spending_attempt(self):
        self.isolated(); self.start(self.reset_request)
        real_load = self.owner.load
        def late(*args):
            value = real_load(*args); self.clock.advance(.201); return value
        self.owner.load = late
        self.apply()
        self.assertEqual(self.finish()['error_code'], 'precondition_failed')
        self.assertTrue(self.model.reset_available('pump'))

    def test_late_pose_loss_is_rechecked_before_reset(self):
        self.isolated(); self.start(self.reset_request)
        real_load = self.owner.load
        def lost(*args):
            value = real_load(*args); self.live.invalid = True; return value
        self.owner.load = lost
        self.apply()
        self.assertEqual(self.finish()['state_change']['disposition'], 'not_applied')
        self.assertTrue(self.model.reset_available('pump'))

    def test_actual_model_heat_after_inspection_refuses_reset(self):
        self.isolated(); self.start(self.reset_request)
        real_load = self.owner.load
        def heated(*args):
            value = real_load(*args); self.model.station.machines['pump'].temperature_c = 41.; return value
        self.owner.load = heated
        self.apply()
        self.assertEqual(self.finish()['state_change']['disposition'], 'not_applied')
        self.assertTrue(self.model.reset_available('pump'))

    def test_unknown_pulse_keeps_ownership_and_reserved_budget(self):
        self.isolated(); self.start(self.reset_request)
        real_actuator = self.model.station.intervene
        def unknown(*args, **kwargs):
            real_actuator(*args, **kwargs)
            raise RuntimeError('controlled failure after actual reset')
        with patch.object(self.model.station, 'intervene', side_effect=unknown): self.apply()
        self.assertEqual(self.owner.active.phase, 'unresolved')
        self.assertEqual(len(self.results), 1)
        self.assertIsNone(self.hold.release_at)
        self.assertFalse(self.model.reset_available('pump'))
        self.assertEqual(self.model.station.machines['pump'].reset_count, 1)
        self.assertEqual(self.model.control_revision, 1)

    def test_replay_and_new_action_cannot_repeat_reset(self):
        value = self.reset_complete()
        reply = self.send({**self.reset_request, 'command_id': uuid.uuid4().hex})
        self.assertEqual(reply['action']['result'], value)
        second = {**self.reset_request, 'action_id': uuid.uuid4().hex,
                  'arguments': {**self.reset_request['arguments'], 'control_revision': 2}}
        self.assertEqual(self.send(second)['error_code'], 'reset_attempt_exhausted')
        changed = {**second, 'action_id': self.reset_request['action_id']}
        self.assertEqual(self.send(changed)['error_code'], 'action_conflict')
        self.assertEqual(len(self.workers), 2)
        self.assertEqual(self.model.station.machines['pump'].reset_count, 1)

    def test_station_reset_during_new_capture_invalidates_old_generation(self):
        self.isolated(); self.start(self.reset_request)
        self.model.sample(100000000)
        self.apply()
        self.assertEqual(self.finish()['error_code'], 'station_run_changed')
        self.assertNotEqual(self.model.run_id, self.reset_request['arguments']['station_run_id'])

    def test_post_pulse_wrong_effect_cannot_release_hold(self):
        self.isolated(); self.start(self.reset_request); self.apply()
        for _ in range(2):
            self.clock.advance(.5)
            packet = self.model.sample(self.clock.sim)
            packet['control']['last_applied']['action_id'] = 'f'*32
            self.owner.telemetry(canonical(packet)); self.owner.tick()
        self.assertEqual(self.owner.active.phase, 'unresolved')
        self.assertEqual(self.owner.active.stop_reason[1], 'reset_not_verified')
        self.assertIsNone(self.hold.release_at)

    def test_post_pulse_expiry_cannot_complete_cached_verification(self):
        self.isolated(); self.start(self.reset_request); self.apply()
        self.sample(); self.sample()
        self.assertIsNotNone(self.hold.release_at)
        self.clock.advance(1.)
        self.hold.released = True; self.owner.tick()
        self.assertIsNotNone(self.owner.active)
        self.assertEqual(len(self.results), 1)

    def test_both_post_reset_samples_must_remain_fresh_through_release(self):
        for wall, sim in ((.6, .6), (.6, 0.), (0., .6)):
            with self.subTest(wall=wall, sim=sim):
                f = ResetFixture(); f.setUp()
                try:
                    f.isolated(); f.start(f.reset_request); f.apply(); f.sample(); f.sample()
                    f.clock.wall += wall; f.clock.sim += round(sim*1e9)
                    f.hold.released = True; f.owner.tick()
                    self.assertIsNotNone(f.owner.active)
                    self.assertEqual(len(f.results), 1)
                finally:
                    f.doCleanups()

    def test_post_pulse_pair_cannot_hide_first_contradictory_measurement(self):
        self.isolated(); self.start(self.reset_request); self.apply()
        self.clock.advance(.5)
        packet = self.model.sample(self.clock.sim)
        packet['machines']['pump']['powered'] = True
        self.owner.telemetry(canonical(packet)); self.owner.tick()
        self.sample()
        self.assertEqual(self.owner.active.phase, 'unresolved')
        self.assertEqual(self.owner.active.stop_reason[1], 'reset_not_verified')
        self.assertIsNone(self.hold.release_at)

    def test_reset_result_rejects_missing_or_mismatched_prerequisites(self):
        value = self.reset_complete()
        for key, changed in (('schema_version', 1), ('eligibility', None), ('isolation', value),
                             ('mission_id', 'f'*32), ('observation', None)):
            bad = copy.deepcopy(value); bad[key] = changed
            with self.subTest(key=key), self.assertRaises(ValueError): result(bad)
        for key, changed in (('temperature_c', 40.01), ('control_revision', 1), ('powered', True)):
            bad = copy.deepcopy(value); bad['state_change']['verification'][key] = changed
            with self.subTest(key=key), self.assertRaises(ValueError): result(bad)

    def test_model_exact_replay_and_used_budget_survive_new_caller(self):
        value = self.reset_complete()
        effect = value['state_change']['effect']
        context = {k:effect[k] for k in ('owner_run_id', 'mission_id', 'action_id', 'request_sha256', 'machine_id')}
        def forbidden(): raise AssertionError('replay reached actuator gate')
        self.assertEqual(self.model.reset_machine(context, self.model.run_id, 1,
            isolation_action_id=self.request['action_id'], admission=forbidden), effect)
        with self.assertRaises(EffectRejected):
            self.model.reset_machine({**context, 'owner_run_id': uuid.uuid4().hex, 'action_id': uuid.uuid4().hex},
                self.model.run_id, 2, isolation_action_id=self.request['action_id'], admission=forbidden)

    def test_long_observation_gap_prevents_later_eligibility(self):
        self.sample(.05); self.sample(1.)
        self.isolated()
        self.assertEqual(self.send(self.reset_request)['error_code'], 'reset_not_eligible')

    def test_uncommitted_unknown_pulse_still_reserves_attempt(self):
        self.isolated(); self.start(self.reset_request)
        with patch.object(self.model.station, 'intervene', side_effect=RuntimeError('unknown actuator boundary')):
            self.apply()
        self.assertEqual(self.model.station.machines['pump'].reset_count, 0)
        self.assertFalse(self.model.reset_available('pump'))
        self.assertEqual(self.owner.active.phase, 'unresolved')

    def test_isolation_observer_refuses_reset_terminal_as_isolation(self):
        self.isolated(); self.start(self.reset_request)
        self.send(self.reset_request, 'cancel'); self.apply()
        value = self.finish()
        target = {k:value[k] for k in ('run_id', 'mission_id', 'action_id', 'machine_id')}
        observer = IsolationTrace(self.model.registry, target)
        # Exercise the assessment boundary with a valid cancelled-reset result;
        # even an otherwise exact request digest cannot make it isolation proof.
        observer.request, observer.terminal = self.reset_request, value
        with self.assertRaisesRegex(ValueError, 'trace_result_binding'):
            observer.assess(round(self.clock.wall*1e9), self.clock.sim)
