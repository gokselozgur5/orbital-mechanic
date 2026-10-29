"""Real station and inspection owner with controlled capture/hold scheduling."""
import copy
import uuid

from orbital.inspection_owner import InspectionOwner
from orbital.inspection_protocol import canonical
from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.maintenance_owner import MaintenanceOwner
from orbital.machines import Fault
from test_machine_registry import RegistryFixture
import test_inspection_owner as capture_fixture


class Hold:
    def __init__(self):
        self.owner = None
        self.acknowledged = self.released = False
        self.release_at = None
    def available(self): return self.owner is None
    def begin(self, token): self.owner = copy.deepcopy(token)
    def held(self): return self.owner is not None and self.acknowledged and not self.released
    def release(self):
        self.release_at = 1.
        if self.released:
            self.owner = None
            return True
        return False


class MaintenanceOwnerTests(RegistryFixture):
    def setUp(self):
        super().setUp()
        self.clock, self.live, self.hold = capture_fixture.Clock(), capture_fixture.Guard(), Hold()
        self.live.current.update(distance_m=.6, stationarity={'max_linear_m_s': .001})
        self.model = MachineTelemetry(MachineRegistry.load(self.root), Fault.LATCHED)
        self.model.sample(self.clock.sim)
        self.workers, self.results = [], []
        self.clock.advance(.05)
        def worker(payload):
            value = capture_fixture.Worker(payload)
            self.workers.append(value)
            return value
        def inspector(*args, **kwargs):
            return InspectionOwner(*args, **kwargs, load=self.load)
        self.owner = MaintenanceOwner(self.root, self.model, self.live, self.hold, worker,
            now=lambda:self.clock.wall, sim_ns=lambda:self.clock.sim, on_result=self.results.append,
            inspection_factory=inspector, load=self.load)
        self.request = {'schema_version': 1, 'run_id': self.owner.run_id, 'mission_id': uuid.uuid4().hex,
            'action_id': uuid.uuid4().hex, 'command_id': uuid.uuid4().hex, 'op': 'isolate',
            'arguments': {'machine_id': 'pump', 'station_run_id': self.model.run_id,
                          'control_revision': 0, 'timeout_s': 5.}}

    def load(self, root, reference, action):
        robot = {'within_settled_inspection_pose': True, **copy.deepcopy(self.live.current)}
        return {'observed_monotonic_s': self.clock.wall, 'sample_stamp_ns': self.clock.sim,
            'observed_sim_stamp_ns': self.clock.sim, 'registry_identity': 'a'*64,
            'camera': {'stamp_ns': self.clock.sim, 'received_age_s': .02, 'info_received_age_s': .02,
                       'robot_state': robot}, 'robot_state': robot, 'telemetry_freshness': {'received_age_s': .03},
            'telemetry_source': {'schema_version': 2, 'run_id': self.request['arguments']['station_run_id'],
                                 'control_revision': 0}, 'measurements': {'powered': True}}

    def send(self, op=None, **changes):
        value = copy.deepcopy(self.request)
        if op is not None:
            value.update(op=op, arguments={})
        value.update(changes)
        return self.owner.handle(canonical(value))

    def start(self):
        self.assertTrue(self.send()['accepted'])
        self.owner.tick()
        self.assertEqual(self.workers, [])  # Acquisition ACK is required first.
        self.hold.acknowledged = True
        self.owner.tick(); self.owner.tick()
        self.assertEqual(len(self.workers), 1)
        return self.workers[0]

    def apply(self):
        worker = self.start()
        worker.exited = True
        self.owner.tick()
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertIsNotNone(self.owner.active.effect)

    def sample(self, delta=.5):
        self.clock.advance(delta)
        packet = self.model.sample(self.clock.sim)
        self.owner.telemetry(canonical(packet))
        self.owner.tick()
        return packet

    def finish(self):
        self.hold.released = True
        self.owner.tick()
        self.assertIsNone(self.owner.active)
        return self.results[-1]

    def test_success_waits_for_capture_reap_new_pair_and_exact_release(self):
        self.apply()
        self.assertEqual(self.results, [])
        self.sample()
        self.assertIsNone(self.hold.release_at)
        packet = self.sample()
        self.assertIsNotNone(self.hold.release_at)
        self.assertEqual(self.results, [])
        value = self.finish()
        self.assertEqual(value['status'], 'succeeded')
        self.assertEqual(value['state_change']['disposition'], 'applied')
        self.assertEqual(value['state_change']['effect'], packet['control']['last_applied'])
        self.assertEqual(value['state_change']['verification']['sequence'], packet['sequence'])
        self.assertEqual(self.live.ends, 1)

    def test_cancel_before_reap_never_applies_and_retains_live_child(self):
        worker = self.start()
        self.send('cancel'); self.owner.tick()
        self.assertEqual(self.results, [])
        self.assertIsNone(self.hold.release_at)
        self.assertEqual(worker.stopped, 1)
        worker.exited = True; self.owner.tick()
        value = self.finish()
        self.assertEqual(value['status'], 'cancelled')
        self.assertEqual(value['state_change']['disposition'], 'not_applied')
        self.assertTrue(self.model.station.machines['pump'].powered)

    def test_cancel_after_effect_reports_verified_applied_effect(self):
        self.apply(); self.send('cancel'); self.owner.tick()
        self.assertEqual(self.results, [])
        self.assertIsNone(self.hold.release_at)
        self.sample(); self.sample()
        value = self.finish()
        self.assertEqual(value['status'], 'cancelled')
        self.assertEqual(value['state_change']['disposition'], 'applied')
        self.assertFalse(self.model.station.machines['pump'].powered)

    def test_cancel_before_acquire_ack_waits_for_tombstone_confirmation(self):
        self.send(); self.send('cancel'); self.owner.tick()
        self.assertEqual(self.results, [])
        self.assertEqual(self.workers, [])
        self.assertIsNotNone(self.hold.release_at)
        self.assertEqual(self.finish()['state_change']['disposition'], 'not_applied')

    def test_deadline_before_capture_reap_prevents_effect(self):
        worker = self.start(); self.clock.advance(5.)
        self.owner.tick(); self.assertEqual(self.results, [])
        worker.exited = True; self.owner.tick()
        self.assertEqual(self.finish()['status'], 'timed_out')
        self.assertTrue(self.model.station.machines['pump'].powered)

    def test_capture_io_delay_and_late_live_loss_prevent_effect(self):
        worker = self.start(); worker.exited = True
        def delayed(*args):
            value = self.load(*args); self.clock.advance(.201); return value
        self.owner.load = delayed
        self.owner.tick()
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertEqual(self.finish()['error_code'], 'precondition_failed')

    def test_late_pose_loss_after_artifact_validation_prevents_effect(self):
        worker = self.start(); worker.exited = True
        def changed(*args):
            value = self.load(*args); self.live.invalid = True; return value
        self.owner.load = changed
        self.owner.tick()
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertEqual(self.finish()['state_change']['disposition'], 'not_applied')

    def test_exception_after_actual_actuator_keeps_unknown_effect_owned(self):
        worker = self.start(); worker.exited = True
        actual = self.model.station.intervene
        def uncertain(*args, **kwargs):
            actual(*args, **kwargs)
            raise OSError('controlled post-effect failure')
        self.model.station.intervene = uncertain
        self.owner.tick()
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertTrue(self.owner.active.unknown)
        self.assertEqual(self.owner.active.phase, 'unresolved')
        self.assertIsNone(self.hold.release_at)
        self.clock.advance(6.); self.send('cancel'); self.owner.tick()
        self.assertEqual(self.results, [])
        self.assertIsNotNone(self.hold.owner)
        self.assertEqual(self.send(action_id=uuid.uuid4().hex)['error_code'], 'busy')

    def test_expired_verification_cannot_finish_and_new_run_never_reconciles(self):
        self.apply(); self.sample(); self.sample()
        self.assertIsNotNone(self.owner.active.verification)
        self.clock.advance(1.01); self.hold.released = True; self.owner.tick()
        self.assertIsNotNone(self.owner.active)
        self.assertIsNone(self.owner.active.verification)
        self.model.sample(500000000)
        self.clock.sim = 600000000; self.sample(.5); self.sample(.5)
        self.assertEqual(self.owner.active.phase, 'unresolved')
        self.assertEqual(self.results, [])

    def test_replay_conflict_and_wrong_revision_do_not_repeat_an_effect(self):
        bad = {**self.request['arguments'], 'control_revision': 1}
        self.assertEqual(self.send(arguments=bad)['error_code'], 'wrong_station_state')
        self.apply(); self.sample(); self.sample(); value = self.finish()
        self.assertEqual(self.send(command_id=uuid.uuid4().hex)['action']['result'], value)
        bad = {**self.request['arguments'], 'machine_id': 'cooler'}
        self.assertEqual(self.send(arguments=bad)['error_code'], 'action_conflict')
        self.assertEqual(self.model.control_revision, 1)
        self.assertEqual(len(self.workers), 1)

    def test_exact_terminal_views_cannot_mutate_history(self):
        self.send(); self.send('cancel'); self.owner.tick(); self.finish()
        reply = self.send('query')
        reply['action']['result']['state_change']['disposition'] = 'applied'
        self.assertEqual(self.send('query')['action']['result']['state_change']['disposition'], 'not_applied')

    def test_station_reset_during_capture_rejects_old_mission(self):
        worker = self.start(); self.model.sample(500000000)
        worker.exited = True; self.owner.tick()
        self.assertEqual(self.finish()['error_code'], 'station_run_changed')
        self.assertTrue(self.model.station.machines['pump'].powered)
        args = {**self.request['arguments'], 'station_run_id': self.model.run_id}
        self.assertEqual(self.send(action_id=uuid.uuid4().hex, arguments=args)['error_code'], 'wrong_station_state')

    def test_non_evicting_history_preserves_old_terminal_without_new_effect(self):
        for number in range(64):
            self.request['action_id'] = '%032x' % number
            self.hold = Hold(); self.owner.hold = self.hold
            self.assertTrue(self.send()['accepted'])
            self.send('cancel'); self.owner.tick(); self.finish()
        self.assertEqual(self.send(action_id=uuid.uuid4().hex)['error_code'], 'history_full')
        self.assertEqual(self.send(action_id='0'*32)['action']['result']['status'], 'cancelled')
        self.assertEqual(self.model.control_revision, 0)
        self.assertEqual(self.workers, [])
