import copy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from orbital.inspection_owner import InspectionOwner, fresh
from orbital.inspection_protocol import MAX_ACTIONS, canonical, command, decode, result

RUN, MISSION, ACTION = '1'*32, '2'*32, '3'*32


def request(op='inspect', **changes):
    value = {'schema_version': 1, 'run_id': RUN, 'mission_id': MISSION, 'action_id': ACTION,
             'command_id': '4'*32, 'op': op,
             'arguments': {'machine_id': 'pump', 'timeout_s': 3.} if op == 'inspect' else {}}
    value.update(changes)
    return value


class Clock:
    wall = 10.
    sim = 1000000000
    def advance(self, delta):
        self.wall += delta
        self.sim += round(delta*1e9)


class Guard:
    def __init__(self):
        self.available = True
        self.settled = True
        self.invalid = False
        self.ends = 0
        self.current = {'health': {'run_id': 'localization', 'epoch': 1}}
    def ready(self): return self.available
    def begin(self, machine): self.machine = machine
    def sample(self):
        if self.invalid: raise ValueError('controlled precondition loss')
        return copy.deepcopy(self.current) if self.settled else None
    def identity(self): return {'registry_identity': 'a'*64, 'authority_identity': 'b'*64}
    def end(self): self.ends += 1


class Worker:
    def __init__(self, payload):
        self.payload = payload
        self.done = False
        self.exited = False
        self.error = None
        self.stopped = 0
        self.reply = {'status': 'succeeded', 'observation_id': 'c'*32, 'manifest_sha256': 'd'*64}
    def stop(self, now): self.stopped += 1
    def poll(self, now):
        self.done = self.exited
        return self.done


class ProtocolTests(unittest.TestCase):
    def test_normalized_int_timeout_and_exact_schema(self):
        self.assertEqual(command(canonical(request()))['arguments']['timeout_s'], 3.)
        bad = [request(schema_version=True), request(op='reset'), request(run_id='wrong'),
               request(extra='x'), request(arguments={'machine_id': 'pump', 'timeout_s': True}),
               request(arguments={'machine_id': 'pump', 'timeout_s': .09}),
               request(arguments={'machine_id': 'pump', 'timeout_s': 10.01}),
               request(arguments={'machine_id': 'pump', 'timeout_s': 2., 'path': '/tmp/file'}),
               request('cancel', arguments={'machine_id': 'pump'})]
        for value in bad:
            with self.subTest(value=value), self.assertRaises(ValueError): command(canonical(value))

    def test_duplicates_nonfinite_nested_and_oversize_are_refused(self):
        values = ['{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}', '['*1000+'0'+']'*1000,
                  'x'*4097, '{"a":"'+'é'*2050+'"}']
        for text in values:
            with self.subTest(size=len(text)), self.assertRaises(ValueError): decode(text)

    def test_oversized_text_is_rejected_before_encoding(self):
        class NoEncode(str):
            def encode(self, *args, **kwargs): raise AssertionError('copied oversized input')
        with self.assertRaises(ValueError): decode(NoEncode('x'*1000000))


class OwnerTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.guard, self.workers, self.events = Clock(), Guard(), [], []
        def factory(payload):
            worker = Worker(payload); self.workers.append(worker); return worker
        self.owner = InspectionOwner(Path('.'), self.guard, factory, now=lambda: self.clock.wall,
            sim_ns=lambda: self.clock.sim, on_result=self.events.append, run_id=RUN, load=self.load)

    def load(self, root, reference, action):
        robot = {'within_settled_inspection_pose': True, **copy.deepcopy(self.guard.current)}
        return {'observed_monotonic_s': self.clock.wall, 'sample_stamp_ns': self.clock.sim,
                'observed_sim_stamp_ns': self.clock.sim, 'registry_identity': 'a'*64,
                'camera': {'stamp_ns': self.clock.sim, 'received_age_s': .02, 'info_received_age_s': .02,
                           'robot_state': robot}, 'robot_state': robot, 'telemetry_freshness': {'received_age_s': .03}}

    def send(self, value=None): return self.owner.handle(canonical(request() if value is None else value))

    def start(self):
        self.assertTrue(self.send()['accepted'])
        self.owner.tick()
        return self.workers[-1]

    def test_success_requires_reap_and_validated_result(self):
        worker = self.start()
        self.assertEqual(self.owner.active.phase, 'running')
        self.assertEqual(self.events, [])
        worker.exited = True; self.owner.tick()
        self.assertIsNone(self.owner.active)
        self.assertEqual(result(self.events[0])['status'], 'succeeded')
        self.assertEqual(self.events[0]['observation']['observation_id'], 'c'*32)
        self.assertIsNone(self.events[0]['state_change'])

    def test_cancel_before_start_never_launches_and_is_queryable(self):
        self.send(); ack = self.send(request('cancel'))
        self.assertIsNone(ack['action']['result'])
        self.owner.tick()
        self.assertEqual(self.workers, [])
        self.assertEqual(self.send(request('query'))['action']['result']['status'], 'cancelled')

    def test_cancel_wins_over_already_exited_unprocessed_success(self):
        worker = self.start(); worker.exited = True
        self.send(request('cancel')); self.owner.tick()
        self.assertEqual(self.events[0]['status'], 'cancelled')
        self.assertIsNone(self.events[0]['observation'])
        self.assertEqual(worker.stopped, 1)

    def test_cancel_ack_does_not_release_a_live_child(self):
        worker = self.start(); self.send(request('cancel')); self.owner.tick()
        self.assertEqual(self.events, [])
        self.assertEqual(self.send(request(action_id='5'*32))['error_code'], 'busy')
        worker.exited = True; self.owner.tick()
        self.assertTrue(self.send(request(action_id='5'*32))['accepted'])
        self.send(request('cancel'))  # Already terminal old action.
        self.assertIsNone(self.owner.active.stop_reason)

    def test_deadline_during_capture_and_after_artifact_io(self):
        worker = self.start(); self.clock.advance(3.)
        self.owner.tick(); self.assertEqual(self.events, [])
        worker.exited = True; self.owner.tick()
        self.assertEqual(self.events[-1]['status'], 'timed_out')
        self.setUp(); worker = self.start(); worker.exited = True
        def delayed(root, ref, action):
            record = self.load(root, ref, action); self.clock.advance(3.); return record
        self.owner.load = delayed; self.owner.tick()
        self.assertEqual(self.events[-1]['status'], 'timed_out')

    def test_guard_loss_is_sticky_even_if_it_recovers_before_reap(self):
        worker = self.start(); self.guard.invalid = True; self.owner.tick()
        self.guard.invalid = False; worker.exited = True; self.owner.tick()
        self.assertEqual(self.events[-1]['status'], 'failed')
        self.assertEqual(worker.stopped, 1)

    def test_freshness_after_load_is_not_renewed(self):
        worker = self.start(); worker.exited = True
        def delayed(root, ref, action):
            record = self.load(root, ref, action); self.clock.advance(.201); return record
        self.owner.load = delayed; self.owner.tick()
        self.assertEqual(self.events[-1]['error_code'], 'observation_unavailable')

    def test_final_live_guard_is_after_file_verification(self):
        worker = self.start(); worker.exited = True
        def changed(root, ref, action):
            record = self.load(root, ref, action); self.guard.invalid = True; return record
        self.owner.load = changed; self.owner.tick()
        self.assertEqual(self.events[-1]['status'], 'failed')

    def test_idempotency_conflict_and_wrong_run_never_start_more_workers(self):
        worker = self.start()
        self.assertTrue(self.send(request(command_id='5'*32))['accepted'])
        self.assertEqual(self.send(request(arguments={'machine_id': 'cooler', 'timeout_s': 3.}))['error_code'], 'action_conflict')
        self.assertEqual(self.send(request(mission_id='5'*32))['error_code'], 'action_conflict')
        self.assertEqual(self.send(request('cancel', run_id='5'*32))['error_code'], 'wrong_run')
        worker.exited = True; self.owner.tick()
        self.assertEqual(self.send()['action']['result'], self.events[0])
        self.assertEqual(len(self.workers), 1)

    def test_capacity_never_evicts_an_action_for_reexecution(self):
        for number in range(MAX_ACTIONS):
            action_id = '%032x' % number
            self.send(request(action_id=action_id)); self.send(request('cancel', action_id=action_id)); self.owner.tick()
        self.assertEqual(self.send()['error_code'], 'history_full')
        self.assertEqual(self.send(request(action_id='0'*32))['action']['result']['status'], 'cancelled')
        self.assertEqual(len(self.owner.actions), MAX_ACTIONS)
        self.assertEqual(self.workers, [])

    def test_shutdown_and_clock_regression_require_reap(self):
        worker = self.start(); self.owner.shutdown(); self.owner.tick()
        self.assertEqual(self.events, []); worker.exited = True; self.owner.tick()
        self.assertEqual(self.events[-1]['error_code'], 'owner_shutdown')
        self.assertEqual(self.send(request(action_id='5'*32))['error_code'], 'unavailable')
        self.setUp(); worker = self.start(); self.clock.sim -= 1; self.owner.tick()
        worker.exited = True; self.owner.tick()
        self.assertEqual(self.events[-1]['error_code'], 'clock_regressed')

    def test_no_initial_settling_times_out_without_a_capture(self):
        self.guard.settled = False; self.send(); self.owner.tick()
        self.assertEqual(self.workers, [])
        self.clock.advance(3.); self.owner.tick()
        self.assertEqual(self.events[-1]['status'], 'timed_out')

    def test_returned_views_cannot_mutate_terminal_history(self):
        worker = self.start(); worker.exited = True; self.owner.tick()
        value = self.send(request('query')); value['action']['result']['status'] = 'failed'
        self.assertEqual(self.send(request('query'))['action']['result']['status'], 'succeeded')

    def test_simulation_age_generation_and_receipt_age_are_independent(self):
        worker = self.start()
        base = self.load(None, None, self.owner.active)
        for change in ('sim', 'camera', 'info', 'telemetry', 'generation', 'pose'):
            record = copy.deepcopy(base)
            if change == 'sim': record['sample_stamp_ns'] -= 200000001
            if change == 'camera': record['camera']['received_age_s'] = .21
            if change == 'info': record['camera']['info_received_age_s'] = .21
            if change == 'telemetry': record['telemetry_freshness']['received_age_s'] = 1.
            if change == 'generation': record['robot_state']['health']['epoch'] += 1
            if change == 'pose': record['robot_state']['within_settled_inspection_pose'] = False
            with self.subTest(change=change), self.assertRaises(ValueError):
                fresh(record, self.clock.wall, self.clock.sim, self.guard.current)
        worker.exited = True; self.owner.tick()


if __name__ == '__main__': unittest.main()
