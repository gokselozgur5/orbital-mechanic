import contextlib
import copy
import importlib.util
import io
import json
import os
from pathlib import Path
import unittest
from unittest.mock import patch
import uuid
from types import SimpleNamespace as NS

from orbital.inspection_protocol import canonical
from orbital.maintenance_client import MaintenanceClient
from orbital.inspection_client import close_client
from orbital import maintenance_protocol as protocol
import test_maintenance_owner as owner_fixture
import test_maintenance_reset as reset_fixture

ROOT = Path(__file__).resolve().parents[1]


class MaintenanceClientTests(unittest.TestCase):
    def setUp(self):
        self.f = owner_fixture.MaintenanceOwnerTests()
        self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.sent = []
        self.complete = self.no_ack = self.drop_results = False
        self.peers = ['station-publisher']
        self.next_sample = self.f.clock.wall+.5
        self.client = MaintenanceClient(self.publish, self.spin, lambda:self.peers, now=lambda:self.f.clock.wall)
        self.f.owner.on_result = lambda v:None if self.drop_results else self.client.observe_result(canonical(v))

    def publish(self, text):
        self.sent.append(json.loads(text))
        reply = self.f.owner.handle(text)
        if not self.no_ack: self.client.observe_reply(canonical(reply))

    def spin(self, delay):
        f = self.f
        f.clock.advance(delay)
        f.hold.acknowledged = True
        if self.complete and f.workers: f.workers[-1].exited = True
        if f.clock.wall >= self.next_sample:
            self.next_sample = f.clock.wall+.5
            packet = f.model.sample(f.clock.sim)
            if packet is not None: f.owner.telemetry(canonical(packet))
        f.owner.tick()
        if f.hold.release_at is not None: f.hold.released = True
        self.client.observe_state(canonical(f.owner.state()))

    def test_acknowledgment_is_not_effect_and_query_recovers_exact_terminal(self):
        self.client.discover()
        ack = self.client.execute(self.f.request, wait=False)
        self.assertEqual(ack['transport'], 'acknowledged')
        self.assertIsNone(ack['result'])
        self.assertTrue(self.f.model.station.machines['pump'].powered)
        self.complete = self.drop_results = True
        for _ in range(100): self.spin(.02)
        request = {**self.f.request, 'op': 'query', 'arguments': {}, 'command_id': uuid.uuid4().hex}
        receipt = self.client.execute(request)
        self.assertEqual(receipt['transport'], 'terminal', receipt)
        self.assertEqual(receipt['result']['state_change']['disposition'], 'applied')
        self.assertEqual([v['op'] for v in self.sent], ['isolate', 'query'])

    def test_exact_state_recovers_lost_ack_without_power_retry(self):
        self.client.discover(); self.complete = self.no_ack = self.drop_results = True
        receipt = self.client.execute(self.f.request)
        self.assertEqual(receipt['transport'], 'terminal', receipt)
        self.assertEqual(receipt['result']['status'], 'succeeded')
        self.assertIsNone(receipt['acknowledgement'])
        self.assertEqual(len(self.sent), 1)

    def test_unreaped_capture_is_uncertain_with_one_exact_cancel(self):
        self.client.discover()
        receipt = self.client.execute(self.f.request)
        self.assertEqual(receipt['transport'], 'uncertain')
        self.assertIsNone(receipt['result'])
        self.assertEqual([v['op'] for v in self.sent], ['isolate', 'cancel'])
        self.assertEqual(self.sent[0]['action_id'], self.sent[1]['action_id'])
        self.assertIsNotNone(self.f.owner.active)
        self.assertIsNotNone(self.f.hold.owner)

    def test_paused_simulation_never_renews_client_readiness(self):
        self.client.discover()
        for _ in range(5):
            self.f.clock.wall += .1
            self.client.observe_state(canonical(self.f.owner.state()))
        receipt = self.client.execute(self.f.request)
        self.assertEqual(receipt['transport'], 'not_sent')
        self.assertEqual(self.sent, [])

    def test_station_restart_invalidates_discovered_state(self):
        self.client.discover(); self.f.model.sample(500000000)
        self.f.clock.advance(.01)
        self.client.observe_state(canonical(self.f.owner.state()))
        self.assertFalse(self.client.valid())
        self.assertEqual(self.client.execute(self.f.request)['transport'], 'not_sent')

    def test_publisher_loss_during_effect_does_not_claim_cancellation(self):
        self.client.discover()
        actual = self.client.spin
        def changed(delay):
            actual(delay)
            self.peers = ['replacement-publisher']
        self.client.spin = changed
        receipt = self.client.execute(self.f.request)
        self.assertEqual(receipt['transport'], 'uncertain')
        self.assertEqual([v['op'] for v in self.sent], ['isolate', 'cancel'])

    def test_malformed_commands_cannot_supply_pose_or_omit_reset_prerequisite(self):
        values = []
        for change in ({'op': 'reset'}, {'schema_version': True}, {'run_id': 'x'}, {'extra': 'x'}):
            values.append({**self.f.request, **change})
        for change in ({'distance_m': 0.}, {'control_revision': True}, {'timeout_s': 31.}, {'station_run_id': 'x'}):
            values.append({**self.f.request, 'arguments': {**self.f.request['arguments'], **change}})
        for value in values:
            with self.subTest(value=value), self.assertRaises(ValueError): protocol.command(canonical(value))
        for text in ('{"op":"isolate","op":"cancel"}', '{"a":NaN}', 'x'*4097):
            with self.assertRaises(ValueError): protocol.command(text)

    def test_terminal_validation_requires_exact_effect_and_verified_power_state(self):
        self.client.discover(); self.complete = True
        value = self.client.execute(self.f.request)['result']
        protocol.result(value)
        for field, change in (('hold_released', False), ('observation', None), ('operation', 'reset')):
            bad = copy.deepcopy(value); bad[field] = change
            with self.subTest(field=field), self.assertRaises(ValueError): protocol.result(bad)
        for field, change in (('powered', True), ('current_a', True), ('run_id', 'f'*32),
                              ('control_revision', 2), ('stamp_ns', value['state_change']['effect']['applied_stamp_ns'])):
            bad = copy.deepcopy(value); bad['state_change']['verification'][field] = change
            with self.subTest(field=field), self.assertRaises(ValueError): protocol.result(bad)
        bad = copy.deepcopy(value); bad['state_change']['effect']['action_id'] = 'f'*32
        self.client.observe_result(canonical(bad))
        self.assertEqual(self.client.failure, 'invalid_terminal_result')

    def test_cli_requires_exact_identity_and_preserves_uncertain_outcome(self):
        spec = importlib.util.spec_from_file_location('maintenance_cli', ROOT/'scripts/maintain-machine.py')
        cli = importlib.util.module_from_spec(spec); spec.loader.exec_module(cli)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): cli.main(['cancel'])
        output = io.StringIO()
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '84', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}), patch.object(cli, 'run_ros', return_value={'transport': 'uncertain', 'result': None}) as invoke, contextlib.redirect_stdout(output):
            code = cli.main(['isolate', 'pump'])
        self.assertEqual(code, 2)
        self.assertEqual(json.loads(output.getvalue())['transport'], 'uncertain')
        self.assertEqual(invoke.call_args.kwargs['machine_id'], 'pump')

    def test_cleanup_failure_preserves_transmitted_outcome_and_attempts_shutdown(self):
        calls = []
        def destroy(): raise RuntimeError('controlled cleanup failure')
        outcome = {'transport': 'uncertain', 'result': None}
        close_client(NS(destroy_node=destroy), NS(try_shutdown=lambda:calls.append('shutdown')), outcome)
        self.assertEqual(outcome['transport'], 'uncertain')
        self.assertTrue(outcome['client_cleanup_failed'])
        self.assertEqual(calls, ['shutdown'])

    def test_cli_accepts_the_full_maintenance_deadline_range(self):
        spec = importlib.util.spec_from_file_location('maintenance_cli_deadline', ROOT/'scripts/maintain-machine.py')
        cli = importlib.util.module_from_spec(spec); spec.loader.exec_module(cli)
        env = {'ROS_DOMAIN_ID': '84', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
        with patch.dict(os.environ, env), patch.object(cli, 'run_ros', return_value={
                'transport': 'acknowledged', 'result': None}) as invoke:
            for value in ('.1', '15', '30'):
                with self.subTest(timeout=value), contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(cli.main(['isolate', 'pump', '--timeout', value]), 0)
                    self.assertEqual(invoke.call_args.kwargs['timeout_s'], float(value))
            for value in ('0', '.09', '30.01', 'nan', 'inf'):
                with self.subTest(timeout=value), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    cli.main(['isolate', 'pump', '--timeout', value])
        self.assertEqual(invoke.call_count, 3)


class ResetClientTests(unittest.TestCase):
    publish = MaintenanceClientTests.publish
    spin = MaintenanceClientTests.spin

    def setUp(self):
        self.f = reset_fixture.ResetFixture()
        self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.f.isolated()
        self.sent = []
        self.complete = self.no_ack = self.drop_results = False
        self.peers = ['station-publisher']
        self.next_sample = self.f.clock.wall+.5
        self.client = MaintenanceClient(self.publish, self.spin, lambda:self.peers, now=lambda:self.f.clock.wall)
        self.f.owner.on_result = lambda v:None if self.drop_results else self.client.observe_result(canonical(v))

    def test_reset_receipt_validates_nested_isolation_and_new_effect(self):
        self.client.discover(); self.complete = self.no_ack = self.drop_results = True
        receipt = self.client.execute(self.f.reset_request)
        self.assertEqual(receipt['transport'], 'terminal', receipt)
        self.assertEqual(receipt['result']['operation'], 'reset')
        self.assertEqual(receipt['result']['status'], 'succeeded')
        self.assertEqual([v['op'] for v in self.sent], ['reset'])

    def test_reset_uses_requested_deadline_and_one_exact_cancel_after_loss(self):
        self.client.discover()
        start = self.f.clock.wall
        receipt = self.client.execute(self.f.reset_request)
        self.assertEqual(receipt['transport'], 'uncertain', receipt)
        self.assertGreaterEqual(self.f.clock.wall-start, 12.)
        self.assertEqual([v['op'] for v in self.sent], ['reset', 'cancel'])
        self.assertEqual(self.sent[0]['action_id'], self.sent[1]['action_id'])
        self.assertTrue(self.f.model.reset_available('pump'))

    def test_reset_refuses_not_ready_server_without_publication(self):
        self.client.discover()
        self.client.latest['ready'] = False
        self.assertEqual(self.client.execute(self.f.reset_request)['transport'], 'not_sent')
        self.assertEqual(self.sent, [])

    def test_reset_ack_digest_mismatch_is_uncertain_and_never_retried(self):
        self.client.discover()
        def publish(text):
            self.sent.append(json.loads(text))
            reply = self.f.owner.handle(text)
            if reply['action'] is not None: reply['action']['request_sha256'] = 'f'*64
            self.client.observe_reply(canonical(reply))
        self.client.publish = publish
        self.assertEqual(self.client.execute(self.f.reset_request)['transport'], 'uncertain')
        self.assertEqual([v['op'] for v in self.sent], ['reset', 'cancel'])
        self.assertEqual(self.client.failure, 'invalid_reply')

    def test_cli_requires_prior_owner_mission_and_isolation_identity(self):
        spec = importlib.util.spec_from_file_location('reset_cli', ROOT/'scripts/maintain-machine.py')
        cli = importlib.util.module_from_spec(spec); spec.loader.exec_module(cli)
        args = ['reset', 'pump', '--run-id', self.f.owner.run_id,
                '--mission-id', self.f.request['mission_id'], '--isolation-action-id', self.f.request['action_id']]
        env = {'ROS_DOMAIN_ID': '84', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
        with patch.dict(os.environ, env), patch.object(cli, 'run_ros', return_value={
                'transport': 'acknowledged', 'result': None}) as invoke:
            for omitted in (2, 4, 6):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    cli.main(args[:omitted]+args[omitted+2:])
            with contextlib.redirect_stdout(io.StringIO()): self.assertEqual(cli.main(args), 0)
        self.assertEqual(invoke.call_count, 1)
        self.assertEqual(invoke.call_args.kwargs['isolation_action_id'], self.f.request['action_id'])
