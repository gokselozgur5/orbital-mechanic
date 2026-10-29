import contextlib
import copy
import importlib.util
import io
import json
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from orbital.inspection_client import InspectionClient
from orbital.inspection_owner import InspectionOwner
from orbital.inspection_protocol import canonical
import test_inspection_owner as fixture

ROOT = Path(__file__).resolve().parents[1]


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.guard, self.workers, self.sent = fixture.Clock(), fixture.Guard(), [], []
        self.no_ack = self.no_result = self.complete = False
        self.peers = ['publisher-one']
        self.owner = InspectionOwner(Path('.'), self.guard, self.factory,
            now=lambda: self.clock.wall, sim_ns=lambda: self.clock.sim, run_id=fixture.RUN,
            load=self.load, on_result=lambda value: None if self.no_result else self.client.observe_result(canonical(value)))
        self.client = InspectionClient(self.publish, self.spin, lambda: self.peers, now=lambda: self.clock.wall)

    def factory(self, payload):
        worker = fixture.Worker(payload); self.workers.append(worker); return worker

    def load(self, root, reference, action):
        robot = {'within_settled_inspection_pose': True, **copy.deepcopy(self.guard.current)}
        return {'observed_monotonic_s': self.clock.wall, 'sample_stamp_ns': self.clock.sim,
                'observed_sim_stamp_ns': self.clock.sim, 'registry_identity': 'a'*64,
                'camera': {'stamp_ns': self.clock.sim, 'received_age_s': .01, 'info_received_age_s': .01,
                           'robot_state': robot}, 'robot_state': robot, 'telemetry_freshness': {'received_age_s': .01}}

    def publish(self, text):
        self.sent.append(json.loads(text))
        reply = self.owner.handle(text)
        if not self.no_ack: self.client.observe_reply(canonical(reply))

    def spin(self, delay):
        self.clock.advance(delay)
        if self.complete and self.workers: self.workers[-1].exited = True
        self.owner.tick()
        self.client.observe_state(canonical(self.owner.state()))

    def test_ack_is_not_terminal_and_query_recovers_a_lost_result(self):
        self.client.discover()
        receipt = self.client.execute(fixture.request(), wait=False)
        self.assertEqual(receipt['transport'], 'acknowledged')
        self.assertIsNone(receipt['result'])
        self.no_result = True; self.workers[-1].exited = True; self.owner.tick()
        receipt = self.client.execute(fixture.request('query', command_id='9'*32))
        self.assertEqual(receipt['transport'], 'terminal')
        self.assertEqual(receipt['result']['status'], 'succeeded')
        self.assertEqual([value['op'] for value in self.sent], ['inspect', 'query'])

    def test_lost_ack_can_complete_from_exact_terminal_state(self):
        self.client.discover(); self.complete = self.no_ack = self.no_result = True
        receipt = self.client.execute(fixture.request())
        self.assertEqual(receipt['transport'], 'terminal')
        self.assertIsNone(receipt['acknowledgement'])
        self.assertEqual(len(self.sent), 1)

    def test_unreaped_worker_is_uncertain_and_inspection_is_not_retransmitted(self):
        self.client.discover()
        receipt = self.client.execute(fixture.request())
        self.assertEqual(receipt['transport'], 'uncertain')
        self.assertIsNone(receipt['result'])
        self.assertEqual([value['op'] for value in self.sent], ['inspect', 'cancel'])
        self.assertEqual(self.sent[0]['action_id'], self.sent[1]['action_id'])
        self.assertIsNotNone(self.owner.active)

    def test_wrong_run_and_stale_state_are_never_sent(self):
        self.client.discover()
        self.assertEqual(self.client.execute(fixture.request(run_id='f'*32))['transport'], 'not_sent')
        self.clock.advance(.3)
        self.assertEqual(self.client.execute(fixture.request())['transport'], 'not_sent')
        self.assertEqual(self.sent, [])

    def test_publisher_change_and_regressing_sequence_are_fatal(self):
        self.client.discover(); self.peers = ['replacement']
        self.assertFalse(self.client.valid())
        self.assertEqual(self.client.failure, 'publisher_generation_changed')
        self.setUp(); self.client.discover()
        self.client.observe_state(canonical(self.client.latest))
        self.assertEqual(self.client.failure, 'invalid_or_changed_server_state')

    def test_conflicting_matching_result_is_not_accepted(self):
        self.client.discover(); self.complete = True
        receipt = self.client.execute(fixture.request())
        wrong = copy.deepcopy(receipt['result']); wrong['request_sha256'] = 'f'*64
        self.client.observe_result(canonical(wrong))
        self.assertEqual(self.client.failure, 'invalid_terminal_result')

    def test_interrupt_attempts_exact_cancel_but_does_not_claim_worker_exit(self):
        self.client.discover(); normal = self.client.spin
        interrupted = False
        def spin(delay):
            nonlocal interrupted
            if not interrupted:
                interrupted = True; self.owner.tick(); raise KeyboardInterrupt
            normal(delay)
        self.client.spin = spin
        receipt = self.client.execute(fixture.request())
        self.assertEqual(receipt['transport'], 'uncertain')
        self.assertEqual([value['op'] for value in self.sent], ['inspect', 'cancel'])
        self.assertEqual(self.owner.active.stop_reason[0], 'cancelled')

    def test_cli_requires_exact_cancel_identity_and_preserves_uncertainty(self):
        spec = importlib.util.spec_from_file_location('inspection_cli', ROOT/'scripts/inspect-machine.py')
        cli = importlib.util.module_from_spec(spec); spec.loader.exec_module(cli)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): cli.main(['cancel'])
        out = io.StringIO()
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '84', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}), patch.object(cli, 'run_ros', return_value={'transport': 'uncertain', 'result': None}) as run, contextlib.redirect_stdout(out):
            code = cli.main(['inspect', 'pump'])
        self.assertEqual(code, 2)
        self.assertEqual(json.loads(out.getvalue())['transport'], 'uncertain')
        self.assertEqual(run.call_args.kwargs['machine_id'], 'pump')


if __name__ == '__main__': unittest.main()
