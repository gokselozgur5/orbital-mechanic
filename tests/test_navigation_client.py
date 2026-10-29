"""Adversarial command/receipt tests; no assumption that an ACK means motion."""
import copy
import json
import unittest

from orbital.navigation_client import NavigationClient, command_timeout, execute, read_target
from orbital.navigation_command import CommandGate

RUN = '1c618ad8-ad03-405e-a5c1-26fcb330ca22'
TARGET = {'run_id': RUN, 'epoch': 2, 'sequence': 1, 'waypoint': 'pump'}


def state(**changes):
    result = {'run_id': RUN, 'command_sequence': 0, 'action_state': 'idle', 'target': None,
              'healthy': True, 'guard_latched': True, 'last_reply': None, 'last_completion': None,
              'forwarded_commands': 0}
    result.update(copy.deepcopy(changes))
    return result


class Channel:
    def __init__(self, initial, reply=None):
        self.now = 10.
        self.client = NavigationClient(lambda: self.now)
        self.initial, self.reply = initial, reply
        self.sent = []
        self.authority = True
        self.after_publish = None
        self.late_poll = False

    def poll(self, duration):
        self.now += 1. if self.late_poll and self.sent else duration
        value = self.reply if self.sent else self.initial
        if value is not None:
            self.client.observe(json.dumps(value))

    def authority_clear(self):
        if isinstance(self.authority, Exception):
            raise self.authority
        return self.authority

    def publish(self, text):
        self.sent.append(json.loads(text))
        if self.after_publish:
            self.after_publish(self)

    def execute(self, operation='request', **args):
        return execute(self.client, self, operation, timeout=.2, **args)


def reply(operation, *, accepted=True, **changes):
    return state(command_sequence=1, target=TARGET, action_state='waiting_for_arm',
                 last_reply={'sequence': 1, 'op': operation, 'accepted': accepted}, **changes)


class NavigationClientTests(unittest.TestCase):
    def test_request_preserves_server_wire_protocol_and_receipt_target(self):
        channel = Channel(state(), reply('request'))
        receipt, code = channel.execute(waypoint='pump')
        self.assertEqual(code, 0)
        self.assertEqual(receipt['delivery'], 'acknowledged')
        self.assertEqual(receipt['target'], TARGET)
        self.assertEqual(read_target(json.dumps(receipt)), TARGET)
        self.assertEqual(CommandGate(RUN, ['pump']).decode(json.dumps(channel.sent[0])), channel.sent[0])
        self.assertEqual(len(channel.sent), 1)

    def test_arm_ack_does_not_claim_dispatch(self):
        channel = Channel(state(target=TARGET, action_state='waiting_for_arm'), reply('arm'))
        receipt, code = channel.execute('arm', target=TARGET)
        self.assertEqual(code, 0)
        self.assertEqual(receipt['state']['action_state'], 'waiting_for_arm')
        self.assertIn('not dispatch', receipt['scope'])

    def test_cancel_can_acknowledge_before_terminal_or_physical_stop(self):
        after = reply('cancel')
        after.update(action_state='cancelling', guard_latched=False)
        channel = Channel(state(target=TARGET, action_state='running', guard_latched=False), after)
        receipt, code = channel.execute('cancel', target=TARGET)
        self.assertEqual(code, 0)
        self.assertEqual(receipt['state']['action_state'], 'cancelling')
        self.assertFalse(receipt['state']['guard_latched'])
        self.assertIn('physical stopping', receipt['scope'])

    def test_cancel_remains_available_without_health_or_fresh_guard(self):
        before = state(target=TARGET, action_state='running', healthy=False, guard_latched=None)
        after = reply('cancel', healthy=False, guard_latched=None)
        receipt, code = Channel(before, after).execute('cancel', target=TARGET)
        self.assertEqual(code, 0)
        self.assertTrue(receipt['accepted'])

    def test_completed_exact_cancel_target_can_acknowledge(self):
        after = state(command_sequence=1, last_reply={'sequence': 1, 'op': 'cancel', 'accepted': True},
                      last_completion={'target': TARGET, 'outcome': 'not_dispatched', 'measured': None})
        receipt, code = Channel(state(target=TARGET, action_state='waiting_for_arm'), after).execute('cancel', target=TARGET)
        self.assertEqual((code, receipt['target']), (0, TARGET))

    def test_stale_target_fields_never_retarget_to_current_state(self):
        for key, value in (('run_id', 'another-run'), ('epoch', 3), ('sequence', 2), ('waypoint', 'cooler')):
            channel = Channel(state(target=TARGET, action_state='waiting_for_arm'))
            target = dict(TARGET, **{key: value})
            with self.subTest(field=key):
                receipt, code = channel.execute('arm', target=target)
                self.assertEqual((code, receipt['delivery'], channel.sent), (2, 'not_sent', []))

    def test_ack_timeout_is_uncertain_and_never_retransmits(self):
        channel = Channel(state(target=TARGET, action_state='waiting_for_arm'))
        receipt, code = channel.execute('arm', target=TARGET)
        self.assertEqual((code, receipt['delivery'], receipt['target']), (3, 'uncertain', TARGET))
        self.assertIsNone(receipt['accepted'])
        self.assertEqual(len(channel.sent), 1)
        self.assertLessEqual(channel.now, 10.201)

    def test_discovery_timeout_sends_nothing(self):
        channel = Channel(state())
        channel.authority = False
        receipt, code = channel.execute(waypoint='pump')
        self.assertEqual((code, receipt['delivery'], channel.sent), (2, 'not_sent', []))

    def test_ownership_failure_before_or_after_send_is_distinguished(self):
        for after in (False, True):
            channel = Channel(state(), reply('request'))
            if after:
                channel.after_publish = lambda c: setattr(c, 'authority', ValueError('Competing publisher'))
            else:
                channel.authority = ValueError('Competing publisher')
            with self.subTest(after=after):
                receipt, code = channel.execute(waypoint='pump')
                self.assertEqual(code, 3 if after else 2)
                self.assertEqual(len(channel.sent), int(after))
                self.assertEqual(receipt['delivery'], 'uncertain' if after else 'not_sent')

    def test_run_restart_after_publication_cannot_match_old_ack(self):
        after = reply('request')
        after['run_id'] = 'new-run'
        after['target']['run_id'] = 'new-run'
        channel = Channel(state(), after)
        receipt, code = channel.execute(waypoint='pump')
        self.assertEqual(code, 3)
        self.assertIn('restarted', receipt['error'])
        self.assertEqual(len(channel.sent), 1)

    def test_other_command_advancing_sequence_makes_ack_ambiguous(self):
        after = reply('request')
        after['command_sequence'] = 2
        receipt, code = Channel(state(), after).execute(waypoint='pump')
        self.assertEqual(code, 3)
        self.assertIn('ambiguous', receipt['error'])

    def test_mismatched_ack_fields_do_not_succeed(self):
        for key, value in (('op', 'arm'), ('accepted', 'true'), ('sequence', True), ('sequence', 0)):
            after = reply('request')
            after['last_reply'][key] = value
            with self.subTest(field=key):
                receipt, code = Channel(state(), after).execute(waypoint='pump')
                self.assertEqual((code, receipt['delivery']), (3, 'uncertain'))

    def test_request_ack_requires_the_requested_owned_waypoint(self):
        for target, action in ((dict(TARGET, waypoint='cooler'), 'waiting_for_arm'), (None, 'idle')):
            after = reply('request')
            after.update(target=target, action_state=action)
            with self.subTest(target=target):
                receipt, code = Channel(state(), after).execute(waypoint='pump')
                self.assertEqual((code, receipt['delivery']), (3, 'uncertain'))

    def test_rejected_command_is_reported_without_retry(self):
        after = state(command_sequence=1, last_reply={'sequence': 1, 'op': 'request', 'accepted': False})
        channel = Channel(state(), after)
        receipt, code = channel.execute(waypoint='pump')
        self.assertEqual((code, receipt['delivery'], receipt['accepted']), (1, 'rejected', False))
        self.assertEqual(len(channel.sent), 1)

    def test_request_refuses_busy_unhealthy_or_unlatched_state(self):
        variants = [state(target=TARGET, action_state='running'), state(healthy=False), state(guard_latched=False)]
        for before in variants:
            channel = Channel(before)
            with self.subTest(before=before):
                receipt, code = channel.execute(waypoint='pump')
                self.assertEqual((code, receipt['delivery'], channel.sent), (2, 'not_sent', []))

    def test_clock_staleness_and_changed_prepared_target_block_publication(self):
        for change in ('stale', 'target', 'health', 'sequence'):
            channel = Channel(state(target=TARGET, action_state='waiting_for_arm'))
            client = channel.client
            client.observe(json.dumps(channel.initial))
            client.prepare('arm', target=TARGET)
            if change == 'stale':
                channel.now += .31
            else:
                changed = copy.deepcopy(channel.initial)
                if change == 'target':
                    changed['target']['epoch'] += 1
                elif change == 'health':
                    changed['healthy'] = False
                else:
                    changed['command_sequence'] += 1
                client.observe(json.dumps(changed))
            with self.subTest(change=change), self.assertRaises(ValueError):
                client.publish_once(channel.publish)
            self.assertEqual(channel.sent, [])

    def test_one_invocation_cannot_publish_twice_or_prepare_another_command(self):
        channel = Channel(state(), reply('request'))
        channel.execute(waypoint='pump')
        with self.assertRaises(ValueError):
            channel.client.publish_once(channel.publish)
        with self.assertRaises(ValueError):
            channel.client.prepare('cancel', target=TARGET)
        self.assertEqual(len(channel.sent), 1)

    def test_transport_exception_after_send_remains_uncertain(self):
        channel = Channel(state(), reply('request'))
        def fail_after_delivery(c):
            raise RuntimeError('Publish transport failed after delivery may have happened')
        channel.after_publish = fail_after_delivery
        receipt, code = channel.execute(waypoint='pump')
        self.assertEqual((code, receipt['delivery'], len(channel.sent)), (3, 'uncertain', 1))

    def test_late_matching_ack_is_not_accepted_after_deadline(self):
        channel = Channel(state(), reply('request'))
        channel.late_poll = True
        receipt, code = channel.execute(waypoint='pump')
        self.assertEqual((code, receipt['delivery']), (3, 'uncertain'))

    def test_discovery_work_cannot_extend_status_or_publication_deadline(self):
        for operation in ('status', 'request'):
            channel = Channel(state(), reply('request'))
            def slow_discovery():
                channel.now += .21
                return True
            channel.authority_clear = slow_discovery
            with self.subTest(operation=operation):
                receipt, code = channel.execute(operation, waypoint='pump')
                self.assertEqual((code, receipt['delivery'], channel.sent), (2, 'not_sent', []))

    def test_interrupt_after_publication_does_not_claim_no_effect(self):
        channel = Channel(state(), reply('request'))
        def interrupt(c):
            raise KeyboardInterrupt()
        channel.after_publish = interrupt
        receipt, code = channel.execute(waypoint='pump')
        self.assertEqual((code, receipt['delivery'], len(channel.sent)), (130, 'uncertain', 1))

    def test_status_observes_unhealthy_state_without_publication(self):
        channel = Channel(state(healthy=False))
        receipt, code = channel.execute('status')
        self.assertEqual((code, receipt['delivery'], channel.sent), (0, 'observed', []))
        self.assertFalse(receipt['state']['healthy'])

    def test_malformed_state_is_fatal_even_after_valid_state_returns(self):
        bad = [json.dumps(state(command_sequence=True)), json.dumps(state(command_sequence=-1)),
               '{"run_id":"one","run_id":"two"}', json.dumps(state(healthy='true')),
               json.dumps(state(target=TARGET)), json.dumps(state(guard_latched=1)),
               json.dumps(state(last_completion={'target':dict(TARGET, sequence=True), 'outcome':'cancelled'})),
               json.dumps(state(extra=float('nan'))), json.dumps(state()).replace('true','1e999',1),
               '[]', 'x'*16385]
        for text in bad:
            client = NavigationClient(lambda: 10.)
            with self.subTest(text=text[:90]):
                client.observe(text)
                client.observe(json.dumps(state()))
                with self.assertRaises(ValueError):
                    client.prepare('request', waypoint='pump')

    def test_sequence_regression_clock_regression_and_exhaustion_refused(self):
        channel = Channel(state(command_sequence=1))
        channel.client.observe(json.dumps(channel.initial))
        channel.client.observe(json.dumps(state()))
        with self.assertRaisesRegex(ValueError, 'regressed'):
            channel.client.prepare('request', waypoint='pump')
        channel = Channel(state())
        channel.client.observe(json.dumps(state()))
        channel.now -= 1
        with self.assertRaisesRegex(ValueError, 'clock'):
            channel.client.prepare('request', waypoint='pump')
        receipt, code = Channel(state(command_sequence=2**63-1)).execute(waypoint='pump')
        self.assertEqual((code, receipt['delivery']), (2, 'not_sent'))

    def test_only_explicit_tokens_or_successful_matching_request_receipts_load(self):
        receipt, _ = Channel(state(), reply('request')).execute(waypoint='pump')
        self.assertEqual(read_target(json.dumps(TARGET)), TARGET)
        for patch in ({'operation':'status'}, {'delivery':'uncertain'}, {'accepted':False},
                      {'target':dict(TARGET, waypoint='cooler')}, {'schema_version':True}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                read_target(json.dumps(dict(receipt, **patch)))
        for token in (dict(TARGET, sequence=True), dict(TARGET, extra='no'), {'target': TARGET}):
            with self.subTest(token=token), self.assertRaises(ValueError):
                read_target(json.dumps(token))

    def test_command_wait_bound_is_finite(self):
        self.assertEqual(command_timeout('.1'), .1)
        self.assertEqual(command_timeout('10'), 10.)
        for value in ('0', '10.01', 'nan', 'inf', '-1'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                command_timeout(value)


if __name__ == '__main__':
    unittest.main()
