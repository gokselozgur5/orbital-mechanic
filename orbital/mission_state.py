"""Bounded mission transitions over existing wire contracts, without actuation.

The controller reserves a step before publication and supplies source-checked
client observations. This object is not a ROS owner, safety interlock, durable
restart store or diagnosis policy. Unknown delivery keeps the step unresolved.
"""
import copy

from orbital import inspection_protocol as inspection, maintenance_protocol as maintenance
from orbital.inspection_protocol import canonical, decode, finite, identifier
from orbital.maintenance_observation import RecoveryEvidence
from orbital.machines import MACHINE_IDS
from orbital.navigation_client import NavigationClient, read_target
from orbital.navigation_command import CommandGate
from orbital.proximity import Unavailable
from orbital.telemetry_observation import decode as telemetry_decode

DECISIONS = frozenset(('healthy', 'recoverable', 'persistent', 'ambiguous', 'inspect_again'))
MAX_RECORD_BYTES = 65536
SCOPE = ('Mission transition contract over source-checked gateway observations; not autonomous '
         'diagnosis, exclusive ROS ownership, physical stopping, durable restart or native acceptance')


def record(value):
    """Bound normalized gateway records; the gateway also bounds raw transport."""
    return decode(canonical(value), MAX_RECORD_BYTES)


class MissionState:
    def __init__(self, registry, mission_id, baseline, *, now, sim_ns, timeout_s=600., max_steps=32):
        self.registry, self.mission_id = registry, identifier(mission_id)
        self.now, self.sim_ns = now, sim_ns
        self.started = self.last_now = finite(now())
        self.started_sim = self.last_sim = maintenance.integer(sim_ns(), 1)
        self.deadline = self.started + finite(timeout_s, .1, 600.)
        if type(max_steps) is not int or not 1 <= max_steps <= 64:
            raise ValueError('invalid_mission_step_limit')
        self.max_steps = max_steps
        sample = telemetry_decode(canonical(baseline), registry)
        if sample['schema_version'] != 2 or not 0 <= self.started_sim-sample['stamp_ns'] < 1000000000:
            raise ValueError('mission_requires_current_control_baseline')
        self.station_run = sample['run_id']
        self.control = copy.deepcopy(sample['control'])
        self.last_sample = None
        self.recovery = RecoveryEvidence(registry)
        self.phase, self.outcome, self.reason = 'inspect', None, None
        self.pending, self.arrival, self.inspection = None, None, None
        self.decision, self.verify_after = None, None
        self.next_operation = None
        self.stop_reason = None
        self.actions, self.owners = {}, {}

    def tick(self):
        if self.outcome is not None:
            return
        now, sim = self.now(), self.sim_ns()
        try:
            finite(now); maintenance.integer(sim, 1)
            if now < self.last_now or sim < self.last_sim:
                raise ValueError('clock_regressed')
        except (ValueError, TypeError, OverflowError):
            self.stop_reason = self.stop_reason or 'clock_invalid'
        else:
            self.last_now, self.last_sim = now, sim
            if now >= self.deadline:
                self.stop_reason = self.stop_reason or 'deadline_exceeded'
        if self.stop_reason and self.outcome is None:
            if self.pending is not None:
                self.phase = 'unresolved'
            else:
                self._finish('cancelled' if self.stop_reason == 'cancel_requested' else
                            'timed_out' if self.stop_reason == 'deadline_exceeded' else 'failed', self.stop_reason)

    def _finish(self, outcome, reason):
        if self.pending is not None:
            raise ValueError('mission_step_still_owned')
        self.outcome, self.reason, self.phase = outcome, reason, 'terminal'

    def _available(self, phase):
        self.tick()
        if self.pending is not None or self.outcome is not None or self.stop_reason or self.phase != phase:
            raise ValueError('mission_transition_not_available')

    def _reserve(self, family, key, request):
        if len(self.actions) >= self.max_steps:
            raise ValueError('mission_step_limit')
        if key in self.actions:
            raise ValueError('mission_step_identity_reused')
        run = request['run_id']
        if family in self.owners and self.owners[family] != run:
            raise ValueError('mission_server_generation_changed')
        self.owners[family] = run
        entry = {'family':family, 'request':copy.deepcopy(request), 'started_sim_stamp_ns':self.last_sim,
                 'acknowledged':False, 'possibly_sent':False, 'result':None, 'target':None,
                 'arm_observed':False, 'arm_command':None, 'last_navigation_state':None}
        self.actions[key] = self.pending = entry
        self.phase = 'navigating' if family == 'navigation' else 'inspecting' if family == 'inspection' else 'intervening'
        return copy.deepcopy(request)

    def begin_navigation(self, request):
        self._available('inspect')
        run, sequence = request.get('run_id'), request.get('sequence')
        maintenance.integer(sequence, 1)
        if not isinstance(run, str) or not 1 <= len(run) <= 128:
            raise ValueError('invalid_navigation_run')
        gate = CommandGate(run, MACHINE_IDS)
        gate.sequence = sequence-1
        value = gate.decode(canonical(request))
        if value['op'] != 'request':
            raise ValueError('mission_requires_navigation_request')
        self.arrival = None
        return self._reserve('navigation', 'navigation:'+run+':'+str(sequence), value)

    def dispatch(self, request, invoke):
        """Reserve before entering the gateway; an exception cannot prove no send."""
        value = self.begin_navigation(request) if request.get('op') == 'request' else self.begin_action(request)
        try:
            receipt = invoke(copy.deepcopy(value))
            if value['op'] == 'request':
                self.navigation_receipt(receipt)
            else:
                self.action_receipt(receipt)
        except BaseException:
            if self.pending is not None:
                self.pending['possibly_sent'] = True
                self.phase = 'unresolved'
            raise
        return copy.deepcopy(receipt)

    def navigation_receipt(self, value):
        self.tick()
        value = record(value)
        step = self._require_pending('navigation')
        if value.get('operation') != 'request' or value.get('command') != step['request']:
            raise ValueError('mission_navigation_receipt_mismatch')
        delivery = value.get('delivery')
        if delivery == 'acknowledged':
            token = read_target(canonical(value))
            if step['target'] is not None and step['target'] != token:
                raise ValueError('mission_navigation_target_changed')
            step['target'], step['acknowledged'], step['possibly_sent'] = token, True, True
        elif (delivery == 'not_sent' and value.get('accepted') is None) or (delivery == 'rejected' and value.get('accepted') is False):
            if step['acknowledged'] or (delivery == 'not_sent' and step['possibly_sent']):
                raise ValueError('mission_delivery_contradiction')
            if delivery == 'rejected':
                reply = (value.get('state') or {}).get('last_reply')
                if (not isinstance(reply, dict) or set(reply) not in ({'sequence','op','accepted'}, {'sequence','op','accepted','error'})
                        or reply['sequence'] != step['request']['sequence'] or reply['op'] != 'request'
                        or reply['accepted'] is not False or reply.get('error') not in (None,'maintenance_hold')):
                    raise ValueError('mission_rejection_unconfirmed')
                step['possibly_sent'] = True
            self._complete_step({'kind':'navigation_delivery','delivery':delivery}, succeeded=False)
        elif delivery == 'uncertain':
            step['possibly_sent'] = True
            self.phase = 'unresolved'
        else:
            raise ValueError('invalid_mission_navigation_receipt')

    def begin_navigation_arm(self, command):
        self.tick()
        step = self._require_pending('navigation')
        if self.stop_reason or step['arm_command'] is not None:
            raise ValueError('mission_arm_not_available')
        command = record(command)
        if not step['acknowledged'] or command.get('target') != step['target']:
            raise ValueError('mission_arm_target_mismatch')
        gate = CommandGate(step['target']['run_id'], MACHINE_IDS)
        gate.sequence = maintenance.integer(command.get('sequence'), 1)-1
        gate.decode(canonical(command))
        if command.get('op') != 'arm' or command.get('target') != step['target']:
            raise ValueError('mission_arm_target_mismatch')
        if command['sequence'] <= step['request']['sequence']:
            raise ValueError('mission_arm_sequence_invalid')
        step['arm_command'] = copy.deepcopy(command)
        return copy.deepcopy(command)

    def dispatch_arm(self, command, invoke):
        value = self.begin_navigation_arm(command)
        try:
            receipt = invoke(copy.deepcopy(value))
            self.navigation_arm_receipt(receipt)
        except BaseException:
            self.phase = 'unresolved'
            raise
        return copy.deepcopy(receipt)

    def navigation_arm_receipt(self, value):
        self.tick()
        value = record(value)
        step = self._require_pending('navigation')
        if (step['arm_command'] is None or value.get('operation') != 'arm'
                or value.get('target') != step['target'] or value.get('command') != step['arm_command']):
            raise ValueError('mission_arm_receipt_mismatch')
        if value.get('delivery') == 'acknowledged' and value.get('accepted') is True:
            step['arm_observed'] = True
        elif value.get('delivery') in ('uncertain', 'not_sent', 'rejected'):
            self.phase = 'unresolved'  # The reserved navigation target still exists.
        else:
            raise ValueError('invalid_mission_arm_receipt')

    def navigation_state(self, value, received_at):
        self.tick()
        value = record(value)
        step = self._require_pending('navigation')
        validator = NavigationClient(lambda:received_at)
        validator.observe(canonical(value))
        if validator.failure:
            raise ValueError('invalid_mission_navigation_state')
        stamp = maintenance.integer(value.get('sim_stamp_ns'), 1)
        if (not 0 <= self.last_now-finite(received_at) < .3 or not 0 <= self.last_sim-stamp < 300000000
                or stamp < step['started_sim_stamp_ns'] or value['run_id'] != step['request']['run_id']):
            raise ValueError('mission_navigation_state_stale_or_changed')
        minimum_sequence = (step['arm_command'] or step['request'])['sequence']
        prior = step['last_navigation_state']
        if (value['command_sequence'] < minimum_sequence or (prior is not None
                and (value['command_sequence'] < prior[0] or stamp < prior[1]))):
            raise ValueError('mission_navigation_state_regressed')
        if prior is not None and stamp == prior[1]:
            return False
        step['last_navigation_state'] = [value['command_sequence'], stamp]
        completion = value['last_completion']
        if (not step['acknowledged'] or value['action_state'] != 'idle' or value['target'] is not None
                or value['healthy'] is not True or value['guard_latched'] is not True
                or completion is None or completion['target'] != step['target']):
            return False
        arrived = completion['outcome'] == 'arrived'
        if arrived and (not step['arm_observed'] or not isinstance(completion.get('measured'), dict)
                        or completion['measured'].get('within_tolerance') is not True):
            raise ValueError('mission_arrival_not_observed')
        machine = step['target']['waypoint']
        self._complete_step(completion, succeeded=arrived)
        if arrived and self.outcome is None:
            self.arrival, self.phase = machine, 'inspect'
        return True

    def begin_action(self, request):
        op = request.get('op')
        family = 'inspection' if op == 'inspect' else 'maintenance'
        protocol = inspection if family == 'inspection' else maintenance
        value = protocol.command(canonical(request))
        if op not in ('inspect', 'isolate', 'reset', 'restore'):
            raise ValueError('unsupported_mission_step')
        self._available('inspect' if op == 'inspect' else 'intervene')
        args = value['arguments']
        if value['mission_id'] != self.mission_id or args['machine_id'] != self.arrival:
            raise ValueError('mission_action_binding')
        if family == 'maintenance':
            if (op != self.next_operation or args['machine_id'] != self.inspection['machine_id']
                    or args['station_run_id'] != self.station_run or args['control_revision'] != self.control['revision']):
                raise ValueError('mission_maintenance_transition')
            field = {'reset':'isolation_action_id', 'restore':'reset_action_id'}.get(op)
            if field and args[field] != self._last_maintenance()['action_id']:
                raise ValueError('mission_prerequisite_identity')
        if args['timeout_s'] > self.deadline-self.last_now:
            raise ValueError('mission_step_exceeds_remaining_budget')
        return self._reserve(family, value['action_id'], value)

    def _require_pending(self, family=None):
        if self.pending is None or (family is not None and self.pending['family'] != family):
            raise ValueError('mission_has_no_matching_step')
        return self.pending

    def action_receipt(self, receipt):
        self.tick()
        receipt = record(receipt)
        step = self._require_pending()
        if step['family'] not in ('inspection', 'maintenance'):
            raise ValueError('mission_wrong_receipt_family')
        if any(receipt.get(k) != step['request'][k] for k in ('run_id','mission_id','action_id')):
            raise ValueError('mission_receipt_identity_mismatch')
        transport = receipt.get('transport')
        if receipt.get('client_cleanup_failed') is True:
            step['possibly_sent'] = True
            self.phase = 'unresolved'
            return
        if transport == 'terminal':
            self.action_result(receipt.get('result'))
        elif transport == 'acknowledged':
            protocol = inspection if step['family'] == 'inspection' else maintenance
            ack = receipt.get('acknowledgement')
            if not isinstance(ack, dict) or ack.get('accepted') is not True:
                raise ValueError('mission_acknowledgement_missing')
            action = protocol.action_view(ack.get('action'))
            if (any(action[k] != step['request'][k] for k in ('run_id','mission_id','action_id'))
                    or action['request_sha256'] != protocol.request_digest(step['request'])):
                raise ValueError('mission_acknowledgement_mismatch')
            step['acknowledged'] = step['possibly_sent'] = True
        elif transport in ('not_sent','rejected'):
            if step['acknowledged'] or (transport == 'not_sent' and step['possibly_sent']):
                raise ValueError('mission_delivery_contradiction')
            if receipt.get('result') is not None:
                raise ValueError('mission_delivery_contradiction')
            if transport == 'rejected':
                protocol = inspection if step['family'] == 'inspection' else maintenance
                ack = receipt.get('acknowledgement')
                if (not isinstance(ack, dict) or ack.get('accepted') is not False or ack.get('action') is not None
                        or ack.get('error_code') not in protocol.REJECTION_CODES
                        or any(ack.get(k) != step['request'][k] for k in ('run_id','mission_id','action_id','command_id','op'))):
                    raise ValueError('mission_rejection_unconfirmed')
                step['possibly_sent'] = True
            elif receipt.get('acknowledgement') is not None:
                raise ValueError('mission_delivery_contradiction')
            self._complete_step({'kind':'action_delivery','transport':transport}, succeeded=False)
        elif transport == 'uncertain':
            step['possibly_sent'] = True
            self.phase = 'unresolved'
        else:
            raise ValueError('invalid_mission_action_receipt')

    def action_result(self, result):
        self.tick()
        step = self._require_pending()
        if step['family'] not in ('inspection','maintenance'):
            raise ValueError('mission_wrong_result_family')
        protocol = inspection if step['family'] == 'inspection' else maintenance
        value = protocol.result(record(result))
        request = step['request']
        if (any(value[k] != request[k] for k in ('run_id','mission_id','action_id'))
                or value['request_sha256'] != protocol.request_digest(request)
                or value['machine_id'] != request['arguments']['machine_id']
                or not step['started_sim_stamp_ns'] <= value['started_sim_stamp_ns']
                       <= value['finished_sim_stamp_ns'] <= self.last_sim):
            raise ValueError('mission_result_binding')
        if value['observation'] is not None and value['observation']['registry_identity'] != self.registry.identity:
            raise ValueError('mission_result_registry_changed')
        if step['family'] == 'maintenance':
            if value['operation'] != request['op'] or value['station_run_id'] != self.station_run:
                raise ValueError('mission_maintenance_result_mismatch')
            field = {'reset':'isolation', 'restore':'reset'}.get(value['operation'])
            if field and value[field] != self._last_maintenance():
                raise ValueError('mission_prerequisite_result_mismatch')
            change = value['state_change']
            if change['disposition'] == 'applied':
                if change['effect']['control_revision'] != self.control['revision']+1:
                    raise ValueError('mission_effect_revision_mismatch')
                self.control = {'revision':change['effect']['control_revision'],'last_applied':copy.deepcopy(change['effect'])}
        step['possibly_sent'] = True
        self._complete_step(value, succeeded=value['status'] == 'succeeded')
        if self.outcome is not None or self.phase == 'escalate':
            return
        if request['op'] == 'inspect':
            self.inspection, self.phase = value, 'diagnose'
        elif request['op'] == 'restore':
            self.phase, self.verify_after = 'verify', self.last_sim
            self.recovery = RecoveryEvidence(self.registry)
        else:
            self.next_operation = 'reset' if request['op'] == 'isolate' else 'restore'
            self.phase = 'intervene'

    def _last_maintenance(self):
        values = [v['result'] for v in self.actions.values() if v['family'] == 'maintenance'
                  and isinstance(v['result'],dict) and v['result'].get('kind') == 'maintenance_result']
        return values[-1] if values else None

    def _complete_step(self, result, *, succeeded):
        step = self._require_pending()
        step['result'] = copy.deepcopy(result)
        self.pending = None
        self.phase = 'inspect' if succeeded else 'escalate'
        self.tick()  # Cancellation/deadline takes precedence over a late success.

    def decide(self, decision):
        self._available('diagnose')
        if decision not in DECISIONS or self.inspection is None:
            raise ValueError('invalid_mission_decision')
        self.decision = {'kind':decision,'inspection_action_id':self.inspection['action_id'],
                         'observation':copy.deepcopy(self.inspection['observation'])}
        if decision == 'healthy':
            self.phase, self.verify_after = 'verify', self.last_sim
            self.recovery = RecoveryEvidence(self.registry)
        elif decision == 'recoverable':
            self.phase, self.next_operation = 'intervene', 'isolate'
        else:
            self.phase = 'inspect' if decision == 'inspect_again' else 'escalate'

    def observe_telemetry(self, text):
        if self.outcome is not None:
            return False  # A closed decision is history, never a renewed live observation.
        self.tick()
        if self.outcome is not None:
            return False
        sample = telemetry_decode(text, self.registry)
        if sample['schema_version'] != 2 or sample['run_id'] != self.station_run:
            self.stop_reason = self.stop_reason or 'station_generation_changed'
            self.tick()
            raise ValueError('mission_station_generation_changed')
        if self.last_sample is not None and sample['control'] != self.last_sample['control']:
            self.recovery = RecoveryEvidence(self.registry)
        if not self.recovery.observe(text, self.last_now, self.last_sim):
            self.stop_reason = self.stop_reason or 'telemetry_invalid'
            self.tick()
            raise ValueError('mission_telemetry_invalid')
        self.last_sample = sample
        return True

    def verify(self):
        self.tick()
        if self.phase != 'verify' or self.pending is not None or self.outcome is not None:
            return False
        try:
            value = self.recovery.measure(self.last_now, self.last_sim)
        except Unavailable:
            return False
        if (not value['verified'] or self.last_sample['control'] != self.control
                or value['window']['first']['stamp_ns'] < self.verify_after):
            return False
        restored = self._last_maintenance()
        if self.decision['kind'] == 'recoverable' and (restored is None or restored['operation'] != 'restore'
                or restored['status'] != 'succeeded' or self.control['last_applied'] != restored['state_change']['effect']):
            raise ValueError('mission_restore_not_verified')
        self._finish('recovered' if self.decision['kind'] == 'recoverable' else 'healthy', 'public_recovery_verified')
        return True

    def cancel(self):
        if self.outcome is None:
            self.stop_reason = self.stop_reason or 'cancel_requested'
            self.tick()
        return self.cancellation_target()

    def cancellation_target(self):
        if self.pending is None:
            return None
        step = self.pending
        if step['family'] == 'navigation':
            return {'family':'navigation','target':copy.deepcopy(step['target'])}
        return {'family':step['family'], **{k:step['request'][k] for k in ('run_id','mission_id','action_id')}}

    def escalate(self):
        self.tick()
        if self.phase != 'escalate' or self.pending is not None:
            raise ValueError('mission_escalation_not_available')
        self._finish('escalated', 'evidence_or_action_requires_operator')

    def snapshot(self):
        self.tick()
        return copy.deepcopy({'mission_id':self.mission_id,'phase':self.phase,'outcome':self.outcome,
            'reason':self.reason,'stop_reason':self.stop_reason,'station_run_id':self.station_run,
            'registry_identity':self.registry.identity,'control':self.control,'decision':self.decision,
            'pending':self.pending,'actions':list(self.actions.values()),'scope':SCOPE})
