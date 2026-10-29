"""Bind reported maintenance outcomes to separately retained public wire samples."""
import hashlib

from orbital import maintenance_protocol as protocol
from orbital.inspection_protocol import canonical, MAX_ACTIONS
from orbital.proximity import Unavailable
from orbital.reset_evidence import MAX_RESET_TEMPERATURE_C, MAX_RESET_VIBRATION_AU


def digest(packet):
    return hashlib.sha256(canonical(packet).encode()).hexdigest()


def pair_fresh(pair):
    first, last = pair
    if (last[0]['sequence'] != first[0]['sequence']+1
            or any(not 0 <= last[1]-entry[1] < 1000000000
                   or not 0 <= last[2]-entry[0]['stamp_ns'] < 1000000000 for entry in pair)):
        raise ValueError('sequence_pair_not_fresh')


class SequenceChecks:
    """Historical sample checks grant no live camera, hold or actuation permission."""
    def __init__(self, trace):
        self.trace = trace
        self.requests = dict(trace.requests)
        self.origins = {key:'command_topic' for key in self.requests}
        self.results = trace.results
        self.controls = {trace.baseline['control']['revision']:trace.baseline['control']}
        self.summaries = {}

    def packet(self, sequence):
        if sequence not in self.trace.samples:
            raise Unavailable('sequence_missing_source_packet')
        return self.trace.samples[sequence]

    def pair(self, sequence):
        value = [self.packet(sequence-1), self.packet(sequence)]
        pair_fresh(value)
        return value

    def request_equal(self, one, two):
        if protocol.request_digest(one) != protocol.request_digest(two):
            raise ValueError('sequence_request_conflict')

    def gather(self):
        children = set()
        for value in self.results.values():
            if value['operation'] != 'cancel_isolate': continue
            child = value['isolation_result']
            if child is None: continue
            key = child['action_id']
            if key in children or key in self.requests:
                raise ValueError('sequence_child_identity_conflict')
            children.add(key)
            self.requests[key] = value['isolation_request']
            self.origins[key] = 'workflow_child_report'
        if len(self.requests) > MAX_ACTIONS:
            raise ValueError('sequence_action_capacity')
        if set(self.requests) != set(self.results):
            raise Unavailable('sequence_missing_command_or_result')
        for key, request in self.requests.items():
            result = self.results[key]
            args = request['arguments']
            if (any(request[k] != result[k] for k in ('run_id','mission_id','action_id'))
                    or protocol.request_digest(request) != result['request_sha256']
                    or request['op'] != result['operation']
                    or any(args[k] != result[k] for k in ('machine_id','station_run_id'))
                    or args['station_run_id'] != self.trace.baseline['run_id']
                    or not self.trace.baseline['stamp_ns'] <= result['started_sim_stamp_ns']
                           <= result['finished_sim_stamp_ns'] <= self.trace.last_sim):
                raise ValueError('sequence_result_binding')
            if result['operation'] == 'cancel_isolate' or result['state_change']['disposition'] != 'applied':
                continue
            effect = result['state_change']['effect']
            revision = effect['control_revision']
            if (revision != args['control_revision']+1 or revision in self.controls
                    or effect['applied_stamp_ns'] < result['started_sim_stamp_ns']):
                raise ValueError('sequence_effect_revision')
            self.controls[revision] = {'revision':revision,'last_applied':effect}
        base = self.trace.baseline['control']['revision']
        if sorted(self.controls) != list(range(base, max(self.controls)+1)):
            raise ValueError('sequence_missing_effect_revision')
        effects = [self.controls[r]['last_applied'] for r in sorted(self.controls) if r > base]
        for one, two in zip(effects, effects[1:]):
            if two['applied_stamp_ns'] < one['applied_stamp_ns']:
                raise ValueError('sequence_effect_clock_regressed')
        if any(sum(e['operation'] == op for e in effects) > 1 for op in ('reset','restore')):
            raise ValueError('sequence_attempt_budget_exceeded')
        for packet, _, _ in self.trace.samples.values():
            if packet['sequence'] < self.trace.baseline['sequence']: continue
            expected = self.controls.get(packet['control']['revision'])
            if expected is None or packet['control'] != expected:
                raise ValueError('sequence_unaccounted_control')
            powered = expected['revision'] == base or expected['last_applied']['operation'] == 'restore'
            measured = packet['machines'][self.trace.target['machine_id']]
            if (measured['powered'] is not powered or (not powered
                    and (measured['current_a'] != 0 or measured['vibration_au'] != 0))):
                raise ValueError('sequence_control_measurement_not_observed')

    def basis(self, result):
        basis = result['eligibility']
        if basis['registry_identity'] != self.trace.registry.identity:
            raise ValueError('sequence_wrong_registry')
        entries = []
        for reference in basis['samples']:
            entry = self.packet(reference['sequence']); packet = entry[0]
            measurements = {'voltage_v':packet['voltage_v'], **{
                key:packet['machines'][result['machine_id']][key]
                for key in ('powered','current_a','vibration_au')}}
            if (digest(packet) != reference['sample_sha256'] or packet['stamp_ns'] != reference['stamp_ns']
                    or packet['control']['revision'] != basis['control_revision']
                    or measurements != reference['measurements']):
                raise ValueError('sequence_reset_basis_not_observed')
            entries.append(entry)
        pair_fresh(entries)
        before = result['state_change']['effect']['applied_stamp_ns'] if result['state_change']['effect'] else result['finished_sim_stamp_ns']
        if any(p['stamp_ns'] <= before and p['machines'][result['machine_id']]['powered']
               and p['machines'][result['machine_id']]['vibration_au'] > MAX_RESET_VIBRATION_AU
               for p, _, _ in self.trace.samples.values()):
            raise ValueError('sequence_high_vibration_history')

    def ordinary(self, key):
        result, request = self.results[key], self.requests[key]
        args, change = request['arguments'], result['state_change']
        previous = self.controls.get(args['control_revision'])
        if previous is None: raise ValueError('sequence_unknown_previous_control')
        for operation, field, argument in (('reset','isolation','isolation_action_id'), ('restore','reset','reset_action_id')):
            if result['operation'] != operation: continue
            prior = self.results.get(args[argument])
            if prior is None: raise Unavailable('sequence_missing_prior_result')
            if (prior != result[field] or prior['state_change']['disposition'] != 'applied'
                    or prior['state_change']['effect'] != previous['last_applied']):
                raise ValueError('sequence_prior_result_conflict')
            if operation == 'reset': self.basis(result)
        if result['observation'] is not None and result['observation']['registry_identity'] != self.trace.registry.identity:
            raise ValueError('sequence_wrong_registry')
        applied = change['disposition'] == 'applied'
        if applied:
            effect, reference = change['effect'], change['verification']
            pair = self.pair(reference['sequence'])
            packet = pair[-1][0]
            measured = packet['machines'][result['machine_id']]
            expected_reference = {'run_id':packet['run_id'],'sequence':packet['sequence'],
                'stamp_ns':packet['stamp_ns'],'control_revision':packet['control']['revision'],
                'sample_sha256':digest(packet),'machine_id':result['machine_id'],
                **{k:measured[k] for k in ('powered','current_a','vibration_au')}}
            if result['operation'] in ('reset','restore'): expected_reference['temperature_c'] = measured['temperature_c']
            if result['operation'] == 'restore': expected_reference['voltage_v'] = packet['voltage_v']
            if expected_reference != reference:
                raise ValueError('sequence_verification_not_observed')
            for packet, _, _ in pair:
                measurement = packet['machines'][result['machine_id']]
                if (packet['stamp_ns'] <= effect['applied_stamp_ns']
                        or packet['control'] != self.controls[effect['control_revision']]
                        or measurement['powered'] is not (result['operation'] == 'restore')
                        or (result['operation'] != 'restore' and (measurement['current_a'] != 0 or measurement['vibration_au'] != 0))
                        or (result['operation'] == 'reset' and measurement['temperature_c'] > MAX_RESET_TEMPERATURE_C)):
                    raise ValueError('sequence_postcondition_not_observed')
        else:
            expected_power = (previous == self.trace.baseline['control'] or previous['last_applied']['operation'] == 'restore')
            candidates = [entry for entry in self.trace.samples.values()
                          if entry[0]['stamp_ns'] >= result['finished_sim_stamp_ns'] and entry[0]['control'] == previous]
            if len(candidates) < 2: raise Unavailable('sequence_unchanged_pair_missing')
            pair = candidates[:2]; pair_fresh(pair)
            if any(p['machines'][result['machine_id']]['powered'] is not expected_power for p, _, _ in pair):
                raise ValueError('sequence_nonapplication_contradicted')
        self.summaries[key] = {'action_id':key,'operation':result['operation'], 'server_status':result['status'],
            'request_source':self.origins[key], 'request_sha256':result['request_sha256'],
            'disposition':change['disposition'], 'sampled_unchanged':not applied,
            'observed_sequences':[p['sequence'] for p, _, _ in pair],
            'observed_sha256':[digest(p) for p, _, _ in pair], 'hold_released_reported':result['hold_released']}

    def workflow(self, key):
        result, request = self.results[key], self.requests[key]
        self.request_equal(request, result['request'])
        target_id = request['arguments']['target_action_id']
        if target_id not in self.requests or target_id not in self.results:
            raise Unavailable('sequence_cancellation_target_missing')
        self.request_equal(self.requests[target_id], result['target_request'])
        if self.results[target_id] != result['target_result']:
            raise ValueError('sequence_cancellation_target_conflict')
        child = result['isolation_result']
        if child is not None and self.results.get(child['action_id']) != child:
            raise ValueError('sequence_cancellation_child_conflict')
        sequences = []
        verification = result['verification']
        if verification is not None:
            if verification['registry_identity'] != self.trace.registry.identity:
                raise ValueError('sequence_wrong_registry')
            entries = []
            for reference in verification['samples']:
                entry = self.packet(reference['sequence']); packet = entry[0]
                measured = packet['machines'][result['machine_id']]
                if (digest(packet) != reference['sample_sha256'] or packet['stamp_ns'] != reference['stamp_ns']
                        or packet['control'] != verification['control']
                        or any(measured[k] != reference[k] for k in ('powered','current_a','vibration_au'))):
                    raise ValueError('sequence_cancellation_not_observed')
                entries.append(entry); sequences.append(packet['sequence'])
            pair_fresh(entries)
        self.summaries[key] = {'action_id':key,'operation':'cancel_isolate','server_status':result['status'],
            'request_source':self.origins[key], 'request_sha256':result['request_sha256'],
            'reported_final_state':result['final_state'], 'reported_state_observed':verification is not None,
            'observed_sequences':sequences, 'target_action_id':target_id,
            'isolation_action_id':result['isolation_action_id'], 'hold_released_reported':result['hold_released']}

    def check(self):
        self.gather()
        for key, result in self.results.items():
            if result['operation'] != 'cancel_isolate': self.ordinary(key)
        for key, result in self.results.items():
            if result['operation'] == 'cancel_isolate': self.workflow(key)
        return self.summaries, self.controls[max(self.controls)]
