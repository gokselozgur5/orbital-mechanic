"""Deterministic inspection order and diagnosis from bound public evidence.

This policy requests no action and supplies no interlock authority. The mission
controller must use the existing source-checked clients and owned server gates.
"""
from collections import deque
import copy
import hashlib

from orbital.inspection_artifacts import read_capture
from orbital.inspection_protocol import canonical, context, finite
from orbital.maintenance_observation import MIN_VOLTAGE_V, MAX_MACHINE_POWER_W
from orbital.machines import MACHINE_IDS
from orbital.reset_evidence import MAX_RESET_VIBRATION_AU
from orbital.telemetry_observation import TelemetryEvidence

POLICY_VERSION = 'public-measurements-v1'
MAX_INSPECTIONS = 6
MAX_PER_MACHINE = 2
MAX_RETAINED_SAMPLES = 64


def fingerprint(packet):
    return hashlib.sha256(canonical(packet).encode()).hexdigest()


def reference(packet):
    return {**{k:packet[k] for k in ('run_id','sequence','stamp_ns')},
            'sample_sha256':fingerprint(packet)}


def classification(packet):
    """Model-specific hypotheses, never a hidden cause or repair conclusion."""
    values, voltage = packet['machines'], packet['voltage_v']
    if any(not v['powered'] for v in values.values()):
        return ('ambiguous', None, 'unpowered_station')
    excess = [name for name in MACHINE_IDS if voltage*values[name]['current_a'] > MAX_MACHINE_POWER_W]
    if not excess:
        if voltage < MIN_VOLTAGE_V:
            return ('ambiguous', None, 'low_voltage_without_excess_load')
        if any(v['vibration_au'] > MAX_RESET_VIBRATION_AU for v in values.values()):
            return ('ambiguous', None, 'vibration_without_excess_load')
        return ('healthy', None, 'normal_public_measurements')
    if len(excess) != 1:
        return ('ambiguous', None, 'multiple_excess_loads')
    if voltage >= MIN_VOLTAGE_V:
        return ('ambiguous', None, 'excess_load_without_bus_degradation')
    machine = excess[0]
    if any(v['vibration_au'] > MAX_RESET_VIBRATION_AU for name,v in values.items() if name != machine):
        return ('ambiguous', None, 'conflicting_machine_signatures')
    if values[machine]['vibration_au'] > MAX_RESET_VIBRATION_AU:
        return ('persistent', machine, 'excess_load_high_vibration')
    return ('recoverable', machine, 'excess_load_low_vibration')


class MissionPolicy:
    """One in-memory policy per mission, at most two inspections per machine."""
    def __init__(self, mission):
        self.mission = mission
        self.registry = mission.registry
        self.reader = TelemetryEvidence(self.registry)
        self.samples = deque(maxlen=MAX_RETAINED_SAMPLES)
        self.visits = {name:0 for name in MACHINE_IDS}
        self.seen = set()
        self.healthy_inspections = set()
        self.high_vibration = {}
        self.plan = self.retry = self.last_decision = self.failure = None

    def observe_telemetry(self, text):
        """Feed the same actual received packet to mission recovery and policy."""
        if self.failure:
            raise ValueError(self.failure)
        if not self.mission.observe_telemetry(text):
            return False
        if not self.reader.observe(text, self.mission.last_now, self.mission.last_sim):
            self.failure = 'policy_telemetry_invalid'
            raise ValueError(self.failure)
        import json
        packet = json.loads(self.reader.candidate)
        self.samples.append((packet,self.mission.last_now))
        for name,values in packet['machines'].items():
            if values['powered'] and values['vibration_au']>MAX_RESET_VIBRATION_AU:
                self.high_vibration.setdefault(name,{'sample':reference(packet),
                    'vibration_au':values['vibration_au']})
        return True

    def _current_pair(self, phase):
        if self.failure:
            raise ValueError(self.failure)
        self.mission._available(phase)
        self.registry.verify_current()
        # Source hashing is I/O: recheck lifecycle and clocks after it returns.
        self.mission._available(phase)
        now, sim = self.mission.last_now, self.mission.last_sim
        latest = self.reader.measure(now,sim)['telemetry']
        if len(self.samples)<2:
            raise ValueError('policy_requires_fresh_pair')
        pair = list(self.samples)[-2:]
        if (latest != pair[-1][0] or pair[-1][0]['sequence'] != pair[0][0]['sequence']+1
                or any(not 0<=now-at<1. or not 0<=sim-p['stamp_ns']<1000000000 for p,at in pair)
                or any(p['schema_version']!=2 or p['run_id']!=self.mission.station_run
                       or p['control']!=self.mission.control for p,at in pair)):
            raise ValueError('policy_source_or_control_changed')
        return [p for p,at in pair]

    def next_machine(self):
        pair = self._current_pair('inspect')
        if sum(self.visits.values()) >= MAX_INSPECTIONS:
            raise ValueError('policy_inspection_budget_exhausted')
        latest = pair[-1]
        available = [name for name in MACHINE_IDS if self.visits[name]<MAX_PER_MACHINE]
        if not available:
            raise ValueError('policy_inspection_budget_exhausted')
        # A newly excessive load can outrank a previously healthy inspection.
        ordered = sorted(available, key=lambda name:(
            -latest['voltage_v']*latest['machines'][name]['current_a'],
            name in self.healthy_inspections, self.visits[name], MACHINE_IDS.index(name)))
        target = self.retry if self.retry in available else ordered[0]
        self.plan = {'policy_version':POLICY_VERSION, 'machine_id':target,
                     'mission_id':self.mission.mission_id, 'registry_identity':self.registry.identity,
                     'station_run_id':latest['run_id'], 'control':copy.deepcopy(latest['control']),
                     'telemetry':[reference(p) for p in pair]}
        return copy.deepcopy(self.plan)

    def _inspection(self):
        """Read actual manifest/image, bind the selected public source, then age it."""
        result = self.mission.inspection
        if (self.plan is None or result is None or result['status']!='succeeded'
                or result['machine_id']!=self.plan['machine_id']
                or result['action_id'] in self.seen
                or self.visits[result['machine_id']]>=MAX_PER_MACHINE):
            raise ValueError('policy_inspection_not_planned')
        ref = result['observation']
        record,_ = read_capture(self.registry.root,ref['observation_id'],ref['manifest_sha256'])
        bound = context(record['inspection_action'])
        if (any(bound[k]!=result[k] for k in ('run_id','mission_id','action_id','request_sha256'))
                or bound['registry_identity']!=self.registry.identity
                or record['kind']!='machine_inspection_observation'
                or record['registry_identity']!=self.registry.identity
                or record['authority_identity']!=bound['authority_identity']
                or record['machine_id']!=result['machine_id']
                or record['machine_binding']!=self.registry.public()['machines'][result['machine_id']]
                or record['source_sha256']!=self.registry.content()['source_sha256']
                or record['sample_stamp_ns']!=ref['sample_stamp_ns']
                or record['camera']['stamp_ns']!=ref['camera_stamp_ns']
                or not result['started_sim_stamp_ns']<=bound['admitted_sim_stamp_ns']
                       <=record['sample_stamp_ns']<=result['finished_sim_stamp_ns']):
            raise ValueError('policy_inspection_binding_mismatch')
        selected = record['telemetry_source']
        expected_keys = {'schema_version','source','run_id','sequence','stamp_ns','time_s',
                         'control_revision','sample_sha256'}
        if not isinstance(selected,dict) or set(selected)!=expected_keys:
            raise ValueError('policy_selected_source_mismatch')
        candidates = [(packet,at) for packet,at in self.samples
                      if fingerprint(packet)==selected['sample_sha256']]
        if len(candidates)!=1:
            raise ValueError('policy_selected_sample_not_retained')
        packet,received = candidates[0]
        if (any(selected[k]!=packet[k] for k in ('schema_version','source','run_id','sequence','stamp_ns','time_s'))
                or selected['control_revision']!=packet['control']['revision']
                or packet['stamp_ns']!=record['sample_stamp_ns'] or record['units']!=packet['units']
                or canonical(record['measurements'])!=canonical(
                    {'voltage_v':packet['voltage_v'],**packet['machines'][result['machine_id']]})):
            raise ValueError('policy_measurement_source_mismatch')
        # Artifact reads and hashing finish before final clock/source/phase fences.
        pair = self._current_pair('diagnose')
        if (packet['run_id']!=self.mission.station_run or packet['control']!=self.mission.control
                or self.plan['control']!=self.mission.control
                or not 0<=self.mission.last_now-received<1.
                or not 0<=self.mission.last_sim-packet['stamp_ns']<1000000000
                or not bound['admitted_monotonic_s']<=finite(record['observed_monotonic_s'])<=self.mission.last_now):
            raise ValueError('policy_inspection_expired_or_changed')
        return result,packet,pair

    def apply(self):
        """Apply one evidence-bound policy decision to the existing mission."""
        self.mission._available('diagnose')
        if self.failure:
            raise ValueError(self.failure)
        try:
            result,selected,pair = self._inspection()
        except (ValueError,TypeError,KeyError,OSError,OverflowError) as error:
            self.failure = 'policy_inspection_unavailable'
            raise ValueError(self.failure) from error
        machine = result['machine_id']
        evidence = [selected,*pair]
        classifications = [classification(p) for p in evidence]
        stable = all(v==classifications[0] for v in classifications)
        kind,target,reason = classifications[0] if stable else ('ambiguous',None,'changing_public_measurements')
        if (kind=='recoverable' and target in self.high_vibration) or (kind=='healthy' and self.high_vibration):
            kind,target,reason = 'ambiguous',None,'contradictory_vibration_history'
        visits = self.visits[machine]+1
        healthy = self.healthy_inspections | ({machine} if kind=='healthy' else set())
        retry = None
        if kind=='healthy':
            decision = 'healthy' if healthy==set(MACHINE_IDS) else 'inspect_again'
            reason = reason if decision=='healthy' else 'inspect_remaining_machines'
        elif kind in ('recoverable','persistent') and target==machine:
            decision = kind
        elif kind in ('recoverable','persistent'):
            decision,reason = (('ambiguous','candidate_inspection_budget_exhausted')
                if self.visits[target]>=MAX_PER_MACHINE else ('inspect_again','inspect_other_candidate'))
        elif visits<MAX_PER_MACHINE and sum(self.visits.values())+1<MAX_INSPECTIONS:
            decision,retry = 'inspect_again',machine
        else:
            decision,reason = 'ambiguous','ambiguous_inspection_budget_exhausted'
        if decision=='inspect_again' and sum(self.visits.values())+1>=MAX_INSPECTIONS:
            decision,reason,retry = 'ambiguous','total_inspection_budget_exhausted',None
        report = {'schema_version':1,'policy_version':POLICY_VERSION,'decision':decision,
            'mission_id':self.mission.mission_id,'registry_identity':self.registry.identity,
            'station_run_id':self.mission.station_run,'control':copy.deepcopy(self.mission.control),
            'hypothesis':kind,'candidate_machine':target,'reason':reason,'machine_id':machine,
            'inspection_action_id':result['action_id'],'observation':copy.deepcopy(result['observation']),
            'telemetry':[reference(p) for p in evidence],
            'historical_high_vibration':copy.deepcopy(self.high_vibration),
            'criteria':{'min_voltage_v':MIN_VOLTAGE_V,'max_normal_power_w':MAX_MACHINE_POWER_W,
                        'max_reset_vibration_au':MAX_RESET_VIBRATION_AU},
            'scope':'deterministic public-model hypothesis; not hidden cause, visual diagnosis, '
                    'live interlock permission, repair proof or independent mission scoring'}
        self.mission.decide(decision)
        self.visits[machine] = visits
        self.seen.add(result['action_id'])
        self.healthy_inspections,self.retry = healthy,retry
        self.last_decision = copy.deepcopy(report)
        self.plan = None
        return report

    def snapshot(self):
        return copy.deepcopy({'policy_version':POLICY_VERSION,'visits':self.visits,
            'retained_samples':len(self.samples),'plan':self.plan,'retry':self.retry,
            'last_decision':self.last_decision,'failure':self.failure,
            'historical_high_vibration':self.high_vibration,
            'limits':{'inspections':MAX_INSPECTIONS,'per_machine':MAX_PER_MACHINE,
                      'retained_samples':MAX_RETAINED_SAMPLES}})
