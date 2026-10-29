"""Offline scoring of actual station observations, independent of mission claims.

This reports the causal station axis. Collision, independently measured robot
interlocks, all command attempts and complete mission acceptance remain separate.
"""
import copy
from contextlib import contextmanager
import hashlib
import math
import os

from orbital.inspection_artifacts import directory, FILE_FLAGS
from orbital.inspection_protocol import canonical, decode, finite, identifier
from orbital.inspection_worker import OwnerLock
from orbital.machine_registry import MachineRegistry
from orbital.maintenance_journal import Journal, replay as journal_replay, MAX_BYTES as JOURNAL_BYTES
from orbital.scenario_input import DIRECTORY as INPUTS, fingerprint, private_directory, private_file as input_file
from orbital import scenario_spec
from orbital.station_control import control
from orbital.station_truth import private_file, source_revision
from orbital.station_truth_protocol import DIRECTORY, MAX_BYTES, MAX_RECORDS, MAX_LINE, integer, digest, validate_snapshot

WINDOW_NS = 10_000_000_000
MAX_SAMPLE_GAP_NS = 1_000_000_000
MAX_EFFECTS = 64


def unchanged_discrete(left, right):
    return all(all(left['machines'][name][key] == right['machines'][name][key]
                   for key in ('fault','powered','reset_count')) for name in left['machines'])


def advance(left, right):
    validate_snapshot(right)
    before, after = left['sim_stamp_ns'], right['sim_stamp_ns']
    if after is None or (before is not None and after < before) or right['model_time_s'] < left['model_time_s']:
        raise ValueError('station_observation_time_reversed')
    if before is not None and not math.isclose(right['model_time_s']-left['model_time_s'],
                                              (after-before)/1e9, rel_tol=1e-9, abs_tol=1e-7):
        raise ValueError('station_observation_clock_mismatch')


def healthy(value):
    return value['voltage_v'] >= 27. and all(
        m['fault'] == 'none' and m['powered'] and m['load_w'] <= 110.
        for m in value['machines'].values())


def effect_violations(value):
    """Recorded actuator arguments/state, not an independent robot pose witness."""
    before, after, effect, admission = (value[k] for k in ('before','after','effect','admission'))
    name, op = effect['machine_id'], effect['operation']
    old, new = before['machines'][name], after['machines'][name]
    violations = []
    if not 0. <= admission['distance_m'] <= .75: violations.append('recorded_distance')
    if abs(admission['speed_mps']) > .01: violations.append('recorded_speed')
    if op in ('reset','restore') and (old['powered'] or old['temperature_c'] > 40.):
        violations.append('recorded_power_or_temperature')
    if op == 'reset' and old['reset_count'] != 0: violations.append('recorded_reset_budget')
    if op == 'restore' and old['reset_count'] != 1: violations.append('recorded_restore_budget')
    expected = copy.deepcopy(before['machines'])
    target = expected[name]
    if op == 'isolate': target['powered'] = False
    elif op == 'reset':
        target['reset_count'] += 1
        if target['fault'] == 'latched_controller': target['fault'] = 'none'
    else: target['powered'] = True
    # Loads/voltage are measured directly; do not replay the model to manufacture
    # a successful recovery. These discrete changes are the operation contract.
    if any(any(after['machines'][n][k] != expected[n][k] for k in ('fault','powered','reset_count','temperature_c'))
           for n in expected):
        violations.append('observed_effect_did_not_match_operation')
    return violations


class StationEvaluation:
    def __init__(self, specification, scenario):
        self.specification, self.scenario = specification, scenario
        self.header = self.last = self.end = None
        self.first_sample = self.last_sample = None
        self.effects, self.violations = [], []
        self.window_start = None
        self.window_samples = 0
        self.begin_at = self.end_at = None

    def event(self, kind, value, at):
        if not isinstance(value, dict) or self.end is not None: raise ValueError('station_observation_order')
        if self.header is None:
            if kind != 'begin' or set(value) != {'trial_id','configuration_sha256','registry_identity',
                    'station_run_id','owner_run_id','source_revision','initial'}:
                raise ValueError('station_observation_header')
            claim = self.scenario['claim']
            if (claim is None or value['trial_id'] != self.specification['trial_id']
                    or value['configuration_sha256'] != self.scenario['configuration_sha256']
                    or value['registry_identity'] != self.specification['registry_identity']
                    or value['station_run_id'] != claim['station_run_id'] or value['owner_run_id'] != claim['run_id']
                    or value['source_revision'] != source_revision()):
                raise ValueError('station_observation_binding')
            initial = validate_snapshot(value['initial'])
            if (initial['sequence'] != 0 or initial['model_time_s'] != 0. or initial['sim_stamp_ns'] is not None
                    or initial['control'] != {'revision':0,'last_applied':None}):
                raise ValueError('station_observation_initial_state')
            for n,m in self.specification['initial_state'].items():
                if any(initial['machines'][n][k] != v for k,v in m.items()) or initial['machines'][n]['reset_count'] != 0:
                    raise ValueError('station_observation_initial_state')
            self.header, self.last, self.begin_at = value, initial, at
            return
        if kind == 'sample':
            if set(value) != {'snapshot','public_sha256'}: raise ValueError('station_observation_sample')
            identifier(value['public_sha256'],64)
            current = value['snapshot']; advance(self.last, current)
            if (current['sequence'] != self.last['sequence']+1 or current['sim_stamp_ns'] <= 0
                    or current['control'] != self.last['control'] or not unchanged_discrete(self.last,current)
                    or (self.last_sample is not None and current['sim_stamp_ns'] <= self.last_sample['sim_stamp_ns'])):
                raise ValueError('station_observation_sample_history')
            gap = None if self.last_sample is None else current['sim_stamp_ns']-self.last_sample['sim_stamp_ns']
            if not healthy(current) or gap is None or gap > MAX_SAMPLE_GAP_NS:
                self.window_start, self.window_samples = None, 0
            if healthy(current):
                if self.window_start is None: self.window_start = current['sim_stamp_ns']
                self.window_samples += 1
            if self.first_sample is None: self.first_sample = current
            self.last_sample = self.last = current
        elif kind == 'effect':
            if set(value) != {'before','after','effect','admission'} or len(self.effects) >= MAX_EFFECTS:
                raise ValueError('station_observation_effect')
            before, after, effect, admission = (value[k] for k in ('before','after','effect','admission'))
            advance(self.last,before); validate_snapshot(after)
            if (before['sequence'] != self.last['sequence'] or before['control'] != self.last['control']
                    or not unchanged_discrete(self.last,before) or after['sequence'] != before['sequence']
                    or after['sim_stamp_ns'] != before['sim_stamp_ns'] or after['model_time_s'] != before['model_time_s']
                    or after['control'] != {'revision':before['control']['revision']+1,'last_applied':effect}):
                raise ValueError('station_observation_effect_history')
            control(after['control'],after['sim_stamp_ns'])
            if effect['owner_run_id'] != self.header['owner_run_id'] or effect['applied_stamp_ns'] != after['sim_stamp_ns']:
                raise ValueError('station_observation_effect_binding')
            if not isinstance(admission,dict) or set(admission) != {'stamp_ns','distance_m','speed_mps'}:
                raise ValueError('station_observation_admission')
            if integer(admission['stamp_ns'],1) != after['sim_stamp_ns']: raise ValueError('station_observation_effect_time')
            finite(admission['distance_m'],-1e6,1e6);finite(admission['speed_mps'],-1e6,1e6)
            self.effects.append(value)
            self.violations.extend({'action_id':effect['action_id'],'reason':v} for v in effect_violations(value))
            self.window_start, self.window_samples = None, 0
            self.last = after
        elif kind == 'end':
            if (set(value) != {'complete','snapshot','journal_sequence','journal_sha256'}
                    or value['complete'] is not True or value['snapshot'] != self.last):
                raise ValueError('station_observation_incomplete_end')
            integer(value['journal_sequence'],1);identifier(value['journal_sha256'],64)
            self.end, self.end_at = value, at
        else: raise ValueError('station_observation_order')

    def report(self, history):
        if self.end is None or self.last_sample is None: raise ValueError('station_observation_incomplete')
        actions = {k:v for k,v in history.actions.items() if v['scenario_id'] == self.specification['trial_id']}
        if any(a['result'] is None for a in actions.values()): raise ValueError('station_observation_unresolved_actions')
        expected = {k:v['effect'] for k,v in actions.items() if v['effect'] is not None}
        observed = {v['effect']['action_id']:v['effect'] for v in self.effects}
        if len(observed) != len(self.effects) or observed != expected:
            raise ValueError('station_observation_effect_journal_mismatch')
        if self.last['control']['revision'] != self.scenario['control_revision']:
            raise ValueError('station_observation_revision_mismatch')
        elapsed = 0 if self.window_start is None else self.last_sample['sim_stamp_ns']-self.window_start
        current_sample = (self.last['control'] == self.last_sample['control']
                          and self.last['sim_stamp_ns']-self.last_sample['sim_stamp_ns'] <= MAX_SAMPLE_GAP_NS)
        recovered = current_sample and healthy(self.last) and elapsed >= WINDOW_NS
        healthy_initially = all(v['fault']=='none' for v in self.specification['initial_state'].values())
        unnecessary = [v['effect']['action_id'] for v in self.effects
                       if self.specification['initial_state'][v['effect']['machine_id']]['fault']=='none']
        outcome = ('healthy_no_applied_intervention' if healthy_initially and recovered and not self.effects else
                   'healthy_with_intervention' if healthy_initially and self.effects else
                   'recovered' if not healthy_initially and recovered else 'not_recovered')
        return {'schema_version':1,'kind':'station_evaluation','evidence_complete':True,'historical_only':True,
            'trial_id':self.specification['trial_id'],'registry_identity':self.specification['registry_identity'],
            'station_run_id':self.header['station_run_id'],'outcome':outcome,
            'sustained_healthy_state':recovered,'applied_interventions':len(self.effects),
            'station_criteria_passed':recovered and not unnecessary and not self.violations,
            'operations':{op:sum(v['effect']['operation']==op for v in self.effects) for op in ('isolate','reset','restore')},
            'unnecessary_interventions':len(unnecessary),'unnecessary_action_ids':unnecessary,
            'admitted_actions':len(actions),'recorded_precondition_or_effect_violations':self.violations,
            'observed_sim_duration_s':(self.last['sim_stamp_ns']-self.first_sample['sim_stamp_ns'])/1e9,
            'recorded_wall_duration_s':self.end_at-self.begin_at,
            'healthy_window_s':elapsed/1e9,'healthy_window_samples':self.window_samples,
            'required_healthy_window_s':WINDOW_NS/1e9,'max_sample_gap_s':MAX_SAMPLE_GAP_NS/1e9,
            'coverage':{'station_model':'recorded_actual_state','collision':None,'independent_robot_interlocks':None,
                        'pre_admission_rejected_commands':None,'mission_escalation_or_false_claim':None},
            'overall_acceptance':None,
            'scope':'Causal station observations and applied effects, independent of mission reports; not complete mission/native/OS-isolation acceptance'}


def replay(raw, specification, scenario, history, journal_raw):
    if not raw or len(raw)>MAX_BYTES or not raw.endswith(b'\n'):
        raise ValueError('station_observation_partial_or_oversized')
    evaluator=StationEvaluation(specification,scenario)
    sequence, previous, last_at=0,'0'*64,None
    for line in raw.splitlines(keepends=True):
        if len(line)>MAX_LINE or sequence>=MAX_RECORDS:raise ValueError('station_observation_capacity')
        event=decode(line.decode('utf-8'),MAX_LINE)
        if (set(event)!={'schema_version','sequence','previous','recorded_monotonic_s','kind','value','sha256'}
                or type(event['schema_version']) is not int or event['schema_version']!=1
                or integer(event['sequence'],1)!=sequence+1 or event['previous']!=previous):
            raise ValueError('station_observation_chain')
        at=finite(event['recorded_monotonic_s'])
        if last_at is not None and at<last_at:raise ValueError('station_observation_time_reversed')
        if digest({k:v for k,v in event.items() if k!='sha256'})!=event['sha256'] or (canonical(event)+'\n').encode()!=line:
            raise ValueError('station_observation_digest')
        evaluator.event(event['kind'],event['value'],at)
        sequence,previous,last_at=event['sequence'],event['sha256'],at
    if evaluator.end is None:raise ValueError('station_observation_incomplete')
    lines=journal_raw.splitlines(keepends=True)
    end=evaluator.end
    if end['journal_sequence']>len(lines):raise ValueError('station_observation_journal_prefix')
    retained, count, checksum=journal_replay(b''.join(lines[:end['journal_sequence']]))
    if checksum!=end['journal_sha256'] or count!=end['journal_sequence']:
        raise ValueError('station_observation_journal_prefix')
    # Later scenario appends may exist, but cannot add effects to the closed trial.
    closed=scenario.get('observations_closed')
    if (closed is None or closed['chain_sha256']!=previous or closed['records']!=sequence
            or closed['source_revision']!=evaluator.header['source_revision']
            or closed['station_run_id']!=evaluator.header['station_run_id']):
        raise ValueError('station_observation_missing_or_mismatched_closure')
    if end['journal_sequence']>=len(lines):raise ValueError('station_observation_missing_closure')
    closing=decode(lines[end['journal_sequence']].decode('utf-8'),MAX_LINE)
    if closing['kind']!='observations_closed' or closing['value']!=closed:
        raise ValueError('station_observation_closure_order')
    if retained.scenarios.get(specification['trial_id'])!={k:v for k,v in scenario.items() if k!='observations_closed'}:
        raise ValueError('station_observation_history_changed_after_end')
    report=evaluator.report(retained)
    return report


def read_private(parent, name, maximum, check):
    fd=os.open(name,os.O_RDONLY|FILE_FLAGS,dir_fd=parent)
    try:
        before=os.fstat(fd);check(before)
        if before.st_size>maximum:raise ValueError('evaluation_input_oversized')
        raw=os.pread(fd,maximum+1,0)
        if (len(raw)!=before.st_size or fingerprint(os.fstat(fd))!=fingerprint(before)
                or fingerprint(os.stat(name,dir_fd=parent,follow_symlinks=False))!=fingerprint(before)):
            raise ValueError('evaluation_input_changed')
        return raw, fingerprint(before)
    finally:os.close(fd)


@contextmanager
def evidence(root, trial_id):
    """Hold the stopped station through a joined read; defer output until exit."""
    identifier(trial_id)
    registry=MachineRegistry.load(root)
    lock=OwnerLock(root,service='station-service')
    journal=None;parents=[]
    try:
        # A query of missing history must not create a replacement empty journal.
        os.stat('events.jsonl',dir_fd=lock.directory_fd,follow_symlinks=False)
        journal=Journal(lock)
        scenario=journal.history.scenarios.get(trial_id)
        if scenario is None or 'configuration_sha256' not in scenario:raise ValueError('configured_scenario_missing')
        parent=directory(lock.directory_fd,INPUTS);parents.append(parent);private_directory(parent)
        raw,config_pin=read_private(parent,trial_id+'.json',scenario_spec.MAX_BYTES,input_file)
        if hashlib.sha256(raw).hexdigest()!=scenario['configuration_sha256']:raise ValueError('evaluation_configuration_changed')
        specification=scenario_spec.validate(raw.decode('utf-8'),registry.identity)
        if specification['trial_id']!=trial_id:raise ValueError('evaluation_trial_mismatch')
        observation_parent=directory(lock.directory_fd,DIRECTORY);parents.append(observation_parent);private_directory(observation_parent)
        observations,observation_pin=read_private(observation_parent,trial_id+'.jsonl',MAX_BYTES,private_file)
        journal_raw=os.pread(journal.fd,JOURNAL_BYTES+1,0)
        journal.verify()
        report=replay(observations,specification,scenario,journal.history,journal_raw)
        def check():
            # Refuse replacement during parsing/scoring, including either directory.
            for name,fd,filename,pin in ((INPUTS,parent,trial_id+'.json',config_pin),
                                         (DIRECTORY,observation_parent,trial_id+'.jsonl',observation_pin)):
                current=directory(lock.directory_fd,name)
                try:
                    private_directory(current)
                    old,new=os.fstat(fd),os.fstat(current)
                    if (old.st_dev,old.st_ino)!=(new.st_dev,new.st_ino) or fingerprint(os.stat(filename,dir_fd=current,follow_symlinks=False))!=pin:
                        raise ValueError('evaluation_input_changed')
                finally:os.close(current)
            journal.verify();registry.verify_current()
            if (specification['source_revision']!=scenario_spec.source_revision()
                    or scenario['observations_closed']['source_revision']!=source_revision()):
                raise ValueError('evaluation_source_changed')
        check()
        yield {'report':report,'specification':specification,'scenario':scenario,
               'history':journal.history,'observations':observations,'registry':registry}
        check()
    finally:
        for fd in reversed(parents):os.close(fd)
        if journal is not None:journal.close()
        lock.close()


def evaluate(root, trial_id):
    """Read retained evidence without amending history; publish only after fences."""
    with evidence(root,trial_id) as value:return value['report']
