"""Join a closed raw mission witness, its journal and actual private station state."""
import hashlib
import os
from pathlib import Path

from orbital.inspection_protocol import canonical, decode, identifier
from orbital.inspection_worker import OwnerLock
from orbital.mission_journal import MissionJournal, MAX_BYTES as JOURNAL_BYTES, MAX_EVENT_BYTES
from orbital import mission_witness_replay as witness
from orbital import mission_observation, mission_outcome, mission_collision
from orbital.station_evaluation import evidence as station_evidence
from orbital.station_truth_protocol import MAX_LINE as STATION_LINE

ROOT=Path(__file__).resolve().parents[1]
SOURCES=('orbital/mission_evaluation.py','orbital/mission_observation.py','orbital/mission_outcome.py',
         'orbital/mission_witness_replay.py','orbital/station_evaluation.py',
         'orbital/inspection_artifacts.py','orbital/inspection_protocol.py','orbital/maintenance_protocol.py',
         'orbital/telemetry_observation.py','orbital/navigation_command.py')
SOURCES+=('orbital/mission_collision.py','orbital/collision_evaluation.py','orbital/collision_geometry.py',
          'orbital/trajectory.py','orbital/navigation_client.py')


def sources():return {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in SOURCES}


def history(journal,events,mission_id):
    raw=os.pread(journal.fd,JOURNAL_BYTES+1,0);journal.verify()
    if len(raw)>JOURNAL_BYTES or not raw.endswith(b'\n'):raise ValueError('mission_journal_incomplete')
    records=[];pins={0:(0,'0'*64)};size=0
    for line in raw.splitlines(keepends=True):
        value=decode(line.decode('utf-8'),MAX_EVENT_BYTES)
        if (canonical(value)+'\n').encode()!=line:raise ValueError('mission_journal_noncanonical')
        records.append(value);size+=len(line)
        pins[len(records)]=(size,value['sha256'])
    def check(value):
        witness.pin(value)
        if pins.get(value['sequence'])!=(value['bytes'],value['sha256']):
            raise ValueError('mission_witness_journal_prefix')
    before,after=events[0]['data']['journal'],events[-1]['data']['journal']
    check(before);check(after)
    selected=records[before['sequence']:after['sequence']]
    if (len(selected)<2 or selected[0]['kind']!='start' or selected[-1]['kind']!='terminal'
            or any(v['mission_id']!=mission_id for v in selected)
            or sum(v['kind']=='start' for v in selected)!=1 or sum(v['kind']=='terminal' for v in selected)!=1):
        raise ValueError('mission_witness_journal_window')
    return selected,check


def evaluate(root,trial_id,mission_id,*,collision_trace=None):
    identifier(trial_id);identifier(mission_id)
    version=sources()
    # Fixed station -> mission lock order, with both held through final validation.
    with station_evidence(root,trial_id) as station:
        lock=OwnerLock(root,service='mission-service');journal=None
        try:
            os.stat('events.jsonl',dir_fd=lock.directory_fd,follow_symlinks=False)
            journal=MissionJournal(lock)
            events,retained=witness.read(lock,mission_id,station['registry'])
            records,check=history(journal,events,mission_id)
            private=[decode(line.decode('utf-8'),STATION_LINE) for line in station['observations'].splitlines()]
            samples={v['value']['snapshot']['sequence']:v['value'] for v in private if v['kind']=='sample'}
            observation=mission_observation.observe(events,station,samples)
            attempts,unsent=mission_observation.publications(events,records,check,observation,station['registry'],mission_id)
            start,terminal=mission_observation.boundaries(records,observation,station,attempts)
            if (start['started_wall_s']<events[0]['recorded_wall_s']
                    or start['started_sim_ns']<events[0]['recorded_sim_ns']
                    or terminal['quiescence']['observed_wall_s']>events[-1]['recorded_wall_s']
                    or terminal['quiescence']['observed_sim_ns']>events[-1]['recorded_sim_ns']):
                raise ValueError('mission_claim_outside_witness_window')
            result=mission_outcome.assess(station,private,attempts,observation,start,terminal,mission_id)
            artifacts=mission_outcome.captures(root,observation,mission_id,station['registry'])
            collision=collision_pin=None
            if collision_trace is not None:
                collision,collision_pin=mission_collision.evaluate(collision_trace,events[0]['data']['runtime'],
                    observation['owners']['navigation'],start,terminal,attempts)
            specification=station['specification']
            matched={k:v for k,v in specification.items() if k!='trial_id'}
            report={'schema_version':1,'kind':'mission_evaluation','evidence_complete':True,'historical_only':True,
                'trial_id':trial_id,'mission_id':mission_id,'station_run_id':station['report']['station_run_id'],
                'registry_identity':station['registry'].identity,'navigation_run_id':observation['owners']['navigation'],
                'runtime_identity':events[0]['data']['runtime'],'witness_sha256':retained['summary']['payload_sha256'],
                # Offline evaluator output is privileged, never a planner input.
                'scenario_input':{**{k:specification[k] for k in ('generator','profile','seed','source_revision')},
                    'configuration_sha256':station['scenario']['configuration_sha256'],
                    'matched_conditions_sha256':hashlib.sha256(canonical(matched).encode()).hexdigest()},
                'station_observations_sha256':hashlib.sha256(station['observations']).hexdigest(),
                'evaluator_source_sha256':version,'publication_attempts':len(attempts),'unpublished_intents':unsent,
                'captured_observations':len(artifacts),'outcome':result,
                'commands':[{'family':a['family'],'operation':a['command']['op'],
                    'native_publish_returned':a['raw']['returned'],
                    'accepted':None if a['reply'] is None else a['reply']['accepted'],
                    'server_result':None if a['result'] is None else a['result']['status']} for a in attempts[:64]],
                'command_details_truncated':len(attempts)>64,
                'collision':collision,
                'coverage':{'actual_station_and_public_packets':True,'gateway_publication_attempts':True,
                    'claimed_terminal_and_raw_owner_states':True,'captured_files':True,
                    'collision':None if collision is None else True,'independent_robot_interlocks':None,'OS_isolation':None},
                'overall_acceptance':None}
            # Reopen the exact referenced image bundles; changing them during the
            # join cannot leave a successful result bound to unavailable artifacts.
            if artifacts!=mission_outcome.captures(root,observation,mission_id,station['registry']):raise ValueError('mission_captures_changed')
            journal.verify();witness.verify(lock,mission_id,station['registry'],retained)
            if collision_pin is not None:mission_collision.verify(collision_trace,collision_pin)
            if version!=sources():raise ValueError('mission_evaluation_source_changed')
        finally:
            if journal is not None:journal.close()
            lock.close()
    return report
