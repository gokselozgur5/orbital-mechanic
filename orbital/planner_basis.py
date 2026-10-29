"""Explicit slow planning observations and fresh executive revalidation.

This does not weaken the exact current-packet gate on native tool requests.
It defines a distinct proposal boundary and records both sides of the handoff.
"""
import copy
import uuid

from orbital.inspection_protocol import canonical, decode, finite, identifier
from orbital.maintenance_protocol import integer
from orbital.machines import MACHINE_IDS
from orbital.mission_policy import classification
from orbital.planner_evidence import observation
from orbital.planner_messages import ProviderError

MAX_PLAN_AGE_S=30.
PHASES={'inspect','navigating','inspecting','diagnose','intervene','intervening','verify','escalate','terminal','unresolved'}


def frame(runtime):
    history=runtime.gateway.state()
    value={'schema_version':1,'kind':'planner_planning_frame','basis_id':uuid.uuid4().hex,
        'executive':{'phase':runtime.mission.phase,'arrival':runtime.mission.arrival,
            'next_operation':runtime.mission.next_operation,'outcome':runtime.mission.outcome,
            'high_vibration':sorted(runtime.policy.high_vibration),
            'evidence_ids':list(history['observations'])[-16:]},
        'observation':runtime.inputs.view()}
    return copy.deepcopy(value)


def validate(value, *, context, run_id, mission_id):
    value=decode(canonical(value),32768)
    if (set(value)!={'schema_version','kind','basis_id','executive','observation'}
            or type(value['schema_version']) is not int or value['schema_version']!=1
            or value['kind']!='planner_planning_frame'):raise ValueError('invalid_planning_frame')
    identifier(value['basis_id'])
    observation(value['observation'],baseline=context['baseline'],run_id=run_id,mission_id=mission_id)
    state=value['executive']
    if (set(state)!={'phase','arrival','next_operation','outcome','high_vibration','evidence_ids'}
            or state['phase'] not in PHASES or state['arrival'] not in (*MACHINE_IDS,None)
            or state['next_operation'] not in ('isolate','reset','restore',None)
            or state['outcome'] not in ('healthy','recovered','escalated','failed','cancelled','timed_out',None)):
        raise ValueError('invalid_planning_frame')
    for key,limit in (('high_vibration',3),('evidence_ids',16)):
        values=state[key]
        if type(values) is not list or len(values)>limit or len(values)!=len(set(values)):
            raise ValueError('invalid_planning_frame')
        for item in values:
            if key=='evidence_ids':identifier(item)
            elif item not in MACHINE_IDS:raise ValueError('invalid_planning_frame')
    return value


def signature(value):
    view=value['observation'];station=view['station']
    return {'registry':view['registry_identity'],'station':station['run_id'],'control':station['control'],
        'classification':classification(station),
        'powered':{name:item['powered'] for name,item in station['machines'].items()},
        'executive':{k:v for k,v in value['executive'].items() if k!='evidence_ids'}}


def compatible(basis, current, *, started_wall, now, started_sim, sim):
    wall=finite(now)-finite(started_wall);elapsed=integer(sim,1)-integer(started_sim,1)
    if not 0<=wall<MAX_PLAN_AGE_S or not 0<=elapsed<MAX_PLAN_AGE_S*1e9:
        raise ProviderError('basis_expired')
    if signature(basis)!=signature(current):raise ProviderError('basis_changed')
