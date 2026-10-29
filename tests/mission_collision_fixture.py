"""Synthetic collision geometry/protocol envelope, never native motion evidence.

The actual retained mission supplies identities and attempted commands only.
All geometry, poses, audit replies and observer counters below are test inputs.
"""
import copy
import json

from orbital.collision_geometry import Scene,IDENTITY
from orbital.inspection_protocol import canonical
from orbital.trajectory import TraceWriter,Trajectory,TRUTH_TOPIC,truth_from_message
from collision_fixture import scene_message
from test_navigation_client import state


def trace(root,mission_id,*,runtime_change=False,wrong_run=False,wall_shift=0,
          omit_commands=False,adverse=False):
    events=[json.loads(line) for line in (root/'runtime/mission-service/mission-observations'/mission_id/'events.jsonl').read_text().splitlines()]
    journal=[json.loads(line) for line in (root/'runtime/mission-service/events.jsonl').read_text().splitlines()]
    start,end=journal[0]['data'],journal[-1]['data']['quiescence']
    commands=[copy.deepcopy(e['data']) for e in events if e['kind']=='published' and e['data']['family']=='navigation']
    nav=next(json.loads(e['data']['text'])['run_id'] for e in events
             if e['kind']=='received' and e['data']['topic']=='/navigation/state')
    if wrong_run:nav='other-navigation-generation'
    parsed=[]
    for item in ([] if omit_commands else commands):
        command=json.loads(item['text']);command['run_id']=nav
        if 'target' in command:command['target']['run_id']=nav
        parsed.append((item,command))
    runtime=copy.deepcopy(events[0]['data']['runtime'])
    if runtime_change:runtime['service_invocations']['orbital-navigation.service']='e'*32
    world=(root/'worlds/maintenance.sdf').read_bytes();scene=Scene(world)
    writer=TraceWriter(root,'synthetic-mission-collision')
    trajectory=Trajectory(writer)
    begin,finish=start['started_sim_ns'],end['observed_sim_ns']
    ticks=sorted(set(range(begin-200000000,finish+300000000,100000000))
                 |{finish+200000000}|{r['entered_sim_ns'] for r,_ in parsed})
    audit_sequence=command_sequence=0;last_reply=None
    for index,stamp in enumerate(ticks):
        at=round(start['started_wall_s']*1e9)+stamp-begin+wall_shift
        for raw,command in parsed:
            if raw['entered_sim_ns']!=stamp:continue
            at=max(at,round(raw['entered_wall_s']*1e9)+wall_shift)
            audit_sequence+=1;command_sequence=command['sequence']
            last_reply={'sequence':command_sequence,'op':command['op'],'accepted':True}
            trajectory.audit(at,canonical({'schema_version':1,'run_id':nav,'sequence':audit_sequence,
                'sim_stamp_ns':stamp,'event':'command_reply','command':command,'reply':last_reply,
                'target':command.get('target'),'command_sequence':command_sequence,'forwarded_commands':0}))
        trajectory.state(at,canonical(state(run_id=nav,sim_stamp_ns=stamp,command_sequence=command_sequence,
            audit_sequence=audit_sequence,audit_ok=True,last_reply=last_reply)))
        pose=(2.5,0.,0.,0.,0.,0.,1.) if adverse else IDENTITY
        trajectory.truth(at,truth_from_message(scene_message(world,stamp,robot_pose=pose),scene))
        trajectory.match(lambda _:list(IDENTITY),at)
        if index==0 and not trajectory.start(at):raise AssertionError('synthetic baseline refused')
    observation=trajectory.finish(at)
    bindings={name:{'path':'/synthetic-fixture/'+name,'sha256':'c'*64}
        for name in ('gz.transport13','gz.transport13._transport','gz.msgs10.pose_v_pb2')}
    writer.finish(True,{'scope':'Synthetic geometry and observer replies; actual mission identities/commands only',
        'truth_topic':TRUTH_TOPIC,'before':runtime,'after':copy.deepcopy(runtime),
        'bindings_before':bindings,'bindings_after':copy.deepcopy(bindings),'observation':observation,
        'geometry':{'schema_version':1,'world_sdf':world.decode(),'world_sha256':scene.sha256,
                    'pose_convention':'world model poses; model-relative robot links'}})
    return writer.path
