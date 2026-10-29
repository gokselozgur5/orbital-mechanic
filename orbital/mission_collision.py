"""Require a generation- and command-bound sampled trace covering the mission."""
from collections import Counter
from pathlib import Path
import stat

from orbital.collision_evaluation import CollisionEvaluation
from orbital.inspection_protocol import canonical
from orbital.scenario_input import fingerprint
from orbital.trajectory import verify_trace


def files(path):
    path=Path(path)
    info=path.lstat()
    if not stat.S_ISDIR(info.st_mode):raise ValueError('mission_collision_directory')
    pins={'directory':(info.st_dev,info.st_ino)}
    for name in ('events.jsonl','summary.json'):
        info=(path/name).lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_nlink!=1:
            raise ValueError('mission_collision_file')
        pins[name]=fingerprint(info)
    return pins


def evaluate(path, runtime, navigation_run, start, terminal, attempts):
    before=files(path)
    evaluator=CollisionEvaluation()
    first=last=None;audits=[]
    begin,end=start['started_sim_ns'],terminal['quiescence']['observed_sim_ns']
    wall_begin,wall_end=start['started_wall_s'],terminal['quiescence']['observed_wall_s']
    def visit(record,summary):
        nonlocal first,last
        evaluator.visit(record,summary)
        value=record['data'];at=record['received_monotonic_ns']
        if record['kind']=='truth' and evaluator.phase=='capture':
            point=(value['sim_stamp_ns'],at)
            if first is None:first=point
            last=point
        if record['kind']=='audit' and value['event']=='command_reply':
            # Include either clock's window; disagreement cannot hide a command.
            if (begin<=value['sim_stamp_ns']<=end or wall_begin<=at/1e9<=wall_end):
                audits.append((value,at))
    summary=verify_trace(path,visit)
    result=evaluator.finish(summary)
    if summary['before']!=runtime or summary['after']!=runtime:
        raise ValueError('mission_collision_runtime_binding')
    if result['observation']['run_id']!=navigation_run:
        raise ValueError('mission_collision_navigation_generation')
    if (first is None or last is None or first[0]>begin or last[0]<end
            or first[1]>round(wall_begin*1e9) or last[1]<round(wall_end*1e9)):
        raise ValueError('mission_collision_does_not_cover_mission')
    expected=[a for a in attempts if a['family']=='navigation']
    if Counter(canonical(a['command']) for a in expected)!=Counter(canonical(v['command']) for v,_ in audits):
        raise ValueError('mission_collision_command_correspondence')
    for attempt in expected:
        matching=[(v,at) for v,at in audits if v['command']==attempt['command']]
        if len(matching)!=1:raise ValueError('mission_collision_ambiguous_command')
        event,at=matching[0]
        if (event['run_id']!=navigation_run or event['sim_stamp_ns']<attempt['raw']['entered_sim_ns']
                or at<round(attempt['raw']['entered_wall_s']*1e9)
                or event['sim_stamp_ns']>end or at>round(wall_end*1e9)):
            raise ValueError('mission_collision_command_time')
        attempt['reply']=event['reply']
    if before!=files(path):raise ValueError('mission_collision_files_changed')
    return {'coverage_complete':True,'matched_navigation_commands':len(expected),
        'first_captured_sim_ns':first[0],'last_captured_sim_ns':last[0],
        'covering_trace':result,'continuous_collision_free':None,
        'scope':'The entire covering trace is evaluated, including samples before/after this mission; sampled clearance is not continuous or physical contact acceptance'},before


def verify(path,pins):
    if files(path)!=pins:raise ValueError('mission_collision_files_changed')
