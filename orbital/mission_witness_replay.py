"""Strict private witness replay; no outcome is inferred from a gateway claim."""
import hashlib
import os

from orbital.inspection_artifacts import directory
from orbital.inspection_protocol import canonical, decode, finite, identifier
from orbital.maintenance_protocol import integer
from orbital.mission_witness import (DIRECTORY,MAX_BYTES,MAX_RECORDS,MAX_LINE,MAX_TEXT,
    digest,private_file,source_revision,runtime,graph_identity)
from orbital.scenario_input import fingerprint,private_directory
from orbital.station_evaluation import read_private


def pin(value):
    if not isinstance(value,dict) or set(value)!={'sequence','sha256','bytes'}:
        raise ValueError('mission_witness_journal_pin')
    integer(value['sequence']);integer(value['bytes']);identifier(value['sha256'],64)


def raw_text(value):
    # Invalid observed JSON is still observable input, not corrupt storage.
    # The independent semantic evaluator classifies it without hiding the bytes.
    if not isinstance(value,str) or len(value.encode('utf-8'))>MAX_TEXT:
        raise ValueError('mission_witness_text')


def replay(raw,summary,mission_id,registry):
    if (not isinstance(summary,dict) or set(summary)!={'schema_version','complete','mission_id','records',
            'chain_sha256','bytes','payload_sha256'} or type(summary['schema_version']) is not int
            or summary['schema_version']!=1 or summary['complete'] is not True
            or summary['mission_id']!=mission_id or integer(summary['bytes'],1)!=len(raw)
            or hashlib.sha256(raw).hexdigest()!=summary['payload_sha256']):
        raise ValueError('mission_witness_manifest')
    identifier(summary['payload_sha256'],64);identifier(summary['chain_sha256'],64)
    if not raw or len(raw)>MAX_BYTES or not raw.endswith(b'\n'):raise ValueError('mission_witness_partial')
    events=[];previous='0'*64;last_wall=last_sim=None
    publications=0
    from orbital.ros_mission import SOURCES
    topics={topic for group in SOURCES.values() for topic in group if topic!='/clock'}
    for line in raw.splitlines(keepends=True):
        if len(line)>MAX_LINE or len(events)>=MAX_RECORDS:raise ValueError('mission_witness_capacity')
        event=decode(line.decode('utf-8'),MAX_LINE)
        if (set(event)!={'schema_version','sequence','previous','recorded_wall_s','recorded_sim_ns','kind','data','sha256'}
                or type(event['schema_version']) is not int or event['schema_version']!=1
                or integer(event['sequence'],1)!=len(events)+1 or event['previous']!=previous
                or digest({k:v for k,v in event.items() if k!='sha256'})!=event['sha256']
                or (canonical(event)+'\n').encode()!=line):raise ValueError('mission_witness_chain')
        wall,sim=finite(event['recorded_wall_s']),integer(event['recorded_sim_ns'],1)
        if last_wall is not None and (wall<last_wall or sim<last_sim):raise ValueError('mission_witness_clock')
        kind,data=event['kind'],event['data']
        if not isinstance(data,dict):raise ValueError('mission_witness_record')
        if not events:
            if kind!='begin' or set(data)!={'mission_id','registry_identity','source_revision','runtime','graph','node_name','journal'}:
                raise ValueError('mission_witness_header')
            if (data['mission_id']!=mission_id or data['registry_identity']!=registry.identity
                    or data['source_revision']!=source_revision()):raise ValueError('mission_witness_binding')
            runtime(data['runtime'],registry.public()['source_sha256']['world'])
            graph_identity(data['graph'],data['node_name']);pin(data['journal'])
        elif events[-1]['kind']=='end':raise ValueError('mission_witness_after_end')
        elif kind=='received':
            if set(data)!={'topic','text','received_wall_s','received_sim_ns'} or data['topic'] not in topics:
                raise ValueError('mission_witness_callback')
            if (finite(data['received_wall_s'])>wall or integer(data['received_sim_ns'],1)>sim
                    or data['received_wall_s']<events[0]['recorded_wall_s']):raise ValueError('mission_witness_receipt_time')
            raw_text(data['text'])
        elif kind=='published':
            if (set(data)!={'publication','family','text','entered_wall_s','entered_sim_ns','finished_wall_s','finished_sim_ns','returned','journal'}
                    or integer(data['publication'],1)!=publications+1 or type(data['returned']) is not bool
                    or data['family'] not in ('navigation','inspection','maintenance')):
                raise ValueError('mission_witness_publication')
            enter,leave=finite(data['entered_wall_s']),finite(data['finished_wall_s'])
            sim_enter,sim_leave=integer(data['entered_sim_ns'],1),integer(data['finished_sim_ns'],1)
            if not events[0]['recorded_wall_s']<=enter<=leave<=wall or not sim_enter<=sim_leave<=sim:
                raise ValueError('mission_witness_publication_time')
            raw_text(data['text']);pin(data['journal']);publications+=1
        elif kind=='end':
            if (set(data)!={'complete','runtime','graph','journal','publications'} or data['complete'] is not True
                    or data['runtime']!=events[0]['data']['runtime'] or data['graph']!=events[0]['data']['graph']
                    or integer(data['publications'])!=publications):raise ValueError('mission_witness_end')
            pin(data['journal'])
        else:raise ValueError('mission_witness_kind')
        events.append(event);previous=event['sha256'];last_wall,last_sim=wall,sim
    if (events[-1]['kind']!='end' or integer(summary['records'],2)!=len(events)
            or summary['chain_sha256']!=previous):raise ValueError('mission_witness_incomplete')
    return events


def read(lock,mission_id,registry):
    """Caller holds mission-service ownership; return pins for a final read fence."""
    identifier(mission_id);lock.verify()
    if lock.service!='mission-service':raise ValueError('mission_witness_wrong_lock')
    parent=run=None
    try:
        parent=directory(lock.directory_fd,DIRECTORY);private_directory(parent)
        run=directory(parent,mission_id);private_directory(run)
        summary_raw,summary_pin=read_private(run,'summary.json',16384,private_file)
        raw,payload_pin=read_private(run,'events.jsonl',MAX_BYTES,private_file)
        summary=decode(summary_raw.decode('utf-8'),16384)
        events=replay(raw,summary,mission_id,registry)
        evidence={'summary':summary,'summary_pin':summary_pin,'payload_pin':payload_pin,
            'source_revision':events[0]['data']['source_revision'],
            'parent_identity':(os.fstat(parent).st_dev,os.fstat(parent).st_ino),
            'run_identity':(os.fstat(run).st_dev,os.fstat(run).st_ino)}
        verify(lock,mission_id,registry,evidence)
        return events,evidence
    finally:
        for fd in (run,parent):
            if fd is not None:os.close(fd)


def verify(lock,mission_id,registry,evidence):
    parent=run=None
    try:
        lock.verify();registry.verify_current()
        if source_revision()!=evidence['source_revision']:raise ValueError('mission_witness_source_changed')
        parent=directory(lock.directory_fd,DIRECTORY);private_directory(parent)
        run=directory(parent,mission_id);private_directory(run)
        for fd,key in ((parent,'parent_identity'),(run,'run_identity')):
            info=os.fstat(fd)
            if (info.st_dev,info.st_ino)!=evidence[key]:raise ValueError('mission_witness_changed')
        for filename,key in (('events.jsonl','payload_pin'),('summary.json','summary_pin')):
            info=os.stat(filename,dir_fd=run,follow_symlinks=False);private_file(info)
            if fingerprint(info)!=evidence[key]:raise ValueError('mission_witness_changed')
    finally:
        for fd in (run,parent):
            if fd is not None:os.close(fd)
