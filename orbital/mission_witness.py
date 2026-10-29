"""Private raw gateway evidence, separate from policy claims and station truth.

The producer shares the trusted gateway process. This is cooperative retained
evidence, not OS isolation, authenticated DDS or independent physical sensing.
"""
import copy
from collections import Counter
import hashlib
import os
from pathlib import Path
import stat
import re

from orbital.inspection_artifacts import directory, FILE_FLAGS
from orbital.inspection_protocol import canonical, finite, identifier
from orbital.maintenance_protocol import integer
from orbital.scenario_input import fingerprint, private_directory

DIRECTORY = 'mission-observations'
MAX_BYTES, MAX_RECORDS, MAX_LINE = 128*1024*1024, 65536, 131072
MAX_TEXT = 32768
SOURCE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_FILES = ('orbital/mission_witness.py','orbital/ros_mission.py','orbital/mission_runtime.py',
                'orbital/mission_journal.py','orbital/mission_policy.py','orbital/mission_state.py',
                'orbital/inspection_client.py','orbital/maintenance_client.py',
                'orbital/navigation_client.py','orbital/ros_trajectory.py')


def digest(value):return hashlib.sha256(canonical(value).encode()).hexdigest()


def source_revision():
    return digest({name:hashlib.sha256((SOURCE_ROOT/name).read_bytes()).hexdigest() for name in SOURCE_FILES})


def runtime(value, world):
    if (not isinstance(value,dict) or set(value)!={'manifest_identity','world_sha256','service_invocations'}
            or value['world_sha256']!=world):raise ValueError('mission_witness_runtime')
    identifier(value['manifest_identity'],64);identifier(value['world_sha256'],64)
    units=value['service_invocations']
    if not isinstance(units,dict) or set(units)!={'orbital-demo.service','orbital-localization.service','orbital-navigation.service'}:
        raise ValueError('mission_witness_services')
    for value in units.values():identifier(value)


def journal_pin(journal):
    journal.verify()
    return {'sequence':journal.sequence,'sha256':journal.previous,'bytes':journal.size}


def graph_identity(value, node_name):
    # Imported after module initialization; the existing transport contract stays
    # the single definition of publisher/subscriber ownership.
    from orbital.ros_mission import SOURCES, OWNERS
    if not isinstance(node_name,str) or re.fullmatch('orbital_mission_[0-9a-f]{16}',node_name) is None:
        raise ValueError('mission_witness_node')
    if not isinstance(value,dict) or set(value)!=set(SOURCES):raise ValueError('mission_witness_graph')
    for family,sources in SOURCES.items():
        wanted=[(topic,owner,'/',kind,'publisher') for topic,owners in sources.items() for owner,kind in owners]
        if family in OWNERS:
            topic='/'+family+'/command'
            wanted.extend(((topic,node_name,'/','std_msgs/msg/String','publisher'),
                           (topic,OWNERS[family],'/','std_msgs/msg/String','subscriber')))
        peers=value[family]
        if not isinstance(peers,(list,tuple)) or len(peers)!=len(wanted):raise ValueError('mission_witness_graph')
        found=[]
        for peer in peers:
            if not isinstance(peer,(list,tuple)) or len(peer) not in (5,6):raise ValueError('mission_witness_graph')
            gid=peer[4]
            if (not isinstance(gid,str) or not 2<=len(gid)<=256 or len(gid)%2
                    or re.fullmatch('[0-9a-f]+',gid) is None or not any(bytes.fromhex(gid))):
                raise ValueError('mission_witness_gid')
            found.append((*peer[:4],'publisher' if len(peer)==5 else peer[5]))
        if Counter(found)!=Counter(wanted):raise ValueError('mission_witness_graph')


def private_file(info):
    if (not stat.S_ISREG(info.st_mode) or info.st_nlink!=1 or info.st_uid!=os.getuid()
            or stat.S_IMODE(info.st_mode)!=0o600 or not 0<=info.st_size<=MAX_BYTES):
        raise ValueError('mission_witness_file')


class MissionWitness:
    def __init__(self, lock, journal, registry, mission_id, before, graph, node_name, *, now, sim_ns):
        self.lock,self.journal,self.registry=lock,journal,registry
        self.mission_id=identifier(mission_id)
        self.now,self.sim_ns=now,sim_ns
        self.parent=self.run=self.fd=None
        self.sequence,self.previous,self.publications=0,'0'*64,0
        self.last_wall=self.last_sim=None
        self.broken=self.closed=False
        self.before,self.graph=copy.deepcopy(before),copy.deepcopy(graph)
        self.revision=source_revision()
        try:
            if lock.service!='mission-service' or journal.active is not None or mission_id in journal.missions:
                raise ValueError('mission_witness_admission')
            runtime(before,registry.public()['source_sha256']['world'])
            graph_identity(graph,node_name)
            self.parent=directory(lock.directory_fd,DIRECTORY,create=True);private_directory(self.parent)
            self.parent_info=os.fstat(self.parent)
            with os.scandir(self.parent) as entries:
                if sum(1 for _ in entries)>=64:raise ValueError('mission_witness_capacity')
            os.mkdir(mission_id,mode=0o700,dir_fd=self.parent)
            self.run=directory(self.parent,mission_id);private_directory(self.run)
            self.run_info=os.fstat(self.run)
            self.fd=os.open('events.jsonl',os.O_RDWR|os.O_APPEND|os.O_CREAT|os.O_EXCL|FILE_FLAGS,0o600,dir_fd=self.run)
            self.expected=fingerprint(os.fstat(self.fd))
            self.append('begin',{'mission_id':mission_id,'registry_identity':registry.identity,
                'source_revision':self.revision,'runtime':self.before,'graph':self.graph,
                'node_name':node_name,'journal':journal_pin(journal)})
            os.fsync(self.fd);os.fsync(self.run);os.fsync(self.parent);os.fsync(lock.directory_fd)
        except BaseException:
            self.close();raise

    def verify(self):
        if self.closed or self.broken or self.fd is None:raise ValueError('mission_witness_unavailable')
        parent=run=None
        try:
            self.lock.verify();self.registry.verify_current()
            if source_revision()!=self.revision:raise ValueError('mission_witness_source_changed')
            parent=directory(self.lock.directory_fd,DIRECTORY);private_directory(parent)
            run=directory(parent,self.mission_id);private_directory(run)
            for fd,old in ((parent,self.parent_info),(run,self.run_info)):
                current=os.fstat(fd)
                if (old.st_dev,old.st_ino)!=(current.st_dev,current.st_ino):raise ValueError('mission_witness_path_changed')
            info=os.stat('events.jsonl',dir_fd=run,follow_symlinks=False);private_file(info);private_file(os.fstat(self.fd))
            if fingerprint(info)!=self.expected or fingerprint(os.fstat(self.fd))!=self.expected:
                raise ValueError('mission_witness_changed')
        except Exception:
            self.broken=True;raise
        finally:
            for fd in (run,parent):
                if fd is not None:os.close(fd)

    def reserve(self):
        self.verify()
        if self.sequence>=MAX_RECORDS-2 or self.expected[2]+2*MAX_LINE>MAX_BYTES:
            self.broken=True;raise ValueError('mission_witness_capacity')

    def append(self, kind, data, *, ended_sim_ns=None):
        self.verify()
        wall=finite(self.now());sim=integer(self.sim_ns() if ended_sim_ns is None else ended_sim_ns,1)
        if self.last_wall is not None and (wall<self.last_wall or sim<self.last_sim):
            self.broken=True;raise ValueError('mission_witness_clock_reversed')
        value={'schema_version':1,'sequence':self.sequence+1,'previous':self.previous,
               'recorded_wall_s':wall,'recorded_sim_ns':sim,'kind':kind,'data':data}
        checksum=digest(value);raw=(canonical({**value,'sha256':checksum})+'\n').encode()
        if len(raw)>MAX_LINE or self.sequence>=MAX_RECORDS or self.expected[2]+len(raw)>MAX_BYTES:
            self.broken=True;raise ValueError('mission_witness_capacity')
        offset=self.expected[2]
        try:
            if os.write(self.fd,raw)!=len(raw):raise OSError('mission_witness_short_write')
            info=os.fstat(self.fd)
            if info.st_size!=offset+len(raw) or os.pread(self.fd,len(raw),offset)!=raw:
                raise ValueError('mission_witness_write_changed')
            self.expected=fingerprint(info);self.verify()
        except BaseException:
            self.broken=True;raise
        self.sequence,self.previous,self.last_wall,self.last_sim=value['sequence'],checksum,wall,sim

    def receive(self, topic, text, received):
        # Callback processing precedes disk work, preserving its real receipt age.
        # Logging failure must not prevent safety-cleanup callbacks from running.
        try:
            if not isinstance(text,str) or len(text.encode())>MAX_TEXT:raise ValueError('mission_witness_text')
            self.append('received',{'topic':topic,'text':text,**received})
        except (ValueError,OSError,TypeError):self.broken=True

    def begin_publication(self, family, text):
        # No I/O after the caller's final freshness gate and before native publish.
        if family not in ('navigation','inspection','maintenance') or not isinstance(text,str) or len(text.encode())>MAX_TEXT:
            raise ValueError('mission_witness_publication')
        self.publications+=1
        return {'publication':self.publications,'family':family,'text':text,
                'entered_wall_s':finite(self.now()),'entered_sim_ns':integer(self.sim_ns(),1),
                # Already persisted journal state, copied without another read
                # or sync after the caller's final live admission checks.
                'journal':{'sequence':self.journal.sequence,'sha256':self.journal.previous,'bytes':self.journal.size}}

    def end_publication(self, value, returned):
        self.append('published',{**value,'returned':returned,
            'finished_wall_s':finite(self.now()),'finished_sim_ns':integer(self.sim_ns(),1)})

    def finish(self, complete, after, graph, *, ended_sim_ns):
        try:
            if complete:
                runtime(after,self.registry.public()['source_sha256']['world'])
                if (after!=self.before or graph!=self.graph or self.journal.active is not None
                        or self.mission_id not in self.journal.missions):raise ValueError('mission_witness_end_binding')
            self.append('end',{'complete':complete,'runtime':after,'graph':graph,
                'journal':journal_pin(self.journal),'publications':self.publications},ended_sim_ns=ended_sim_ns)
            # The manifest is never created until the exact payload is synced.
            os.fsync(self.fd);self.verify()
            summary={'schema_version':1,'complete':complete,'mission_id':self.mission_id,
                     'records':self.sequence,'chain_sha256':self.previous,'bytes':self.expected[2],
                     'payload_sha256':hashlib.sha256(os.pread(self.fd,MAX_BYTES+1,0)).hexdigest()}
            raw=(canonical(summary)+'\n').encode()
            fd=os.open('summary.json',os.O_WRONLY|os.O_CREAT|os.O_EXCL|FILE_FLAGS,0o600,dir_fd=self.run)
            try:
                if os.write(fd,raw)!=len(raw):raise OSError('mission_witness_manifest_short_write')
                os.fsync(fd)
            finally:os.close(fd)
            os.fsync(self.run)
            return summary
        except Exception:
            if complete:raise
            return None
        finally:self.close()

    def close(self):
        self.closed=True
        for name in ('fd','run','parent'):
            fd=getattr(self,name)
            if fd is not None:os.close(fd);setattr(self,name,None)
