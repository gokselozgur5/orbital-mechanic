"""Durable mission admission and public event history, never automatic resume.

The gateway owns the lock and must record intent before transport publication.
An unfinished or unreadable history refuses a new mission. This local integrity
chain is not authentication or protection from coherent same-user rollback.
"""
import copy
import hashlib
import os
import stat
import uuid

from orbital.inspection_artifacts import FILE_FLAGS
from orbital.inspection_protocol import canonical, decode, identifier
from orbital.inspection_protocol import finite
from orbital.maintenance_protocol import integer
from orbital import inspection_protocol, maintenance_protocol
from orbital.navigation_client import NavigationClient
from orbital.station_control import control
from orbital.planner_history import ToolHistory, KINDS as TOOL_KINDS
from orbital.planner_dialog import ProviderHistory, KINDS as PROVIDER_KINDS

NAME = 'events.jsonl'
MAX_BYTES = 32*1024*1024
MAX_EVENTS = 16384
MAX_MISSIONS = 64
MAX_EVENT_BYTES = 65536
KINDS = frozenset(('start','command','receipt','telemetry','decision','error','cancel_intent','terminal',
                   'operator_tool_reset')) | TOOL_KINDS | PROVIDER_KINDS


class MissionJournal:
    def __init__(self, lock):
        if lock.service != 'mission-service':
            raise ValueError('wrong_mission_journal_lock')
        self.lock, self.fd, self.broken = lock, None, False
        self.sequence, self.previous, self.size = 0, '0'*64, 0
        self.active = None
        self.context = None
        self.missions = set()
        self.tools = ToolHistory()
        self.provider = ProviderHistory()
        try:
            lock.verify()
            parent = os.fstat(lock.directory_fd)
            if parent.st_uid != os.getuid() or parent.st_mode & 0o077:
                raise ValueError('mission_directory_not_private')
            self.fd = os.open(NAME, os.O_RDWR|os.O_CREAT|os.O_APPEND|FILE_FLAGS, 0o600,
                              dir_fd=lock.directory_fd)
            info = os.fstat(self.fd)
            self._file(info)
            self.identity = (info.st_dev,info.st_ino)
            raw = os.pread(self.fd, MAX_BYTES+1, 0)
            after = os.fstat(self.fd)
            if self._version(after)!=self._version(info) or len(raw)!=after.st_size:
                raise ValueError('mission_journal_changed_during_read')
            if len(raw)>MAX_BYTES or (raw and not raw.endswith(b'\n')):
                raise ValueError('mission_journal_incomplete_or_full')
            for line in raw.splitlines():
                event = decode(line.decode('utf-8'),MAX_EVENT_BYTES)
                if set(event)!={'schema_version','sequence','previous_sha256','mission_id','kind','data','sha256'}:
                    raise ValueError('invalid_mission_journal_event')
                digest = event.pop('sha256')
                if digest != hashlib.sha256(canonical(event).encode()).hexdigest():
                    raise ValueError('mission_journal_digest_mismatch')
                if (type(event['schema_version']) is not int or event['schema_version']!=1
                        or type(event['sequence']) is not int or event['sequence']!=self.sequence+1
                        or event['previous_sha256']!=self.previous):
                    raise ValueError('mission_journal_chain_mismatch')
                self._transition(event['mission_id'],event['kind'],event['data'])
                self.sequence,self.previous=event['sequence'],digest
                if self.sequence>MAX_EVENTS:
                    raise ValueError('mission_journal_full')
            self.size=len(raw)
            self.version=self._version(os.fstat(self.fd))
            self.verify()
            os.fsync(lock.directory_fd)
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _version(info):
        return (info.st_size,info.st_mtime_ns,info.st_ctime_ns)

    @staticmethod
    def _file(info):
        if (not stat.S_ISREG(info.st_mode) or info.st_nlink!=1
                or info.st_uid!=os.getuid() or info.st_mode & 0o077):
            raise ValueError('mission_journal_not_private_regular_file')

    def verify(self):
        if self.broken or self.fd is None:
            raise ValueError('mission_journal_unavailable')
        self.lock.verify()
        info=os.stat(NAME,dir_fd=self.lock.directory_fd,follow_symlinks=False)
        self._file(info)
        if ((info.st_dev,info.st_ino)!=self.identity or info.st_size!=self.size
                or self._version(info)!=self.version):
            raise ValueError('mission_journal_changed')

    def _transition(self, mission_id, kind, data):
        identifier(mission_id)
        if kind not in KINDS or not isinstance(data,dict):
            raise ValueError('invalid_mission_journal_event')
        if kind == 'operator_tool_reset':
            if self.active is not None or mission_id in self.missions:
                raise ValueError('unfinished_mission_requires_reconciliation')
            self.tools = self.tools.operator_reset(mission_id,data)
            return
        if kind=='start':
            if self.active is not None:
                raise ValueError('unfinished_mission_requires_reconciliation')
            if self.tools.stop_owner is not None:raise ValueError('planner_operator_reset_required')
            if mission_id in self.missions or mission_id in self.tools.audit_ids or len(self.missions)>=MAX_MISSIONS:
                raise ValueError('mission_identity_reused_or_history_full')
            if set(data)!={'registry_identity','station_run_id','policy_version','baseline','started_wall_s','started_sim_ns','deadline_wall_s'}:
                raise ValueError('invalid_mission_start_record')
            identifier(data['registry_identity'],64);identifier(data['station_run_id'])
            integer(data['started_sim_ns'],1)
            start=finite(data['started_wall_s'],0.,float('inf'))
            finite(data['deadline_wall_s']-start,0.,600.)
            if (not isinstance(data['policy_version'],str) or not 1<=len(data['policy_version'])<=96
                    or not isinstance(data['baseline'],dict)
                    or data['baseline'].get('run_id')!=data['station_run_id']
                    or data['baseline'].get('schema_version')!=2):
                raise ValueError('invalid_mission_start_record')
            control(data['baseline'].get('control'),data['started_sim_ns'])
            self.missions.add(mission_id)
            self.active=mission_id
            self.context=copy.deepcopy(data)
        else:
            if self.active!=mission_id:
                raise ValueError('mission_journal_has_no_matching_owner')
            self.tools = self.tools.advance(mission_id,kind,data,self.context)
            self.provider = self.provider.advance(mission_id,kind,data,self.context,self.tools)
            if kind=='terminal':
                if (set(data)!={'outcome','reason','control','pending','quiescence'} or data['pending'] is not None
                        or data['outcome'] not in ('healthy','recovered','escalated','cancelled','timed_out','failed')
                        or not isinstance(data['quiescence'],dict)):
                    raise ValueError('invalid_mission_terminal_record')
                self._terminal(data)
                self.active=None
                self.context=None

    def _terminal(self, data):
        """Require retained owner observations, not an empty completion marker."""
        q=data['quiescence']
        if set(q)!={'navigation','inspection','maintenance','received_wall_s',
                    'observed_wall_s','observed_sim_ns','scope'}:
            raise ValueError('invalid_mission_quiescence')
        wall=finite(q['observed_wall_s'],self.context['started_wall_s'],float('inf'))
        sim=integer(q['observed_sim_ns'],self.context['started_sim_ns'])
        received=q['received_wall_s']
        if not isinstance(received,dict) or set(received)!={'navigation','inspection','maintenance'}:
            raise ValueError('invalid_mission_quiescence')
        for value in received.values():
            if not 0<=wall-finite(value,0.,float('inf'))<.3:
                raise ValueError('stale_mission_quiescence')
        n=NavigationClient(lambda:wall)
        n.observe(canonical(q['navigation']))
        i=inspection_protocol.state(canonical(q['inspection']))
        m=maintenance_protocol.state(canonical(q['maintenance']))
        c=control(data['control'],sim)
        if (n.failure or n.state['action_state']!='idle' or n.state['target'] is not None
                or n.state['guard_latched'] is not True or i['active'] is not None or m['active'] is not None
                or m['station_run_id']!=self.context['station_run_id']
                or c['revision']<self.context['baseline']['control']['revision']
                or m['control_revision']!=c['revision']
                or not 0<=sim-n.state.get('sim_stamp_ns',-1)<300000000
                or not 0<=sim-m['sim_stamp_ns']<300000000):
            raise ValueError('invalid_mission_quiescence')

    def append(self, mission_id, kind, data):
        """A partial/failed append is sticky and is never retried or truncated."""
        self.verify()
        trial=copy.copy(self);trial.missions=set(self.missions)
        trial._transition(mission_id,kind,data)
        event={'schema_version':1,'sequence':self.sequence+1,'previous_sha256':self.previous,
               'mission_id':mission_id,'kind':kind,'data':data}
        digest=hashlib.sha256(canonical(event).encode()).hexdigest()
        raw=(canonical({**event,'sha256':digest})+'\n').encode()
        if len(raw)>MAX_EVENT_BYTES or self.size+len(raw)>MAX_BYTES or self.sequence>=MAX_EVENTS:
            raise ValueError('mission_journal_full')
        self.verify()
        try:
            if os.write(self.fd,raw)!=len(raw):
                raise OSError('short_mission_journal_write')
            os.fsync(self.fd)
            self.size+=len(raw)
            self.version=self._version(os.fstat(self.fd))
            self.verify()
        except BaseException:
            self.broken=True
            raise
        self.active,self.missions,self.context,self.tools,self.provider=(
            trial.active,trial.missions,trial.context,trial.tools,trial.provider)
        self.sequence,self.previous=event['sequence'],digest
        return digest

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd=None

    def reset_planner_stop(self):
        """Explicit operator-only storage action; never resume an unfinished mission."""
        self.verify()
        stopped = self.tools.stop_owner
        if stopped is None or self.active is not None:raise ValueError('planner_stop_reset_unavailable')
        audit = uuid.uuid4().hex
        digest = self.append(audit,'operator_tool_reset',{'stop_mission_id':stopped,'operator_requested':True})
        return {'schema_version':1,'cleared':True,'stop_mission_id':stopped,'audit_id':audit,
                'journal_sha256':digest,'scope':'Explicit planner admission reset; no actuator or guard reset.'}
