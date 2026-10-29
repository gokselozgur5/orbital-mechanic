"""Bounded durable station claims and action history under the existing owner lock."""
import copy
import hashlib
import os
import stat
import uuid

from orbital.inspection_artifacts import DIRECTORY_FLAGS, FILE_FLAGS, directory
from orbital.inspection_protocol import canonical, decode, identifier
from orbital.inspection_worker import OwnerLock
from orbital.maintenance_history import History
from orbital.machine_registry import MachineRegistry

MAX_BYTES = 16 * 1024 * 1024
MAX_EVENTS = 8192
MAX_EVENT_BYTES = 32768
NAME = 'events.jsonl'


class JournalUnavailable(Exception):
    """A failed/ambiguous write is not a known mutation-free actuator rejection."""


def fingerprint(info):
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)


def replay(raw):
    if len(raw) > MAX_BYTES or (raw and not raw.endswith(b'\n')):
        raise ValueError('journal_partial_or_oversized')
    history, sequence, previous = History(), 0, '0'*64
    for line in raw.splitlines(keepends=True):
        if len(line) > MAX_EVENT_BYTES or sequence >= MAX_EVENTS:
            raise ValueError('journal_capacity_exceeded')
        event = decode(line.decode('utf-8'), MAX_EVENT_BYTES)
        if set(event) != {'schema_version','sequence','previous','kind','value','sha256'}:
            raise ValueError('invalid_journal_record')
        if type(event['schema_version']) is not int or event['schema_version'] != 1:
            raise ValueError('invalid_journal_version')
        if type(event['sequence']) is not int or event['sequence'] != sequence+1 or event['previous'] != previous:
            raise ValueError('journal_chain_mismatch')
        digest = identifier(event['sha256'],64)
        content = {k:v for k,v in event.items() if k != 'sha256'}
        if hashlib.sha256(canonical(content).encode()).hexdigest() != digest or (canonical(event)+'\n').encode() != line:
            raise ValueError('journal_digest_mismatch')
        history.apply(event['kind'], event['value'])
        sequence, previous = event['sequence'], digest
    return history, sequence, previous


class Journal:
    def __init__(self, lock, *, create=False):
        if lock.service != 'station-service': raise ValueError('wrong_journal_owner_lock')
        self.lock, self.fd, self.broken = lock, None, False
        self.parent = lock.directory_fd
        self.claimed = None
        self.lock.verify()
        directory_info = os.fstat(self.parent)
        if directory_info.st_uid != os.getuid() or directory_info.st_mode & 0o077:
            raise ValueError('journal_directory_not_private')
        created = False
        try:
            if create:
                try:
                    self.fd = os.open(NAME, os.O_RDWR|os.O_APPEND|os.O_CREAT|os.O_EXCL|FILE_FLAGS, 0o600, dir_fd=self.parent)
                    created = True
                except FileExistsError: pass
            if self.fd is None:
                self.fd = os.open(NAME, os.O_RDWR|os.O_APPEND|FILE_FLAGS, dir_fd=self.parent)
            info = os.fstat(self.fd)
            self._check_file(info)
            raw = os.pread(self.fd, MAX_BYTES+1, 0)
            if len(raw) != info.st_size: raise ValueError('journal_changed')
            self.history, self.sequence, self.previous = replay(raw)
            self.expected = fingerprint(info)
            if created:
                os.fsync(self.fd); os.fsync(self.parent)
            self.verify()
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _check_file(info):
        if (not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) != 0o600 or not 0 <= info.st_size <= MAX_BYTES):
            raise ValueError('invalid_journal_file')

    def verify(self):
        if self.broken or self.fd is None: raise JournalUnavailable('journal_unavailable')
        opened = []
        try:
            self.lock.verify()
            parent = os.open(self.lock.root, DIRECTORY_FLAGS); opened.append(parent)
            for name in ('runtime','station-service'):
                parent = directory(parent,name); opened.append(parent)
            expected_parent, actual_parent = os.fstat(self.parent), os.fstat(parent)
            if (expected_parent.st_dev,expected_parent.st_ino) != (actual_parent.st_dev,actual_parent.st_ino):
                raise ValueError('journal_directory_changed')
            if actual_parent.st_uid != os.getuid() or actual_parent.st_mode & 0o077:
                raise ValueError('journal_directory_not_private')
            self._check_file(os.fstat(self.fd))
            info = os.stat(NAME,dir_fd=parent,follow_symlinks=False)
            if fingerprint(info) != self.expected or fingerprint(os.fstat(self.fd)) != self.expected:
                raise ValueError('journal_changed')
        except Exception as error:
            self.broken = True
            raise JournalUnavailable('journal_unavailable') from error
        finally:
            for fd in reversed(opened): os.close(fd)

    def record(self, kind, value):
        self.verify()
        value = copy.deepcopy(value)
        candidate = copy.deepcopy(self.history)
        candidate.apply(kind,value)  # Refuse invalid transitions before any write.
        event = {'schema_version':1,'sequence':self.sequence+1,'previous':self.previous,'kind':kind,'value':value}
        digest = hashlib.sha256(canonical(event).encode()).hexdigest()
        raw = (canonical({**event,'sha256':digest})+'\n').encode()
        # Leave bounded headroom for outcomes/terminal records of admitted work.
        headroom = 4*MAX_EVENT_BYTES if kind in ('scenario','scenario_v2','claim','admit') else 0
        if (len(raw) > MAX_EVENT_BYTES or self.sequence >= MAX_EVENTS-(8 if headroom else 0)
                or self.expected[2]+len(raw)+headroom > MAX_BYTES):
            raise ValueError('journal_full')
        size = self.expected[2]
        try:
            view = memoryview(raw)
            while view:
                count = os.write(self.fd,view)
                if count <= 0: raise OSError('journal_short_write')
                view = view[count:]
            os.fsync(self.fd)
            info = os.fstat(self.fd)
            if info.st_size != size+len(raw) or os.pread(self.fd,len(raw),size) != raw:
                raise ValueError('journal_write_changed')
            self.expected = fingerprint(info)
            self.verify()
        except Exception as error:
            self.broken = True
            raise JournalUnavailable('journal_write_unavailable') from error
        self.history, self.sequence, self.previous = candidate, event['sequence'], digest

    def prepare(self, registry_identity):
        scenario = uuid.uuid4().hex
        self.record('scenario',{'scenario_id':scenario,'registry_identity':registry_identity})
        return scenario

    def prepare_configuration(self, scenario_id, registry_identity, configuration_sha256):
        self.record('scenario_v2', {'scenario_id':scenario_id, 'registry_identity':registry_identity,
                                   'configuration_sha256':configuration_sha256})

    def claim(self, run_id, station_run_id, registry_identity, configuration_sha256=None):
        current = self.history.scenarios.get(self.history.current)
        if current is None: raise ValueError('scenario_not_prepared')
        if current['registry_identity'] != registry_identity: raise ValueError('scenario_registry_changed')
        if current.get('configuration_sha256') != configuration_sha256:
            raise ValueError('scenario_configuration_mismatch')
        value = {'scenario_id':self.history.current,'run_id':run_id,'station_run_id':station_run_id}
        self.record('claim',value)
        self.claimed = value

    def admit(self, request, *, child_id=None, parent_action_id=None):
        if self.claimed is None: raise ValueError('journal_not_claimed')
        self.record('admit',{'scenario_id':self.claimed['scenario_id'],'request':request,
                            'child_id':child_id,'parent_action_id':parent_action_id})

    def intent(self, operation, context, station_run_id, revision, isolation_action_id, reset_action_id):
        if self.claimed is None: raise ValueError('journal_not_claimed')
        self.record('intent',{'scenario_id':self.claimed['scenario_id'],'action_id':context['action_id'],
            'context':context,'operation':operation,'station_run_id':station_run_id,'control_revision':revision,
            'isolation_action_id':isolation_action_id,'reset_action_id':reset_action_id})

    def outcome(self, action_id, effect=None):
        value = {'scenario_id':self.claimed['scenario_id'],'action_id':action_id}
        if effect is not None: value['effect'] = effect
        self.record('applied' if effect is not None else 'rejected',value)

    def terminal(self, result):
        self.record('terminal',{'scenario_id':self.claimed['scenario_id'],'result':result})

    def close(self):
        if self.fd is not None: os.close(self.fd); self.fd = None


def prepare_scenario(root):
    """Legacy model-fixture generation; insufficient for the live ROS publisher."""
    registry = MachineRegistry.load(root)
    lock = OwnerLock(root,service='station-service')
    journal = None
    try:
        journal = Journal(lock,create=True)
        scenario = journal.prepare(registry.identity)
        registry.verify_current()
        return {'scenario_id':scenario,'registry_identity':registry.identity,'new_world_required':True}
    finally:
        if journal is not None: journal.close()
        lock.close()


def prepare_configured_scenario(root, profile, seed):
    """An explicit new-world input, never live mutation or station-only resume."""
    from orbital import scenario_spec
    from orbital.scenario_input import write_input
    registry = MachineRegistry.load(root)
    value = scenario_spec.create(profile, seed, registry.identity)
    scenario_spec.require_station_fixture(value)
    lock = OwnerLock(root, service='station-service')
    journal = None
    try:
        journal = Journal(lock, create=True)
        digest = write_input(lock, value)
        registry.verify_current()
        journal.prepare_configuration(value['trial_id'], registry.identity, digest)
        return {**scenario_spec.assignment(value), 'new_world_required':True}
    finally:
        if journal is not None: journal.close()
        lock.close()


def read_history(root):
    opened = []
    try:
        parent = os.open(root,DIRECTORY_FLAGS); opened.append(parent)
        for name in ('runtime','station-service'):
            parent = directory(parent,name); opened.append(parent)
        directory_info = os.fstat(parent)
        if directory_info.st_uid != os.getuid() or directory_info.st_mode & 0o077:
            raise ValueError('journal_directory_not_private')
        fd = os.open(NAME,os.O_RDONLY|FILE_FLAGS,dir_fd=parent); opened.append(fd)
        info = os.fstat(fd)
        Journal._check_file(info)
        raw = os.pread(fd,MAX_BYTES+1,0)
        if (len(raw) != info.st_size or fingerprint(os.fstat(fd)) != fingerprint(info)
                or fingerprint(os.stat(NAME,dir_fd=parent,follow_symlinks=False)) != fingerprint(info)):
            raise ValueError('journal_changed_during_read')
        return replay(raw)[0]
    finally:
        for fd in reversed(opened): os.close(fd)
