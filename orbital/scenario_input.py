"""Bound private inputs to an explicit durable station generation.

File privacy prevents accidental exposure through ordinary public artifacts; it
does not isolate a malicious process running as the same OS user.
"""
import hashlib
import os
import stat

from orbital.inspection_artifacts import directory, FILE_FLAGS
from orbital.inspection_protocol import canonical, identifier
from orbital import scenario_spec as spec

DIRECTORY = 'scenario-inputs'
MAX_INPUTS = 256


def fingerprint(info):
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)


def private_directory(fd):
    value = os.fstat(fd)
    if not stat.S_ISDIR(value.st_mode) or value.st_uid != os.getuid() or stat.S_IMODE(value.st_mode) != 0o700:
        raise ValueError('scenario_directory_not_private')


def private_file(info):
    if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_nlink != 1
            or stat.S_IMODE(info.st_mode) != 0o600 or not 0 < info.st_size <= spec.MAX_BYTES):
        raise ValueError('scenario_input_not_private_regular_file')


def write_input(lock, value):
    lock.verify()
    if lock.service != 'station-service': raise ValueError('scenario_input_wrong_lock')
    private_directory(lock.directory_fd)
    value = spec.validate(canonical(value), value['registry_identity'])
    spec.require_station_fixture(value)
    raw = (canonical(value)+'\n').encode()
    parent = directory(lock.directory_fd, DIRECTORY, create=True)
    try:
        private_directory(parent)
        with os.scandir(parent) as entries:
            for count, _ in enumerate(entries, 1):
                if count >= MAX_INPUTS: raise ValueError('scenario_input_capacity')
        name = value['trial_id']+'.json'
        fd = os.open(name, os.O_RDWR|os.O_CREAT|os.O_EXCL|FILE_FLAGS,
                     0o600, dir_fd=parent)
        try:
            if os.write(fd, raw) != len(raw): raise OSError('scenario_input_short_write')
            os.fsync(fd)
            info = os.fstat(fd); private_file(info)
            if (os.pread(fd, spec.MAX_BYTES+1, 0) != raw
                    or fingerprint(os.stat(name, dir_fd=parent, follow_symlinks=False)) != fingerprint(info)):
                raise ValueError('scenario_input_changed')
        finally: os.close(fd)
        os.fsync(parent); os.fsync(lock.directory_fd)
        lock.verify()
        return hashlib.sha256(raw).hexdigest()
    finally: os.close(parent)


class PreparedInput:
    def __init__(self, journal, registry, *, expected_trial_id=None):
        self.journal, self.registry = journal, registry
        self.parent = self.fd = None
        self.broken = False
        try:
            journal.verify()
            current = journal.history.scenarios.get(journal.history.current)
            if current is not None and current['claim'] is not None: raise ValueError('scenario_already_claimed')
            if current is None or 'configuration_sha256' not in current:
                raise ValueError('private_scenario_not_prepared')
            if current['registry_identity'] != registry.identity: raise ValueError('scenario_source_mismatch')
            self.scenario_id = identifier(current['scenario_id'])
            if expected_trial_id is not None and self.scenario_id != identifier(expected_trial_id):
                raise ValueError('scenario_launch_identity_mismatch')
            self.digest = identifier(current['configuration_sha256'], 64)
            self.name = self.scenario_id+'.json'
            self.parent = directory(journal.lock.directory_fd, DIRECTORY)
            private_directory(self.parent)
            self.parent_identity = os.fstat(self.parent)
            self.fd = os.open(self.name, os.O_RDONLY|FILE_FLAGS, dir_fd=self.parent)
            before = os.fstat(self.fd); private_file(before)
            raw = os.pread(self.fd, spec.MAX_BYTES+1, 0)
            after = os.fstat(self.fd)
            if (fingerprint(before) != fingerprint(after) or len(raw) != before.st_size
                    or hashlib.sha256(raw).hexdigest() != self.digest):
                raise ValueError('scenario_input_changed')
            self.value = spec.validate(raw.decode('utf-8'), registry.identity)
            if self.value['trial_id'] != self.scenario_id: raise ValueError('scenario_identity_mismatch')
            spec.require_station_fixture(self.value)
            self.expected = fingerprint(after)
            self.verify()
        except BaseException:
            self.close()
            raise

    def verify(self):
        if self.broken or self.fd is None: raise ValueError('scenario_input_unavailable')
        parent = None
        try:
            self.journal.verify(); self.registry.verify_current()
            if self.journal.history.current != self.scenario_id: raise ValueError('scenario_generation_changed')
            parent = directory(self.journal.lock.directory_fd, DIRECTORY)
            private_directory(parent)
            info = os.fstat(parent)
            if (info.st_dev, info.st_ino) != (self.parent_identity.st_dev, self.parent_identity.st_ino):
                raise ValueError('scenario_directory_changed')
            info = os.stat(self.name, dir_fd=parent, follow_symlinks=False); private_file(info)
            if fingerprint(info) != self.expected or fingerprint(os.fstat(self.fd)) != self.expected:
                raise ValueError('scenario_input_changed')
            if self.value['source_revision'] != spec.source_revision(): raise ValueError('scenario_source_mismatch')
        except (ValueError, OSError, RuntimeError):
            self.broken = True
            raise
        finally:
            if parent is not None: os.close(parent)

    def model(self):
        from orbital.machine_telemetry import MachineTelemetry
        self.verify()
        model = MachineTelemetry(self.registry, initial=self.value['initial_state'])
        model.configuration = self
        model.configuration_sha256 = self.digest
        return model

    def close(self):
        for name in ('fd', 'parent'):
            fd = getattr(self, name)
            if fd is not None: os.close(fd); setattr(self, name, None)


def check_prepared(root, trial_id):
    """Verify an unclaimed input; this is not a claim or a live-world assertion."""
    from orbital.inspection_worker import OwnerLock
    from orbital.machine_registry import MachineRegistry
    from orbital.maintenance_journal import Journal
    identifier(trial_id)
    lock = OwnerLock(root, service='station-service')
    journal = prepared = None
    try:
        journal = Journal(lock)
        prepared = PreparedInput(journal, MachineRegistry.load(root), expected_trial_id=trial_id)
        return spec.assignment(prepared.value)
    finally:
        if prepared is not None: prepared.close()
        if journal is not None: journal.close()
        lock.close()
