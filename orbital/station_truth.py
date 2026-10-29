"""Retain actual causal-model observations privately, independent of mission reports.

The producer remains in the trusted station process. Its files do not establish
OS isolation, physical robot truth or authentication against same-user edits.
"""
import hashlib
import os
from pathlib import Path
import stat

from orbital.inspection_artifacts import directory, FILE_FLAGS
from orbital.inspection_protocol import canonical, finite
from orbital.scenario_input import fingerprint, private_directory
from orbital.station_truth_protocol import DIRECTORY, MAX_BYTES, MAX_RECORDS, MAX_LINE, snapshot, digest

SOURCE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_FILES = ('orbital/station_truth.py', 'orbital/station_truth_protocol.py',
                'orbital/station_evaluation.py', 'orbital/ros_machine_telemetry.py',
                'orbital/ros_maintenance.py', 'orbital/maintenance_journal.py',
                'orbital/maintenance_history.py', 'orbital/maintenance_owner.py',
                'orbital/station_control.py', 'scripts/evaluate-station.py')


class ObservationUnavailable(RuntimeError):
    pass


def source_revision():
    return digest({name:hashlib.sha256((SOURCE_ROOT/name).read_bytes()).hexdigest() for name in SOURCE_FILES})


def private_file(info):
    if (not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_uid != os.getuid()
            or stat.S_IMODE(info.st_mode) != 0o600 or not 0 <= info.st_size <= MAX_BYTES):
        raise ValueError('invalid_station_observation_file')


class StationTruth:
    def __init__(self, lock, journal, configuration, model, *, now):
        self.lock, self.journal, self.configuration, self.model = lock, journal, configuration, model
        self.now = now
        self.parent = self.fd = None
        self.broken = self.closed = False
        self.sequence, self.previous = 0, '0'*64
        self.last_time = None
        self.revision = source_revision()
        self.claim = None
        self.name = configuration.scenario_id+'.jsonl'
        try:
            configuration.verify()
            if (journal.claimed is None or model.configuration is not configuration
                    or model.control_revision != 0 or model.sequence != 0 or model.previous is not None
                    or model.station.time_s != 0.):
                raise ValueError('station_observation_requires_new_model')
            self.claim = dict(journal.claimed)
            self.parent = directory(lock.directory_fd, DIRECTORY, create=True)
            private_directory(self.parent)
            self.parent_identity = os.fstat(self.parent)
            self.fd = os.open(self.name, os.O_RDWR|os.O_APPEND|os.O_CREAT|os.O_EXCL|FILE_FLAGS,
                              0o600, dir_fd=self.parent)
            self.expected = fingerprint(os.fstat(self.fd))
            os.fsync(self.fd); os.fsync(self.parent); os.fsync(lock.directory_fd)
            self.append('begin', {'trial_id':configuration.scenario_id,
                'configuration_sha256':configuration.digest, 'registry_identity':model.registry.identity,
                'station_run_id':model.run_id, 'owner_run_id':self.claim['run_id'],
                'source_revision':self.revision, 'initial':snapshot(model)})
        except BaseException:
            self.close()
            raise

    def verify(self):
        if self.closed or self.broken or self.fd is None:
            raise ObservationUnavailable('station_observations_unavailable')
        parent = None
        try:
            self.configuration.verify()
            if (self.journal.claimed != self.claim or self.model.run_id != self.claim['station_run_id']
                    or source_revision() != self.revision):
                raise ValueError('station_observation_identity_changed')
            parent = directory(self.lock.directory_fd, DIRECTORY)
            private_directory(parent)
            info = os.fstat(parent)
            if (info.st_dev, info.st_ino) != (self.parent_identity.st_dev, self.parent_identity.st_ino):
                raise ValueError('station_observation_directory_changed')
            info = os.stat(self.name, dir_fd=parent, follow_symlinks=False); private_file(info)
            private_file(os.fstat(self.fd))
            if fingerprint(info) != self.expected or fingerprint(os.fstat(self.fd)) != self.expected:
                raise ValueError('station_observation_file_changed')
        except Exception as error:
            self.broken = True
            raise ObservationUnavailable('station_observations_unavailable') from error
        finally:
            if parent is not None: os.close(parent)

    def reserve(self):
        """All file work occurs before the control's final live admission gate."""
        self.verify()
        if self.sequence >= MAX_RECORDS-2 or self.expected[2]+2*MAX_LINE > MAX_BYTES:
            self.broken = True
            raise ObservationUnavailable('station_observation_capacity')

    def append(self, kind, value):
        self.verify()
        at = finite(self.now())
        if self.last_time is not None and at < self.last_time:
            self.broken = True
            raise ObservationUnavailable('station_observation_clock_reversed')
        event = {'schema_version':1, 'sequence':self.sequence+1, 'previous':self.previous,
                 'recorded_monotonic_s':at, 'kind':kind, 'value':value}
        checksum = digest(event)
        raw = (canonical({**event,'sha256':checksum})+'\n').encode()
        if (len(raw) > MAX_LINE or self.sequence >= MAX_RECORDS
                or self.expected[2]+len(raw) > MAX_BYTES):
            self.broken = True
            raise ObservationUnavailable('station_observation_capacity')
        size = self.expected[2]
        try:
            # A short/ambiguous append is retained, never retried into validity.
            if os.write(self.fd, raw) != len(raw): raise OSError('station_observation_short_write')
            os.fsync(self.fd)
            info = os.fstat(self.fd)
            if info.st_size != size+len(raw) or os.pread(self.fd, len(raw), size) != raw:
                raise ValueError('station_observation_write_changed')
            self.expected = fingerprint(info)
            self.verify()
        except BaseException:
            self.broken = True
            raise
        self.sequence, self.previous, self.last_time = event['sequence'], checksum, at

    def sample(self, packet):
        self.append('sample', {'snapshot':snapshot(self.model), 'public_sha256':digest(packet)})

    def effect(self, before, admission, effect):
        self.append('effect', {'before':before, 'after':snapshot(self.model), 'effect':effect,
            'admission':{k:admission[k] for k in ('stamp_ns','distance_m','speed_mps')}})

    def finish(self, complete):
        try:
            self.append('end', {'complete':complete, 'snapshot':snapshot(self.model),
                'journal_sequence':self.journal.sequence, 'journal_sha256':self.journal.previous})
            if complete:
                # This marker means the observation payload was already synced
                # and read back. A failed final-payload sync cannot authorize it.
                self.journal.complete_observations(self.previous, self.sequence, self.revision)
        except Exception:
            if complete: raise
        finally: self.close()

    def close(self):
        self.closed = True
        for name in ('fd','parent'):
            fd = getattr(self, name)
            if fd is not None: os.close(fd); setattr(self, name, None)
