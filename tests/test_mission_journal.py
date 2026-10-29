"""Actual files and process loss preserve unresolved mission admission."""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import unittest
from unittest.mock import patch
import uuid

from orbital.inspection_worker import OwnerLock
from orbital.mission_journal import MissionJournal
from orbital import mission_journal as storage
from test_maintenance_reset import ResetFixture

ROOT=Path(__file__).resolve().parents[1]


class MissionJournalTests(unittest.TestCase):
    def setUp(self):
        self.f=ResetFixture();self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.root=self.f.root
        self.lock=OwnerLock(self.root,service='mission-service');self.addCleanup(self.lock.close)
        self.journal=MissionJournal(self.lock);self.addCleanup(self.journal.close)
        self.path=self.root/'runtime/mission-service/events.jsonl'
        baseline=self.f.sample(.05)
        self.start={'registry_identity':self.f.model.registry.identity,'station_run_id':baseline['run_id'],
            'policy_version':'public-measurements-v1','baseline':baseline,
            'started_wall_s':self.f.clock.wall,'started_sim_ns':self.f.clock.sim,'deadline_wall_s':self.f.clock.wall+60.}
        self.mission=uuid.uuid4().hex

    def reopen(self):
        self.journal.close()
        self.journal=MissionJournal(self.lock);self.addCleanup(self.journal.close)

    def test_reopening_does_not_renew_unfinished_mission_or_allow_another(self):
        self.journal.append(self.mission,'start',self.start)
        before=self.path.read_bytes()
        self.reopen()
        self.assertEqual(self.journal.active,self.mission)
        with self.assertRaisesRegex(ValueError,'unfinished_mission'):
            self.journal.append(uuid.uuid4().hex,'start',self.start)
        self.assertEqual(self.path.read_bytes(),before)

    def test_other_mission_cannot_append_to_owned_history(self):
        self.journal.append(self.mission,'start',self.start)
        before=self.path.read_bytes()
        with self.assertRaises(ValueError):self.journal.append(uuid.uuid4().hex,'command',{})
        self.assertEqual(self.path.read_bytes(),before)

    def test_actual_sigkill_leaves_durable_admission_after_process_lock_releases(self):
        self.journal.close();self.lock.close()
        source='''import json,signal,sys
from orbital.inspection_worker import OwnerLock
from orbital.mission_journal import MissionJournal
value=json.loads(sys.stdin.readline())
lock=OwnerLock(sys.argv[1],service='mission-service')
journal=MissionJournal(lock)
journal.append(value['mission'],'start',value['start'])
print('durable',flush=True)
signal.pause()
'''
        child=subprocess.Popen([sys.executable,'-B','-c',source,str(self.root)],cwd=ROOT,
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            child.stdin.write(json.dumps({'mission':self.mission,'start':self.start})+'\n');child.stdin.flush()
            self.assertTrue(select.select([child.stdout],[],[],3.)[0])
            self.assertEqual(child.stdout.readline().strip(),'durable')
            child.kill();child.wait(timeout=3.)
        finally:
            if child.poll() is None:child.kill();child.wait(timeout=3.)
            for stream in (child.stdin,child.stdout,child.stderr):stream.close()
        self.lock=OwnerLock(self.root,service='mission-service');self.addCleanup(self.lock.close)
        self.journal=MissionJournal(self.lock);self.addCleanup(self.journal.close)
        self.assertEqual(self.journal.active,self.mission)
        with self.assertRaisesRegex(ValueError,'unfinished_mission'):
            self.journal.append(uuid.uuid4().hex,'start',self.start)

    def test_partial_write_is_sticky_and_unreadable_after_reopen(self):
        actual=os.write
        def short(fd,raw):return actual(fd,raw[:len(raw)//2])
        with patch.object(storage.os,'write',side_effect=short):
            with self.assertRaises(OSError):self.journal.append(self.mission,'start',self.start)
        before=self.path.read_bytes()
        with self.assertRaisesRegex(ValueError,'unavailable'):self.journal.append(self.mission,'start',self.start)
        self.journal.close()
        with self.assertRaises(ValueError):MissionJournal(self.lock)
        self.assertEqual(self.path.read_bytes(),before)

    def test_failed_sync_cannot_be_retried_as_a_fresh_start(self):
        actual=os.fsync
        def failed(fd):
            if fd==self.journal.fd:raise OSError('controlled sync failure')
            return actual(fd)
        with patch.object(storage.os,'fsync',side_effect=failed):
            with self.assertRaises(OSError):self.journal.append(self.mission,'start',self.start)
        self.assertTrue(self.journal.broken)
        self.reopen()
        self.assertEqual(self.journal.active,self.mission)

    def test_changed_prefix_and_replaced_file_are_refused(self):
        self.journal.append(self.mission,'start',self.start)
        raw=self.path.read_bytes()
        self.path.write_bytes(raw.replace(b'public-measurements-v1',b'public-measurements-v2'))
        with self.assertRaisesRegex(ValueError,'changed'):self.journal.verify()
        self.journal.close()
        with self.assertRaisesRegex(ValueError,'digest'):MissionJournal(self.lock)

    def test_public_mode_and_symlink_are_refused(self):
        self.journal.close()
        self.path.chmod(0o644)
        with self.assertRaisesRegex(ValueError,'private'):MissionJournal(self.lock)
        self.path.chmod(0o600)
        original=self.path.with_suffix('.original');self.path.rename(original)
        self.path.symlink_to(original.name)
        with self.assertRaises(OSError):MissionJournal(self.lock)

    def test_concurrent_change_while_reading_is_refused(self):
        self.journal.append(self.mission,'start',self.start);self.journal.close()
        actual=os.pread
        def changed(fd,size,offset):
            data=actual(fd,size,offset)
            with self.path.open('ab') as stream:stream.write(b'\n')
            return data
        with patch.object(storage.os,'pread',side_effect=changed):
            with self.assertRaisesRegex(ValueError,'changed_during_read'):MissionJournal(self.lock)

    def test_bounds_refuse_before_appending_and_preserve_history(self):
        self.journal.append(self.mission,'start',self.start)
        before=self.path.read_bytes()
        with patch.object(storage,'MAX_BYTES',len(before)):
            with self.assertRaisesRegex(ValueError,'full'):self.journal.append(self.mission,'telemetry',{'text':'x'})
        self.assertEqual(self.path.read_bytes(),before)
