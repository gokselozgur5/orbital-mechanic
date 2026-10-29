"""SIGKILL cannot turn an admitted logical call into fresh executable work."""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import unittest

from orbital.inspection_worker import OwnerLock
from orbital.mission_journal import MissionJournal
from orbital import ros_mission

ROOT=Path(__file__).resolve().parents[1]


class PlannerCrashTests(unittest.TestCase):
    def check_cut(self,mode):
        with tempfile.TemporaryDirectory(prefix='planner-crash-') as directory:
            env={**os.environ,'TMPDIR':directory,'PYTHONDONTWRITEBYTECODE':'1',
                 'PYTHONPATH':str(ROOT)+os.pathsep+str(ROOT/'tests')}
            child=subprocess.Popen([sys.executable,'-B',str(ROOT/'tests/planner_crash_fixture.py'),mode],
                cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            try:
                self.assertTrue(select.select([child.stdout],[],[],60.)[0],'checkpoint deadline')
                line=child.stdout.readline()
                if not line:self.fail('child exited before checkpoint: '+child.stderr.read())
                checkpoint=json.loads(line)
                child.kill();self.assertEqual(child.wait(timeout=5.),-9)
            finally:
                if child.poll() is None:child.kill();child.wait(timeout=5.)
                child.stdout.close();child.stderr.close()
            root=Path(checkpoint['root'])
            self.assertTrue(root.is_relative_to(directory))
            lock=OwnerLock(root,service='mission-service')
            try:
                journal=MissionJournal(lock)
                try:
                    self.assertEqual(journal.active,checkpoint['mission_id'])
                    call=journal.tools.get(journal.active)['calls'][checkpoint['action_id']]
                    self.assertEqual(len(call['bindings']),1)
                    if mode=='after_result':
                        self.assertEqual(call['response']['result']['effect'],'reported')
                    else:self.assertIsNone(call['response'])
                    with self.assertRaisesRegex(ValueError,'reset_unavailable'):journal.reset_planner_stop()
                finally:journal.close()
            finally:lock.close()
            before=(root/'runtime/mission-service/events.jsonl').read_bytes()
            def never(**kwargs):self.fail('unfinished ownership must refuse before transport construction')
            with self.assertRaisesRegex(ValueError,'unfinished_mission'):
                ros_mission.run(root,transport_factory=never,tool_contract=True)
            self.assertEqual((root/'runtime/mission-service/events.jsonl').read_bytes(),before)
            if mode=='before_send':
                self.assertEqual(checkpoint['published_commands'],[])
                self.assertEqual(checkpoint['control_revision'],0)
            else:
                self.assertEqual(checkpoint['control_revision'],1)
                self.assertFalse(checkpoint['powered'])
                operations=[command['op'] for family,command in checkpoint['published_commands']
                            if family=='maintenance']
                self.assertEqual(operations,['isolate'])

    def test_kill_after_binding_before_publication_retains_pending_call(self):self.check_cut('before_send')
    def test_kill_after_actual_effect_before_result_retains_uncertainty(self):self.check_cut('after_effect')
    def test_kill_after_result_before_return_still_refuses_unfinished_mission(self):self.check_cut('after_result')
