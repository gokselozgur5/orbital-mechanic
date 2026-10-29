"""Provider ownership survives actual child SIGKILL without automatic replay."""
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
from orbital.planner_recording import RecordedProvider
from orbital import ros_mission
from planner_provider_fixture import recording,step

ROOT=Path(__file__).resolve().parents[1]


class PlannerProviderCrashTests(unittest.TestCase):
    def check_cut(self,mode):
        with tempfile.TemporaryDirectory(prefix='provider-crash-') as directory:
            env={**os.environ,'TMPDIR':directory,'PYTHONDONTWRITEBYTECODE':'1',
                 'PYTHONPATH':str(ROOT)+os.pathsep+str(ROOT/'tests')}
            child=subprocess.Popen([sys.executable,'-B',str(ROOT/'tests/planner_provider_crash_fixture.py'),mode],
                cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            try:
                self.assertTrue(select.select([child.stdout],[],[],60.)[0],'checkpoint deadline')
                line=child.stdout.readline()
                if not line:self.fail('child exited before checkpoint: '+child.stderr.read())
                checkpoint=json.loads(line);child.kill();self.assertEqual(child.wait(timeout=5.),-9)
            finally:
                if child.poll() is None:child.kill();child.wait(timeout=5.)
                child.stdout.close();child.stderr.close()
            root=Path(checkpoint['root']);self.assertTrue(root.is_relative_to(directory))
            lock=OwnerLock(root,service='mission-service')
            try:
                journal=MissionJournal(lock)
                try:
                    self.assertEqual(journal.active,checkpoint['mission_id'])
                    dialog=journal.provider.get(journal.active)
                    self.assertFalse(dialog['closed'])
                    call=dialog['calls'][checkpoint['provider_id']]
                    native=journal.tools.get(journal.active)['calls'].get(checkpoint['action_id'])
                    self.assertEqual(call['request']['action_id'],checkpoint['action_id'])
                    if mode=='before_send':self.assertIsNone(native)
                    else:self.assertEqual(native['response']['result']['effect'],'reported')
                    self.assertEqual(call['response'] is not None,mode=='after_receipt')
                    with self.assertRaisesRegex(ValueError,'reset_unavailable'):journal.reset_planner_stop()
                finally:journal.close()
            finally:lock.close()
            before=(root/'runtime/mission-service/events.jsonl').read_bytes()
            provider=RecordedProvider(recording([step('stop',{'reason':'uncertain'})]))
            def never(**kwargs):self.fail('unfinished ownership must refuse before transport construction')
            with self.assertRaisesRegex(ValueError,'unfinished_mission'):
                ros_mission.run(root,transport_factory=never,provider=provider)
            self.assertTrue(provider.closed);self.assertEqual(provider.index,0)
            self.assertEqual((root/'runtime/mission-service/events.jsonl').read_bytes(),before)
            self.assertEqual(checkpoint['control_revision'],0 if mode=='before_send' else 1)
            if mode=='before_send':self.assertEqual(checkpoint['published_commands'],[])
            else:
                self.assertFalse(checkpoint['powered'])
                self.assertEqual([v['op'] for family,v in checkpoint['published_commands'] if family=='maintenance'],['isolate'])

    def test_kill_after_provider_binding_before_native_admission(self):self.check_cut('before_send')
    def test_kill_after_actual_isolation_before_provider_receipt(self):self.check_cut('after_effect')
    def test_kill_after_durable_receipt_before_model_result_return(self):self.check_cut('after_receipt')
