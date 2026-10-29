"""Durable scenario claims fence replay without adopting historical effects."""
import copy
import hashlib
import json
import os
import select
from pathlib import Path
import shutil
import subprocess
import sys
import uuid
from unittest.mock import patch

from orbital.inspection_protocol import canonical
from orbital.inspection_worker import OwnerLock
from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.maintenance_owner import MaintenanceOwner
from orbital.maintenance_journal import Journal, JournalUnavailable, prepare_scenario, read_history, replay
from orbital import maintenance_journal as storage
from orbital.machines import Fault
from test_machine_registry import ROOT
from test_machine_telemetry_adapter import AdapterHarness
from test_maintenance_cancellation import CancellationFixture
import test_maintenance_owner as isolation


class JournalFixture(CancellationFixture):
    def setUp(self):
        super().setUp()
        self.attach()

    def attach(self):
        prepare_scenario(self.root)
        self.lock = OwnerLock(self.root, service='station-service')
        self.addCleanup(self.lock.close)
        self.journal = Journal(self.lock)
        self.addCleanup(self.journal.close)
        old = self.owner
        self.owner = MaintenanceOwner(self.root, self.model, self.live, self.hold, old.worker,
            now=old.now, sim_ns=old.sim_ns, on_result=self.results.append,
            inspection_factory=old.inspection_factory, load=self.load, journal=self.journal)
        self.request.update(run_id=self.owner.run_id)
        self.path = self.root/'runtime/station-service/events.jsonl'

    def new_scenario(self):
        self.journal.close(); self.lock.close()
        old = self.owner
        self.model = MachineTelemetry(MachineRegistry.load(self.root), Fault.LATCHED)
        self.hold = isolation.Hold()
        begin = self.hold.begin
        def acquire(token):
            self.hold.acknowledged = self.hold.released = False
            self.hold.release_at = None
            begin(token)
        self.hold.begin = acquire
        self.owner = MaintenanceOwner(self.root, self.model, self.live, self.hold, old.worker,
            now=old.now, sim_ns=old.sim_ns, on_result=self.results.append,
            inspection_factory=old.inspection_factory, load=self.load)
        self.request = {**self.request, 'action_id':uuid.uuid4().hex, 'command_id':uuid.uuid4().hex,
            'arguments':{**self.request['arguments'],'station_run_id':self.model.run_id,'control_revision':0}}
        self.attach()
        self.sample(.05); self.sample(.05)

    def failed_sync(self, number):
        actual, calls = os.fsync, [0]
        def sync(fd):
            if fd == self.journal.fd:
                calls[0] += 1
                if calls[0] == number: raise OSError('controlled journal sync failure')
            return actual(fd)
        return patch.object(storage.os, 'fsync', sync)


class JournalTests(JournalFixture):
    def test_complete_workflow_retains_exact_original_child_and_terminal_history(self):
        value = self.completed()
        history = read_history(self.root)
        self.assertEqual(len(history.actions),5)
        self.assertEqual(history.actions[value['action_id']]['result'],value)
        self.assertEqual(history.actions[value['target_request']['action_id']]['result'],value['target_result'])
        self.assertEqual(history.actions[value['isolation_action_id']]['result'],value['isolation_result'])
        self.assertEqual(history.status()['unresolved_count'],0)
        self.assertEqual(len(history.budgets),2)
        before = self.path.read_bytes()
        reply = self.send(self.cancel_request)
        self.assertTrue(reply['accepted'])
        self.assertEqual(reply['action']['result'],value)
        self.assertEqual(self.path.read_bytes(),before)
        self.assertEqual(self.model.control_revision,4)

    def test_station_restart_cannot_claim_generation_or_publish_new_model(self):
        self.reset_complete()
        before = self.path.read_bytes()
        self.journal.close(); self.lock.close()
        harness = AdapterHarness()
        with self.assertRaisesRegex(ValueError, 'scenario_already_claimed'):
            harness.run(self.root,prepare=False)
        self.assertEqual(harness.messages['/station/telemetry'],[])
        self.assertEqual(self.path.read_bytes(),before)
        self.assertTrue(harness.destroyed and harness.shutdown)

    def test_explicit_new_scenario_preserves_old_mission_and_action_refusals(self):
        original = copy.deepcopy(self.reset_complete())
        old_action = self.request['action_id']
        self.new_scenario()
        self.assertEqual(self.send()['error_code'],'mission_reused')
        self.request['mission_id'] = uuid.uuid4().hex
        self.assertEqual(self.send({**self.request,'action_id':old_action})['error_code'],'action_reused')
        value = self.reset_complete()
        self.assertEqual(value['status'],'succeeded')
        self.assertEqual(self.model.station.machines['pump'].reset_count,1)
        history = read_history(self.root)
        self.assertEqual(history.actions[original['action_id']]['result'],original)
        self.assertEqual(len(history.scenarios),2)
        self.assertNotEqual(value['mission_id'],original['mission_id'])

    def test_admission_is_durable_before_guard_acquire(self):
        self.sample(.05); self.sample(.05)
        begin = self.hold.begin
        seen = []
        def acquire(token):
            record = read_history(self.root).actions[token['action_id']]
            self.assertEqual(record['request']['mission_id'],token['mission_id'])
            self.assertIsNone(record['intent'])
            seen.append(token)
            begin(token)
        self.hold.begin = acquire
        self.assertTrue(self.send()['accepted'])
        self.assertEqual(len(seen),1)

    def test_cancellation_reservation_is_durable_before_original_is_stopped(self):
        self.prepare(); self.start(self.restore_request)
        stop = self.owner.stop
        seen = []
        def stopped(*args):
            history = read_history(self.root)
            workflow = self.owner.cancellation
            self.assertEqual(history.reserved[workflow.isolation_id],workflow.request['action_id'])
            self.assertEqual(history.actions[workflow.request['action_id']]['request'],workflow.request)
            seen.append(True)
            stop(*args)
        self.owner.stop = stopped
        self.cancel_isolate(self.restore_request)
        self.assertTrue(seen)
        self.assertTrue(self.workers[-1].stopped)

    def test_failed_admission_sync_never_acquires_hold_or_reports_acceptance(self):
        self.sample(.05); self.sample(.05)
        with self.failed_sync(1): reply = self.send()
        self.assertFalse(reply['accepted'])
        self.assertEqual(reply['error_code'],'journal_unavailable')
        self.assertIsNone(self.hold.owner)
        self.assertIsNone(self.owner.active)
        self.assertTrue(self.journal.broken)
        self.assertFalse(self.owner.ready())
        self.assertEqual(self.model.control_revision,0)

    def test_effect_intent_delay_requires_final_freshness_after_disk_io(self):
        self.sample(.05); self.sample(.05); self.start(self.request)
        record = self.journal.record
        def delayed(kind,value):
            record(kind,value)
            if kind == 'intent': self.clock.advance(.201)
        self.journal.record = delayed
        self.apply()
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertIsNone(self.owner.active.effect)
        value = self.finish()
        self.assertEqual(value['status'],'failed')
        history = read_history(self.root).actions[self.request['action_id']]
        self.assertTrue(history['rejected'])
        self.assertIsNone(history['effect'])
        self.assertEqual(history['result'],value)

    def test_intent_sync_failure_retains_owner_without_effect_or_release(self):
        self.sample(.05); self.sample(.05); self.start(self.request)
        with self.failed_sync(1): self.apply()
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertTrue(self.owner.active.unknown)
        self.assertEqual(self.owner.active.phase,'unresolved')
        self.assertTrue(self.hold.held())
        self.assertIsNone(self.hold.release_at)
        self.assertEqual(self.results,[])

    def test_applied_sync_failure_keeps_actual_effect_and_forbids_retry(self):
        self.sample(.05); self.sample(.05); self.start(self.request)
        with self.failed_sync(2): self.apply()
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertEqual(self.model.control_revision,1)
        self.assertTrue(self.owner.active.unknown)
        self.assertTrue(self.hold.held())
        self.assertIsNone(self.hold.release_at)
        self.assertEqual(self.results,[])
        before = self.path.read_bytes()
        self.owner.tick(); self.send()
        self.assertEqual(self.path.read_bytes(),before)
        self.assertEqual(self.model.control_revision,1)

    def test_terminal_sync_failure_retains_uncertain_publication_and_no_new_action(self):
        self.sample(.05); self.sample(.05); self.start(self.request); self.apply()
        self.sample(.05); self.sample(.05)
        self.hold.released = True
        with self.failed_sync(1): self.owner.tick()
        self.assertTrue(self.owner.active.unknown)
        self.assertEqual(self.owner.active.phase,'unresolved')
        self.assertEqual(self.results,[])
        self.assertIsNone(self.hold.owner)  # Normal release completed before the result decision.
        self.assertFalse(self.owner.ready())
        self.assertFalse(self.send({**self.request,'action_id':uuid.uuid4().hex})['accepted'])

    def test_cancellation_terminal_sync_failure_never_retries_or_publishes(self):
        self.isolated(); self.cancel_isolate(self.request)
        self.sample(.05)
        with self.failed_sync(1): self.sample(.05)
        self.assertIsNone(self.workflow.result)
        self.assertTrue(self.workflow.persistence_failed)
        self.assertEqual(self.workflow.phase,'unresolved')
        before = self.path.read_bytes()
        for _ in range(3): self.owner.tick()
        self.assertEqual(self.path.read_bytes(),before)
        self.assertEqual(len(self.results),1)

    def test_release_io_crossing_deadline_cannot_persist_success(self):
        self.request['arguments']['timeout_s'] = .2
        self.sample(.05); self.sample(.05); self.start(self.request); self.apply()
        self.sample(.05); self.sample(.05)
        self.hold.released = True
        release = self.hold.release
        def delayed():
            self.clock.advance(.11)
            return release()
        self.hold.release = delayed
        self.owner.tick()
        value = self.results[-1]
        self.assertEqual((value['status'],value['error_code']),('timed_out','deadline_exceeded'))
        self.assertEqual(read_history(self.root).actions[value['action_id']]['result'],value)

    def test_expired_after_release_is_evidence_loss_without_storage_failure(self):
        self.sample(.05); self.sample(.05); self.start(self.request); self.apply()
        self.sample(.05); self.sample(.05)
        self.hold.released = True
        release = self.hold.release
        def delayed():
            self.clock.advance(1.001)
            return release()
        self.hold.release = delayed
        self.owner.tick()
        self.assertFalse(self.journal.broken)
        self.assertEqual(self.results,[])
        self.assertIsNotNone(self.owner.active)
        self.assertFalse(self.owner.active.unknown)
        self.assertEqual(self.owner.active.phase,'unresolved')

    def test_terminal_disk_delay_does_not_renew_the_completed_decision_time(self):
        self.request['arguments']['timeout_s'] = .2
        self.sample(.05); self.sample(.05); self.start(self.request); self.apply()
        self.sample(.05); self.sample(.05)
        decision_sim, decision_wall = self.clock.sim, self.clock.wall
        record = self.journal.record
        def delayed(kind,value):
            record(kind,value)
            if kind == 'terminal': self.clock.advance(2.)
        self.journal.record = delayed
        value = self.finish()
        self.assertEqual(value['status'],'succeeded')
        self.assertEqual(value['finished_sim_stamp_ns'],decision_sim)
        self.assertLess(value['wall_duration_s'],.2)
        self.assertGreater(self.clock.wall,decision_wall+1.)
        self.assertEqual(read_history(self.root).actions[value['action_id']]['result'],value)

    def test_journal_capacity_refuses_before_hold_or_effect(self):
        self.sample(.05); self.sample(.05)
        before = self.path.read_bytes()
        with patch.object(storage,'MAX_BYTES',len(before)+storage.MAX_EVENT_BYTES):
            reply = self.send()
        self.assertFalse(reply['accepted'])
        self.assertEqual(reply['error_code'],'journal_full')
        self.assertIsNone(self.hold.owner)
        self.assertEqual(self.path.read_bytes(),before)

    def test_clock_discontinuity_cannot_rebuild_durable_station_or_attempts(self):
        self.reset_complete()
        identity, attempts = self.model.run_id, copy.deepcopy(self.model.reset_attempts)
        for stamp in (0, self.clock.sim+10000000001):
            with self.subTest(stamp=stamp), self.assertRaisesRegex(ValueError,'requires_new_scenario'):
                self.model.sample(stamp)
            self.assertEqual(self.model.run_id,identity)
            self.assertEqual(self.model.reset_attempts,attempts)
            self.assertFalse(self.model.station.machines['pump'].powered)

    def test_partial_corrupt_duplicate_or_invalid_transition_records_refuse(self):
        raw = self.path.read_bytes()
        first = json.loads(raw.splitlines()[0])
        wrong_sequence = {**first,'sequence':2}
        wrong_hash = {**first,'sha256':'f'*64}
        duplicate = raw.splitlines()[0].replace(b'"schema_version":1',b'"schema_version":1,"schema_version":1')+b'\n'
        variants = [raw[:-1], raw[:-10], raw+b'{}\n', (canonical(wrong_sequence)+'\n').encode(),
                    (canonical(wrong_hash)+'\n').encode(), duplicate, raw+raw.splitlines()[0]+b'\n']
        for value in variants:
            with self.subTest(value=value[:60]), self.assertRaises((ValueError,UnicodeError)):
                replay(value)
        self.assertEqual(self.path.read_bytes(),raw)

    def test_replaced_file_and_changed_directory_refuse_before_admission(self):
        self.sample(.05); self.sample(.05)
        original = self.path.read_bytes()
        displaced = self.path.with_name('retained-original.jsonl')
        self.path.rename(displaced)
        self.path.write_bytes(original); self.path.chmod(0o600)
        self.assertEqual(self.send()['error_code'],'journal_unavailable')
        self.assertIsNone(self.hold.owner)
        self.assertEqual(displaced.read_bytes(),original)

    def test_links_permissions_and_foreign_record_bytes_are_not_adopted(self):
        original = self.path.read_bytes()
        self.journal.close(); self.lock.close()
        alternate = self.path.with_name('retained.jsonl')
        self.path.rename(alternate)
        for kind in ('symlink','hardlink','permission'):
            if kind == 'symlink': self.path.symlink_to(alternate)
            elif kind == 'hardlink': os.link(alternate,self.path)
            else: self.path.write_bytes(original); self.path.chmod(0o644)
            lock = OwnerLock(self.root,service='station-service')
            try:
                with self.subTest(kind=kind), self.assertRaises((ValueError,OSError,JournalUnavailable)):
                    Journal(lock)
            finally:
                lock.close(); self.path.unlink()
        self.assertEqual(alternate.read_bytes(),original)

    def test_operator_cli_is_read_only_and_prepare_requires_explicit_new_world(self):
        value = self.reset_complete()
        script = self.root/'scripts/maintenance-journal.py'
        script.parent.mkdir(exist_ok=True)
        script.write_bytes((ROOT/'scripts/maintenance-journal.py').read_bytes())
        environment = {**os.environ,'PYTHONPATH':str(ROOT)}
        before = self.path.read_bytes()
        for arguments in [('status',),('query','--action-id',value['action_id'])]:
            process = subprocess.run([sys.executable,str(script),*arguments],env=environment,capture_output=True,text=True)
            self.assertEqual(process.returncode,0,process.stderr)
            self.assertTrue(json.loads(process.stdout)['historical_only'])
        for arguments in [('prepare',),('prepare','--new-world')]:
            process = subprocess.run([sys.executable,str(script),*arguments],env=environment,capture_output=True,text=True)
            self.assertEqual(process.returncode,2)
        self.assertEqual(self.path.read_bytes(),before)

    def test_partial_write_is_retained_and_never_truncated_into_permission(self):
        self.sample(.05); self.sample(.05)
        before = self.path.read_bytes()
        write, calls = os.write, [0]
        def partial(fd, raw):
            if fd == self.journal.fd:
                calls[0] += 1
                if calls[0] == 1: return write(fd,raw[:len(raw)//2])
                raise OSError('controlled interrupted append')
            return write(fd,raw)
        with patch.object(storage.os,'write',partial): reply = self.send()
        self.assertEqual(reply['error_code'],'journal_unavailable')
        self.assertIsNone(self.hold.owner)
        damaged = self.path.read_bytes()
        self.assertTrue(damaged.startswith(before))
        self.assertGreater(len(damaged),len(before))
        self.assertFalse(damaged.endswith(b'\n'))
        self.journal.close(); self.lock.close()
        with self.assertRaisesRegex(ValueError,'partial'): prepare_scenario(self.root)
        self.assertEqual(self.path.read_bytes(),damaged)

    def test_directory_privacy_change_prevents_admission_and_history_read(self):
        self.sample(.05); self.sample(.05)
        self.path.parent.chmod(0o777)
        self.assertEqual(self.send()['error_code'],'journal_unavailable')
        self.assertIsNone(self.hold.owner)
        with self.assertRaisesRegex(ValueError,'not_private'): read_history(self.root)

    def test_known_rejected_reset_intent_does_not_spend_a_physical_attempt(self):
        self.isolated()
        record, delayed = self.journal.record, [False]
        def delay(kind,value):
            record(kind,value)
            if kind == 'intent' and value['operation'] == 'reset' and not delayed[0]:
                delayed[0] = True
                self.clock.advance(.201)
        self.journal.record = delay
        self.start(self.reset_request); self.apply()
        failed = self.finish()
        self.assertEqual(failed['state_change']['disposition'],'not_applied')
        self.assertTrue(self.model.reset_available('pump'))
        self.assertEqual(read_history(self.root).budgets,{})
        self.sample(.05); self.sample(.05)
        request = {**self.reset_request,'action_id':uuid.uuid4().hex,'command_id':uuid.uuid4().hex}
        self.start(request); self.apply(); self.sample(.05); self.sample(.05)
        value = self.finish()
        self.assertEqual(value['status'],'succeeded')
        self.assertEqual(self.model.station.machines['pump'].reset_count,1)
        self.assertEqual(read_history(self.root).actions[failed['action_id']]['result'],failed)

    def test_actual_process_crash_cuts_never_allow_restart_or_old_identity_reuse(self):
        from maintenance_journal_crash import build
        self.crash_receipts = []
        for cut in ('admit','intent','mutated','applied','terminal'):
            with self.subTest(cut=cut):
                root = self.root/('crash-'+cut)
                (root/'config').mkdir(parents=True); (root/'worlds').mkdir()
                shutil.copyfile(self.root/'config/known-map.json',root/'config/known-map.json')
                shutil.copyfile(self.root/'worlds/maintenance.sdf',root/'worlds/maintenance.sdf')
                process = subprocess.Popen([sys.executable,str(ROOT/'tests/maintenance_journal_crash.py'),str(root),cut],
                    stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,cwd=ROOT)
                try:
                    ready,_,_ = select.select([process.stdout],[],[],10.)
                    self.assertTrue(ready,'Child did not reach the persisted boundary')
                    line = process.stdout.readline()
                    self.assertTrue(line,process.stderr.read() if process.poll() is not None else 'Child produced no receipt')
                    receipt = json.loads(line)
                    self.assertEqual(receipt['pid'],process.pid)
                    self.assertEqual(receipt['cut'],cut)
                    self.assertIsNone(process.poll())
                    path = root/'runtime/station-service/events.jsonl'
                    before = path.read_bytes()
                    with self.assertRaises(BlockingIOError): prepare_scenario(root)
                    process.kill(); process.wait(timeout=5.)
                    self.assertEqual(process.returncode,-9)
                    self.assertEqual(path.read_bytes(),before)
                    history = read_history(root)
                    action = history.actions[receipt['action_id']]
                    self.assertEqual(receipt['published_results'],0)
                    self.assertEqual(receipt['control_revision'],int(cut in ('applied','terminal')))
                    self.assertEqual(receipt['powered'],cut in ('admit','intent'))
                    self.assertEqual(action['intent'] is not None,cut != 'admit')
                    self.assertEqual(action['effect'] is not None,cut in ('applied','terminal'))
                    self.assertEqual(action['result'] is not None,cut == 'terminal')
                    harness = AdapterHarness()
                    with self.assertRaisesRegex(ValueError,'scenario_already_claimed'):
                        harness.run(root,prepare=False)
                    self.assertEqual(harness.messages['/station/telemetry'],[])
                    self.assertEqual(path.read_bytes(),before)
                    restarted = build(root)
                    try:
                        restarted.sample(.05); restarted.sample(.05)
                        reused = {**restarted.request,'mission_id':receipt['mission_id']}
                        self.assertEqual(restarted.send(reused)['error_code'],'mission_reused')
                        reused = {**restarted.request,'action_id':receipt['action_id']}
                        self.assertEqual(restarted.send(reused)['error_code'],'action_reused')
                        value = restarted.reset_complete()
                        self.assertEqual(value['status'],'succeeded')
                        self.assertEqual(restarted.model.station.machines['pump'].reset_count,1)
                        self.assertEqual(read_history(root).actions[receipt['action_id']],action)
                    finally: restarted.doCleanups()
                    receipt.update(exit_code=process.returncode,old_history_sha256=hashlib.sha256(before).hexdigest(),
                        old_record_unchanged=True,station_restart_refused=True,old_mission_and_action_refused=True,
                        new_mission_result=value['status'],retained_state=history.status())
                    (root/'process-proof.json').write_text(json.dumps(receipt,indent=2)+'\n')
                    self.crash_receipts.append(receipt)
                finally:
                    if process.poll() is None: process.kill(); process.wait(timeout=5.)
                    process.stdout.close(); process.stderr.close()

    def test_raw_station_without_prepared_generation_publishes_nothing(self):
        # A separate fixture root has no issued world generation.
        fresh = self.root/'unprepared'
        (fresh/'config').mkdir(parents=True); (fresh/'worlds').mkdir()
        shutil.copyfile(self.root/'config/known-map.json',fresh/'config/known-map.json')
        shutil.copyfile(self.root/'worlds/maintenance.sdf',fresh/'worlds/maintenance.sdf')
        harness = AdapterHarness()
        with self.assertRaises(FileNotFoundError): harness.run(fresh,prepare=False)
        self.assertEqual(harness.messages['/station/telemetry'],[])
