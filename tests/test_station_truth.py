"""Failures in actual private observation storage cannot manufacture a score."""
import json
import os
import subprocess
import sys
from unittest.mock import patch

from orbital import station_truth as storage, ros_maintenance
from orbital.maintenance_journal import read_history
from orbital.station_evaluation import evaluate
from orbital.station_truth import ObservationUnavailable
from station_evaluation_fixture import StationFixture
from test_machine_registry import RegistryFixture, ROOT
from test_machine_telemetry_adapter import AdapterHarness


class StationTruthTests(StationFixture):
    def test_partial_write_is_retained_and_never_retried(self):
        self.attach('healthy');before=self.path.read_bytes()
        write=os.write
        def partial(fd,raw):return write(fd,raw[:len(raw)//2]) if fd==self.truth.fd else write(fd,raw)
        with patch.object(storage.os,'write',side_effect=partial):
            with self.assertRaises(OSError):self.model.sample(1_000_000_000)
        after=self.path.read_bytes()
        self.assertTrue(after.startswith(before));self.assertFalse(after.endswith(b'\n'))
        with self.assertRaises(ObservationUnavailable):self.model.sample(2_000_000_000)
        self.assertEqual(self.path.read_bytes(),after)
        self.close_station(False)
        with self.assertRaises(ValueError):evaluate(self.root,self.assignment['trial_id'])

    def test_failed_effect_observation_sync_retains_actual_effect_and_unknown_owner(self):
        self.attach();self.sample(.05);self.sample();self.start(self.request)
        sync=os.fsync
        def fail(fd):
            if fd==self.truth.fd:raise OSError('controlled private observation sync failure')
            return sync(fd)
        with patch.object(storage.os,'fsync',side_effect=fail):self.apply()
        self.assertEqual(self.model.control_revision,1)
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertTrue(self.owner.active.unknown)
        self.assertEqual(self.results,[])
        action=read_history(self.root).actions[self.request['action_id']]
        self.assertEqual(action['effect']['control_revision'],1)
        self.assertIsNone(action['result'])
        self.close_station(False)
        with self.assertRaises(ValueError):evaluate(self.root,self.assignment['trial_id'])

    def test_changed_file_refuses_before_an_actual_effect(self):
        self.attach();self.sample(.05);self.sample();self.start(self.request)
        with self.path.open('ab') as out:out.write(b' ')
        self.apply()
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertEqual(self.model.control_revision,0)
        self.assertTrue(read_history(self.root).actions[self.request['action_id']]['rejected'])

    def test_private_preflight_delay_cannot_spend_stale_live_permission(self):
        self.attach();self.sample(.05);self.sample(.05);self.start(self.request)
        reserve=self.truth.reserve
        def late():
            reserve();self.clock.advance(.4)
        with patch.object(self.truth,'reserve',side_effect=late):self.apply()
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertEqual(self.model.control_revision,0)
        self.assertTrue(read_history(self.root).actions[self.request['action_id']]['rejected'])

    def test_storage_capacity_refuses_before_mutation_and_keeps_end_headroom(self):
        self.attach();self.sample(.05);self.sample();self.start(self.request)
        with patch.object(storage,'MAX_RECORDS',self.truth.sequence+2):self.apply()
        self.assertEqual(self.model.control_revision,0)
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertTrue(self.truth.broken)

    def test_directory_replacement_and_file_permissions_invalidate_writer(self):
        self.attach('healthy')
        parent=self.path.parent;parent.rename(parent.with_name('retained-station-observations'))
        parent.mkdir(mode=0o700)
        self.path.write_bytes((parent.with_name('retained-station-observations')/self.path.name).read_bytes())
        self.path.chmod(0o600)
        with self.assertRaises(ObservationUnavailable):self.model.sample(1_000_000_000)
        self.assertTrue(self.truth.broken)

    def test_linked_or_nonprivate_observations_cannot_be_scored(self):
        self.attach('healthy')
        for _ in range(24):self.sample()
        self.close_station();original=self.path.with_suffix('.retained');self.path.rename(original)
        for kind in ('symlink','hardlink','mode'):
            if kind=='symlink':self.path.symlink_to(original.name)
            elif kind=='hardlink':os.link(original,self.path)
            else:self.path.write_bytes(original.read_bytes());self.path.chmod(0o644)
            try:
                with self.subTest(kind=kind),self.assertRaises((ValueError,OSError)):evaluate(self.root,self.assignment['trial_id'])
            finally:self.path.unlink()

    def test_changed_source_refuses_before_new_public_measurement(self):
        self.attach('healthy')
        with patch.object(storage,'source_revision',return_value='f'*64):
            with self.assertRaises(ObservationUnavailable):self.model.sample(1_000_000_000)
        self.assertEqual(self.model.sequence,0)

    def test_final_marker_sync_failure_cannot_return_a_successful_close(self):
        self.attach('healthy')
        for _ in range(24):self.sample()
        sync=os.fsync
        def fail(fd):
            if fd==self.truth.fd:raise OSError('controlled final sync failure')
            return sync(fd)
        with patch.object(storage.os,'fsync',side_effect=fail):
            with self.assertRaises(OSError):self.truth.finish(True)
        self.assertTrue(self.truth.closed)
        self.configuration.close();self.journal.close();self.lock.close()
        with self.assertRaises(ValueError):evaluate(self.root,self.assignment['trial_id'])

    def test_closed_observation_generation_refuses_new_admission(self):
        self.attach('healthy');self.sample()
        self.truth.finish(True)
        history=read_history(self.root)
        self.assertIn('observations_closed',history.scenarios[history.current])
        with self.assertRaisesRegex(ValueError,'observations_closed'):self.journal.admit(self.request)
        self.assertEqual(history.status()['action_count'],0)

    def test_missing_or_corrupt_authoritative_closure_cannot_accept_a_complete_file(self):
        self.attach('healthy')
        for _ in range(24):self.sample()
        self.close_station()
        journal_path=self.root/'runtime/station-service/events.jsonl'
        lines=journal_path.read_bytes().splitlines(keepends=True)
        self.assertEqual(json.loads(lines[-1])['kind'],'observations_closed')
        journal_path.write_bytes(b''.join(lines[:-1]))
        with self.assertRaisesRegex(ValueError,'closure'):evaluate(self.root,self.assignment['trial_id'])


class StationTruthAdapterTests(RegistryFixture):
    def test_actual_adapter_startup_failure_releases_resources_without_public_packets(self):
        harness=AdapterHarness()
        with patch.object(storage.StationTruth,'append',side_effect=OSError('controlled startup failure')):
            with self.assertRaisesRegex(OSError,'startup failure'):harness.run(self.root)
        self.assertEqual(harness.messages['/station/telemetry'],[])
        self.assertTrue(harness.destroyed and harness.shutdown and harness.listener_closed)
        current=read_history(self.root).current
        self.assertIsNotNone(read_history(self.root).scenarios[current]['claim'])
        with self.assertRaises(ValueError):evaluate(self.root,current)

    def test_failed_owner_cleanup_never_writes_a_complete_observation_end(self):
        real=ros_maintenance.StationMaintenance
        def broken(*args,**kw):
            owner=real(*args,**kw)
            def fail():raise RuntimeError('controlled owner cleanup failure')
            owner.owner.shutdown=fail
            return owner
        harness=AdapterHarness(times=tuple(n*500_000_000 for n in range(1,26)))
        with patch.object(ros_maintenance,'StationMaintenance',side_effect=broken):
            with self.assertRaisesRegex(RuntimeError,'cleanup failure'):harness.run(self.root)
        current=read_history(self.root).current
        self.assertTrue(harness.destroyed and harness.shutdown and harness.listener_closed)
        with self.assertRaisesRegex(ValueError,'incomplete'):evaluate(self.root,current)

    def test_actual_sigkill_before_and_after_effect_record_refuses_score_and_station_restart(self):
        for cut in ('sample','before_effect_record','after_effect_record'):
            with self.subTest(cut=cut):
                process=subprocess.Popen([sys.executable,'-B',str(ROOT/'tests/station_truth_crash.py'),str(self.root),cut],
                    env={**os.environ,'PYTHONPATH':str(ROOT)+os.pathsep+str(ROOT/'tests'),'PYTHONDONTWRITEBYTECODE':'1'},
                    stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
                def cleanup(p=process):
                    if p.poll() is None:p.kill()
                    p.wait(timeout=5.)
                    for stream in (p.stdin,p.stdout,p.stderr):
                        if stream is not None:stream.close()
                self.addCleanup(cleanup)
                # The child owns the tested model and waits only after its exact cut.
                import select
                self.assertTrue(select.select([process.stdout],[],[],5.)[0])
                line=process.stdout.readline();self.assertTrue(line,process.stderr.read() if process.poll() is not None else 'no cut receipt')
                receipt=json.loads(line)
                self.assertEqual(receipt['cut'],cut)
                process.kill();out,err=process.communicate(timeout=5.)
                self.assertEqual(process.returncode,-9,err)
                self.assertEqual(receipt['control_revision'],0 if cut=='sample' else 1)
                with self.assertRaisesRegex(ValueError,'incomplete'):evaluate(self.root,receipt['trial_id'])
                harness=AdapterHarness()
                with self.assertRaisesRegex(ValueError,'already_claimed'):harness.run(self.root,prepare=False)
                self.assertEqual(harness.messages['/station/telemetry'],[])
