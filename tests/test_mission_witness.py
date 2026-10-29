"""Actual ROS gateway publication/receipt and private storage boundary controls."""
import copy
import json
import os
from unittest.mock import patch

from orbital.inspection_worker import OwnerLock
from orbital import mission_witness as storage, mission_witness_replay as replay, ros_mission
from orbital.machines import Fault
from mission_ros_fixture import runtime_identity
from mission_runtime_fixture import RuntimeFixture
import test_ros_mission as adapter


class MissionWitnessTests(RuntimeFixture):
    composed=adapter.RosMissionTests.composed
    release_initial_gateway=adapter.RosMissionTests.release_initial_gateway

    def files(self):
        return self.f.root/'runtime/mission-service/mission-observations'/self.f.request['mission_id']

    def read(self):
        lock=OwnerLock(self.f.root,service='mission-service')
        try:return replay.read(lock,self.f.request['mission_id'],self.f.model.registry)
        finally:lock.close()

    def test_actual_publications_and_callbacks_are_retained_and_reopened(self):
        report=self.composed();self.assertTrue(report['complete'],report)
        events,evidence=self.read()
        published=[e['data'] for e in events if e['kind']=='published']
        self.assertEqual([(e['family'],json.loads(e['text'])) for e in published],self.bus.sent)
        self.assertEqual([e['publication'] for e in published],list(range(1,len(self.bus.sent)+1)))
        self.assertTrue(all(e['returned'] for e in published))
        topics={e['data']['topic'] for e in events if e['kind']=='received'}
        self.assertEqual(topics,set(self.ros.subscribers))
        self.assertEqual(events[0]['data']['journal']['sequence'],0)
        self.assertEqual(events[-1]['data']['journal']['sequence'],report['journal']['sequence'])
        self.assertEqual(evidence['summary']['payload_sha256'],report['witness']['sha256'])
        self.assertTrue(self.ros.destroyed and self.ros.shutdown)

    def test_storage_preflight_delay_cannot_publish_stale_maintenance(self):
        real=storage.MissionWitness.reserve;delayed=False
        def late(witness):
            nonlocal delayed
            real(witness)
            raw=os.pread(witness.journal.fd,witness.journal.size,0)
            if not delayed and b'"op":"isolate"' in raw:
                delayed=True;self.clock.wall+=1.
        with patch.object(storage.MissionWitness,'reserve',late):report=self.composed()
        self.assertTrue(delayed)
        self.assertFalse(any(f=='maintenance' for f,c in self.bus.sent))
        self.assertEqual(self.f.model.control_revision,0)
        self.assertNotEqual(report['mission']['outcome'],'recovered')

    def test_record_failure_after_native_arm_keeps_exact_cancellation_available(self):
        original=os.write;failed=False
        def fail(fd,raw):
            nonlocal failed
            if not failed and b'"kind":"published"' in raw:
                event=json.loads(raw)
                if json.loads(event['data']['text'])['op']=='arm':
                    failed=True;raise OSError('controlled post-publication write failure')
            return original(fd,raw)
        self.bus.hold_navigation=True
        with patch.object(storage.os,'write',side_effect=fail):report=self.composed()
        self.assertTrue(failed)
        self.assertFalse(report['complete'])
        self.assertEqual([c['op'] for f,c in self.bus.sent],['request','arm','cancel'])
        self.assertEqual(self.bus.nav['action_state'],'idle')
        self.assertTrue(self.bus.nav['guard_latched'])
        self.assertEqual(self.bus.nav['last_completion']['outcome'],'cancelled')
        self.assertFalse((self.files()/'summary.json').exists())
        with self.assertRaises((ValueError,OSError)):self.read()

    def test_final_payload_sync_failure_never_creates_complete_manifest(self):
        finish=storage.MissionWitness.finish;sync=os.fsync
        def fail(witness,*args,**kwargs):
            def sync_fail(fd):
                if fd==witness.fd:raise OSError('controlled final payload sync failure')
                return sync(fd)
            with patch.object(storage.os,'fsync',side_effect=sync_fail):return finish(witness,*args,**kwargs)
        self.f.model.station.machines['pump'].fault=Fault.JAM
        with patch.object(storage.MissionWitness,'finish',fail):report=self.composed()
        self.assertFalse(report['complete']);self.assertFalse(report['witness']['complete'])
        self.assertEqual(json.loads((self.files()/'events.jsonl').read_text().splitlines()[-1])['kind'],'end')
        self.assertFalse((self.files()/'summary.json').exists())
        with self.assertRaises((ValueError,OSError)):self.read()

    def test_replaced_runtime_generation_refuses_completed_gateway_evidence(self):
        original=runtime_identity(self.f.model.registry);changed=copy.deepcopy(original)
        changed['service_invocations']['orbital-demo.service']='e'*32
        calls=0
        def current(root):
            nonlocal calls
            calls+=1
            return original if calls==1 else changed
        with patch.object(ros_mission,'witness_runtime',side_effect=current):report=self.composed()
        self.assertEqual(report['mission']['outcome'],'recovered')
        self.assertFalse(report['complete']);self.assertFalse(report['witness']['complete'])
        self.assertFalse((self.files()/'summary.json').exists())

    def test_replayed_file_mode_link_and_final_source_changes_are_refused(self):
        self.f.model.station.machines['pump'].fault=Fault.JAM
        self.assertTrue(self.composed()['complete'])
        events,evidence=self.read()
        path=self.files()/'events.jsonl';path.chmod(0o644)
        with self.assertRaises(ValueError):self.read()
        path.chmod(0o600)
        other=self.files()/'linked.jsonl';os.link(path,other)
        try:
            with self.assertRaises(ValueError):self.read()
        finally:other.unlink()
        events,evidence=self.read()
        lock=OwnerLock(self.f.root,service='mission-service')
        try:
            with patch.object(replay,'source_revision',return_value='f'*64):
                with self.assertRaisesRegex(ValueError,'source_changed'):
                    replay.verify(lock,self.f.request['mission_id'],self.f.model.registry,evidence)
        finally:lock.close()
