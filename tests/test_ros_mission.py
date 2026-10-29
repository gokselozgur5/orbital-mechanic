"""Real ROS adapter wiring and admission; controlled peers are not native DDS."""
import json
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch

from orbital import ros_mission
from orbital.machines import Fault
from orbital.mission_journal import MissionJournal
from mission_ros_fixture import MissionRos, runtime_identity
from mission_runtime_fixture import RuntimeFixture


class RosMissionTests(RuntimeFixture):
    def release_initial_gateway(self):
        self.mission_journal.close();self.mission_lock.close()
        self.bus.callbacks={}

    def composed(self, fake=None, timeout=60.):
        self.release_initial_gateway()
        self.ros=fake or MissionRos(self.bus)
        with patch.dict(sys.modules,self.ros.modules), patch.object(ros_mission,'runtime_snapshot',
                return_value=runtime_identity(self.f.model.registry)):
            return ros_mission.run(self.f.root,mission_id=self.f.request['mission_id'],
                                   timeout_s=timeout,now=self.bus.now)

    def test_actual_adapter_runs_recovery_and_cleans_all_subscriptions(self):
        report=self.composed()
        self.assertTrue(report['complete'],report)
        self.assertTrue(report['witness']['complete'])
        self.assertEqual(report['mission']['outcome'],'recovered')
        self.assertEqual(self.f.model.control_revision,3)
        self.assertEqual(len(self.ros.subscribers),8)
        self.assertTrue(self.ros.destroyed and self.ros.shutdown)
        self.assertEqual(self.bus.callbacks,{})

    def test_cleanup_failure_is_visible_after_observed_terminal(self):
        self.f.model.station.machines['pump'].fault=Fault.JAM
        fake=MissionRos(self.bus);fake.cleanup_error=True
        report=self.composed(fake)
        self.assertFalse(report['complete'])
        self.assertTrue(report['client_cleanup_failed'])
        self.assertEqual(report['mission']['outcome'],'escalated')
        self.assertIsNone(report['journal']['active'])
        self.assertTrue(fake.shutdown)

    def test_startup_counts_against_the_same_mission_deadline(self):
        fake=MissionRos(self.bus);fake.startup_delay=5.
        start=self.clock.wall
        report=self.composed(fake,timeout=20.)
        events=[json.loads(v) for v in (self.f.root/'runtime/mission-service/events.jsonl').read_text().splitlines()]
        self.assertLessEqual(events[0]['data']['deadline_wall_s'],start+20.)
        self.assertIsNotNone(report['mission'])

    def test_expired_startup_releases_lock_without_admission_or_publication(self):
        fake=MissionRos(self.bus);fake.startup_delay=5.
        with self.assertRaisesRegex(ValueError,'startup_deadline'):self.composed(fake,timeout=2.)
        self.assertEqual(self.bus.sent,[])
        self.assertTrue(fake.destroyed and fake.shutdown)
        self.assertEqual((self.f.root/'runtime/mission-service/events.jsonl').read_bytes(),b'')

    def test_unfinished_mission_refuses_before_transport_construction(self):
        self.driver.start()
        self.release_initial_gateway()
        with patch.object(ros_mission,'RosMissionTransport'):
            def never(**kwargs):self.fail('transport must not start')
            with self.assertRaisesRegex(ValueError,'unfinished_mission'):
                ros_mission.run(self.f.root,now=self.bus.now,transport_factory=never)
        self.assertEqual(self.bus.sent,[])

    def test_graph_identity_and_command_ownership_changes_refuse(self):
        self.bus.callbacks={}
        fake=MissionRos(self.bus)
        with patch.dict(sys.modules,fake.modules):
            transport=ros_mission.RosMissionTransport(now=self.bus.now)
            self.addCleanup(transport.close)
            for topic,subscriber in (('/clock',False),('/inspection/state',False),
                                     ('/maintenance/command',True),('/navigation/command',False)):
                with self.subTest(topic=topic,subscriber=subscriber):
                    fake.mutations[(topic,subscriber)]=lambda peers:peers+peers
                    with self.assertRaises(ValueError):transport.fence()
                    fake.mutations.clear()
                    def changed(peers):peers[0].endpoint_gid[0]^=1;return peers
                    fake.mutations[(topic,subscriber)]=changed
                    with self.assertRaises(ValueError):transport.fence()
                    fake.mutations.clear()
            fake.mode=False
            with self.assertRaises(ValueError):transport.fence()
            # Exact cancellation may still inspect its own source generation.
            transport.fence('navigation')

    def test_cli_rejects_invalid_input_before_ros_or_runtime_mutation(self):
        script=Path(__file__).resolve().parents[1]/'scripts/run-scripted-mission.py'
        for args in (['run','--timeout','nan'],['run','--mission-id','bad'],['run']):
            with self.subTest(args=args):
                result=subprocess.run([sys.executable,'-B',str(script),*args],capture_output=True,text=True,
                    env={'PATH':'/usr/bin:/bin','ROS_DOMAIN_ID':'77','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST',
                         'TMPDIR':'/Users/gok/codex/.tmp','PYTHONDONTWRITEBYTECODE':'1'},timeout=10.)
                self.assertNotEqual(result.returncode,0)
                self.assertNotIn('ModuleNotFoundError',result.stderr)

    def test_empty_terminal_marker_cannot_clear_admission(self):
        self.driver.start()
        before=(self.f.root/'runtime/mission-service/events.jsonl').read_bytes()
        with self.assertRaisesRegex(ValueError,'quiescence'):
            self.mission_journal.append(self.driver.mission_id,'terminal',{'outcome':'failed','reason':'test',
                'control':self.driver.mission.control,'pending':None,'quiescence':{}})
        self.assertEqual((self.f.root/'runtime/mission-service/events.jsonl').read_bytes(),before)
        self.mission_journal.close()
        with_journal=MissionJournal(self.mission_lock);self.addCleanup(with_journal.close)
        self.assertEqual(with_journal.active,self.driver.mission_id)

    def test_existing_context_is_not_shutdown_after_init_refusal(self):
        fake=MissionRos(self.bus);fake.active=True
        def refuse(**kwargs):raise RuntimeError('already initialized')
        fake.modules['rclpy'].init=refuse
        with patch.dict(sys.modules,fake.modules):
            with self.assertRaisesRegex(RuntimeError,'already initialized'):
                ros_mission.RosMissionTransport(now=self.bus.now)
        self.assertTrue(fake.active)
        self.assertFalse(fake.shutdown)

    def test_own_context_is_shutdown_after_node_construction_failure(self):
        fake=MissionRos(self.bus)
        def fail(*args,**kwargs):raise RuntimeError('controlled node creation failure')
        fake.modules['rclpy'].create_node=fail
        with patch.dict(sys.modules,fake.modules):
            with self.assertRaisesRegex(RuntimeError,'node creation failure'):
                ros_mission.RosMissionTransport(now=self.bus.now)
        self.assertFalse(fake.active)
        self.assertTrue(fake.shutdown)
