import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from orbital.inspection_artifacts import read_capture
from orbital.inspection_protocol import canonical, result, state
from orbital import inspection_owner, ros_inspection
from orbital import ros_proximity
from orbital.inspection_artifacts import CaptureStore
from orbital.machine_registry import MachineRegistry
from proximity_fixture import Harness
from inspection_transport_fixture import transport
from test_inspection_owner import request
import test_map_binding as map_fixture

ROOT = Path(__file__).resolve().parents[1]


class EndpointTests(unittest.TestCase):
    def setUp(self):
        map_fixture.MapBindingTests.setUp(self)
        for name in ('config/robot.urdf', 'config/bridge.yaml'):
            shutil.copyfile(ROOT/name, self.root/name)

    def run_endpoint(self, mode='success'):
        origin = time.monotonic()
        h = transport(self.root, origin)
        original_node = h.modules['rclpy'].create_node
        original_spin = h.modules['rclpy'].spin_once
        outputs = {topic: [] for topic in ros_inspection.TOPICS}
        created, callbacks = set(), {}
        submitted = cancelled = False
        real_popen = subprocess.Popen
        workers = []
        process_type = ros_inspection.CaptureProcess

        def process(*args, **kwargs):
            worker = process_type(*args, **kwargs)
            if mode == 'orphan':
                os.close(worker.lifeline); worker.lifeline = None
            return worker

        def create(*args, **kwargs):
            node = original_node(*args, **kwargs)
            subscribe, peers = node.create_subscription, node.get_publishers_info_by_topic
            def subscription(kind, topic, callback, qos):
                if topic == '/inspection/command':
                    callbacks[topic] = callback; return NS(topic=topic)
                return subscribe(kind, topic, callback, qos)
            def publish(kind, topic, qos):
                self.assertIn(topic, outputs)
                created.add(topic)
                return NS(publish=lambda message: outputs[topic].append(json.loads(message.data)))
            def publishers(topic):
                if topic in ros_inspection.TOPICS:
                    return [NS(node_name=ros_inspection.OWNER, node_namespace='/', topic_type='std_msgs/msg/String',
                               endpoint_gid=[9]*24)] if topic in created else []
                return peers(topic)
            node.create_subscription, node.create_publisher, node.get_publishers_info_by_topic = subscription, publish, publishers
            return node

        def spin(node, timeout_sec):
            nonlocal submitted, cancelled
            original_spin(node, timeout_sec)
            statuses = outputs['/inspection/state']
            if statuses and '/inspection/command' in callbacks:
                latest = statuses[-1]
                if latest['ready'] and not submitted:
                    callbacks['/inspection/command'](NS(data=canonical(request(run_id=latest['run_id'], arguments={'machine_id': 'pump', 'timeout_s': 8.}))))
                    submitted = True
                if mode == 'cancel' and latest['active'] is not None and latest['active']['phase'] == 'running' and not cancelled:
                    callbacks['/inspection/command'](NS(data=canonical(request('cancel', run_id=latest['run_id']))))
                    cancelled = True
            if time.monotonic()-origin > 13.:
                raise AssertionError('endpoint fixture exceeded its owned deadline')

        def launch(args, **kwargs):
            self.assertEqual(Path(args[1]).name, 'inspection-worker.py')
            kwargs['env']['ORBITAL_TEST_ORIGIN'] = str(origin)
            child = real_popen([args[0], str(ROOT/'tests/inspection_transport_fixture.py'), *args[2:]], **kwargs)
            workers.append(child)
            return child

        real_read = inspection_owner.read_capture
        def read(*args):
            value = real_read(*args)
            if mode == 'late': time.sleep(.25)
            if mode == 'wrong_identity': value[0]['inspection_action']['action_id'] = 'f'*32
            return value
        h.modules['rclpy'].create_node, h.modules['rclpy'].spin_once = create, spin
        try:
            with patch.dict(sys.modules, h.modules), patch('orbital.inspection_worker.subprocess.Popen', side_effect=launch), patch.object(inspection_owner, 'read_capture', side_effect=read), patch.object(ros_inspection, 'CaptureProcess', side_effect=process):
                ros_inspection.serve(self.root, stop_requested=lambda: bool(outputs['/inspection/result']))
        finally:
            for child in workers:
                if child.poll() is None:
                    child.kill(); child.wait(timeout=2.)
        self.assertTrue(h.destroyed and h.unregistered and h.shutdown)
        self.assertEqual(len(workers), 1)
        self.assertTrue(all(child.returncode is not None for child in workers))
        for status in outputs['/inspection/state']: state(canonical(status))
        self.assertEqual(len(outputs['/inspection/result']), 1)
        terminal = result(outputs['/inspection/result'][0])
        diagnostic = self.root/'runtime/test-worker-error.txt'
        if mode != 'orphan':
            self.assertNotEqual(terminal['error_code'], 'capture_unavailable',
                                diagnostic.read_text() if diagnostic.exists() else terminal)
        self.assertTrue(outputs['/inspection/reply'][0]['accepted'])
        self.assertIsNone(outputs['/inspection/reply'][0]['action']['result'])
        return terminal

    def test_server_process_actual_capture_and_manifest_reach_one_terminal_success(self):
        terminal = self.run_endpoint()
        self.assertEqual(terminal['status'], 'succeeded', terminal)
        reference = terminal['observation']
        record, image = read_capture(self.root, reference['observation_id'], reference['manifest_sha256'])
        self.assertEqual(record['inspection_action']['action_id'], terminal['action_id'])
        self.assertEqual(record['inspection_action']['run_id'], terminal['run_id'])
        self.assertGreater(record['observed_monotonic_s'], record['inspection_action']['admitted_monotonic_s'])
        self.assertEqual(image, b'P6\n640 480\n255\n'+bytes((7,31,193))*(640*480))
        self.assertNotIn('fault', json.dumps(record))

    def test_cancel_reaps_real_worker_without_a_success_reference(self):
        terminal = self.run_endpoint('cancel')
        self.assertEqual(terminal['status'], 'cancelled', terminal)
        self.assertIsNone(terminal['observation'])

    def test_post_capture_io_delay_does_not_renew_evidence(self):
        terminal = self.run_endpoint('late')
        self.assertEqual(terminal['status'], 'failed', terminal)
        self.assertEqual(terminal['error_code'], 'observation_unavailable')
        self.assertIsNone(terminal['observation'])

    def test_other_action_artifact_cannot_complete_this_request(self):
        terminal = self.run_endpoint('wrong_identity')
        self.assertEqual(terminal['status'], 'failed', terminal)
        self.assertIsNone(terminal['observation'])

    def test_real_worker_lifeline_loss_refuses_completion(self):
        terminal = self.run_endpoint('orphan')
        self.assertEqual(terminal['status'], 'failed', terminal)
        self.assertIsNone(terminal['observation'])

    def test_cooperative_cancel_during_storage_removes_acceptance_marker(self):
        h = Harness(self)
        node = h.modules['rclpy'].create_node()
        sources = {**ros_proximity.proximity_sources(), **ros_inspection.CAMERA_TOPICS,
                   '/station/telemetry': [('station_telemetry', 'std_msgs/msg/String')]}
        identity = hashlib.sha256(json.dumps(ros_proximity.authority(node, sources), separators=(',', ':')).encode()).hexdigest()
        context = {'run_id': '1'*32, 'mission_id': '2'*32, 'action_id': '3'*32,
                   'request_sha256': '4'*64, 'registry_identity': MachineRegistry.load(self.root).identity,
                   'authority_identity': identity, 'admitted_monotonic_s': h.wall, 'admitted_sim_stamp_ns': h.sim}
        cancelled = False
        actual = CaptureStore.write
        def write(store, name, chunks):
            nonlocal cancelled
            actual(store, name, chunks)
            if name == 'image.ppm': cancelled = True
        with patch.object(CaptureStore, 'write', write), self.assertRaisesRegex(ValueError, 'inspection_cancelled'):
            h.run(invoke=lambda: ros_proximity.capture_machine_observation(self.root, 'pump', timeout=3.,
                now=lambda: h.wall, action_context=context, cancelled=lambda: cancelled))
        self.assertEqual(list((self.root/'runtime').glob('inspection/*/observation.json')), [])
        self.assertTrue(h.destroyed and h.unregistered and h.shutdown)

    def test_cancellation_during_preparation_does_not_initialize_ros(self):
        actual = ros_proximity.MapSnapshot.prepare
        cancelled = False
        def prepare(registry):
            nonlocal cancelled
            value = actual(registry); cancelled = True; return value
        with patch.object(ros_proximity.MapSnapshot, 'prepare', side_effect=prepare), self.assertRaisesRegex(ValueError, 'inspection_cancelled'):
            ros_proximity.capture_machine_observation(self.root, 'pump', cancelled=lambda: cancelled)


if __name__ == '__main__': unittest.main()
