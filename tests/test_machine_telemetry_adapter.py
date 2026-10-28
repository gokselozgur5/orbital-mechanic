"""Execute the shipped entry point and real adapter with controlled ROS transport."""
import json
import runpy
import sys
import types
from unittest.mock import patch

from test_machine_registry import ROOT, RegistryFixture

SCRIPT = (ROOT/'scripts/machine-telemetry.py').read_bytes()


class AdapterHarness:
    def __init__(self, *, times=(10**9,2*10**9), sim_time=True, scenario='latched_controller', before=None, publish_error=False):
        self.times, self.before, self.publish_error = times,before,publish_error
        self.stamp = 0
        self.messages = {'/station/telemetry':[], '/station/bus_voltage':[]}
        self.initialized = self.destroyed = self.shutdown = False
        self.parameters = {'use_sim_time':sim_time,'scenario':scenario}
        owner = self

        class Message:
            def __init__(self, data):
                self.data = data

        class Node:
            def __init__(self, name):
                assert name == 'station_telemetry'
            def get_parameter(self, name):
                return types.SimpleNamespace(value=owner.parameters[name])
            def declare_parameter(self, name, value):
                owner.parameters.setdefault(name,value)
            def create_publisher(self, kind, topic, depth):
                assert topic in owner.messages and depth == 5
                def publish(value):
                    if owner.publish_error:
                        raise RuntimeError('controlled publication failure')
                    owner.messages[topic].append(value.data)
                return types.SimpleNamespace(publish=publish)
            def create_timer(self, period, callback):
                assert period == .5
                owner.callback = callback
            def get_clock(self):
                return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=owner.stamp))
            def destroy_node(self):
                owner.destroyed = True

        def init():
            self.initialized = True
        def shutdown():
            self.shutdown = True
        def spin(node):
            for i, stamp in enumerate(self.times):
                self.stamp = stamp
                if self.before:
                    self.before(i)
                self.callback()
        ros = types.ModuleType('rclpy')
        ros.init, ros.create_node, ros.spin = init, Node, spin
        ros.ok, ros.shutdown = lambda:True, shutdown
        node_module = types.ModuleType('rclpy.node')
        node_module.Node = Node
        msg = types.ModuleType('std_msgs.msg')
        msg.String = msg.Float32 = Message
        self.modules = {'rclpy':ros,'rclpy.node':node_module,'std_msgs':types.ModuleType('std_msgs'),'std_msgs.msg':msg}

    def run(self, root, source=SCRIPT):
        (root/'scripts').mkdir(exist_ok=True)
        script = root/'scripts/machine-telemetry.py'
        script.write_bytes(source)
        with patch.dict(sys.modules,self.modules), patch.object(sys,'path',list(sys.path)):
            runpy.run_path(str(script),run_name='__main__')


class MachineTelemetryAdapterTests(RegistryFixture):
    def test_actual_entrypoint_publishes_bound_measurements_and_voltage(self):
        harness = AdapterHarness()
        harness.run(self.root)
        values = [json.loads(text) for text in harness.messages['/station/telemetry']]
        self.assertEqual(len(values),2)
        self.assertEqual([v['stamp_ns'] for v in values],[10**9,2*10**9])
        self.assertEqual([v['voltage_v'] for v in values],harness.messages['/station/bus_voltage'])
        self.assertEqual(values[0]['registry'],values[1]['registry'])
        self.assertTrue(harness.destroyed and harness.shutdown)

    def test_bad_registry_fails_before_ros_initialization(self):
        self.config['inspection_poses']['pump']['control_xy_m'] = [0,0]
        self.save_config()
        harness = AdapterHarness()
        with self.assertRaises(ValueError):
            harness.run(self.root)
        self.assertFalse(harness.initialized)
        self.assertEqual(harness.messages['/station/telemetry'],[])

    def test_wall_clock_mode_is_refused_before_publishing(self):
        harness = AdapterHarness(sim_time=False)
        with self.assertRaisesRegex(ValueError,'use_sim_time'):
            harness.run(self.root)
        self.assertTrue(harness.destroyed and harness.shutdown)
        self.assertEqual(harness.messages['/station/telemetry'],[])

    def test_changed_source_stops_before_next_publication(self):
        def change(index):
            if index == 1:
                self.world_path.write_bytes(self.world_path.read_bytes()+b'\n')
        harness = AdapterHarness(before=change)
        with self.assertRaisesRegex(ValueError,'source changed'):
            harness.run(self.root, source=SCRIPT)
        self.assertEqual(len(harness.messages['/station/telemetry']),1)
        self.assertEqual(len(harness.messages['/station/bus_voltage']),1)
        self.assertTrue(harness.destroyed and harness.shutdown)

    def test_switching_clock_mode_cannot_label_wall_time_as_simulation_time(self):
        harness = AdapterHarness()
        def change(index):
            if index == 1:
                harness.parameters['use_sim_time'] = False
        harness.before = change
        with self.assertRaisesRegex(ValueError,'use_sim_time'):
            harness.run(self.root)
        self.assertEqual(len(harness.messages['/station/telemetry']),1)
        self.assertTrue(harness.destroyed and harness.shutdown)

    def test_reset_has_new_identity_and_paused_clock_is_not_republished(self):
        harness = AdapterHarness(times=(0,10**9,10**9,500000000))
        harness.run(self.root)
        values = [json.loads(text) for text in harness.messages['/station/telemetry']]
        self.assertEqual(len(values),2)
        self.assertNotEqual(values[0]['run_id'],values[1]['run_id'])
        self.assertEqual([v['sequence'] for v in values],[1,1])

    def test_publication_failure_is_not_hidden_by_the_adapter(self):
        harness = AdapterHarness(publish_error=True)
        with self.assertRaisesRegex(RuntimeError,'publication failure'):
            harness.run(self.root)
        self.assertTrue(harness.destroyed and harness.shutdown)

    def test_invalid_operator_scenario_does_not_create_measurements(self):
        harness = AdapterHarness(scenario='unknown')
        with self.assertRaises(ValueError):
            harness.run(self.root)
        self.assertTrue(harness.destroyed and harness.shutdown)
        self.assertEqual(harness.messages['/station/telemetry'],[])

    def test_operator_interrupt_cleans_up(self):
        def interrupt(index):
            if index == 1:
                raise KeyboardInterrupt()
        harness = AdapterHarness(before=interrupt)
        harness.run(self.root)
        self.assertTrue(harness.destroyed and harness.shutdown)
        self.assertEqual(len(harness.messages['/station/telemetry']),1)
