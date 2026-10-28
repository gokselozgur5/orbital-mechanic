"""Run the actual CLI transport against production supervisor callbacks.

ROS transport is controlled; this is not a native DDS or physical-motion proof.
"""
from contextlib import redirect_stderr, redirect_stdout
import fcntl
import importlib.util
from io import StringIO
import json
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from orbital.navigation_client import NavigationClient, read_target
import test_supervisor_arm as arm_fixture

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('orbital_navigate_cli_test', ROOT/'scripts/orbital-navigate.py')
cli = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cli)


class NavigationCliTransportTests(unittest.TestCase):
    def setUp(self):
        self.fixture = arm_fixture.SupervisorArmTests(methodName='runTest')
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.supervisor = self.fixture.supervisor
        self.callback = None
        self.nodes = []
        self.publications = []
        self.publisher_topics = []
        self.extra_peer = False
        self.discovery_delay = 0
        self.polls = 0
        self.shutdowns = 0
        owner = self

        class Node:
            def __init__(self, name):
                self.name, self.destroyed = name, False

            def get_name(self):
                return self.name

            def create_subscription(self, kind, topic, callback, depth):
                if topic != '/navigation/state':
                    raise AssertionError('Unexpected CLI subscription: '+topic)
                owner.callback = callback
                return NS()

            def create_publisher(self, kind, topic, depth):
                owner.publisher_topics.append(topic)
                if topic != '/navigation/command':
                    raise AssertionError('CLI tried to bypass supervisor: '+topic)
                def publish(message):
                    owner.publications.append(json.loads(message.data))
                    owner.supervisor.on_command(message)
                return NS(publish=publish, get_subscription_count=lambda: 1)

            def get_publishers_info_by_topic(self, topic):
                name = 'orbital_navigation_supervisor' if topic == '/navigation/state' else self.name
                if owner.polls <= owner.discovery_delay:
                    name = '_NODE_NAME_UNKNOWN_'
                values = [NS(node_name=name, node_namespace='/', topic_type='std_msgs/msg/String')]
                if owner.extra_peer and topic == '/navigation/command':
                    values.append(NS(node_name='another_operator', node_namespace='/', topic_type='std_msgs/msg/String'))
                return values

            def get_subscriptions_info_by_topic(self, topic):
                return [NS(node_name='orbital_navigation_supervisor', node_namespace='/', topic_type='std_msgs/msg/String')]

            def destroy_node(self):
                self.destroyed = True

        def create(name):
            node = Node(name)
            self.nodes.append(node)
            return node

        def spin(node, timeout_sec):
            self.polls += 1
            self.fixture.health(dt=timeout_sec)
            self.fixture.guard(latched=True)
            self.supervisor.tick()

        def shutdown():
            self.shutdowns += 1

        self.supervisor.nav_state_pub.publish = lambda message: self.callback(message) if self.callback else None
        ros = ModuleType('rclpy')
        ros.init = lambda **kwargs: None
        ros.create_node, ros.spin_once, ros.try_shutdown = create, spin, shutdown
        messages = ModuleType('std_msgs.msg')
        messages.String = arm_fixture.Message
        self.modules = patch.dict(sys.modules, {'rclpy':ros, 'std_msgs':ModuleType('std_msgs'), 'std_msgs.msg':messages})
        self.modules.start()
        self.addCleanup(self.modules.stop)

    def invoke(self, arguments):
        args = cli.parse_args(arguments)
        return cli.ros_command(args, NavigationClient(lambda: self.fixture.now))

    def test_request_arm_cancel_use_production_exact_target_protocol(self):
        receipt, code = self.invoke(['request', 'pump'])
        self.assertEqual(code, 0)
        token = json.dumps(read_target(json.dumps(receipt)))
        arm, code = self.invoke(['arm', '--target', token])
        self.assertEqual(code, 0)
        self.assertEqual(arm['state']['action_state'], 'waiting_for_arm')
        self.assertIsNotNone(self.supervisor.arm_future)
        self.assertEqual(self.supervisor.session.client.calls, [])
        cancel, code = self.invoke(['cancel', '--target', token])
        self.assertEqual(code, 0)
        self.assertEqual(cancel['state']['action_state'], 'terminal')
        self.assertEqual(self.supervisor.session.outcome, 'not_dispatched')
        self.assertEqual([p['sequence'] for p in self.publications], [1,2,3])
        self.assertEqual([p['op'] for p in self.publications], ['request','arm','cancel'])
        self.assertEqual(set(self.publisher_topics), {'/navigation/command'})
        self.assertTrue(all(node.destroyed for node in self.nodes))
        self.assertEqual(self.shutdowns, 3)

    def test_status_creates_no_command_publisher(self):
        result, code = self.invoke(['status'])
        self.assertEqual((code, result['delivery']), (0, 'observed'))
        self.assertEqual(self.publisher_topics, [])
        self.assertEqual(self.publications, [])
        self.assertEqual(self.supervisor.commands.sequence, 0)

    def test_second_command_publisher_is_refused_before_server_sequence_changes(self):
        self.extra_peer = True
        result, code = self.invoke(['request','pump'])
        self.assertEqual((code, result['delivery']), (2, 'not_sent'))
        self.assertEqual(self.publications, [])
        self.assertEqual(self.supervisor.commands.sequence, 0)

    def test_discovery_placeholders_resolve_before_one_publication(self):
        self.discovery_delay = 3
        receipt, code = self.invoke(['request','pump'])
        self.assertEqual(code, 0)
        self.assertGreater(self.polls, self.discovery_delay)
        self.assertEqual(len(self.publications), 1)
        self.assertEqual(receipt['target']['waypoint'], 'pump')

    def test_old_token_cannot_arm_after_production_cancel_and_release(self):
        receipt, code = self.invoke(['request','pump'])
        self.assertEqual(code, 0)
        token = json.dumps(receipt['target'])
        self.assertEqual(self.invoke(['cancel','--target',token])[1], 0)
        for _ in range(6):
            self.fixture.health(dt=.05)
            self.fixture.guard(latched=True)
            self.supervisor.tick()
        self.assertIsNone(self.supervisor.session.current)
        result, code = self.invoke(['arm','--target',token])
        self.assertEqual((code, result['delivery']), (2, 'not_sent'))
        self.assertEqual([p['op'] for p in self.publications], ['request','cancel'])
        self.assertEqual(self.supervisor.arm_client.calls, [])

    def test_cancel_does_not_depend_on_reloading_current_map_config(self):
        receipt, code = self.invoke(['request','pump'])
        self.assertEqual(code, 0)
        (ROOT/'runtime').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT/'runtime') as directory:
            root = Path(directory)
            (root/'config').mkdir()
            (root/'config/known-map.json').write_text('{}')
            output = StringIO()
            with patch.object(cli,'ROOT',root), patch.dict(os.environ, {
                'ROS_DOMAIN_ID':'84','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST',
            }), redirect_stdout(output):
                code = cli.main(['cancel','--target',json.dumps(receipt['target'])])
            self.assertEqual(code, 0)
            self.assertEqual(json.loads(output.getvalue())['delivery'], 'acknowledged')
            self.assertEqual(self.supervisor.session.outcome, 'not_dispatched')


class NavigationCliBoundaryTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'runtime').mkdir(exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(dir=ROOT/'runtime')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.token = {'run_id':'test-run','epoch':0,'sequence':1,'waypoint':'pump'}

    def test_help_and_invalid_arguments_do_not_need_ros(self):
        with redirect_stdout(StringIO()), self.assertRaises(SystemExit) as raised:
            cli.parse_args(['--help'])
        self.assertEqual(raised.exception.code, 0)
        variants = [['arm'], ['request','unknown'], ['--timeout','nan','status'],
                    ['arm','--target','{"run_id":"x","run_id":"y"}']]
        for args in variants:
            with self.subTest(args=args), redirect_stderr(StringIO()), self.assertRaises(SystemExit) as raised:
                cli.parse_args(args)
            self.assertEqual(raised.exception.code, 2)

    def test_target_file_is_bounded_and_requires_explicit_identity(self):
        path = self.root/'target.json'
        path.write_text(json.dumps(self.token))
        self.assertEqual(cli.parse_args(['arm','--target-file',str(path)]).target_value, self.token)
        path.write_text('x'*20000)
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            cli.parse_args(['arm','--target-file',str(path)])

    def test_wrong_domain_and_nonlocal_discovery_refused_before_ros(self):
        for domain, discovery in (('77','LOCALHOST'), ('42','SUBNET'), ('','')):
            output = StringIO()
            with self.subTest(domain=domain, discovery=discovery), patch.dict(os.environ, {
                'ROS_DOMAIN_ID':domain, 'ROS_AUTOMATIC_DISCOVERY_RANGE':discovery,
            }), patch.object(cli, 'ros_command', side_effect=AssertionError('ROS must not start')), redirect_stdout(output):
                code = cli.main(['status'])
            self.assertEqual(code, 2)
            self.assertEqual(json.loads(output.getvalue())['delivery'], 'not_sent')

    def test_existing_local_command_lock_refused_before_ros(self):
        (self.root/'runtime').mkdir()
        with (self.root/'runtime/navigation-client-84.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            output = StringIO()
            with patch.object(cli,'ROOT',self.root), patch.dict(os.environ, {
                'ROS_DOMAIN_ID':'84','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST',
            }), patch.object(cli,'ros_command',side_effect=AssertionError('ROS must not start')), redirect_stdout(output):
                code = cli.main(['cancel','--target',json.dumps(self.token)])
            self.assertEqual(code, 2)
            result = json.loads(output.getvalue())
            self.assertEqual(result['delivery'], 'not_sent')
            self.assertIn('owns this checkout', result['error'])

    def test_request_checks_geometry_before_ros_or_local_lock(self):
        (self.root/'config').mkdir()
        (self.root/'config/known-map.json').write_text('{}')
        output = StringIO()
        with patch.object(cli,'ROOT',self.root), patch.dict(os.environ, {
            'ROS_DOMAIN_ID':'84','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST',
        }), patch.object(cli,'ros_command',side_effect=AssertionError('ROS must not start')), redirect_stdout(output):
            code = cli.main(['request','pump'])
        self.assertEqual(code, 2)
        self.assertEqual(json.loads(output.getvalue())['delivery'], 'not_sent')
        self.assertFalse((self.root/'runtime').exists())


if __name__ == '__main__':
    unittest.main()
