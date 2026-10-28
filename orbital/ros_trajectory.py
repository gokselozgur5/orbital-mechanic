"""Native read-only ROS/Gazebo adapter for the independent evaluator.

No truth bridge, command publisher, service request or automatic stop is created.
The operator's trial guardian, not this observer, owns movement containment.
"""
import hashlib
import importlib
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

from orbital.service_owner import dependency_identity
from orbital.trajectory import Inbox, Trajectory, TRUTH_TOPIC, transform_at, truth_from_message


def message_text(message):
    if not isinstance(message.data, str) or len(message.data) > 16384:
        raise ValueError('Observer JSON message exceeds 16,384 characters')
    return message.data


def runtime_snapshot(root):
    result = subprocess.run([
        sys.executable, str(root/'scripts/navigation-manifest.py'), '--verify',
        str(root/'runtime/navigation/source-manifest.json')],
        text=True, capture_output=True, timeout=20, check=True)
    manifest = json.loads(result.stdout)
    if manifest.get('passed') is not True:
        raise ValueError('Navigation source manifest did not verify')
    return {'manifest_identity': manifest['manifest_identity'], 'service_invocations': {
        unit: dependency_identity(unit) for unit in (
            'orbital-demo.service', 'orbital-localization.service', 'orbital-navigation.service')}}


def binding_sources():
    result = {}
    for name in ('gz.transport13', 'gz.transport13._transport', 'gz.msgs10.pose_v_pb2'):
        module = importlib.import_module(name)
        path = Path(module.__file__).resolve()
        result[name] = {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    return result


def graph_ready(node, gz):
    for topic in ('/navigation/state', '/navigation/audit'):
        peers = node.get_publishers_info_by_topic(topic)
        if not peers:
            return False
        if len(peers) != 1:
            raise ValueError('Ambiguous evaluator publisher: '+topic)
        peer = peers[0]
        if peer.node_name in ('', '_NODE_NAME_UNKNOWN_') or peer.node_namespace == '_NODE_NAMESPACE_UNKNOWN_':
            return False
        if (peer.node_name, peer.node_namespace, peer.topic_type) != (
                'orbital_navigation_supervisor', '/', 'std_msgs/msg/String'):
            raise ValueError('Unexpected evaluator publisher: '+topic)
    publishers, _ = gz.topic_info(TRUTH_TOPIC)
    if not publishers:
        return False
    if len(publishers) != 1 or publishers[0].msg_type_name != 'gz.msgs.Pose_V':
        raise ValueError('Ambiguous or wrong-type simulator truth publisher')
    return True


def collect(trace, inbox, channel, *, duration, warmup, now=time.monotonic_ns):
    """Same loop in production and transport-controlled regression tests."""
    warmup_deadline = now()+round(warmup*1e9)
    next_graph_check = 0
    graph = False
    cutoff = None
    while True:
        channel.poll(.01)
        items, error = inbox.drain()
        # Preserve the queued prefix even when a later callback failed.
        for kind, at, value in items:
            getattr(trace, kind)(at, value)
        if error:
            raise ValueError(error)
        current = now()
        trace.match(channel.lookup, current)
        if current >= next_graph_check:
            graph = channel.authority_clear()
            next_graph_check = current+100000000
            if trace.started_ns is not None and not graph:
                raise ValueError('Evaluator publisher disappeared')
        if trace.started_ns is None:
            if current >= warmup_deadline:
                raise ValueError('Recorder warmup timed out; requires healthy idle/latched state and exact-time TF')
            if graph:
                trace.start(current)
        elif cutoff is None and current-trace.started_ns >= round(duration*1e9):
            cutoff = inbox.close_kind('truth')
            channel.stop_truth()
        elif cutoff is not None:
            # Bounded post-roll permits time-specific TF and reliable audit/state
            # callbacks to catch up. Truth ends at cutoff; command counters may
            # cover the additional post-roll interval, recorded separately.
            if current-cutoff >= 200000000 and not trace.pending:
                if not channel.authority_clear():
                    raise ValueError('Evaluator publisher missing at final fence')
                return trace.finish(current, ended_ns=cutoff)
            if current-cutoff >= 600000000:
                raise ValueError('Recorder post-roll timed out')


def run(root, writer, *, duration, warmup):
    trace, inbox = Trajectory(writer), Inbox()
    details = {'scope': 'Sampled independent evaluator evidence, not mission acceptance or a motion watchdog',
               'truth_topic': TRUTH_TOPIC, 'duration_requested_s': duration, 'warmup_limit_s': warmup,
               'graph_scope': 'Sampled endpoint/type uniqueness; no authentication or per-message publisher attribution'}
    node = listener = gz = None
    initialized = subscribed = False
    complete = False
    code = 1
    try:
        details['before'] = runtime_snapshot(root)
        details['bindings_before'] = binding_sources()
        import rclpy
        from rclpy.clock import ClockType
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.time import Time
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformException, TransformListener
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V

        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node('orbital_trajectory_'+uuid.uuid4().hex[:12],
                                parameter_overrides=[Parameter('use_sim_time', value=True)])
        buffer = Buffer(node=node)
        listener = TransformListener(buffer, node)
        node.create_subscription(String, '/navigation/state',
                                 lambda msg: inbox.receive('state', lambda: message_text(msg)), 100)
        node.create_subscription(String, '/navigation/audit',
                                 lambda msg: inbox.receive('audit', lambda: message_text(msg)),
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        gz = GzNode()
        if not gz.subscribe(Pose_V, TRUTH_TOPIC, lambda msg: inbox.receive('truth', lambda: truth_from_message(msg))):
            raise ValueError('Gazebo truth subscription was refused')
        subscribed = True

        class Channel:
            def poll(self, seconds):
                rclpy.spin_once(node, timeout_sec=seconds)

            def lookup(self, stamp):
                try:
                    value = buffer.lookup_transform('map', 'base_footprint',
                                                    Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME))
                except TransformException as error:
                    raise LookupError(str(error)) from error
                return transform_at(value, stamp)

            def authority_clear(self):
                return graph_ready(node, gz)

            def stop_truth(self):
                nonlocal subscribed
                if not gz.unsubscribe(TRUTH_TOPIC):
                    raise ValueError('Gazebo truth unsubscribe failed')
                subscribed = False

        details['observation'] = collect(trace, inbox, Channel(), duration=duration, warmup=warmup)
        inbox.close()
        details['after'] = runtime_snapshot(root)
        details['bindings_after'] = binding_sources()
        if details['before'] != details['after'] or details['bindings_before'] != details['bindings_after']:
            raise ValueError('Service, deployment or observer binding identity changed during capture')
        complete, code = True, 0
    except KeyboardInterrupt:
        details['error'], code = 'Interrupted; recorder does not cancel a navigation goal', 130
    except Exception as error:
        details['error'] = f'{type(error).__name__}: {error}'
    finally:
        inbox.close()
        for cleanup in (lambda: gz.unsubscribe(TRUTH_TOPIC) if subscribed else None,
                        lambda: listener.unregister() if listener is not None else None,
                        lambda: node.destroy_node() if node is not None else None,
                        lambda: rclpy.try_shutdown() if initialized else None):
            try:
                cleanup()
            except Exception as error:
                complete, code = False, 1
                details.setdefault('cleanup_errors', []).append(type(error).__name__+': '+str(error))
    result = writer.finish(complete, details)
    return result, code
