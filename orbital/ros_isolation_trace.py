"""Read-only ROS observer; interruption never cancels or retries maintenance."""
import time
import uuid

from orbital.isolation_trace import IsolationTrace, TOPICS
from orbital.inspection_protocol import finite
from orbital.machine_registry import MachineRegistry
from orbital.navigation_graph import BASE_TOPICS
from orbital.proximity import Unavailable
from orbital.ros_proximity import authority
from orbital.ros_trajectory import runtime_snapshot, message_text


def run(root, writer, identity, *, duration=60., warmup=10., now=time.monotonic_ns,
        trace_factory=IsolationTrace, node_prefix='orbital_isolation_observer'):
    node = None
    initialized = complete = False
    code, details = 1, {}
    callback_error = None
    trace = None
    try:
        finite(duration, .1, 120.); finite(warmup, .1, 30.)
        registry = MachineRegistry.load(root)
        before = runtime_snapshot(root)
        trace = trace_factory(registry, identity, writer=writer)
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node(node_prefix+'_'+uuid.uuid4().hex[:12],
                                parameter_overrides=[Parameter('use_sim_time', value=True)])
        expected = {name:[('station_telemetry', 'std_msgs/msg/String')] for name in TOPICS[1:]}
        expected['/clock'] = BASE_TOPICS['/clock']
        pinned = None
        def peers():
            if node.get_parameter('use_sim_time').value is not True:
                raise ValueError('trace_clock_mode_changed')
            current = authority(node, expected)
            if pinned is not None and current != pinned:
                raise ValueError('trace_publisher_changed')
            return current

        def receive(topic, message):
            nonlocal callback_error
            if trace.source is None or callback_error is not None:
                return
            try:
                peers()
                trace.receive(topic, now(), node.get_clock().now().nanoseconds, message_text(message))
            except Exception as error:
                callback_error = error

        for topic in TOPICS:
            node.create_subscription(String, topic, lambda msg, topic=topic:receive(topic, msg), 32)
        deadline = now()+round(warmup*1e9)
        while rclpy.ok():
            current, sim = now(), node.get_clock().now().nanoseconds
            if pinned is None:
                try:
                    candidate = peers()
                except Unavailable:
                    candidate = None  # Startup discovery only; loss after pinning is fatal.
                if candidate is not None and sim > 0:
                    pinned = candidate
                    trace.begin(current, sim, {'runtime': before, 'publishers': pinned})
            else:
                peers()
            if callback_error is not None:
                raise callback_error
            if trace.source is not None:
                if trace.ready_at is None:
                    if trace.ready(current, sim):
                        writer.sync()
                        writer.marker('ready.json', {'target': trace.target,
                            'station_run_id': trace.baseline['run_id'],
                            'control_revision': trace.baseline['control']['revision'],
                            'scope': 'Observer ready; not maintenance permission'})
                elif trace.assess(current, sim) is not None:
                    # The observation time precedes potentially slow service/file reads.
                    # Those reads do not renew source sample freshness.
                    after = runtime_snapshot(root)
                    registry.verify_current()
                    details['observation'] = trace.end(current, sim, {'runtime': after, 'publishers': peers()})
                    complete, code = True, 0
                    break
            if trace.ready_at is None and current >= deadline:
                raise ValueError('trace_warmup_timeout')
            if trace.ready_at is not None and current-trace.ready_at >= round(duration*1e9):
                raise ValueError('trace_action_timeout')
            rclpy.spin_once(node, timeout_sec=.01)
        if not complete:
            raise ValueError('trace_transport_ended')
    except KeyboardInterrupt:
        details['error'], code = 'Recorder interrupted; maintenance outcome remains unchanged', 130
    except Exception as error:
        details['error'] = type(error).__name__+': '+str(error)
    finally:
        for cleanup in (lambda: node.destroy_node() if node is not None else None,
                        lambda: rclpy.try_shutdown() if initialized else None):
            try:
                cleanup()
            except Exception as error:
                complete, code = False, 1
                details.setdefault('cleanup_errors', []).append(type(error).__name__)
    return writer.finish(complete, details), code
