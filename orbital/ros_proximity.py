"""Bounded read-only ROS query; map services are reads, no control RPC is made."""
from collections import Counter
import hashlib
import json
import math
import time
import uuid

from orbital.machine_registry import MachineRegistry
from orbital.navigation_graph import BASE_TOPICS, NAV_TOPICS
from orbital.proximity import ProximityEvidence, Unavailable

MAP_MAX_AGE_S = .5


def authority(node):
    expected = {name:BASE_TOPICS[name] for name in ('/tf','/tf_static','/clock','/map')}
    expected['/navigation/health'] = NAV_TOPICS['/navigation/health']
    snapshot = []
    for topic, owners in expected.items():
        peers = node.get_publishers_info_by_topic(topic)
        actual = Counter((p.node_name,p.node_namespace,p.topic_type) for p in peers)
        if actual != Counter((name,'/',kind) for name,kind in owners):
            raise Unavailable('publisher_authority_unavailable')
        for peer in peers:
            gid = list(peer.endpoint_gid)
            if not 1 <= len(gid) <= 128 or not any(gid) or any(type(v) is not int or not 0<=v<=255 for v in gid):
                raise Unavailable('publisher_identity_unavailable')
            snapshot.append((topic,peer.node_name,peer.node_namespace,peer.topic_type,bytes(gid).hex()))
    return sorted(snapshot)


def run(root, machine_id, *, timeout=10., now=time.monotonic):
    if not math.isfinite(timeout) or not .1 <= timeout <= 10.:
        raise ValueError('timeout_must_be_between_0.1_and_10_seconds')
    deadline = now()+timeout
    registry = MachineRegistry.load(root)
    evidence = ProximityEvidence(registry,machine_id)
    import rclpy
    from rclpy.clock import ClockType
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener
    from orbital.ros_map_binding import collect

    node = listener = None
    rclpy.init(args=[])
    try:
        node = rclpy.create_node('orbital_proximity_'+uuid.uuid4().hex[:12],
                                parameter_overrides=[Parameter('use_sim_time',value=True)])
        buffer = Buffer(node=node)
        listener = TransformListener(buffer,node)
        node.create_subscription(String,'/navigation/health',
                                 lambda msg:evidence.health(msg.data,now()),10)

        def remaining():
            left = deadline-now()
            if left <= 0:
                raise Unavailable('proximity_deadline_exceeded')
            return left

        initial = None
        while initial is None:
            rclpy.spin_once(node,timeout_sec=min(.02,remaining()))
            try:
                initial = authority(node)
            except Unavailable:
                pass
        mapping = collect(node,root,timeout=remaining())
        remaining()
        if mapping.get('passed') is not True or mapping['binding']['config_sha256'] != registry.config_sha256:
            raise Unavailable('map_registry_binding_mismatch')
        bound_at = now()

        def fence():
            if node.get_parameter('use_sim_time').value is not True:
                raise Unavailable('simulation_clock_mode_changed')
            if authority(node) != initial:
                raise Unavailable('publisher_generation_changed')
            if evidence.error:
                raise Unavailable(evidence.error)
            observed_at = now()
            if not 0 <= observed_at-bound_at < MAP_MAX_AGE_S:
                raise Unavailable('map_binding_observation_expired')
            return observed_at

        while True:
            rclpy.spin_once(node,timeout_sec=min(.02,remaining()))
            fence()
            try:
                requested = evidence.requested(node.get_clock().now().nanoseconds)
                transform = buffer.lookup_transform('map','base_footprint',
                    Time(nanoseconds=requested,clock_type=ClockType.ROS_TIME))
            except (Unavailable,TransformException):
                continue
            registry.verify_current()
            # A successful lookup can finish after discovery, clock mode or map
            # evidence changes. Recheck before accepting the measured candidate.
            observed_at = fence()
            try:
                result = evidence.measure(requested,node.get_clock().now().nanoseconds,observed_at,transform)
            except Unavailable:
                continue
            remaining()
            result.update(observation_id=uuid.uuid4().hex, map_binding=mapping['binding'],
                          map_observed_age_s=observed_at-bound_at,
                          authority_identity=hashlib.sha256(json.dumps(initial,separators=(',',':')).encode()).hexdigest())
            return result
    finally:
        if listener is not None:
            listener.unregister()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
