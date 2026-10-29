"""Bounded read-only public station observation; no motion or maintenance clients."""
import hashlib
import json
import math
import time
import uuid

from orbital.machine_registry import MachineRegistry
from orbital.navigation_graph import BASE_TOPICS
from orbital.proximity import Unavailable
from orbital.ros_proximity import authority
from orbital.telemetry_observation import TelemetryEvidence

SOURCES = {'/clock':BASE_TOPICS['/clock'],
           '/station/telemetry':[('station_telemetry','std_msgs/msg/String')]}


def run(root, *, timeout=5., now=time.monotonic):
    if type(timeout) not in (int,float) or not math.isfinite(timeout) or not .1<=timeout<=10.:
        raise ValueError('timeout_must_be_between_0.1_and_10_seconds')
    deadline = now()+timeout
    registry = MachineRegistry.load(root)
    evidence = TelemetryEvidence(registry)

    def remaining():
        left = deadline-now()
        if not math.isfinite(left) or left<=0:
            raise Unavailable('station_observation_deadline_exceeded')
        return left

    remaining()
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    node = None
    rclpy.init(args=[])
    try:
        node = rclpy.create_node('orbital_station_observer_'+uuid.uuid4().hex[:12],
                                parameter_overrides=[Parameter('use_sim_time',value=True)])

        def clock():
            return node.get_clock().now().nanoseconds

        def mode():
            if node.get_parameter('use_sim_time').value is not True:
                raise Unavailable('simulation_clock_mode_changed')

        initial = None
        while initial is None:
            rclpy.spin_once(node,timeout_sec=min(.02,remaining()))
            mode()
            evidence.tick(now(),clock())
            if clock()>0:
                try:
                    initial = authority(node,SOURCES)
                except Unavailable:
                    pass
        # Do not accept a packet before the clock and publisher inventory exist.
        node.create_subscription(String,'/station/telemetry',
            lambda msg:evidence.observe(msg.data,now(),clock()),5)

        def fence():
            mode()
            if authority(node,SOURCES)!=initial:
                raise Unavailable('publisher_generation_changed')
            evidence.tick(now(),clock())
            remaining()

        while True:
            rclpy.spin_once(node,timeout_sec=min(.02,remaining()))
            fence()
            try:
                evidence.measure(now(),clock())
            except Unavailable:
                continue  # Fatal input/clock failures were already raised by the fence.
            identity = {'observation_id':uuid.uuid4().hex,
                'authority_identity':hashlib.sha256(json.dumps(initial,separators=(',',':')).encode()).hexdigest()}
            registry.verify_current()
            fence()
            result = evidence.measure(now(),clock())
            result.update(identity)
            remaining()
            return result
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
