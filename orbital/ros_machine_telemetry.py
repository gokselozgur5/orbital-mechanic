"""Publish the causal station and serialize maintenance on that same instance."""
import json
import os
import time

from orbital.machine_registry import MachineRegistry


def run(root, *, stop_requested=lambda:False, now=time.monotonic):
    registry = MachineRegistry.load(root)  # Refuse bad bindings before joining ROS.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String, Float32
    from orbital.ros_maintenance import StationMaintenance

    node = maintenance = None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node('station_telemetry')
        def require_sim_time():
            if node.get_parameter('use_sim_time').value is not True:
                raise ValueError('Machine telemetry requires use_sim_time=true')
        require_sim_time()
        # Private initial conditions are read under the station owner lock. ROS
        # parameters/events and process arguments never carry a fault selector.
        maintenance = StationMaintenance(root, node, None, now=now,
                                         expected_trial_id=os.environ.get('ORBITAL_TRIAL_ID'))
        model = maintenance.model
        telemetry = node.create_publisher(String, '/station/telemetry', 5)
        voltage = node.create_publisher(Float32, '/station/bus_voltage', 5)

        def update():
            require_sim_time()
            value = model.sample(node.get_clock().now().nanoseconds)
            if value is not None:
                telemetry.publish(String(data=json.dumps(value, allow_nan=False)))
                voltage.publish(Float32(data=value['voltage_v']))

        node.create_timer(.5, update)
        while rclpy.ok() and not stop_requested():
            rclpy.spin_once(node, timeout_sec=.01)
            maintenance.tick()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if maintenance is not None:
                maintenance.close()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
