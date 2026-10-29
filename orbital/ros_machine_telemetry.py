"""Publish source-bound public machine measurements; no maintenance endpoints."""
import json

from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.machines import Fault


def run(root):
    registry = MachineRegistry.load(root)  # Refuse bad bindings before joining ROS.
    import rclpy
    from std_msgs.msg import String, Float32

    node = None
    rclpy.init()
    try:
        node = rclpy.create_node('station_telemetry')
        def require_sim_time():
            if node.get_parameter('use_sim_time').value is not True:
                raise ValueError('Machine telemetry requires use_sim_time=true')
        require_sim_time()
        node.declare_parameter('scenario', 'latched_controller')
        model = MachineTelemetry(registry, Fault(node.get_parameter('scenario').value))
        telemetry = node.create_publisher(String, '/station/telemetry', 5)
        voltage = node.create_publisher(Float32, '/station/bus_voltage', 5)

        def update():
            require_sim_time()
            value = model.sample(node.get_clock().now().nanoseconds)
            if value is not None:
                telemetry.publish(String(data=json.dumps(value, allow_nan=False)))
                voltage.publish(Float32(data=value['voltage_v']))

        node.create_timer(.5, update)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
