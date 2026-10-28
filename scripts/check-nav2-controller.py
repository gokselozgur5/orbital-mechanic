#!/usr/bin/env python3
"""Configure a real inactive Nav2 controller on empty domain 84; never activate it."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '84':
        raise RuntimeError('Requires isolated ROS_DOMAIN_ID=84')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Requires localhost-only discovery')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unstamped-control', action='store_true', help='Deliberately break the contract; must fail.')
    args = parser.parse_args()
    import rclpy
    from lifecycle_msgs.srv import ChangeState, GetState
    from rcl_interfaces.srv import GetParameters
    rclpy.init(args=[])
    node = rclpy.create_node('orbital_controller_configuration_probe')
    child, stream = None, None
    root = Path(__file__).resolve().parents[1]
    params = root / 'config/nav2-controller.yaml'
    report = {'passed': False, 'activated': False,
              'params_sha256': hashlib.sha256(params.read_bytes()).hexdigest(),
              'negative_control': args.unstamped_control}

    def wait(predicate, seconds=10):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if child is not None and child.poll() is not None:
                raise RuntimeError('Controller exited; inspect native log')
            if predicate():
                return
        raise RuntimeError('Timed out waiting for controller evidence')

    def call(kind, name, request):
        client = node.create_client(kind, name)
        try:
            wait(client.service_is_ready)
            future = client.call_async(request)
            wait(future.done)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        end = time.monotonic() + 2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        others = [name for name in node.get_node_names() if name != node.get_name()]
        if others:
            raise RuntimeError('Domain is not empty: ' + repr(others))
        (root / 'runtime').mkdir(exist_ok=True)
        stream = (root / 'runtime/nav2-controller.log').open('w')
        command = ['/opt/ros/jazzy/lib/nav2_controller/controller_server', '--ros-args',
                   '--params-file', str(params), '-r', 'cmd_vel:=/navigation/raw_cmd_vel']
        if args.unstamped_control:
            command += ['-p', 'enable_stamped_cmd_vel:=false']
        child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        before = call(GetState, '/controller_server/get_state', GetState.Request())
        assert before.current_state.id == 1, 'Controller must start unconfigured'
        request = ChangeState.Request()
        request.transition.id = 1  # CONFIGURE, never ACTIVATE
        assert call(ChangeState, '/controller_server/change_state', request).success, 'Configure failed'
        state = call(GetState, '/controller_server/get_state', GetState.Request())
        assert state.current_state.id == 2, 'Controller must remain inactive'
        wait(lambda: bool(node.get_publishers_info_by_topic('/navigation/raw_cmd_vel')))
        publishers = node.get_publishers_info_by_topic('/navigation/raw_cmd_vel')
        report['publisher_types'] = [p.topic_type for p in publishers]
        report['publisher_nodes'] = [p.node_name for p in publishers]
        assert len(publishers) == 1 and publishers[0].node_name == 'controller_server', 'Ambiguous output authority'
        assert publishers[0].topic_type == 'geometry_msgs/msg/TwistStamped', 'Unstamped controller output rejected'
        for topic in ('/cmd_vel', '/motion/request', '/motion/guarded_cmd_vel'):
            assert not node.get_publishers_info_by_topic(topic), 'Controller bypass detected: ' + topic
        names = ['enable_stamped_cmd_vel', 'FollowPath.desired_linear_vel', 'FollowPath.rotate_to_heading_angular_vel']
        response = call(GetParameters, '/controller_server/get_parameters', GetParameters.Request(names=names))
        assert response.values[0].bool_value
        assert 0 < response.values[1].double_value <= .2
        assert 0 < response.values[2].double_value <= .5
        report.update(passed=True, lifecycle_state=state.current_state.label,
                      linear_speed=response.values[1].double_value,
                      rotation_speed=response.values[2].double_value)
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=3)
        if stream is not None:
            stream.close()
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
