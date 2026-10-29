#!/usr/bin/env python3
"""Exercise the native planner against the versioned map on empty domain 84.

No controller, simulator or velocity publisher is launched. The synthetic TF
and wall clock are private-domain fixtures, never localization evidence.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.mapping import build_grid, FREE, OCCUPIED
from orbital.path_clearance import require_clear_path


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '84' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Requires empty localhost-only ROS domain 84')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from lifecycle_msgs.srv import ChangeState, GetState
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.srv import GetCostmap
    from tf2_ros import StaticTransformBroadcaster

    config = json.loads((ROOT / 'config/known-map.json').read_text())
    grid = build_grid(ROOT / config['world'], config)
    params = ROOT / 'config/nav2-planner.yaml'
    rclpy.init(args=[])
    node = rclpy.create_node('orbital_planner_probe')
    child, stream, action = None, None, None
    report = {'passed': False, 'scope': 'native planning only; no executed motion', 'routes': [],
              'sources': {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                          for name in ('config/nav2-planner.yaml', 'config/known-map.json',
                                       config['world'], 'orbital/mapping.py',
                                       'orbital/path_clearance.py', 'scripts/check-nav2-planner.py')}}

    def wait(predicate, seconds=15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if child is not None and child.poll() is not None:
                raise RuntimeError('Planner exited; inspect native log')
            if predicate():
                return
        raise RuntimeError('Timed out waiting for planner evidence')

    def call(kind, name, request):
        client = node.create_client(kind, name)
        try:
            wait(client.service_is_ready)
            future = client.call_async(request)
            wait(future.done)
            return future.result()
        finally:
            node.destroy_client(client)

    def pose(values):
        result = PoseStamped()
        result.header.frame_id = 'map'
        result.header.stamp = node.get_clock().now().to_msg()
        result.pose.position.x, result.pose.position.y = map(float, values[:2])
        result.pose.orientation.z = math.sin(values[2]/2)
        result.pose.orientation.w = math.cos(values[2]/2)
        return result

    def plan(name, start, goal, error_code=0):
        request = ComputePathToPose.Goal()
        request.start, request.goal = pose(start), pose(goal)
        request.use_start, request.planner_id = True, 'GridBased'
        future = action.send_goal_async(request)
        wait(future.done)
        handle = future.result()
        assert handle.accepted, 'Planning request rejected before evaluation'
        future = handle.get_result_async()
        wait(future.done)
        response = future.result()
        result = response.result
        summary = {'name': name, 'status': response.status, 'error_code': result.error_code,
                   'start': start, 'goal': goal, 'poses': len(result.path.poses)}
        report['routes'].append(summary)
        assert result.error_code == error_code, repr(summary) + result.error_msg
        if error_code:
            assert response.status == GoalStatus.STATUS_ABORTED and not result.path.poses
            return
        assert response.status == GoalStatus.STATUS_SUCCEEDED
        assert result.path.header.frame_id == 'map' and result.path.poses
        assert all(p.header.frame_id == 'map' for p in result.path.poses)
        points = [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]
        assert math.dist(points[0], start[:2]) <= grid.resolution*1.5, 'Wrong start'
        assert math.dist(points[-1], goal[:2]) <= .05, 'Wrong goal'
        # Include the connectors, even if Navfn omits the exact start cell center.
        require_clear_path(grid, [start[:2], *points, goal[:2]], config['robot_radius_m'])
        summary['length_m'] = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        summary['swept_footprint_clear'] = True
        summary['path_xy'] = points

    try:
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert not [n for n in node.get_node_names() if n != node.get_name()], 'Domain is not empty'
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        publisher = node.create_publisher(OccupancyGrid, '/map', qos)
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.header.stamp = node.get_clock().now().to_msg()
        message.info.resolution, message.info.width, message.info.height = grid.resolution, grid.width, grid.height
        message.info.origin.position.x, message.info.origin.position.y = grid.origin
        message.info.origin.orientation.w = 1.
        message.data = [0 if c == FREE else 100 if c == OCCUPIED else -1 for c in grid.cells]
        publisher.publish(message)
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.rotation.w = 1.
        broadcaster.sendTransform(transform)
        (ROOT / 'runtime').mkdir(exist_ok=True)
        stream = (ROOT / 'runtime/nav2-planner.log').open('w')
        child = subprocess.Popen(['/opt/ros/jazzy/lib/nav2_planner/planner_server', '--ros-args',
                                  '--params-file', str(params), '-p', 'use_sim_time:=false'],
                                 stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        assert call(GetState, '/planner_server/get_state', GetState.Request()).current_state.id == 1
        for transition, expected in ((1, 2), (3, 3)):
            request = ChangeState.Request()
            request.transition.id = transition
            assert call(ChangeState, '/planner_server/change_state', request).success
            assert call(GetState, '/planner_server/get_state', GetState.Request()).current_state.id == expected
        costmap = call(GetCostmap, '/global_costmap/get_costmap', GetCostmap.Request()).map
        assert costmap.metadata.size_x == grid.width and costmap.metadata.size_y == grid.height, 'Map not received'
        action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        wait(action.server_is_ready)
        previous = [0., 0., 0.]
        for name, entry in config['inspection_poses'].items():
            goal = entry['goal_xy_yaw']
            plan('origin_to_'+name, [0., 0., 0.], goal)
            if previous != [0., 0., 0.]:
                plan('tour_to_'+name, previous, goal)
            previous = goal
        plan('around_cooler', [0., 0., 0.], [4., 0., 0.])
        plan('occupied_goal', [0., 0., 0.], [3., 0., 0.], ComputePathToPose.Result.GOAL_OCCUPIED)
        plan('outside_goal', [0., 0., 0.], [20., 0., 0.], ComputePathToPose.Result.GOAL_OUTSIDE_MAP)
        try:
            require_clear_path(grid, [(0., 0.), (4., 0.)], config['robot_radius_m'])
        except ValueError:
            report['straight_through_machine_rejected'] = True
        else:
            raise AssertionError('Independent checker accepted an impossible direct path')
        for topic in ('/cmd_vel', '/navigation/raw_cmd_vel', '/motion/request', '/motion/guarded_cmd_vel'):
            assert not node.get_publishers_info_by_topic(topic), 'Unexpected velocity publisher: '+topic
        report['passed'] = True
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
        if action is not None:
            action.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
