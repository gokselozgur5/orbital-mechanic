#!/usr/bin/env python3
"""Verify owned native Nav2 lifecycle/teardown on empty localhost domain 84.

Static TF and zero odometry are fixtures. Only a goal at the fixture's current
pose is sent. There is no drive, guard, localization or world in this trial.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.mapping import build_grid, FREE, OCCUPIED


def process_tree(pid):
    """Read only descendants of our launch, retaining process start identities."""
    # Jetson's kernel does not expose /proc/PID/task/TID/children. Build the
    # parent relation from stat instead; never select processes by executable.
    snapshot = {}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            stat = (proc/'stat').read_text().rsplit(')', 1)[1].split()
            snapshot[int(proc.name)] = (int(stat[1]), stat[19], stat[0])
        except (FileNotFoundError, PermissionError):
            pass
    found = {}
    queue = [pid]
    while queue:
        current = queue.pop()
        if current not in snapshot or current in found:
            continue
        try:
            proc = Path('/proc')/str(current)
            _, start, state = snapshot[current]
            found[current] = {'start': start, 'state': state,
                              'command': (proc/'cmdline').read_bytes().split(b'\0')[0].decode()}
            queue.extend(p for p, info in snapshot.items() if info[0]==current)
        except FileNotFoundError:
            pass
    return found


def still_alive(pid, identity):
    try:
        stat = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
        return stat[19] == identity['start'] and stat[0] != 'Z'
    except FileNotFoundError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kill-controller', action='store_true')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '84' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Requires empty localhost-only domain84')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import TransformStamped, TwistStamped
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import ManageLifecycleNodes
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init(args=[])
    node = rclpy.create_node('orbital_navigation_launch_probe')
    child, stream, action = None, None, None
    owned = {}
    names = ('controller_server', 'planner_server', 'bt_navigator')
    velocities = []
    report = {'passed': False, 'controller_killed': args.kill_controller,
              'scope': 'native Nav2 lifecycle with fixture stationary pose; no world or drive'}
    source_names = ['scripts/navigation.launch.py', 'config/nav2-controller.yaml', 'config/nav2-planner.yaml',
                    'config/nav2-navigator.yaml', 'config/navigate-guarded.xml',
                    'scripts/check-navigation-launch.py', 'orbital/mapping.py',
                    'config/known-map.json', 'worlds/maintenance.sdf']
    report['sources'] = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in source_names}

    def wait(predicate, seconds=20, require_running=True):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if require_running and child is not None and child.poll() is not None:
                raise RuntimeError('Owned launch exited early')
            if predicate():
                return
        raise RuntimeError('Timed out waiting for launch evidence')

    def call(kind, name, request):
        client = node.create_client(kind, name)
        try:
            wait(client.service_is_ready)
            future = client.call_async(request)
            wait(future.done)
            return future.result()
        finally:
            node.destroy_client(client)

    def state(name):
        return call(GetState, '/'+name+'/get_state', GetState.Request()).current_state.id

    try:
        end = time.monotonic()+2
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.05)
        assert not [n for n in node.get_node_names() if n != node.get_name()], 'Domain is not empty'
        config = json.loads((ROOT/'config/known-map.json').read_text())
        grid = build_grid(ROOT/config['world'], config)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        map_pub = node.create_publisher(OccupancyGrid, '/map', retained)
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.resolution, message.info.width, message.info.height = grid.resolution, grid.width, grid.height
        message.info.origin.position.x, message.info.origin.position.y = grid.origin
        message.info.origin.orientation.w = 1.
        message.data = [0 if c==FREE else 100 if c==OCCUPIED else -1 for c in grid.cells]
        map_pub.publish(message)
        broadcaster = StaticTransformBroadcaster(node)
        transforms = []
        for parent, name in (('map','odom'), ('odom','base_footprint')):
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, name
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.transform.rotation.w = 1.
            transforms.append(transform)
        broadcaster.sendTransform(transforms)
        odom_pub = node.create_publisher(Odometry, '/odom', 10)
        scan_pub = node.create_publisher(LaserScan, '/scan', 10)

        def sensors():
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
            odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.
            odom_pub.publish(odom)
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = 'base_footprint', stamp
            scan.angle_min, scan.angle_max, scan.angle_increment = -3.14, 3.14, 6.28/359
            scan.range_min, scan.range_max = .1, 12.
            scan.ranges = [12.]*360
            scan_pub.publish(scan)

        timer = node.create_timer(.05, sensors)
        sub = node.create_subscription(TwistStamped, '/navigation/raw_cmd_vel',
                                       lambda m: velocities.append((m.twist.linear.x,m.twist.angular.z)), 10)
        (ROOT/'runtime').mkdir(exist_ok=True)
        stream = (ROOT/'runtime/navigation-launch.log').open('w')
        child = subprocess.Popen(['/opt/ros/jazzy/bin/ros2', 'launch', str(ROOT/'scripts/navigation.launch.py'),
                                  'use_sim_time:=false'], stdout=stream, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        assert all(state(name)==1 for name in names), 'Launch activated without explicit startup'
        report['initial_states'] = 'all_unconfigured'
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        assert call(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', request).success
        assert all(state(name)==3 for name in names), 'Native lifecycle activation failed'
        report['active_states'] = list(names)
        raw = node.get_publishers_info_by_topic('/navigation/raw_cmd_vel')
        assert len(raw)==1 and raw[0].node_name=='controller_server' and raw[0].topic_type=='geometry_msgs/msg/TwistStamped'
        for topic in ('/cmd_vel', '/motion/request', '/motion/guarded_cmd_vel'):
            assert not node.get_publishers_info_by_topic(topic), 'Command bypass: '+topic
        graph = node.get_node_names()
        assert not any(n in graph for n in ('behavior_server','velocity_smoother','waypoint_follower','map_server','amcl'))
        action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        wait(action.server_is_ready)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.orientation.w = 1.
        future = action.send_goal_async(goal)
        wait(future.done)
        handle = future.result()
        assert handle.accepted
        future = handle.get_result_async()
        wait(future.done)
        assert future.result().status == GoalStatus.STATUS_SUCCEEDED, 'Stationary goal failed'
        assert velocities and all(abs(v)<1e-9 and abs(w)<1e-9 for v,w in velocities), 'Stationary goal commanded movement'
        report['stationary_goal_zero_commands'] = len(velocities)
        owned.update(process_tree(child.pid))
        report['owned_processes'] = owned
        controller = [pid for pid, info in owned.items() if info['command']=='/opt/ros/jazzy/lib/nav2_controller/controller_server']
        assert len(controller)==1, 'Cannot identify our owned controller process'
        if args.kill_controller:
            os.kill(controller[0], signal.SIGKILL)
        else:
            os.kill(child.pid, signal.SIGINT)
        wait(lambda: child.poll() is not None, seconds=12, require_running=False)
        wait(lambda: not any(still_alive(pid,info) for pid,info in owned.items()), seconds=5, require_running=False)
        report.update(passed=True, launch_exit=child.returncode, owned_processes_reaped=len(owned))
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if child is not None:
            owned.update(process_tree(child.pid))
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
            for pid, info in owned.items():
                if still_alive(pid,info):
                    os.kill(pid, signal.SIGKILL)
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
