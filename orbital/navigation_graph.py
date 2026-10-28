"""Expected trusted ROS graph before startup and before lifecycle activation.

Node names and endpoint counts detect accidental conflicting owners. They are
not authentication, nor do they attribute each TF sample to a publisher.
"""
from collections import Counter

TF = 'tf2_msgs/msg/TFMessage'
STAMPED = 'geometry_msgs/msg/TwistStamped'
STRING = 'std_msgs/msg/String'
BASE_TOPICS = {
    '/tf': [('amcl',TF),('robot_state_publisher',TF),('ros_gz_bridge',TF)],
    '/tf_static': [('robot_state_publisher',TF)],
    '/clock': [('ros_gz_bridge','rosgraph_msgs/msg/Clock')],
    '/odom': [('ros_gz_bridge','nav_msgs/msg/Odometry')],
    '/scan': [('ros_gz_bridge','sensor_msgs/msg/LaserScan')],
    '/imu': [('ros_gz_bridge','sensor_msgs/msg/Imu')],
    '/map': [('map_server','nav_msgs/msg/OccupancyGrid')],
    '/amcl_pose': [('amcl','geometry_msgs/msg/PoseWithCovarianceStamped')],
    '/motion/state': [('motion_guard',STRING)],
    '/motion/guarded_cmd_vel': [('motion_guard','geometry_msgs/msg/Twist')],
}
NAV_TOPICS = {
    '/navigation/raw_cmd_vel': [('controller_server',STAMPED)],
    '/motion/request': [('orbital_navigation_supervisor',STAMPED)],
    '/navigation/state': [('orbital_navigation_supervisor',STRING)],
    '/navigation/audit': [('orbital_navigation_supervisor',STRING)],
    '/navigation/interlock': [('orbital_navigation_supervisor',STRING)],
    '/navigation/health': [('orbital_localization_monitor',STRING)],
}
BYPASS_TOPICS = ('/cmd_vel','/demo/private/cmd_vel')
NAV_ACTIONS = {
    '/navigate_to_pose': ('bt_navigator','nav2_msgs/action/NavigateToPose'),
    '/compute_path_to_pose': ('planner_server','nav2_msgs/action/ComputePathToPose'),
    '/follow_path': ('controller_server','nav2_msgs/action/FollowPath'),
}
NAV_NODES = ('controller_server','planner_server','bt_navigator','lifecycle_manager_navigation',
             'orbital_navigation_supervisor','orbital_localization_monitor')
PARENTS = {'base_footprint':'odom', 'base_link':'base_footprint', 'lidar_link':'base_link',
           'camera_link':'base_link', 'camera_optical_frame':'camera_link',
           'left_wheel':'base_link','right_wheel':'base_link'}


def graph_errors(snapshot, phase):
    if phase not in ('before','unconfigured','configured'):
        raise ValueError('Unknown graph phase')
    errors = []
    expected = dict(BASE_TOPICS)
    expected.update({topic: owners if phase!='before' and (phase=='configured' or topic!='/navigation/raw_cmd_vel') else []
                     for topic,owners in NAV_TOPICS.items()})
    expected.update({topic:[] for topic in BYPASS_TOPICS})
    for topic, owners in expected.items():
        actual = Counter((p['node'],p['namespace'],p['type']) for p in snapshot['publishers'].get(topic,[]))
        wanted = Counter((name,'/',kind) for name,kind in owners)
        if actual != wanted:
            errors.append('Publisher ownership mismatch: '+topic)
    nodes = Counter((n['name'],n['namespace']) for n in snapshot['nodes'])
    for name in ('ros_gz_bridge','robot_state_publisher','motion_guard','map_server','amcl'):
        if nodes[(name,'/')] != 1:
            errors.append('Missing or duplicate node: '+name)
    for name in NAV_NODES:
        count = sum(v for (n,_),v in nodes.items() if n==name)
        if count != (0 if phase=='before' else 1) or (phase!='before' and nodes[(name,'/')] != 1):
            errors.append('Unexpected navigation node ownership: '+name)
    action_types = {kind for _,kind in NAV_ACTIONS.values()}
    actions = [a for a in snapshot['actions'] if a['name'] in NAV_ACTIONS or a['type'] in action_types]
    actual = Counter((a['name'],a['node'],a['namespace'],a['type']) for a in actions)
    wanted = Counter((path,name,'/',kind) for path,(name,kind) in NAV_ACTIONS.items()) if phase=='configured' else Counter()
    if actual != wanted:
        errors.append('Navigation action ownership mismatch')
    for service in ('/motion/stop','/motion/reset_stop'):
        owners = [(s['node'],s['namespace'],s['type']) for s in snapshot['services'] if s['name']==service]
        if owners != [('motion_guard','/','std_srvs/srv/Trigger')]:
            errors.append('Guard service ownership mismatch: '+service)
    for child,parent in PARENTS.items():
        if set(snapshot['parents'].get(child,[])) != {parent}:
            errors.append('Missing or conflicting TF parent: '+child)
    # AMCL can be present but unseeded before navigation startup. If it emits
    # map->odom, the parent must already be consistent; health gates seeding.
    if 'odom' in snapshot['parents'] and set(snapshot['parents']['odom']) != {'map'}:
        errors.append('Conflicting TF parent: odom')
    return errors
