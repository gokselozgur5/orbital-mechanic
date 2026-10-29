"""Own only the Nav2 route stack; localization and guarded motion are external.

The parent service owns the full process tree. Any child exit ends this launch;
there is no respawn, recovery velocity producer or implicit lifecycle activation.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    autostart = ParameterValue(LaunchConfiguration('autostart'), value_type=bool)
    servers = []
    for name, package, config in (
            ('controller_server', 'nav2_controller', 'nav2-controller.yaml'),
            ('planner_server', 'nav2_planner', 'nav2-planner.yaml'),
            ('bt_navigator', 'nav2_bt_navigator', 'nav2-navigator.yaml')):
        overrides = {'use_sim_time': clock}
        if name == 'bt_navigator':
            overrides['default_nav_to_pose_bt_xml'] = str(root/'config/navigate-guarded.xml')
        servers.append(Node(package=package, executable=name, name=name, output='screen',
                            parameters=[str(root/'config'/config), overrides],
                            remappings=[('cmd_vel', '/navigation/raw_cmd_vel')],
                            respawn=False))
    manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                   name='lifecycle_manager_navigation', output='screen', respawn=False,
                   parameters=[{'use_sim_time': clock, 'autostart': autostart,
                                'node_names': ['controller_server', 'planner_server', 'bt_navigator'],
                                'bond_timeout': 2.0, 'attempt_respawn_reconnection': False}])
    owned = [*servers, manager]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason='Owned navigation process exited'))]))
                for node in owned]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        *handlers, *owned])
