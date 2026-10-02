"""Slim Nav2 navigation stack for the real robot on the Raspberry Pi 4.

Same nodes and remappings as nav2_bringup/navigation_launch.py, minus the servers
this robot does not use for goals and waypoints (route_server, docking_server,
smoother_server). With the full set the Pi sat at a load
average of 25-37 and Nav2 kept resetting itself on missed heartbeats.

Run localization (nav2_bringup localization_launch.py) first, then:
  ros2 launch /ros2_ws/src/my_robot_navigation/launch/nav2_real_lite.launch.py
For AprilTag docking use the full navigation_launch.py instead.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',   # needed by the RViz Nav2 panel's waypoint mode
    ]

    def nav2_node(package, executable, name, extra_remaps=()):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[params_file, {'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=remappings + list(extra_remaps),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='/ros2_ws/src/my_robot_navigation/config/nav2_params_real.yaml',
            description='Nav2 parameters file'
        ),
        nav2_node('nav2_controller', 'controller_server', 'controller_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_planner', 'planner_server', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{
                'autostart': True,
                'node_names': lifecycle_nodes,
                # default 4 s: one late heartbeat on the loaded Pi reset the whole stack
                'bond_timeout': 15.0,
            }],
        ),
    ])
