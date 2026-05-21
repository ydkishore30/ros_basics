from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='controller_node',
            name='my_robot_controller',
            output='screen',
            parameters=[
                {'cmd_vel_topic': '/cmd_vel'},
                {'output_topic': '/wheel_vel_cmd'},
            ],
        ),
    ])
