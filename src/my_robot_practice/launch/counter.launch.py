import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_practice'),
        'config', 'params.yaml')

    hello = Node(
        package='my_robot_practice',
        executable='hello_node',
        name='hello_node',
        parameters=[config],
    )

    # TODO: count_listener కి కూడా parameters=[config] తో Node రాయండి
    count_listener = Node(
        package='my_robot_practice',
        executable='count_listener',
        name='count_listener',   # must match the key in params.yaml
        parameters=[config],
    )

    return LaunchDescription([hello, count_listener])