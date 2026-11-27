import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_practice'),
        'config', 'temperature.yaml')

    temperature_sensor = Node(
        package='my_robot_practice',
        executable='temperature_sensor',
        name='temperature_sensor',
        parameters=[config]
    )

    temperature_monitor = Node(
        package='my_robot_practice',
        executable='temperature_monitor',
        name='temperature_monitor',
        parameters=[config]
    )

    return LaunchDescription([
        temperature_sensor,
        temperature_monitor
    ])