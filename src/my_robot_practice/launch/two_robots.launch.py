import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_practice'),
        'config', 'params.yaml')

    robot1_battery = Node(
        package='my_robot_practice',
        executable='battery_node',
        namespace='robot1',
        parameters=[config],
    )
    robot2_battery = Node(
            package='my_robot_practice',
            executable='battery_node',
            namespace='robot2',
            parameters=[config],
        )
    robot1_monitor = Node(
        package='my_robot_practice',
        executable='battery_monitor',
        namespace='robot1',
        parameters=[config],
    )
    robot2_monitor = Node(
        package='my_robot_practice',
        executable='battery_monitor',
        namespace='robot2',
        parameters=[config],
    )

    return LaunchDescription([robot1_battery, robot2_battery, robot1_monitor, robot2_monitor])