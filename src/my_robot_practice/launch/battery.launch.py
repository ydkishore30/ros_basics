from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    
    battery = Node(
        package='my_robot_practice',
        executable='battery_node',
        name='battery_node',
    )

    # TODO: battery_monitor కి ఇలాగే ఇంకో Node రాయండి
    battery_monitor = Node(
        package='my_robot_practice',
        executable='battery_monitor',
        name='battery_monitor_node',
    )

    return LaunchDescription([
        battery,
        battery_monitor
    ])