from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    xacro_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        'my_robot.urdf.xacro'
    ])

    robot_description = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'rviz',
            'my_robot.rviz'  # (optional config file)
        ])]
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz2
    ])
