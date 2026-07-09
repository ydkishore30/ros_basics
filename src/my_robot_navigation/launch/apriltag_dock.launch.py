import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_navigation')
    apriltag_params = os.path.join(pkg_share, 'config', 'apriltag.yaml')

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[apriltag_params],
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
    )

    dock_pose_publisher = Node(
        package='my_robot_navigation',
        executable='dock_pose_publisher',
        name='dock_pose_publisher',
        output='screen',
    )

    return LaunchDescription([
        apriltag_node,
        dock_pose_publisher,
    ])
