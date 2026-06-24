from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument('model_path', default_value='yolo26n.pt')
    confidence_arg = DeclareLaunchArgument('confidence_threshold', default_value='0.5')
    safety_distance_arg = DeclareLaunchArgument('safety_distance', default_value='1.0')

    detector_node = Node(
        package='my_robot_yolo',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
        }],
    )

    safety_monitor_node = Node(
        package='my_robot_yolo',
        executable='safety_monitor_node',
        name='yolo_safety_monitor',
        output='screen',
        parameters=[{
            'safety_distance': LaunchConfiguration('safety_distance'),
        }],
    )

    return LaunchDescription([
        model_path_arg,
        confidence_arg,
        safety_distance_arg,
        detector_node,
        safety_monitor_node,
    ])
