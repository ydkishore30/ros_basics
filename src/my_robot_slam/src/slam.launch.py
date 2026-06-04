from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    return LaunchDescription([

        # Use simulation time (Gazebo)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),

        # SLAM parameters file
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'config',
                'mapper_params_online_async.yaml'
            ]),
            description='Full path to SLAM params file'
        ),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('/scan', '/scan'),
                ('/map', '/map')
            ]
        ),

    ])