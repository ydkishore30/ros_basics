from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_navigation'),
            'maps',
            'my_map.yaml'
        ]),
        description='Full path to map yaml file to load'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_navigation'),
            'config',
            'amcl_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for AMCL'
    )

    # Map Server - Load the map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # AMCL - Localization node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Lifecycle Manager for map server and AMCL
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        params_file_arg,
        map_server,
        amcl_node,
        lifecycle_manager
    ])
