"""Launch file for robot simulation and controller startup."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart


def generate_launch_description():
    """Generate launch description for robot bringup."""
    description_pkg = FindPackageShare('my_robot_description')

    xacro_file = PathJoinSubstitution([
        description_pkg,
        'urdf',
        'my_robot.urdf.xacro',
    ])

    world_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'worlds',
        'industrial-warehouse',
        'industrial-warehouse.sdf'
    ])

    robot_description = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-v', '4',
            '-r',
            world_file
        ],
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-allow_renaming', 'true'
        ],
    )

    bridge_config = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'config',
        'gazebo_bridge.yaml',
    ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config
        }],
        output='screen'
    )

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config',
        'my_robot_controllers.yaml',
    ])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }, robot_controllers],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_controller_spawn = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            robot_controllers,
        ],
        output='screen'
    )

    # =========================================================================
    # EKF Localization Integration
    # =========================================================================
    ekf_config = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'config',
        'ekf.yaml',
    ])

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True} # Keeps EKF synced with Gazebo time clock
        ]
    )
    # =========================================================================

    # Delay robot spawn by 5 seconds
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[gz_spawn_entity]
    )

    spawn_robot_after_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[delayed_spawn]
        )
    )

    controller_manager_after_spawn = RegisterEventHandler(
        OnProcessStart(
            target_action=gz_spawn_entity,
            on_start=[controller_manager]
        )
    )

    joint_state_after_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    diff_drive_after_joint = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawn]
        )
    )

    use_rviz = LaunchConfiguration('use_rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', PathJoinSubstitution([
            description_pkg,
            'rviz',
            'my_robot.rviz'
        ])]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('my_robot_bringup'),
            'config',
            'teleop_joy.yaml',
        ])]
    )

    twist_converter_node = Node(
        package='my_robot_bringup',
        executable='twist_converter.py',
        name='twist_converter',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz2',
        ),

        robot_state_publisher,
        gazebo,
        bridge,

        spawn_robot_after_gazebo,
        controller_manager_after_spawn,
        joint_state_after_controller,
        diff_drive_after_joint,

        # Added EKF node execution here
        ekf_node,

        rviz_node,
        joy_node,
        teleop_node,
        twist_converter_node,
    ])
