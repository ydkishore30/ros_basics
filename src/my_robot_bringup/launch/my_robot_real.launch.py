"""Launch file for robot simulation and controller startup."""

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution
)


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for robot bringup."""
    # ------------------------------
    # Find description package
    # ------------------------------
    description_pkg = FindPackageShare('my_robot_description')

    # Xacro file
    xacro_file = PathJoinSubstitution([
        description_pkg,
        'urdf',
        'my_robot.urdf.xacro',
    ])

    # Convert Xacro -> URDF
    robot_description = Command(['xacro ', xacro_file])

    # ------------------------------
    # Robot State Publisher
    # ------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,  # Set to False for real hardware
            'robot_description': robot_description
        }]
    )

    # ------------------------------
    # Controller YAML
    # ------------------------------
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config',
        'my_robot_controllers.yaml',
    ])

    # ------------------------------
    # Controller Manager (for hardware interfacing)
    # ------------------------------
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False  # Set to False for real hardware
        }, robot_controllers],
        output='screen'
    )

    # ------------------------------
    # Spawners
    # ------------------------------
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

    # ------------------------------
    # Event handlers for ordered controller startup
    # Start joint_state_broadcaster after controller_manager
    joint_state_after_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    # Start diff_drive_controller after joint_state_broadcaster finishes
    diff_drive_after_joint = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawn]
        )
    )

    # ------------------------------
    # RViz
    # ------------------------------
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

    # ------------------------------
    # Teleop (Gamepad Joystick Control)
    # Converts joystick input to Twist messages and then to TwistStamped
    # ------------------------------
    # Joy node (reads gamepad input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # Teleop node (converts joystick to Twist)
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

    # Twist converter node (converts Twist to TwistStamped)
    twist_converter_node = Node(
        package='my_robot_bringup',
        executable='twist_converter.py',
        name='twist_converter',
        output='screen'
    )

    ekf_config = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config',
        'ekf.yaml',
    ])

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {
            'use_sim_time': False
        }]
    )

    # ------------------------------
    # Launch description
    # ------------------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz2',
        ),
        robot_state_publisher,
        controller_manager,
        joint_state_after_controller,
        diff_drive_after_joint,
        ekf_node,   # ✅ ADD THIS

        rviz_node,
        # Joystick control with twist converter
        joy_node,
        teleop_node,
        twist_converter_node,
    ])
