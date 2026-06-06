"""Launch file for robot simulation and controller startup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    description_pkg = FindPackageShare('my_robot_description')
    bringup_pkg = FindPackageShare('my_robot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # ---------------- URDF ----------------
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

    # ---------------- GAZEBO ----------------
    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-v', '4',
            '-r',
            world_file,
        ],
        output='screen'
    )

    # ---------------- ROBOT STATE PUBLISHER ----------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # ---------------- SPAWN ROBOT ----------------
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

    spawn_robot = TimerAction(
        period=5.0,
        actions=[gz_spawn_entity]
    )

    # ---------------- ROS2 CONTROL ----------------
    robot_controllers = PathJoinSubstitution([
        bringup_pkg,
        'config',
        'my_robot_controllers.yaml',
    ])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
            robot_controllers
        ]
    )

    # ---------------- CONTROLLER SPAWNERS (SAFE DELAYED) ----------------
    joint_state_broadcaster = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    diff_drive_controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller'],
                output='screen'
            )
        ]
    )

    # ---------------- BRIDGE ----------------
    bridge_config = PathJoinSubstitution([
        description_pkg,
        'config',
        'gazebo_bridge.yaml',
    ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config
        }]
    )

    # ---------------- EKF ----------------
    ekf_config = PathJoinSubstitution([
        description_pkg,
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
            {'use_sim_time': use_sim_time}
        ]
    )

    # ---------------- RVIZ ----------------
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
        ])],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---------------- JOYSTICK ----------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     
        'device_id': 0,
        'device_name': '/dev/input/js0',
        'deadzone': 0.05}]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                bringup_pkg,
                'config',
                'teleop_joy.yaml',
            ]),
            {'use_sim_time': use_sim_time}
        ]
    )

    twist_converter_node = Node(
        package='my_robot_bringup',
        executable='twist_converter.py',
        name='twist_converter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---------------- LAUNCH ARGUMENTS ----------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time'
        ),

        # CORE SYSTEM
        gazebo,
        robot_state_publisher,
        # controller_manager,
        bridge,
        spawn_robot,

        # CONTROLLERS
        joint_state_broadcaster,
        diff_drive_controller,

        # HIGHER LEVEL
        ekf_node,
        rviz_node,

        # INPUT STACK
        joy_node,
        teleop_node,
        twist_converter_node,
    ])