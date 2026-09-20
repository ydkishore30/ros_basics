"""Launch file for robot simulation and controller startup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    description_pkg = FindPackageShare('my_robot_description')
    bringup_pkg = FindPackageShare('my_robot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    # use_rviz = LaunchConfiguration('use_rviz')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_slam = LaunchConfiguration('use_slam')
    use_dock = LaunchConfiguration('use_dock')

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


    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' use_ros2_control:=',
            use_ros2_control
        ]),
        value_type=str
    )

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
        condition=IfCondition(use_ros2_control),
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
            robot_controllers
        ]
    )

    # ---------------- CONTROLLER SPAWNERS (CHAINED) ----------------
    # Use a timer to wait for the robot to spawn in Gazebo before activating controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    joint_state_broadcaster = TimerAction(
        period=7.0,
        condition=IfCondition(use_ros2_control),
        actions=[joint_state_broadcaster_spawner]
    )

    diff_drive_controller = RegisterEventHandler(
        condition=IfCondition(use_ros2_control),
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller'],
                    output='screen',
                )
            ]
        )
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
            'config_file': bridge_config,
            'use_sim_time': use_sim_time  # <--- CRITICAL FIX: Forces bridge node to match simulation clock
        }]
    )

    # ---------------- BNO IMU SIM (bridges /imu/gz → /imu with BNO covariances) ----------------
    bno_imu_sim = Node(
        package='my_robot_bringup',
        executable='bno_imu_sim.py',
        name='bno_imu_sim',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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

    # ---------------- SLAM ----------------
    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_slam'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        condition=IfCondition(use_slam),
    )

    # ---------------- NAV2 ----------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': PathJoinSubstitution([
                description_pkg,
                'maps',
                'industrial-warehouse.yaml'
            ]),
            'params_file': PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'config',
                'nav2_params.yaml'
            ]),
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_nav2),
    )

    # ---------------- DOCK (AprilTag + docking_server) ----------------
    dock_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'launch',
                'apriltag_dock.launch.py'
            ])
        ]),
        condition=IfCondition(use_dock),
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

        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ROS2 control with controller_manager'
        ),

        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Launch Nav2 navigation stack'
        ),

        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Launch SLAM toolbox for mapping'
        ),

        DeclareLaunchArgument(
            'use_dock',
            default_value='false',
            description='Launch AprilTag detection and docking server'
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
        bno_imu_sim,
        ekf_node,
        rviz_node,
        slam_bringup,
        nav2_bringup,
        dock_bringup,

        # INPUT STACK
        joy_node,
        teleop_node,
        twist_converter_node,
    ])