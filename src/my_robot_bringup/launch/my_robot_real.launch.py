"""Launch file for real hardware bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for real hardware bringup."""
    joy_dev = LaunchConfiguration('joy_dev')
    serial_port = LaunchConfiguration('serial_port')
    use_bno = LaunchConfiguration('use_bno')

    description_pkg = FindPackageShare('my_robot_description')

    # Xacro file
    xacro_file = PathJoinSubstitution([
        description_pkg,
        'urdf',
        'my_robot.urdf.xacro',
    ])

    # Convert Xacro -> URDF
    robot_description = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=true',
        ' use_sim_hardware:=false',
        ' serial_port:=', serial_port,
    ])

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

    # # ------------------------------
    # # RViz
    # # ------------------------------
    # use_rviz = LaunchConfiguration('use_rviz')

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     condition=IfCondition(use_rviz),
    #     arguments=['-d', PathJoinSubstitution([
    #         description_pkg,
    #         'rviz',
    #         'my_robot.rviz'
    #     ])]
    # )

    # ------------------------------
    # Teleop (Gamepad Joystick Control)
    # Converts joystick input to Twist messages and then to TwistStamped
    # ------------------------------
    # Joy node (reads gamepad input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': joy_dev}]
    )

    # Teleop node: publishes Twist on /cmd_vel
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('my_robot_bringup'),
            'config',
            'teleop_joy.yaml',
        ])]
    )

    # Twist → TwistStamped converter (diff_drive_controller v4+ requires TwistStamped)
    twist_converter = Node(
        package='my_robot_bringup',
        executable='twist_converter.py',
        name='twist_converter',
        output='screen'
    )

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
        parameters=[ekf_config, {
            'use_sim_time': False
        }]
    )

    bno_imu_node = Node(
        package='my_robot_bringup',
        executable='bno_imu_node.py',
        name='bno_imu_node',
        output='screen',
        condition=IfCondition(use_bno),
        parameters=[{
            'serial_port': LaunchConfiguration('imu_port'),
            'baud_rate': 115200,
            'frame_id': 'imu_link',
        }]
    )

    # ------------------------------
    # YDLidar X2 → /scan (frame lidar_link)
    # ------------------------------
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'), 'config', 'ydlidar_x2.yaml',
            ]),
            {'port': LaunchConfiguration('lidar_port')},
        ],
    )

    # ------------------------------
    # Launch description
    # ------------------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            # Fixed USB-port path: ESP32 and X2 both use a CP2102, so ttyUSB0/1
            # can swap between boots. ESP32 must stay in Pi USB port 1.3.
            default_value='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
            description='Serial port for motor controller + BNO IMU (Arduino/ESP32)'
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/ttyACM0',
            description='Serial port for BNO IMU (must be DIFFERENT from serial_port)'
        ),
        DeclareLaunchArgument(
            'use_bno',
            default_value='false',
            description='Deprecated — BNO IMU is now published by the hardware interface directly'
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='false',  # temporarily off; enable with use_lidar:=true
            description='Start the YDLidar X2 driver (publishes /scan)'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            # Fixed USB-port path — YDLidar X2 must stay in Pi USB port 1.2.
            default_value='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0',
            description='Serial port for YDLidar X2 (must be DIFFERENT from serial_port)'
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/event0',
            description='Joystick device path (check with: ls /dev/input/by-id/)'
        ),
        robot_state_publisher,
        controller_manager,
        joint_state_after_controller,
        diff_drive_after_joint,
        bno_imu_node,
        lidar_node,
        ekf_node,
        joy_node,
        teleop_node,
        twist_converter,
    ])
