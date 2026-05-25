from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart

def generate_launch_description():

    # ------------------------------
    # Find description package
    # ------------------------------
    description_pkg = FindPackageShare("my_robot_description")

    # Xacro file
    xacro_file = PathJoinSubstitution([
        description_pkg,
        "urdf",
        "my_robot.urdf.xacro",
    ])

    # Convert Xacro -> URDF
    robot_description = Command(["xacro ", xacro_file])

    # ------------------------------
    # Robot State Publisher
    # ------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description
        }]
    )

    # ------------------------------
    # Gazebo
    # ------------------------------
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", "/ros2_ws/src/my_robot_description/worlds/sample.sdf"],
        output="screen"
    )

    # ------------------------------
    # Spawn robot entity in Gazebo
    # ------------------------------
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot',
                   '-allow_renaming', 'true'],
    )

    # ------------------------------
    # ROS-Gazebo Bridge Configuration
    # Uses existing gazebo_bridge.yaml from my_robot_description
    # ------------------------------
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
    # Event handlers for sequential startup
    # ------------------------------
    # Spawn robot entity after Gazebo starts
    spawn_robot_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo,
            on_start=[gz_spawn_entity]
        )
    )

    # Start controller_manager after robot is spawned
    controller_manager_after_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_spawn_entity,
            on_start=[controller_manager]
        )
    )

    # Start joint_state_broadcaster after controller_manager
    joint_state_after_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    # Start diff_drive_controller after joint_state_broadcaster
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
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", PathJoinSubstitution([
            description_pkg,
            "rviz",
            "my_robot.rviz"
        ])]
    )

    # ------------------------------
    # Teleop (Gamepad Joystick Control)
    # Converts joystick input to Twist messages and then to TwistStamped
    # ------------------------------
    # Joy node (reads gamepad input)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )
    
    # Teleop node (converts joystick to Twist)
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        output="screen",
        parameters=[PathJoinSubstitution([
            FindPackageShare('my_robot_bringup'),
            'config',
            'teleop_joy.yaml',
        ])]
    )
    
    # Twist converter node (converts Twist to TwistStamped)
    twist_converter_node = Node(
        package="my_robot_bringup",
        executable="twist_converter.py",
        name="twist_converter",
        output="screen"
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
        gazebo,
        bridge,
        spawn_robot_after_gazebo,
        controller_manager_after_spawn,
        joint_state_after_controller,
        diff_drive_after_joint,
        rviz_node,
        # Joystick control with twist converter
        joy_node,
        teleop_node,
        twist_converter_node,
    ])