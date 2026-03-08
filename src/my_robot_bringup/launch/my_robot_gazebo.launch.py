from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
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
        cmd=["gz", "sim", "-v", "4", "-r",
             "/home/ubuntu/ros_basics/src/my_robot_description/worlds/sample.sdf"],
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
    # ROS-Gazebo Bridge
    # ------------------------------
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
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

    # Start joint_state_broadcaster after robot is spawned
    joint_state_after_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_spawn_entity,
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
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            description_pkg,
            "rviz",
            "my_robot.rviz"
        ])]
    )

    # ------------------------------
    # Launch description
    # ------------------------------
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        bridge,
        spawn_robot_after_gazebo,
        joint_state_after_spawn,
        diff_drive_after_joint,
        rviz_node
    ])