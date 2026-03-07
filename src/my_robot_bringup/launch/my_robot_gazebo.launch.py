from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Find description package
    description_pkg = FindPackageShare("my_robot_description")

    # Xacro file
    xacro_file = PathJoinSubstitution([
        description_pkg,
        "urdf",
        "my_robot.urdf.xacro",
        
    ])

    # Convert Xacro -> URDF
    robot_description = Command(["xacro ", xacro_file])

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description
        }]
    )

    # Gazebo-ROS Bridge
    bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    parameters=[{
        "config_file": PathJoinSubstitution([
            description_pkg,
            "config",
            "gazebo_bridge.yaml"
        ])
    }],
    output="screen"
)

    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        )
        

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('my_robot_bringup'),
            'config',
            'my_robot_controllers.yaml',
        ]
    )


    diff_drive_controller_spawn = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    # Start Gazebo Harmonic with ground
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", "/home/ubuntu/ros_basics/src/my_robot_description/worlds/sample.sdf"],
        output="screen"
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'diff_drive', '-allow_renaming', 'true'],
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        bridge,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawn
        # diff_drive_controller_spawn,
        #         RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gz_spawn_entity,
        #         on_exit=[joint_state_broadcaster_spawner],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[diff_drive_controller_spawn],
        #     )
        # )
            ,
        Node(
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
    ])
