from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command

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
        parameters=[PathJoinSubstitution([
            description_pkg,
            "config",
            "gazebo_bridge.yaml"
        ])],
        output="screen"
    )

    # Start Gazebo Harmonic with ground
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", "empty.sdf"],
        output="screen"
    )

    # Spawn robot slightly above ground
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-z", "0.2"
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot
            ,
            bridge,
            # Launch RViz2 with my_robot.rviz config
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
