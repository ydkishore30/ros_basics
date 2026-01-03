from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command

def generate_launch_description():

    description_pkg = FindPackageShare("my_robot_description")
    bringup_pkg = FindPackageShare("my_robot_bringup")
    gazebo_pkg = FindPackageShare("gazebo_ros")

    # URDF / Xacro
    xacro_file = PathJoinSubstitution([
        description_pkg,
        "urdf",
        "my_robot.urdf.xacro"
    ])

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

    # Joint State Publisher GUI
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    # RViz2
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_pkg, "launch", "gazebo.launch.py"])
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "my_robot"
        ],
        output="screen"
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        rviz2
    ])
