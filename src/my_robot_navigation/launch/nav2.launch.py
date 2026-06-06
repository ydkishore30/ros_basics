from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('my_robot_navigation')

    map_file = os.path.join(
        pkg_share,
        'maps',
        'my_map.yaml'
    )

    params_file = os.path.join(
        pkg_share,
        'config',
        'nav2_params.yaml'
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_bringup_dir,
                    'launch',
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'true'
            }.items()
        )
    ])