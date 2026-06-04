import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. yolo_bringup package location ni automatic ga ethukuntundi
    yolo_bringup_dir = get_package_share_directory('yolo_bringup')
    
    # 2. YOLO26 original launch node config payload
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_bringup_dir, 'launch', 'yolo.launch.py')
        ),
        launch_arguments={
            'model': 'yolo26n.pt',
            'input_image_topic': '/camera/image_raw',
            'device': 'cpu',
            'threshold': '0.5'
        }.items()
    )

    return LaunchDescription([
        yolo_launch
    ])
