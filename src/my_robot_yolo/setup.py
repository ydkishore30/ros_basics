import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files workspace share directory loki copy avvadaniki ee line sahayam chestundi:
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dinesh',
    maintainer_email='ydkishore30@gmail.com',
    description='YOLO26 Perception and tracking integration package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Kavalante custom python script entry points ikkada add cheskovachu
        ],
    },
)
