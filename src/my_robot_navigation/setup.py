from setuptools import find_packages, setup

package_name = 'my_robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/nav2.launch.py',
            'launch/localization.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
            'config/amcl_params.yaml',
        ]),
        ('share/' + package_name + '/maps', [
            'maps/my_map.yaml',
            'maps/my_map_gimp.pgm',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dinesh',
    maintainer_email='ydkishore30@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
