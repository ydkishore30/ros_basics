from setuptools import find_packages, setup

package_name = 'my_robot_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dinesh',
    maintainer_email='ydkishore30@gmail.com',
    description='Reinforcement learning for autonomous robot navigation',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'train_rl = my_robot_rl.train_rl:main',
            'inference_rl = my_robot_rl.inference_rl:main',
        ],
    },
)
