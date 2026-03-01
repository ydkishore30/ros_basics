#!/bin/bash
# Script to build, source, and launch Gazebo for my_robot_bringup

set -e

# Build the workspace
colcon build --packages-select my_robot_description my_robot_bringup --symlink-install
colcon build 

# Source the setup script
touch install/setup.bash  # Ensure the file exists
test -f install/setup.bash && source install/setup.bash

# Launch Gazebo
ros2 launch my_robot_bringup my_robot_gazebo.launch.py
