#!/bin/bash
# Script to build, source, and launch Gazebo for my_robot_bringup

set -e
clear
rm -rf build/ install/ log/
# Build the workspace
colcon build 
source install/setup.bash
# Launch Gazebo
ros2 launch my_robot_bringup my_robot_gazebo.launch.py

