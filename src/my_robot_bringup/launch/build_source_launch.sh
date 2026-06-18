#!/bin/bash
set -e
clear

rm -rf build install log

# Build
colcon build --symlink-install

# Source new workspace
source install/setup.bash

# Launch
ros2 launch my_robot_bringup my_robot_gazebo.launch.py use_sim_time:=true