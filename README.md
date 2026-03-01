# My Robot Simulation - Build & Launch Instructions

## Prerequisites
- ROS 2 (e.g., Jazzy, Humble, or compatible)
- Gazebo and required ROS 2 Gazebo bridge packages
- All dependencies installed (see your ROS 2 installation guide)

## How to Build and Launch the Simulation

1. **Open a terminal and navigate to your workspace root:**
   ```bash
   cd /home/ubuntu/ros_basics
   ```

2. **Run the build, source, and launch script:**
   ```bash
   ./src/my_robot_bringup/launch/build_source_launch.sh
   ```
   This script will:
   - Build the workspace using `colcon build`
   - Source the setup file
   - Launch the Gazebo simulation with your robot

3. **(Optional) If you get a permission error, make the script executable:**
   ```bash
   chmod +x ./src/my_robot_bringup/launch/build_source_launch.sh
   ```
   Then re-run the script as above.

## Troubleshooting
- Ensure all dependencies are installed and sourced.
- If you modify URDF/Xacro or launch files, re-run the script to rebuild and relaunch.
- For parameter changes, edit the appropriate `.xacro` or `.yaml` files, then rebuild and relaunch.

## Additional Notes
- To visualize the robot in RViz, use the display launch file:
  ```bash
  ros2 launch src/my_robot_description/launch/display.launch.py
  ```
- For customizations, edit the files in `src/my_robot_description/` and `src/my_robot_bringup/`.

---
For more help, see the ROS 2 and Gazebo documentation or contact the project maintainer.


./src/my_robot_bringup/launch/build_source_launch.sh