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

## Teleop / Robot Control

### Option 1: Keyboard Teleop (Recommended)

The robot controller expects `TwistStamped` messages, but `teleop_twist_keyboard` publishes `Twist` messages by default. A converter node is included to handle this conversion automatically.

**Setup:**

First, install the teleop package if you haven't already:
```bash
sudo apt install ros-<distro>-teleop-twist-keyboard
```
Replace `<distro>` with your ROS 2 distribution (e.g., `jazzy`, `humble`).

**Enable in Launch File:**

Edit the launch file (`src/my_robot_bringup/launch/my_robot_gazebo.launch.py`) and uncomment both teleop nodes in the launch description:
```python
# Uncomment below to enable keyboard teleop
teleop_node,
twist_converter_node,
```

Then rebuild and relaunch:
```bash
./src/my_robot_bringup/launch/build_source_launch.sh
```

**Manual Teleop (without modifying launch file):**

In a new terminal with the simulation running, start the converter first:
```bash
source install/setup.bash
ros2 run my_robot_bringup twist_converter.py
```

Then in another terminal, start teleop:
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i`: Move forward
- `u` / `o`: Move forward + turn left/right
- `j` / `l`: Turn left/right (in place)
- `m` / `.`: Move backward + turn left/right
- `,`: Move backward
- `[` / `]`: Increase/decrease max speed
- `v` / `b`: Increase/decrease turn speed
- `k` or `Space`: Stop

### Option 2: Manual Topic Publishing

You can manually publish velocity commands to control the robot's movement. Open a new terminal and run:

```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: -1.0}, angular: {z: 0.0}}}"
```

**Parameters:**
- `linear.x`: Linear velocity (m/s). Positive = forward, Negative = backward. Set to `-1.0` for backward motion.
- `angular.z`: Angular velocity (rad/s). Positive = counterclockwise, Negative = clockwise. Set to `0.0` for straight motion.

**Example Commands:**
- Move forward: `linear: {x: 1.0}, angular: {z: 0.0}`
- Move backward: `linear: {x: -1.0}, angular: {z: 0.0}`
- Rotate counterclockwise: `linear: {x: 0.0}, angular: {z: 1.0}`
- Rotate clockwise: `linear: {x: 0.0}, angular: {z: -1.0}`

---
For more help, see the ROS 2 and Gazebo documentation or contact the project maintainer.


./src/my_robot_bringup/launch/build_source_launch.sh