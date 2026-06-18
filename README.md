# My Robot Simulation - Build & Launch Instructions

## Prerequisites
- ROS 2 (e.g., Jazzy, Humble, or compatible)
- Gazebo and required ROS 2 Gazebo bridge packages
- All dependencies installed (see your ROS 2 installation guide)

## How to Build and Launch the Simulation

1. **Open a terminal and navigate to your workspace root:**
   ```bash
   cd /ros2_ws
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

## Real Robot (Raspberry Pi) Build & Launch
```bash
./src/my_robot_bringup/launch/build_real_source_launch.sh
```

## Troubleshooting
- Ensure all dependencies are installed and sourced.
- If you modify URDF/Xacro or launch files, re-run the script to rebuild and relaunch.
- For parameter changes, edit the appropriate `.xacro` or `.yaml` files, then rebuild and relaunch.
- If topics/nodes seem stale or out of sync, reset the ROS 2 daemon:
  ```bash
  export ROS_DOMAIN_ID=0
  ros2 daemon stop
  ros2 daemon start
  ```

## Additional Notes
- To visualize the robot in RViz, use the display launch file:
  ```bash
  ros2 launch src/my_robot_description/launch/display.launch.py
  ```
- For customizations, edit the files in `src/my_robot_description/` and `src/my_robot_bringup/`.
- Enable the `ros2_control` Gazebo system plugin (add to `~/.bashrc`):
  ```bash
  echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
  ```

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

You can manually publish velocity commands to control the robot's movement.

Directly to the controller (bypasses the converter):
```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: base_link}, twist: {linear: {x: -1.0}, angular: {z: 0.0}}}'
```

Or to `/cmd_vel` (requires `twist_converter` running to bridge to the controller):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

**Parameters:**
- `linear.x`: Linear velocity (m/s). Positive = forward, Negative = backward.
- `angular.z`: Angular velocity (rad/s). Positive = counterclockwise, Negative = clockwise.

**Example Commands:**
- Move forward: `linear: {x: 1.0}, angular: {z: 0.0}`
- Move backward: `linear: {x: -1.0}, angular: {z: 0.0}`
- Rotate counterclockwise: `linear: {x: 0.0}, angular: {z: 1.0}`
- Rotate clockwise: `linear: {x: 0.0}, angular: {z: -1.0}`

### Joystick
Check the controller is connected before launching `joy_node`:
```bash
ls /dev/input/js0
```

## SLAM (Mapping)

Default config reference: `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml`

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

With a custom params file:
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/ros2_ws/src/my_robot_description/nav2_params.yaml use_sim_time:=True
```

Save the generated map once mapping is complete:
```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/my_robot_description/maps/<world_name>
```

## Navigation (Nav2)

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  map:=./src/my_robot_description/maps/my_first_map.yaml \
  params_file:=/ros2_ws/src/my_robot_description/nav2_params.yaml
```

```bash
ros2 launch nav2_bringup bringup_launch.py map:=./src/my_robot_description/maps/my_map.yaml use_sim_time:=True
```

```bash
ros2 launch nav2_bringup localization_launch.py map:=/src/my_robot_description/maps/my_map.yaml use_sim_time:=True
```

```bash
ros2 launch my_robot_navigation nav2.launch.py
```

For testing without SLAM/AMCL running, publish a static `map`→`odom` transform:
```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --frame-id map --child-frame-id odom
```

## Custom / Test Worlds
```bash
export GZ_SIM_RESOURCE_PATH=/home/dinesh/ros_basics/src/my_robot_description/worlds/sample_blender
gz sim /home/dinesh/ros_basics/src/my_robot_description/worlds/sample_blender/model.sdf
```

## Docker / Raspberry Pi Deployment

Enter a running container:
```bash
docker exec -it ros_basics-ros-1 bash
```

Run the prebuilt Pi image:
```bash
sudo docker run -d --name ros_basic_container --restart unless-stopped --net-host ghcr.io/ydkishore30/ros_basics:pi
```

Or with compose:
```bash
docker compose --profile pi up -d
```

**One-time setup on your laptop for ARM emulation** (needed to build/run ARM images on x86):
```bash
docker run --privileged --rm tonistiigi/binfmt --install all
```

---
For more help, see the ROS 2 and Gazebo documentation or contact the project maintainer.
