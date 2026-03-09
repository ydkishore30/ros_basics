# Joystick Control Setup

This guide explains how to use a gamepad joystick to control the my_robot.

## Prerequisites

Install the required ROS2 packages:

```bash
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

## Hardware Setup

1. Connect your gamepad/joystick to your computer via USB
2. Verify the device is recognized:
   ```bash
   ls -la /dev/input/js*
   ```

## Joystick Controls

### Default Mapping (Xbox-style controller):
- **Left Stick Horizontal (LX)**: Robot rotation (yaw)
- **Right Trigger (RT)**: Forward/backward movement
- **A Button**: Enable control
- **X Button**: Turbo mode (increased speed)

### Configuration

The joystick configuration is in `config/teleop_joy.yaml`. You can customize:
- `axis_linear`: Which axis controls forward/backward motion
- `axis_angular`: Which axis controls rotation
- `scale_linear`: Maximum linear velocity
- `scale_angular`: Maximum angular velocity

## Axis Reference

Common joystick axis mappings:
- 0: Left Stick X (LX)
- 1: Left Stick Y (LY)
- 2: Right Stick X (RX)
- 3: Right Stick Y (RY)
- 4: Left Trigger (LT) / Right Trigger (RT) combined
- 5+: Additional triggers and buttons

## Launching

Simply run the main launch file:

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.py
```

The joy_node, teleop_twist_joy, and twist_converter will start automatically.

## Tested Devices

- Xbox 360 Controller
- Xbox One Controller
- Generic USB gamepads

## Troubleshooting

If the joystick isn't detected:
- Check `/dev/input/js*` permissions
- Run with `sudo` if needed
- Test with: `ros2 run joy joy_node`
- Echo Joy topic: `ros2 topic echo /joy`
