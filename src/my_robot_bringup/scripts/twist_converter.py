#!/usr/bin/env python3
"""
Converts Twist messages to TwistStamped messages.

Subscribes to /cmd_vel (Twist) and publishes to /diff_drive_controller/cmd_vel 

(TwistStamped).
"""

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool

import rclpy
from rclpy.node import Node


class TwistConverter(Node):
    """ROS2 node that converts Twist messages to TwistStamped messages."""

    def __init__(self):
        """Initialize the Twist Converter node."""
        super().__init__('twist_converter')

        self.yolo_safety_stop = False

        # Subscriber for Twist messages from teleop
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

        # Subscriber for the YOLO semantic safety layer's stop signal
        self.safety_sub = self.create_subscription(
            Bool,
            '/yolo_safety/stop',
            self.safety_callback,
            10
        )

        # Publisher for TwistStamped messages to controller
        self.twist_stamped_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )

        self.get_logger().info('Twist Converter node started')

    def safety_callback(self, msg: Bool):
        """Track whether the YOLO safety layer wants the robot stopped."""
        self.yolo_safety_stop = msg.data

    def twist_callback(self, msg: Twist):
        """Convert Twist to TwistStamped, zeroing velocity if a safety stop is active."""
        twist_stamped = TwistStamped()

        # Set header
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'

        # Copy twist data, unless the YOLO safety layer has called a stop
        twist_stamped.twist = Twist() if self.yolo_safety_stop else msg

        # Publish the converted message
        self.twist_stamped_pub.publish(twist_stamped)


def main(args=None):
    """Generate launch description for robot bringup."""
    rclpy.init(args=args)
    node = TwistConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
