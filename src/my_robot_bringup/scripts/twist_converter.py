#!/usr/bin/env python3
"""
Converts Twist messages to TwistStamped messages.

Subscribes to /cmd_vel (Twist) and publishes to /diff_drive_controller/cmd_vel 

(TwistStamped).
"""

from geometry_msgs.msg import Twist, TwistStamped

import rclpy
from rclpy.node import Node


class TwistConverter(Node):
    """ROS2 node that converts Twist messages to TwistStamped messages."""

    def __init__(self):
        """Initialize the Twist Converter node."""
        super().__init__('twist_converter')

        # Subscriber for Twist messages from teleop
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

        # Publisher for TwistStamped messages to controller
        self.twist_stamped_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )

        self.get_logger().info('Twist Converter node started')

    def twist_callback(self, msg: Twist):
        """Convert Twist to TwistStamped."""
        twist_stamped = TwistStamped()

        # Set header
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'

        # Copy twist data
        twist_stamped.twist = msg

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
