#!/usr/bin/env python3
"""
Converts Twist messages to TwistStamped messages.
Subscribes to /cmd_vel (Twist) and publishes to /diff_drive_controller/cmd_vel (TwistStamped)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from builtin_interfaces.msg import Time


class TwistConverter(Node):
    def __init__(self):
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
        """Convert Twist to TwistStamped"""
        twist_stamped = TwistStamped()
        
        # Set header
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        
        # Copy twist data
        twist_stamped.twist = msg
        
        # Publish the converted message
        self.twist_stamped_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
