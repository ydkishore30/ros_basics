#!/usr/bin/env python3
"""
BNO IMU simulator for PC Docker testing.

Subscribes to Gazebo's /imu topic and re-publishes it in the same
sensor_msgs/Imu format the real BNO node would produce, so the EKF
and rest of the stack see identical data in simulation and on hardware.

Also prints the equivalent 'I qx qy qz gx gy gz' serial line for
verifying the parse logic matches the real sensor output.

Usage (inside container):
  ros2 run my_robot_bringup bno_imu_sim.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class BnoImuSim(Node):
    def __init__(self):
        super().__init__('bno_imu_sim')
        self.sub = self.create_subscription(Imu, '/imu/gz', self._cb, 10)
        self.pub = self.create_publisher(Imu, '/imu', 10)
        self.get_logger().info('BNO IMU sim: bridging /imu/gz → /imu')

    def _cb(self, msg: Imu):
        # Re-stamp and re-publish with BNO-style covariances
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'imu_link'

        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        out.orientation_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.01,
        ]
        out.angular_velocity_covariance = [
            0.001, 0, 0,
            0, 0.001, 0,
            0, 0, 0.001,
        ]
        out.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(out)

        # Print equivalent serial line for debugging
        q = msg.orientation
        g = msg.angular_velocity
        self.get_logger().debug(
            f'I {q.x:.3f} {q.y:.3f} {q.z:.3f} {g.x:.3f} {g.y:.3f} {g.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BnoImuSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
