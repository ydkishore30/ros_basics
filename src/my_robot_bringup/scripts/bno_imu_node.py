#!/usr/bin/env python3
"""
BNO IMU serial driver node.

Reads lines from the BNO sensor over serial in the format:
  I <qx> <qy> <qz> <gx> <gy> <gz>

Where:
  qx qy qz  — quaternion i,j,k components (w is computed as sqrt(1 - qx²-qy²-qz²))
  gx gy gz  — gyroscope angular velocity (rad/s)

Publishes: sensor_msgs/msg/Imu on /imu

Usage:
  ros2 run my_robot_bringup bno_imu_node.py --ros-args -p serial_port:=/dev/ttyACM0
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial


class BnoImuNode(Node):
    def __init__(self):
        super().__init__('bno_imu_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub = self.create_publisher(Imu, '/imu', 10)

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.ser.dtr = False  # prevent Arduino reset on port open
            self.get_logger().info(f'Opened BNO serial port: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port {port}: {e}')
            raise

        self.create_timer(0.005, self.read_and_publish)  # 200 Hz poll

    def read_and_publish(self):
        if not self.ser.in_waiting:
            return
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception:
            return

        if not line.startswith('I'):
            return

        parts = line.split()
        if len(parts) != 7:
            return

        try:
            qx, qy, qz = float(parts[1]), float(parts[2]), float(parts[3])
            gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
        except ValueError:
            return

        # Compute qw from unit quaternion constraint
        norm_sq = qx * qx + qy * qy + qz * qz
        qw = math.sqrt(max(0.0, 1.0 - norm_sq))

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Covariance — diagonal, tuned for BNO085 typical noise
        msg.orientation_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.01,
        ]
        msg.angular_velocity_covariance = [
            0.001, 0, 0,
            0, 0.001, 0,
            0, 0, 0.001,
        ]
        # Linear acceleration not in protocol — mark as unknown
        msg.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BnoImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
