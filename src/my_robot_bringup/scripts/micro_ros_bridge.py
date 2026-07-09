#!/usr/bin/env python3
"""Bridge: cmd_vel → /wheel_cmd and /telemetry → /odom + tf."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

WHEEL_BASE      = 0.34    # m
WHEEL_RADIUS    = 0.035   # m
COUNTS_PER_REV  = 1260.0
MAX_WHEEL_SPEED = 0.35    # m/s at command=1.0 (calibrated from encoder counts)


class MicroRosBridge(Node):
    def __init__(self):
        super().__init__('micro_ros_bridge')

        self._wheel_pub = self.create_publisher(Float32MultiArray, '/wheel_cmd', 10)
        self._odom_pub  = self.create_publisher(Odometry, '/odom', 10)
        self._tf_br     = TransformBroadcaster(self)

        self.create_subscription(Twist,              '/cmd_vel',   self._cmd_cb,  10)
        self.create_subscription(Float32MultiArray,  '/telemetry', self._telem_cb, 10)

        self._x = self._y = self._yaw = 0.0
        self._prev_left = self._prev_right = None

    def _cmd_cb(self, msg: Twist):
        v, w = msg.linear.x, msg.angular.z
        vl = (v - w * WHEEL_BASE / 2) / MAX_WHEEL_SPEED
        vr = (v + w * WHEEL_BASE / 2) / MAX_WHEEL_SPEED
        vl = max(-1.0, min(1.0, vl))
        vr = max(-1.0, min(1.0, vr))
        out = Float32MultiArray()
        # Left motor is mounted mirrored — negate to go forward
        out.data = [-vl, vr]
        self._wheel_pub.publish(out)

    def _telem_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        lc, rc = msg.data[0], msg.data[1]
        if self._prev_left is None:
            self._prev_left, self._prev_right = lc, rc
            return

        m_per_count = 2.0 * math.pi * WHEEL_RADIUS / COUNTS_PER_REV
        dl = (lc - self._prev_left)  * m_per_count
        dr = (rc - self._prev_right) * m_per_count
        self._prev_left, self._prev_right = lc, rc

        d     = (dl + dr) / 2.0
        dtheta = (dr - dl) / WHEEL_BASE
        self._x   += d * math.cos(self._yaw + dtheta / 2)
        self._y   += d * math.sin(self._yaw + dtheta / 2)
        self._yaw += dtheta

        now = self.get_clock().now().to_msg()

        # odom message
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        cy, sy = math.cos(self._yaw / 2), math.sin(self._yaw / 2)
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        self._odom_pub.publish(odom)

        # tf odom → base_link
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.rotation.z    = sy
        t.transform.rotation.w    = cy
        self._tf_br.sendTransform(t)


def main():
    rclpy.init()
    node = MicroRosBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
