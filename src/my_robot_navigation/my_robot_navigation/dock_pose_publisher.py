#!/usr/bin/env python3
"""Bridges an AprilTag TF detection into the PoseStamped topic nav2_docking expects."""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

CAMERA_FRAME = 'camera_link_optical'
DOCK_TAG_FRAME = 'dock'
PUBLISH_RATE_HZ = 10.0


class DockPosePublisher(Node):

    def __init__(self):
        super().__init__('dock_pose_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_dock_pose)

    def publish_dock_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                CAMERA_FRAME, DOCK_TAG_FRAME, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except (LookupException, ExtrapolationException):
            return

        pose = PoseStamped()
        pose.header.frame_id = CAMERA_FRAME
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation

        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = DockPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
