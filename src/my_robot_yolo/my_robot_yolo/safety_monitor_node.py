#!/usr/bin/env python3
"""Stops the robot when a watched object class is closer than a safety distance.

Distance is estimated monocularly from bounding-box pixel height using the
pinhole camera model: distance = (real_world_height * focal_length_y) / bbox_height_px.
This is a rough heuristic (no depth sensing on this robot) intended as a
semantic safety layer on top of nav2's LiDAR-based obstacle avoidance, not a
replacement for it.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray

DEFAULT_OBJECT_HEIGHTS_M = {
    'person': 1.7,
}


class YoloSafetyMonitorNode(Node):
    """Publishes /yolo_safety/stop=True while a watched class is within the safety distance."""

    def __init__(self):
        super().__init__('yolo_safety_monitor')

        self.declare_parameter('watch_classes', list(DEFAULT_OBJECT_HEIGHTS_M.keys()))
        self.declare_parameter('safety_distance', 1.0)

        self.watch_classes = set(
            self.get_parameter('watch_classes').get_parameter_value().string_array_value)
        self.safety_distance = self.get_parameter(
            'safety_distance').get_parameter_value().double_value

        self.focal_length_y = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.detections_sub = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.detections_callback, 10)
        self.stop_pub = self.create_publisher(Bool, '/yolo_safety/stop', 10)

    def camera_info_callback(self, msg: CameraInfo):
        self.focal_length_y = msg.k[4]

    def detections_callback(self, msg: Detection2DArray):
        if self.focal_length_y is None:
            return

        danger = False
        for detection in msg.detections:
            if not detection.results:
                continue

            class_id = detection.results[0].hypothesis.class_id
            real_height = DEFAULT_OBJECT_HEIGHTS_M.get(class_id)
            if class_id not in self.watch_classes or real_height is None:
                continue
            if detection.bbox.size_y <= 0:
                continue

            distance = (real_height * self.focal_length_y) / detection.bbox.size_y
            if distance < self.safety_distance:
                danger = True
                self.get_logger().warn(
                    f'{class_id} detected at ~{distance:.2f}m, '
                    f'below safety distance {self.safety_distance}m')

        self.stop_pub.publish(Bool(data=danger))


def main(args=None):
    rclpy.init(args=args)
    node = YoloSafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
