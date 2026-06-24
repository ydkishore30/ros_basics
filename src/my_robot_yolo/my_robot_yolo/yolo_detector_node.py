#!/usr/bin/env python3
"""Runs an Ultralytics YOLO model on the robot camera feed and publishes detections."""

import cv_bridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class YoloDetectorNode(Node):
    """Subscribes to the camera feed and publishes vision_msgs/Detection2DArray."""

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path', 'yolo26n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/camera/image_raw')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter(
            'confidence_threshold').get_parameter_value().double_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.model = YOLO(model_path)
        self.bridge = cv_bridge.CvBridge()

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.detections_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)

        self.get_logger().info(f"YOLO detector loaded '{model_path}', watching {image_topic}")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, verbose=False)[0]

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for box in results.boxes:
            confidence = float(box.conf[0])
            if confidence < self.confidence_threshold:
                continue

            x_center, y_center, width, height = box.xywh[0].tolist()
            class_name = self.model.names[int(box.cls[0])]

            detection = Detection2D()
            detection.header = msg.header
            detection.bbox = BoundingBox2D()
            detection.bbox.center.position.x = x_center
            detection.bbox.center.position.y = y_center
            detection.bbox.size_x = width
            detection.bbox.size_y = height

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = confidence
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

        self.detections_pub.publish(detection_array)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
