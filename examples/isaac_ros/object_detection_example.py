#!/usr/bin/env python3
"""
Isaac ROS Object Detection Example
This script demonstrates how to use Isaac ROS for object detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional


class IsaacObjectDetectionNode(Node):
    """
    A node that demonstrates Isaac ROS object detection functionality.
    Note: This is a conceptual example. Actual Isaac ROS object detection
    implementation uses specialized Isaac ROS DNN packages with TensorRT.
    """

    def __init__(self):
        super().__init__('isaac_object_detection_node')

        # Create CV bridge for image conversion
        self.bridge = CvBridge()

        # Subscription to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publisher for detection results
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/isaac_ros/detections',
            10
        )

        # Detection parameters
        self.confidence_threshold = 0.5
        self.class_labels = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
            'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
            'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
            'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
            'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
            'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
            'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
            'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
            'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        self.get_logger().info('Isaac ROS Object Detection node initialized')

    def image_callback(self, msg: Image):
        """Process incoming image and perform object detection"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            detections = self.perform_detection(cv_image)

            # Create and publish detection message
            detection_msg = self.create_detection_message(detections, msg.header)
            self.detection_pub.publish(detection_msg)

            # Log detection results
            if detections:
                self.get_logger().info(f'Detected {len(detections)} objects')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def perform_detection(self, image: np.ndarray) -> List[Dict]:
        """
        Perform object detection on the input image.
        In actual Isaac ROS implementation, this would use TensorRT-accelerated
        neural networks through Isaac ROS DNN packages.
        """
        height, width = image.shape[:2]

        # Simulated detection results (in practice, these come from Isaac ROS DNN)
        # This simulates what would be the output of a real neural network
        simulated_detections = [
            {
                'class_name': 'person',
                'confidence': 0.89,
                'bbox': [int(0.3 * width), int(0.2 * height),
                         int(0.2 * width), int(0.6 * height)]  # [x, y, w, h]
            },
            {
                'class_name': 'chair',
                'confidence': 0.76,
                'bbox': [int(0.6 * width), int(0.4 * height),
                         int(0.25 * width), int(0.5 * height)]
            },
            {
                'class_name': 'bottle',
                'confidence': 0.68,
                'bbox': [int(0.1 * width), int(0.5 * height),
                         int(0.08 * width), int(0.15 * height)]
            }
        ]

        # Filter detections by confidence threshold
        filtered_detections = [
            det for det in simulated_detections
            if det['confidence'] >= self.confidence_threshold
        ]

        return filtered_detections

    def create_detection_message(self, detections: List[Dict], header: Header) -> Detection2DArray:
        """Create Detection2DArray message from detection results"""
        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            detection_2d = Detection2D()
            detection_2d.header = header

            # Set bounding box
            bbox = BoundingBox2D()
            bbox.center.position.x = float(detection['bbox'][0] + detection['bbox'][2] / 2)
            bbox.center.position.y = float(detection['bbox'][1] + detection['bbox'][3] / 2)
            bbox.size_x = float(detection['bbox'][2])
            bbox.size_y = float(detection['bbox'][3])
            detection_2d.bbox = bbox

            # Create object hypothesis for the detected object
            from vision_msgs.msg import ObjectHypothesisWithPose
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = detection['class_name']
            hypothesis.hypothesis.score = detection['confidence']
            detection_2d.results.append(hypothesis)

            detection_array.detections.append(detection_2d)

        return detection_array

    def visualize_detections(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """Visualize detections on the image (for debugging purposes)"""
        vis_image = image.copy()

        for detection in detections:
            # Extract bounding box coordinates
            x, y, w, h = detection['bbox']
            x, y, w, h = int(x), int(y), int(w), int(h)

            # Draw bounding box
            cv2.rectangle(vis_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Draw label and confidence
            label = f"{detection['class_name']}: {detection['confidence']:.2f}"
            cv2.putText(vis_image, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return vis_image


def main(args=None):
    """Main function to run the object detection node"""
    rclpy.init(args=args)

    detection_node = IsaacObjectDetectionNode()

    try:
        # Spin the node
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        detection_node.get_logger().info('Shutting down Isaac Object Detection node')
    finally:
        detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()