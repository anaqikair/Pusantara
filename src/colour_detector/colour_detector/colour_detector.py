#!/usr/bin/env python3
"""ROS 2 HSV colour detector for the PUSANTARA camera."""

import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ColourDetector(Node):
    """Detect one configured colour and publish boxes and pixel coordinates."""

    @staticmethod
    def colour_ranges():
        """HSV thresholds; OpenCV hue values range from 0 through 179."""
        return {
            "red": [
                (np.array([0, 100, 70]), np.array([10, 255, 255])),
                (np.array([170, 100, 70]), np.array([179, 255, 255])),
            ],
            "green": [
                (np.array([35, 70, 50]), np.array([85, 255, 255])),
            ],
            "blue": [
                (np.array([90, 80, 50]), np.array([130, 255, 255])),
            ],
            "yellow": [
                (np.array([20, 100, 100]), np.array([35, 255, 255])),
            ],
            "orange": [
                (np.array([8, 100, 100]), np.array([20, 255, 255])),
            ],
            "purple": [
                (np.array([130, 60, 50]), np.array([160, 255, 255])),
            ],
        }

    def __init__(self):
        super().__init__("colour_detector")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/colour/detections")
        self.declare_parameter("annotated_topic", "/colour/annotated")
        self.declare_parameter("target_colour", "red")
        self.declare_parameter("min_area", 500.0)
        self.declare_parameter("kernel_size", 5)
        self.declare_parameter("show_ui", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detections_topic = str(
            self.get_parameter("detections_topic").value
        )
        self.annotated_topic = str(
            self.get_parameter("annotated_topic").value
        )
        self.target_colour = str(
            self.get_parameter("target_colour").value
        ).lower()
        self.min_area = float(self.get_parameter("min_area").value)
        self.kernel_size = int(self.get_parameter("kernel_size").value)
        self.show_ui = bool(self.get_parameter("show_ui").value)

        if self.target_colour not in self.colour_ranges():
            raise ValueError(
                "target_colour must be one of: "
                + ", ".join(self.colour_ranges().keys())
            )
        if self.min_area < 0:
            raise ValueError("min_area must be >= 0")
        if self.kernel_size < 1:
            raise ValueError("kernel_size must be >= 1")

        self.bridge = CvBridge()
        self.kernel = np.ones(
            (self.kernel_size, self.kernel_size), dtype=np.uint8
        )
        self.last_time = time.monotonic()

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.detection_pub = self.create_publisher(
            String, self.detections_topic, 10
        )
        self.annotated_pub = self.create_publisher(
            Image, self.annotated_topic, 10
        )

        self.get_logger().info(
            f"Colour detector started; colour={self.target_colour}, "
            f"input={self.image_topic}"
        )

    def create_mask(self, hsv_image):
        """Create and clean a binary mask for the selected HSV colour."""
        mask = None
        for lower, upper in self.colour_ranges()[self.target_colour]:
            part = cv2.inRange(hsv_image, lower, upper)
            mask = part if mask is None else cv2.bitwise_or(mask, part)

        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, self.kernel, iterations=1
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, self.kernel, iterations=2
        )
        return mask

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as exc:  # keep the node alive on a bad frame
            self.get_logger().error(f"Could not convert camera image: {exc}")
            return

        frame_height, frame_width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self.create_mask(hsv)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area:
                continue

            x, y, box_width, box_height = cv2.boundingRect(contour)
            moments = cv2.moments(contour)
            if moments["m00"] > 0.0:
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
            else:
                center_x = x + box_width // 2
                center_y = y + box_height // 2

            detection_index = len(detections)
            detections.append({
                "index": detection_index,
                "colour": self.target_colour,
                "center_x": center_x,
                "center_y": center_y,
                "x": int(x),
                "y": int(y),
                "width": int(box_width),
                "height": int(box_height),
                "area": round(area, 1),
                "frame_width": int(frame_width),
                "frame_height": int(frame_height),
                "normalized_x": round(center_x / frame_width, 4),
                "normalized_y": round(center_y / frame_height, 4),
            })

            # Draw the detection rectangle and contour centroid.
            cv2.rectangle(
                frame,
                (x, y),
                (x + box_width, y + box_height),
                (0, 255, 0),
                2,
            )
            cv2.circle(
                frame, (center_x, center_y), 5, (0, 0, 255), -1
            )
            label = f"{self.target_colour} #{detection_index}"
            cv2.putText(
                frame,
                label,
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"({center_x}, {center_y})",
                (x, min(frame_height - 8, y + box_height + 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1,
            )

        # Publish an empty detections list when no target is present.
        payload = {
            "colour": self.target_colour,
            "frame_width": int(frame_width),
            "frame_height": int(frame_height),
            "count": len(detections),
            "detections": detections,
            "timestamp": time.time(),
        }
        detection_msg = String()
        detection_msg.data = json.dumps(payload)
        self.detection_pub.publish(detection_msg)

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(
                frame, encoding="bgr8"
            )
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as exc:
            self.get_logger().error(f"Could not publish annotated image: {exc}")

        if self.show_ui:
            now = time.monotonic()
            fps = 1.0 / max(now - self.last_time, 1e-6)
            self.last_time = now
            cv2.putText(
                frame,
                f"{self.target_colour}: {len(detections)}  FPS: {fps:.1f}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 0, 0),
                2,
            )
            cv2.imshow("PUSANTARA Colour Detector", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_ui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ColourDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
