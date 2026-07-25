#!/usr/bin/env python3

import time
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('debug', False)
        self.declare_parameter('confidence_threshold', 0.5)

        self.debug = self.get_parameter(
            'debug'
        ).value

        self.conf_threshold = self.get_parameter(
            'confidence_threshold'
        ).value

        # Utilities
        self.bridge = CvBridge()

        self.model = YOLO("yolov8n.pt")

        self.last_time = time.time()

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Detection publisher
        self.detection_pub = self.create_publisher(
            String,
            '/yolo/detections',
            10
        )

        # Debug image publisher
        self.debug_pub = self.create_publisher(
            Image,
            '/yolo/debug_image',
            10
        )

        self.get_logger().info("YOLO Detector Started")

        self.get_logger().info(
            f"Debug Mode: {self.debug}"
        )

        self.get_logger().info(
            f"Confidence Threshold: {self.conf_threshold}"
        )

    def image_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            results = self.model(
                frame,
                verbose=False
            )

            detected_people = []

            for result in results:

                for box in result.boxes:

                    cls_id = int(box.cls[0])

                    class_name = self.model.names[
                        cls_id
                    ]

                    # Detect only people
                    if class_name != "person":
                        continue

                    conf = float(box.conf[0])

                    if conf < self.conf_threshold:
                        continue

                    x1, y1, x2, y2 = box.xyxy[0]

                    x1 = int(x1)
                    y1 = int(y1)
                    x2 = int(x2)
                    y2 = int(y2)

                    center_x = int(
                        (x1 + x2) / 2
                    )

                    center_y = int(
                        (y1 + y2) / 2
                    )

                    width = x2 - x1
                    height = y2 - y1

                    detection_string = (
                        f"person,"
                        f"{conf:.2f},"
                        f"{center_x},"
                        f"{center_y},"
                        f"{width},"
                        f"{height}"
                    )

                    detected_people.append(
                        detection_string
                    )

                    # Debug drawing
                    if self.debug:

                        cv2.rectangle(
                            frame,
                            (x1, y1),
                            (x2, y2),
                            (0, 255, 0),
                            2
                        )

                        cv2.circle(
                            frame,
                            (center_x, center_y),
                            5,
                            (0, 0, 255),
                            -1
                        )

                        cv2.putText(
                            frame,
                            f"Person {conf:.2f}",
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2
                        )

            # Publish detections
            if detected_people:

                output = String()

                output.data = ";".join(
                    detected_people
                )

                self.detection_pub.publish(
                    output
                )

            # FPS calculation
            current_time = time.time()

            fps = 1.0 / (
                current_time - self.last_time
            )

            self.last_time = current_time

            people_count = len(
                detected_people
            )

            # Debug overlays
            if self.debug:

                cv2.putText(
                    frame,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 0, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"People: {people_count}",
                    (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 0, 0),
                    2
                )

                # Publish debug image
                debug_msg = (
                    self.bridge.cv2_to_imgmsg(
                        frame,
                        encoding='bgr8'
                    )
                )

                self.debug_pub.publish(
                    debug_msg
                )

                # Local display window
                cv2.imshow(
                    "YOLO Detector",
                    frame
                )

                key = cv2.waitKey(1)

                if key == ord('q'):
                    self.get_logger().info(
                        "Q pressed"
                    )

        except Exception as e:

            self.get_logger().error(
                f"Error: {str(e)}"
            )

    def destroy_node(self):

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = YoloDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
