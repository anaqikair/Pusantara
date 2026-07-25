
import time
import json

import cv2
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import String

from cv_bridge import CvBridge

from pupil_apriltags import Detector


class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('apriltag_detector')

        self.declare_parameter('show_ui', True)

        self.show_ui = (
            self.get_parameter('show_ui')
            .get_parameter_value()
            .bool_value
        )

        self.bridge = CvBridge()

        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.id_pub = self.create_publisher(
            Int32,
            '/apriltag/id',
            10
        )

        self.detection_pub = self.create_publisher(
            String,
            '/apriltag/detections',
            10
        )

        self.last_time = time.time()

        self.get_logger().info(
            f'AprilTag Detector Started (UI={self.show_ui})'
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        frame_height, frame_width = frame.shape[:2]

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        tags = self.detector.detect(gray)

        if self.show_ui:

            cv2.line(
                frame,
                (frame_width // 2, 0),
                (frame_width // 2, frame_height),
                (255, 0, 255),
                1
            )

            cv2.line(
                frame,
                (0, frame_height // 2),
                (frame_width, frame_height // 2),
                (255, 0, 255),
                1
            )

        for tag in tags:

            tag_id = int(tag.tag_id)

            id_msg = Int32()
            id_msg.data = tag_id
            self.id_pub.publish(id_msg)

            center_x = int(tag.center[0])
            center_y = int(tag.center[1])

            corners = tag.corners.astype(int)

            width = int(
                (
                    (
                        corners[1][0] - corners[0][0]
                    ) ** 2 +
                    (
                        corners[1][1] - corners[0][1]
                    ) ** 2
                ) ** 0.5
            )

            height = int(
                (
                    (
                        corners[2][0] - corners[1][0]
                    ) ** 2 +
                    (
                        corners[2][1] - corners[1][1]
                    ) ** 2
                ) ** 0.5
            )

            area = int(
                abs(
                    cv2.contourArea(
                        corners.astype('float32')
                    )
                )
            )

            normalized_x = (
                center_x / frame_width
            )

            normalized_y = (
                center_y / frame_height
            )

            detection = {
                "id": tag_id,
                "center_x": center_x,
                "center_y": center_y,
                "width": width,
                "height": height,
                "area": area,
                "frame_width": frame_width,
                "frame_height": frame_height,
                "normalized_x": round(
                    normalized_x,
                    4
                ),
                "normalized_y": round(
                    normalized_y,
                    4
                ),
                "timestamp": time.time()
            }

            det_msg = String()
            det_msg.data = json.dumps(detection)

            self.detection_pub.publish(det_msg)

            if self.show_ui:

                for i in range(4):

                    pt1 = tuple(corners[i])
                    pt2 = tuple(
                        corners[
                            (i + 1) % 4
                        ]
                    )

                    cv2.line(
                        frame,
                        pt1,
                        pt2,
                        (0, 255, 0),
                        2
                    )

                cv2.circle(
                    frame,
                    (
                        center_x,
                        center_y
                    ),
                    5,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    frame,
                    f"ID {tag_id}",
                    (
                        center_x + 10,
                        center_y
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"Area: {area}",
                    (
                        center_x + 10,
                        center_y + 25
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"W:{width} H:{height}",
                    (
                        center_x + 10,
                        center_y + 45
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"X:{normalized_x:.2f}",
                    (
                        center_x + 10,
                        center_y + 65
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    2
                )

        if self.show_ui:

            now = time.time()

            fps = 1.0 / max(
                now - self.last_time,
                0.0001
            )

            self.last_time = now

            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 0, 0),
                2
            )

            cv2.putText(
                frame,
                f"Resolution: {frame_width}x{frame_height}",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 0),
                2
            )

            cv2.imshow(
                "AprilTag Detector",
                frame
            )

            cv2.waitKey(1)

    def destroy_node(self):

        if self.show_ui:
            cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = AprilTagDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


