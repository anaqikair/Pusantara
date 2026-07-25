#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image


class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('webcam_publisher')

        self.publisher_ = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.bridge = CvBridge()

        camera_device = (
            "/dev/v4l/by-id/"
            "usb-046d_C270_HD_WEBCAM_E01FE110-video-index0"
        )

        self.cap = cv2.VideoCapture(camera_device)

        if not self.cap.isOpened():
            self.get_logger().error(
                f"Failed to open camera: {camera_device}"
            )
            raise RuntimeError("Camera not found")

        # Resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.get_logger().info(
            f"Camera opened: {camera_device}"
        )

        # 30 FPS
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.publish_frame
        )

    def publish_frame(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn(
                "Failed to capture frame"
            )
            return

        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        self.publisher_.publish(msg)

    def destroy_node(self):

        if self.cap.isOpened():
            self.cap.release()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = WebcamPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
