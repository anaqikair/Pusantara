#!/usr/bin/env python3

import time
import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class HeritageExplainer(Node):

    def __init__(self):
        super().__init__('heritage_explainer')

        self.subscription = self.create_subscription(
            String,
            '/apriltag/detections',
            self.tag_callback,
            10
        )

        self.tts_pub = self.create_publisher(
            String,
            '/tts_text',
            10
        )

        self.last_tag = None
        self.last_time = 0

        # Tags that should be detected but not explained
        self.ignored_tags = {"0"}

        # AprilTag ID = rock art panel marker
        self.heritage_data = {

            "1": {
                "name": "Gua Tambun Rock Art Panel",
                "text": (
                    "Panel one detected. This marker represents the Gua Tambun "
                    "rock art panel, a real ancient rock art site in Perak, Malaysia. "
                    "PUSANTARA CaveCare will inspect this panel without touching the "
                    "rock surface. The robot checks humidity using the DHT11 sensor, "
                    "scans the panel using the RGB camera, and uses LiDAR to keep a "
                    "safe distance. The purpose is to detect early mold or moss risk "
                    "before the rock art is damaged."
                )
            },

            "2": {
                "name": "Humidity Risk Panel",
                "text": (
                    "Panel two detected. This is a humidity risk inspection point. "
                    "PUSANTARA will check temperature and humidity near the rock art "
                    "panel and classify the condition as safe, warning, or danger."
                )
            },

            "3": {
                "name": "Mold and Moss Visual Scan Panel",
                "text": (
                    "Panel three detected. This area is used for visual inspection. "
                    "The RGB camera scans the rock art surface for green, dark, or "
                    "spreading patches that may indicate mold or moss growth."
                )
            },

            "4": {
                "name": "Safe Distance Inspection Panel",
                "text": (
                    "Panel four detected. PUSANTARA uses LiDAR to stop at a safe "
                    "distance so the robot does not touch or collide with the ancient "
                    "rock art panel."
                )
            },

            "5": {
                "name": "Conservation Report Panel",
                "text": (
                    "Panel five detected. After inspection, PUSANTARA saves the panel "
                    "ID, humidity, temperature, camera image, and risk status for "
                    "human conservators."
                )
            }
        }

        self.get_logger().info('PUSANTARA CaveCare Heritage Explainer Started')

    def parse_detection(self, msg):
        """
        Supports JSON AprilTag format:
        {"id": 1, "area": 2025, ...}

        Also supports older simple format:
        "1,320,240"
        """

        data = msg.data.strip()

        try:
            detection = json.loads(data)

            tag_id = str(detection.get("id"))
            area = int(detection.get("area", 0))

            return tag_id, area

        except Exception:
            pass

        try:
            parts = data.split(",")
            tag_id = str(parts[0]).strip()

            # If old format has no area, allow it by giving safe default area
            area = 2000

            return tag_id, area

        except Exception as e:
            self.get_logger().error(f"AprilTag Parse Error: {e}")
            return None, 0

    def tag_callback(self, msg):

        tag_id, area = self.parse_detection(msg)

        if tag_id is None:
            return

        # Ignore ID 0 completely
        if tag_id in self.ignored_tags:
            self.get_logger().info("Detected Tag 0: ignored")
            return

        # Ignore very small detections
        if area < 1000:
            return

        current_time = time.time()

        # Prevent repeating the same explanation too often
        if (
            tag_id == self.last_tag and
            current_time - self.last_time < 15
        ):
            return

        self.last_tag = tag_id
        self.last_time = current_time

        if tag_id in self.heritage_data:

            info = self.heritage_data[tag_id]

            speech = String()
            speech.data = info["text"]

            self.tts_pub.publish(speech)

            self.get_logger().info(
                f"Detected Tag {tag_id}: {info['name']}"
            )

            self.get_logger().info(
                f"Speaking: {info['text']}"
            )

        else:

            speech = String()
            speech.data = (
                f"I detected rock art panel marker number {tag_id}, "
                f"but this panel has not been added to the conservation "
                f"database yet."
            )

            self.tts_pub.publish(speech)

            self.get_logger().warn(
                f"Unknown Rock Art Panel Tag: {tag_id}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = HeritageExplainer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
