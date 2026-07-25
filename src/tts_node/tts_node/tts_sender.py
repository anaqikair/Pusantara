#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TTSSender(Node):

    def __init__(self):
        super().__init__('tts_sender')

        self.publisher = self.create_publisher(
            String,
            '/tts_text',
            10
        )

        self.get_logger().info(
            "TTS Sender Ready. Type text and press Enter."
        )

        self.run()

    def run(self):

        while rclpy.ok():

            try:
                text = input("\nSpeak> ")

                if not text.strip():
                    continue

                msg = String()
                msg.data = text

                self.publisher.publish(msg)

                self.get_logger().info(
                    f'Published: "{text}"'
                )

            except KeyboardInterrupt:
                break


def main(args=None):

    rclpy.init(args=args)

    node = TTSSender()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
