#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import ollama


class QwenNode(Node):

    def __init__(self):
        super().__init__('qwen_node')

        self.sub = self.create_subscription(
            String,
            '/user_text',
            self.text_callback,
            10
        )

        self.tts_pub = self.create_publisher(
            String,
            '/tts_text',
            10
        )

        self.get_logger().info("Qwen Node Started")

    def text_callback(self, msg):

        user_text = msg.data

        self.get_logger().info(f"User: {user_text}")

        try:
            response = ollama.chat(
                model='qwen2.5:0.5b',
                messages=[
                    {
                        'role': 'system',
                        'content':
                        'You are PUSANTARA, a cultural heritage service robot. Keep answers short and friendly.'
                    },
                    {
                        'role': 'user',
                        'content': user_text
                    }
                ]
            )

            answer = response['message']['content']

            self.get_logger().info(f"Qwen: {answer}")

            out = String()
            out.data = answer

            self.tts_pub.publish(out)

        except Exception as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)

    node = QwenNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
