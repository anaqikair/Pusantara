#!/usr/bin/env python3

import random
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class VisitorGreeter(Node):

    def __init__(self):

        super().__init__('visitor_greeter')

        self.subscription = self.create_subscription(
            String,
            '/yolo/detections',
            self.detection_callback,
            10
        )

        self.tts_pub = self.create_publisher(
            String,
            '/tts_text',
            10
        )

        self.person_present = False

        self.last_seen_time = 0.0
        self.last_greet_time = 0.0
        self.last_engagement_time = 0.0

        # Settings
        self.greeting_cooldown = 15.0
        self.person_timeout = 5.0

        self.engagement_interval = random.randint(30, 60)

        # Greeting messages
        self.greetings = [

            "Welcome to PUSANTARA.",

            "Hello visitor. Would you like to learn about Malaysian cultural heritage?",

            "Selamat datang. I can help you explore traditional food, batik and cultural artifacts.",

            "Welcome. Feel free to ask me about our cultural heritage.",

            "Hello. I am PUSANTARA, your cultural heritage guide.",

            "Greetings visitor. I am ready to assist your cultural journey.",

            "Welcome. Explore Malaysia's rich heritage with me."
        ]

        # While visitor is present
        self.engagement_messages = [

            "Did you know batik patterns often tell cultural stories?",

            "You can try the Batik Designer application.",

            "Feel free to ask me about traditional Malaysian food.",

            "I can explain cultural artifacts and historical objects.",

            "Would you like to learn about traditional clothing?",

            "Ask me about Malaysian heritage and traditions.",

            "I am here to make cultural learning fun and interactive.",

            "You may also try our cultural heritage quiz.",

            "Malaysia has a rich heritage of art, food and traditions."
        ]

        # Goodbye messages
        self.goodbye_messages = [

            "Goodbye visitor. Thank you for exploring our heritage.",

            "Thank you for visiting. Have a wonderful day.",

            "Selamat jalan. I hope you enjoyed learning about our culture.",

            "See you again soon.",

            "Thank you for spending time with PUSANTARA.",

            "Farewell. I hope to guide you again in the future.",

            "Goodbye and thank you for supporting cultural heritage."
        ]

        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.get_logger().info(
            "Visitor Greeter Started"
        )

    def speak(self, text):

        msg = String()
        msg.data = text

        self.tts_pub.publish(msg)

        self.get_logger().info(
            f"TTS: {text}"
        )

    def detection_callback(self, msg):

        detections = msg.data.lower()

        if "person" not in detections:
            return

        now = time.time()

        self.last_seen_time = now

        # New visitor arrived
        if not self.person_present:

            self.person_present = True

            if (
                now - self.last_greet_time
                > self.greeting_cooldown
            ):

                greeting = random.choice(
                    self.greetings
                )

                self.speak(greeting)

                self.last_greet_time = now

                self.last_engagement_time = now

                self.engagement_interval = random.randint(
                    30,
                    60
                )

                self.get_logger().info(
                    "Visitor detected"
                )

    def timer_callback(self):

        now = time.time()

        # Visitor currently present
        if self.person_present:

            # Visitor left
            if (
                now - self.last_seen_time
                > self.person_timeout
            ):

                goodbye = random.choice(
                    self.goodbye_messages
                )

                self.speak(goodbye)

                self.person_present = False

                self.get_logger().info(
                    "Visitor left"
                )

                return

            # Engagement messages
            if (
                now - self.last_engagement_time
                > self.engagement_interval
            ):

                message = random.choice(
                    self.engagement_messages
                )

                self.speak(message)

                self.last_engagement_time = now

                self.engagement_interval = random.randint(
                    30,
                    60
                )

    def destroy_node(self):

        self.get_logger().info(
            "Visitor Greeter Stopped"
        )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = VisitorGreeter()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
