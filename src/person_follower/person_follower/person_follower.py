#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import String


class PersonFollower(Node):

    def __init__(self):

        super().__init__('person_follower')

        self.subscription = self.create_subscription(
            String,
            '/yolo/detections',
            self.detection_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            String,
            '/robot_command',
            10
        )

        self.get_logger().info(
            "Person Follower Started"
        )

    def send_command(self, cmd):

        msg = String()
        msg.data = cmd

        self.cmd_pub.publish(msg)

    def detection_callback(self, msg):

        try:

            detections = msg.data.split(";")

            if len(detections) == 0:
                return

            largest_person = None
            largest_width = 0

            for det in detections:

                parts = det.split(",")

                if len(parts) != 6:
                    continue

                label = parts[0]

                if label != "person":
                    continue

                center_x = int(parts[2])
                width = int(parts[4])

                if width > largest_width:

                    largest_width = width
                    largest_person = (
                        center_x,
                        width
                    )

            if largest_person is None:
                return

            center_x, width = largest_person

            # CLOSE ENOUGH
            if width > 250:

                self.send_command(
                    "M,0,0"
                )

                return

            # TURN LEFT
            if center_x < 260:

                self.send_command(
                    "M,-20,20"
                )

            # TURN RIGHT
            elif center_x > 380:

                self.send_command(
                    "M,20,-20"
                )

            # FORWARD
            else:

                self.send_command(
                    "M,30,30"
                )

        except Exception as e:

            self.get_logger().error(
                str(e)
            )


def main(args=None):

    rclpy.init(args=args)

    node = PersonFollower()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
