#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class AprilTagFollower(Node):

    def __init__(self):

        super().__init__('apriltag_follower')

        #
        # Parameters
        #

        self.declare_parameter('kp', 80.0)

        self.declare_parameter('target_x', 0.5)

        self.declare_parameter('normal_speed', 40)
        self.declare_parameter('slow_speed', 20)

        self.declare_parameter('stop_area', 6000)
        self.declare_parameter('slow_area', 3000)

        self.declare_parameter('lost_timeout', 0.5)
        self.declare_parameter('search_timeout', 2.0)

        self.declare_parameter('search_speed', 15)
        self.declare_parameter('search_switch_time', 3.0)

        self.kp = self.get_parameter(
            'kp'
        ).value

        self.target_x = self.get_parameter(
            'target_x'
        ).value

        self.normal_speed = self.get_parameter(
            'normal_speed'
        ).value

        self.slow_speed = self.get_parameter(
            'slow_speed'
        ).value

        self.stop_area = self.get_parameter(
            'stop_area'
        ).value

        self.slow_area = self.get_parameter(
            'slow_area'
        ).value

        self.lost_timeout = self.get_parameter(
            'lost_timeout'
        ).value

        self.search_timeout = self.get_parameter(
            'search_timeout'
        ).value

        self.search_speed = self.get_parameter(
            'search_speed'
        ).value

        self.search_switch_time = self.get_parameter(
            'search_switch_time'
        ).value

        #
        # State
        #

        self.filtered_x = None

        self.last_detection_time = 0.0

        self.last_command = None

        self.search_direction = 1
        self.last_search_switch = time.time()

        #
        # ROS
        #

        self.subscription = self.create_subscription(
            String,
            '/apriltag/detections',
            self.detection_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/robot_command',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'AprilTag follower started'
        )

    # --------------------------------------------------
    # Utilities
    # --------------------------------------------------

    def clamp(self, value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def publish_command(self, command):

        #
        # Avoid spamming identical commands
        #

        if command == self.last_command:
            return

        msg = String()
        msg.data = command

        self.publisher.publish(msg)

        self.last_command = command

    # --------------------------------------------------
    # Detection callback
    # --------------------------------------------------

    def detection_callback(self, msg):

        try:
            data = json.loads(msg.data)

        except Exception as e:

            self.get_logger().warn(
                f'JSON decode error: {e}'
            )
            return

        current_x = data.get(
            'normalized_x',
            0.5
        )

        area = data.get(
            'area',
            0
        )

        #
        # Exponential smoothing
        #

        if self.filtered_x is None:
            self.filtered_x = current_x
        else:
            self.filtered_x = (
                0.7 * self.filtered_x +
                0.3 * current_x
            )

        self.current_area = area

        self.last_detection_time = time.time()

    # --------------------------------------------------
    # Main control loop
    # --------------------------------------------------

    def control_loop(self):

        now = time.time()

        #
        # No detections yet
        #

        if self.last_detection_time == 0.0:
            self.publish_command("M,0,0")
            return

        elapsed = now - self.last_detection_time

        #
        # Search mode
        #

        if elapsed > self.search_timeout:

            if (
                now - self.last_search_switch
                >
                self.search_switch_time
            ):

                self.search_direction *= -1

                self.last_search_switch = now

            right_speed = (
                self.search_speed *
                self.search_direction
            )

            left_speed = (
                -self.search_speed *
                self.search_direction
            )

            cmd = (
                f"M,{int(right_speed)},{int(left_speed)}"
            )

            self.publish_command(cmd)

            return

        #
        # Lost but not searching
        #

        if elapsed > self.lost_timeout:

            self.publish_command("M,0,0")

            return

        #
        # Distance control
        #

        area = getattr(
            self,
            'current_area',
            0
        )

        if area >= self.stop_area:

            self.publish_command("M,0,0")

            return

        elif area >= self.slow_area:

            base_speed = self.slow_speed

        else:

            base_speed = self.normal_speed

        #
        # Steering control
        #

        error = (
            self.target_x -
            self.filtered_x
        )

        turn = self.kp * error

        #
        # Differential drive
        #

        right_speed = (
            base_speed + turn
        )

        left_speed = (
            base_speed - turn
        )

        #
        # Clamp
        #

        right_speed = self.clamp(
            int(right_speed),
            -100,
            100
        )

        left_speed = self.clamp(
            int(left_speed),
            -100,
            100
        )

        cmd = (
            f"M,{right_speed},{left_speed}"
        )

        self.publish_command(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = AprilTagFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.publish_command("M,0,0")

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
