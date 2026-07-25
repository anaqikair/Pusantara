
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from rclpy.qos import qos_profile_sensor_data


class MotionManager(Node):

    def __init__(self):

        super().__init__('motion_manager')

        # =====================================
        # Parameters
        # =====================================

        self.declare_parameter('mode', 'SAFE')
        self.declare_parameter('stop_distance', 0.15)
        self.declare_parameter('warning_distance', 0.50)
        self.declare_parameter('front_angle', 30.0)

        self.mode = self.get_parameter(
            'mode'
        ).value.upper()

        self.stop_distance = self.get_parameter(
            'stop_distance'
        ).value

        self.warning_distance = self.get_parameter(
            'warning_distance'
        ).value

        self.front_angle = self.get_parameter(
            'front_angle'
        ).value

        # =====================================
        # State
        # =====================================

        self.front_distance = 999.0

        self.last_command = "M,0,0"

        self.last_sent_command = ""

        # =====================================
        # Subscribers
        # =====================================

        self.command_sub = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # =====================================
        # Publisher
        # =====================================

        self.safe_pub = self.create_publisher(
            String,
            '/safe_robot_command',
            10
        )

        # =====================================
        # Safety Timer
        # =====================================

        self.safety_timer = self.create_timer(
            0.1,  # 10Hz
            self.safety_loop
        )

        self.get_logger().info(
            "Motion Manager Started"
        )

    # =====================================
    # LIDAR
    # =====================================

    def scan_callback(self, msg):

        front = []

        total = len(msg.ranges)

        if total == 0:
            return

        angle_points = int(
            total * self.front_angle / 360.0
        )

        # Front left

        for i in range(angle_points):

            d = msg.ranges[i]

            if (
                math.isfinite(d)
                and d > 0.05
            ):
                front.append(d)

        # Front right

        for i in range(
            total - angle_points,
            total
        ):

            d = msg.ranges[i]

            if (
                math.isfinite(d)
                and d > 0.05
            ):
                front.append(d)

        if front:

            self.front_distance = min(front)

        else:

            self.front_distance = 999.0

    # =====================================
    # Command Input
    # =====================================

    def command_callback(self, msg):

        self.last_command = msg.data

    # =====================================
    # Utilities
    # =====================================

    def parse_motor_command(self, cmd):

        try:

            if not cmd.startswith("M,"):
                return None

            parts = cmd.split(",")

            right = int(parts[1])
            left = int(parts[2])

            return right, left

        except Exception:

            return None

    def moving_forward(self, cmd):

        result = self.parse_motor_command(cmd)

        if result:

            right, left = result

            return (
                right > 0
                or left > 0
            )

        if cmd == "W":
            return True

        return False

    def publish_if_changed(self, cmd):

        if cmd == self.last_sent_command:
            return

        msg = String()

        msg.data = cmd

        self.safe_pub.publish(msg)

        self.last_sent_command = cmd

    # =====================================
    # Main Safety Loop
    # =====================================

    def safety_loop(self):

        self.mode = self.get_parameter(
            'mode'
        ).value.upper()

        cmd = self.last_command

        # -------------------------------
        # OVERRIDE MODE
        # -------------------------------

        if self.mode == "OVERRIDE":

            self.publish_if_changed(cmd)

            return

        # -------------------------------
        # Obstacle Check
        # -------------------------------

        if self.moving_forward(cmd):

            if self.front_distance <= self.stop_distance:

                self.get_logger().warn(
                    f"STOP obstacle "
                    f"{self.front_distance:.2f}m"
                )

                self.publish_if_changed(
                    "M,0,0"
                )

                return

            if (
                self.mode == "ASSIST"
                and
                self.front_distance <= self.warning_distance
            ):

                self.get_logger().warn(
                    f"SLOW obstacle "
                    f"{self.front_distance:.2f}m"
                )

                self.publish_if_changed(
                    "M,10,10"
                )

                return

        # -------------------------------
        # Pass Through
        # -------------------------------

        self.publish_if_changed(cmd)

    # =====================================
    # Cleanup
    # =====================================


def main():

    rclpy.init()

    node = MotionManager()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()

