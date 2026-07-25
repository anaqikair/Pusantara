#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelToRobotCommand(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_robot_command")

        # Topics
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("robot_command_topic", "/robot_command")

        # Robot physical settings
        self.declare_parameter("wheel_separation", 0.33)

        # Safe speed limits
        self.declare_parameter("max_linear_speed", 0.35)
        self.declare_parameter("max_angular_speed", 1.20)

        # Motor command limits
        self.declare_parameter("max_motor_power", 55)
        self.declare_parameter("min_motor_power", 18)

        # Small values ignored
        self.declare_parameter("linear_deadband", 0.02)
        self.declare_parameter("angular_deadband", 0.05)

        # Safety timeout
        self.declare_parameter("cmd_timeout", 0.50)

        # Motor direction correction
        self.declare_parameter("left_invert", False)
        self.declare_parameter("right_invert", False)
        self.declare_parameter("swap_wheels", False)

        # If false, backward command becomes STOP
        self.declare_parameter("allow_reverse", True)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.robot_command_topic = self.get_parameter("robot_command_topic").value

        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        self.max_motor_power = int(self.get_parameter("max_motor_power").value)
        self.min_motor_power = int(self.get_parameter("min_motor_power").value)

        self.linear_deadband = float(self.get_parameter("linear_deadband").value)
        self.angular_deadband = float(self.get_parameter("angular_deadband").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.left_invert = bool(self.get_parameter("left_invert").value)
        self.right_invert = bool(self.get_parameter("right_invert").value)
        self.swap_wheels = bool(self.get_parameter("swap_wheels").value)

        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)

        self.last_cmd_time = None
        self.last_output = None

        self.pub = self.create_publisher(String, self.robot_command_topic, 10)

        self.sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        self.timer = self.create_timer(0.10, self.timeout_check)

        self.get_logger().info("cmd_vel_bridge started")
        self.get_logger().info(f"Subscribing: {self.cmd_vel_topic}")
        self.get_logger().info(f"Publishing : {self.robot_command_topic}")
        self.get_logger().info("Output format: M,<right>,<left>")

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def apply_deadband(self, value, deadband):
        if abs(value) < deadband:
            return 0.0
        return value

    def wheel_speed_to_power(self, wheel_speed):
        if abs(wheel_speed) < 1e-6:
            return 0

        power = int((wheel_speed / self.max_linear_speed) * self.max_motor_power)

        power = self.clamp(power, -self.max_motor_power, self.max_motor_power)

        if power != 0 and abs(power) < self.min_motor_power:
            power = self.min_motor_power if power > 0 else -self.min_motor_power

        return int(power)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.pub.publish(msg)
        self.last_output = command

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        linear_x = self.clamp(
            linear_x,
            -self.max_linear_speed,
            self.max_linear_speed
        )

        angular_z = self.clamp(
            angular_z,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        linear_x = self.apply_deadband(linear_x, self.linear_deadband)
        angular_z = self.apply_deadband(angular_z, self.angular_deadband)

        if not self.allow_reverse and linear_x < 0:
            linear_x = 0.0

        if linear_x == 0.0 and angular_z == 0.0:
            self.publish_command("X")
            return

        # Standard differential drive formula
        left_speed = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_separation / 2.0)

        left_power = self.wheel_speed_to_power(left_speed)
        right_power = self.wheel_speed_to_power(right_speed)

        # Normalize if one side exceeds limit
        max_abs = max(abs(left_power), abs(right_power))
        if max_abs > self.max_motor_power:
            scale = self.max_motor_power / max_abs
            left_power = int(left_power * scale)
            right_power = int(right_power * scale)

        if self.left_invert:
            left_power *= -1

        if self.right_invert:
            right_power *= -1

        if self.swap_wheels:
            left_power, right_power = right_power, left_power

        # Your robot command format:
        # M,<right>,<left>
        command = f"M,{right_power},{left_power}"
        self.publish_command(command)

    def timeout_check(self):
        if self.last_cmd_time is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.cmd_timeout:
            if self.last_output != "X":
                self.publish_command("X")
                self.get_logger().warn("cmd_vel timeout -> X")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRobotCommand()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_command("X")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
