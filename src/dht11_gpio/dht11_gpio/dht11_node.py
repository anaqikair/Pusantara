#!/usr/bin/env python3

import time
import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Temperature, RelativeHumidity

import Jetson.GPIO as GPIO


class DHT11Reader:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

    def _wait_for_level(self, level, timeout_us):
        start = time.monotonic_ns()
        timeout_ns = timeout_us * 1000

        while GPIO.input(self.pin) != level:
            if time.monotonic_ns() - start > timeout_ns:
                return False

        return True

    def read(self):
        data_bits = []

        # Start signal
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.LOW)
        time.sleep(0.020)  # 20 ms low
        GPIO.output(self.pin, GPIO.HIGH)
        time.sleep(0.00004)  # 40 us high

        GPIO.setup(self.pin, GPIO.IN)

        # DHT11 response: LOW then HIGH
        if not self._wait_for_level(GPIO.LOW, 100):
            raise RuntimeError("DHT11 no initial LOW response")

        if not self._wait_for_level(GPIO.HIGH, 100):
            raise RuntimeError("DHT11 no initial HIGH response")

        if not self._wait_for_level(GPIO.LOW, 100):
            raise RuntimeError("DHT11 response did not end")

        # Read 40 bits
        for _ in range(40):
            if not self._wait_for_level(GPIO.HIGH, 100):
                raise RuntimeError("Timeout waiting for bit HIGH")

            high_start = time.monotonic_ns()

            if not self._wait_for_level(GPIO.LOW, 120):
                raise RuntimeError("Timeout waiting for bit LOW")

            high_time_us = (time.monotonic_ns() - high_start) / 1000.0

            # Around 26-28 us = 0, around 70 us = 1
            data_bits.append(1 if high_time_us > 50 else 0)

        # Convert bits into 5 bytes
        values = []
        for i in range(5):
            byte = 0
            for bit in data_bits[i * 8:(i + 1) * 8]:
                byte = (byte << 1) | bit
            values.append(byte)

        humidity_int, humidity_dec, temp_int, temp_dec, checksum = values

        if ((humidity_int + humidity_dec + temp_int + temp_dec) & 0xFF) != checksum:
            raise RuntimeError(
                f"Checksum failed: data={values}"
            )

        humidity = float(humidity_int) + float(humidity_dec) / 10.0
        temperature = float(temp_int) + float(temp_dec) / 10.0

        return temperature, humidity


class DHT11GpioNode(Node):
    def __init__(self):
        super().__init__("dht11_gpio_node")

        self.declare_parameter("board_pin", 7)
        self.declare_parameter("read_period", 2.0)
        self.declare_parameter("max_retries", 3)

        self.pin = int(self.get_parameter("board_pin").value)
        self.read_period = float(self.get_parameter("read_period").value)
        self.max_retries = int(self.get_parameter("max_retries").value)

        self.reader = DHT11Reader(self.pin)

        self.temp_pub = self.create_publisher(
            Temperature,
            "/dht11/temperature",
            10
        )

        self.humidity_pub = self.create_publisher(
            RelativeHumidity,
            "/dht11/humidity",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/dht11/status",
            10
        )

        self.data_pub = self.create_publisher(
            String,
            "/dht11/data",
            10
        )

        self.timer = self.create_timer(self.read_period, self.timer_callback)

        self.get_logger().info(
            f"DHT11 GPIO node started on Jetson physical pin {self.pin}"
        )

    def get_status(self, humidity):
        if humidity < 70.0:
            return "SAFE"
        elif humidity < 80.0:
            return "WARNING"
        else:
            return "DANGER"

    def read_with_retries(self):
        last_error = None

        for _ in range(self.max_retries):
            try:
                return self.reader.read()
            except Exception as error:
                last_error = error
                time.sleep(0.2)

        raise RuntimeError(last_error)

    def timer_callback(self):
        try:
            temperature, humidity = self.read_with_retries()
            status = self.get_status(humidity)

            now = self.get_clock().now().to_msg()

            temp_msg = Temperature()
            temp_msg.header.stamp = now
            temp_msg.header.frame_id = "dht11_link"
            temp_msg.temperature = temperature
            temp_msg.variance = 0.0

            humidity_msg = RelativeHumidity()
            humidity_msg.header.stamp = now
            humidity_msg.header.frame_id = "dht11_link"
            humidity_msg.relative_humidity = humidity / 100.0
            humidity_msg.variance = 0.0

            status_msg = String()
            status_msg.data = status

            data_msg = String()
            data_msg.data = json.dumps({
                "temperature_c": round(temperature, 1),
                "humidity_percent": round(humidity, 1),
                "status": status
            })

            self.temp_pub.publish(temp_msg)
            self.humidity_pub.publish(humidity_msg)
            self.status_pub.publish(status_msg)
            self.data_pub.publish(data_msg)

            self.get_logger().info(
                f"Temp: {temperature:.1f} C | Humidity: {humidity:.1f}% | Status: {status}"
            )

        except Exception as error:
            self.get_logger().warn(f"DHT11 read failed: {error}")

    def destroy_node(self):
        GPIO.cleanup(self.pin)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DHT11GpioNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
