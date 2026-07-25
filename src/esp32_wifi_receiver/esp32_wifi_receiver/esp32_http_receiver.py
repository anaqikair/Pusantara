#!/usr/bin/env python3

import json
import threading
from flask import Flask, request, jsonify

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


app = Flask(__name__)
ros_node = None


class ESP32HttpReceiver(Node):
    def __init__(self):
        super().__init__('esp32_http_receiver')

        self.publisher_ = self.create_publisher(
            String,
            '/esp32/sensor_data',
            10
        )

        self.get_logger().info('ESP32 Wi-Fi HTTP Receiver started')
        self.get_logger().info('Waiting for ESP32 data on port 5000...')
        self.get_logger().info('ROS topic: /esp32/sensor_data')


@app.route('/esp32_data', methods=['POST'])
def receive_esp32_data():
    global ros_node

    try:
        data = request.get_json(force=True)

        if data is None:
            return jsonify({'status': 'error', 'message': 'No JSON received'}), 400

        msg = String()
        msg.data = json.dumps(data)

        ros_node.publisher_.publish(msg)

        ros_node.get_logger().info(f'Received ESP32 data: {msg.data}')

        return jsonify({
            'status': 'ok',
            'message': 'Data received by ROS 2',
            'data': data
        }), 200

    except Exception as e:
        if ros_node:
            ros_node.get_logger().error(f'Error receiving ESP32 data: {e}')

        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500


def run_flask_server():
    app.run(host='0.0.0.0', port=5000, debug=False)


def main(args=None):
    global ros_node

    rclpy.init(args=args)
    ros_node = ESP32HttpReceiver()

    flask_thread = threading.Thread(target=run_flask_server)
    flask_thread.daemon = True
    flask_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
