#!/usr/bin/env python3

import os
import re
import queue
import threading
import subprocess

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TTSNode(Node):

    def __init__(self):

        super().__init__('tts_node')

        # -----------------------------
        # Piper Paths
        # -----------------------------

        self.en_model = (
            "/home/orin_nano/tts/en_US-lessac-medium.onnx"
        )

        # Future Malay model
        self.ms_model = (
            "/home/orin_nano/tts/ms_MY-medium.onnx"
        )

        self.piper_bin = (
            "/home/orin_nano/tts/piper/piper"
        )

        # -----------------------------
        # Queue
        # -----------------------------

        self.tts_queue = queue.Queue()

        self.worker_thread = threading.Thread(
            target=self.tts_worker,
            daemon=True
        )

        self.worker_thread.start()

        # -----------------------------
        # ROS Subscriber
        # -----------------------------

        self.subscription = self.create_subscription(
            String,
            '/tts_text',
            self.speak_callback,
            10
        )

        self.get_logger().info("TTS Node Ready")

    # ==================================
    # Language Detection
    # ==================================

    def detect_language(self, text):

        text = text.lower()

        malay_keywords = [
            "selamat",
            "muzium",
            "warisan",
            "budaya",
            "sejarah",
            "melaka",
            "terima kasih",
            "sila",
            "anda",
            "bangunan",
            "tradisional",
            "kesultanan",
            "malaysia"
        ]

        score = 0

        for word in malay_keywords:
            if word in text:
                score += 1

        if score >= 2:
            return "ms"

        return "en"

    # ==================================
    # ROS Callback
    # ==================================

    def speak_callback(self, msg):

        text = msg.data.strip()

        if not text:
            return

        self.tts_queue.put(text)

    # ==================================
    # Worker Thread
    # ==================================

    def tts_worker(self):

        while True:

            text = self.tts_queue.get()

            try:

                language = self.detect_language(text)

                if (
                    language == "ms"
                    and os.path.exists(self.ms_model)
                ):
                    model = self.ms_model
                else:
                    model = self.en_model

                self.get_logger().info(
                    f"[{language.upper()}] {text}"
                )

                wav_file = "/tmp/pusantara_tts.wav"

                piper_process = subprocess.Popen(
                    [
                        self.piper_bin,
                        "--model",
                        model,
                        "--output_file",
                        wav_file
                    ],
                    stdin=subprocess.PIPE,
                    text=True
                )

                piper_process.communicate(text)

                subprocess.run(
                    ["aplay", wav_file],
                    check=False
                )

            except Exception as e:

                self.get_logger().error(
                    f"TTS Error: {e}"
                )

            self.tts_queue.task_done()


def main(args=None):

    rclpy.init(args=args)

    node = TTSNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
