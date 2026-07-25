#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from faster_whisper import WhisperModel

import subprocess
import tempfile
import threading
import os


class SpeechToTextNode(Node):

    def __init__(self):

        super().__init__('speech_to_text')

        self.publisher_ = self.create_publisher(
            String,
            '/speech_text',
            10
        )

        self.wake_word = "robot"

        self.audio_device = "sysdefault:CARD=WEBCAM"

        self.record_seconds = 4

        self.get_logger().info(
            "Loading Whisper Base..."
        )

        self.model = WhisperModel(
            "base",
            device="cpu",
            compute_type="int8"
        )

        self.get_logger().info(
            "Whisper Ready"
        )

        self.get_logger().info(
            f'Wake word: "{self.wake_word}"'
        )

        self.worker_thread = threading.Thread(
            target=self.listen_loop,
            daemon=True
        )

        self.worker_thread.start()

    def listen_loop(self):

        while rclpy.ok():

            try:

                with tempfile.NamedTemporaryFile(
                    suffix=".wav",
                    delete=False
                ) as temp_file:

                    wav_file = temp_file.name

                subprocess.run(
                    [
                        "arecord",
                        "-D",
                        self.audio_device,
                        "-f",
                        "S16_LE",
                        "-r",
                        "16000",
                        "-c",
                        "1",
                        "-d",
                        str(self.record_seconds),
                        wav_file
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )

                segments, info = self.model.transcribe(
                    wav_file,
                    beam_size=5
                )

                text = ""

                for segment in segments:
                    text += segment.text + " "

                text = text.strip().lower()

                if len(text) == 0:

                    if os.path.exists(wav_file):
                        os.remove(wav_file)

                    continue

                self.get_logger().info(
                    f"Heard: {text}"
                )

                if self.wake_word in text:

                    command = text.replace(
                        self.wake_word,
                        ""
                    ).strip()

                    if len(command) > 0:

                        msg = String()
                        msg.data = command

                        self.publisher_.publish(msg)

                        self.get_logger().info(
                            f'Published: "{command}"'
                        )

                if os.path.exists(wav_file):
                    os.remove(wav_file)

            except Exception as e:

                self.get_logger().error(
                    str(e)
                )


def main(args=None):

    rclpy.init(args=args)

    node = SpeechToTextNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
