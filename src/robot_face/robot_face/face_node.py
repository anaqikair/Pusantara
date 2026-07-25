import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import pygame
import random
import time
import math


class RobotFace(Node):

    def __init__(self):

        super().__init__('robot_face')

        self.subscription = self.create_subscription(
            String,
            '/yolo/detections',
            self.callback,
            10
        )

        self.tts_subscription = self.create_subscription(
            String,
            '/tts_text',
            self.tts_callback,
            10
        )

        pygame.init()

        self.width = 1000
        self.height = 700

        self.screen = pygame.display.set_mode(
            (self.width, self.height),
            pygame.RESIZABLE
        )

        pygame.display.set_caption("PUSANTARA Face")

        self.person_count = 0
        self.target_x = 320

        self.last_detection = time.time()

        self.next_blink = time.time() + random.uniform(3, 6)
        self.blinking = False
        self.blink_end = 0

        self.pupil = 0
        self.target_pupil = 0

        self.speaking = False
        self.speaking_end = 0

        self.timer = self.create_timer(
            0.03,
            self.update
        )

    def callback(self, msg):

        people = []

        for det in msg.data.split(';'):

            parts = det.split(',')

            if len(parts) < 6:
                continue

            if parts[0] != "person":
                continue

            x = int(parts[2])
            w = int(parts[4])

            center = x + w / 2

            people.append(center)

        self.person_count = len(people)

        if len(people) > 0:

            self.target_x = sum(people) / len(people)

            self.last_detection = time.time()

    def tts_callback(self, msg):

        self.speaking = True
        duration = max(1.0, len(msg.data) * 0.08)
        self.speaking_end = time.time() + duration

    def draw_eye(self, x, y):

        if self.blinking:

            pygame.draw.line(
                self.screen,
                (40, 60, 120),
                (x - 60, y),
                (x + 60, y),
                12
            )

            return

        pygame.draw.circle(
            self.screen,
            (255, 255, 255),
            (x, y),
            85
        )

        pygame.draw.circle(
            self.screen,
            (100, 180, 255),
            (int(x + self.pupil), y),
            35
        )

        pygame.draw.circle(
            self.screen,
            (20, 20, 20),
            (int(x + self.pupil), y),
            18
        )

        pygame.draw.circle(
            self.screen,
            (255,255,255),
            (int(x + self.pupil - 8), y - 8),
            5
        )

    def draw_face(self):

        panel = pygame.Rect(
            150,
            100,
            700,
            500
        )

        pygame.draw.rect(
            self.screen,
            (245,245,250),
            panel,
            border_radius=50
        )

    def draw_mouth(self):

        color = (50, 80, 150)

        if self.speaking:
            mouth_open = math.sin(time.time() * 20) > 0
            if mouth_open:
                pygame.draw.ellipse(
                    self.screen,
                    color,
                    (450, 440, 100, 60),
                    0
                )
            else:
                pygame.draw.line(
                    self.screen,
                    color,
                    (450, 470),
                    (550, 470),
                    6
                )
            return

        if self.person_count == 0:

            pygame.draw.line(
                self.screen,
                color,
                (450, 470),
                (550, 470),
                6
            )

        elif self.person_count == 1:

            pygame.draw.arc(
                self.screen,
                color,
                (430, 420, 140, 80),
                0,
                math.pi,
                6
            )

        else:

            pygame.draw.circle(
                self.screen,
                color,
                (500, 470),
                30,
                5
            )

    def update(self):

        for event in pygame.event.get():

            if event.type == pygame.QUIT:

                self.destroy_node()
                raise SystemExit

            elif event.type == pygame.VIDEORESIZE:

                self.width = event.w
                self.height = event.h

                self.screen = pygame.display.set_mode(
                    (self.width, self.height),
                    pygame.RESIZABLE
                )

        now = time.time()

        if now - self.last_detection > 1.0:

            self.person_count = 0

        if now > self.next_blink:

            self.blinking = True

            self.blink_end = now + 0.12

            self.next_blink = now + random.uniform(3, 6)

        if self.blinking and now > self.blink_end:

            self.blinking = False

        if self.speaking and now > self.speaking_end:

            self.speaking = False

        if self.person_count > 0:

            normalized = self.target_x / 640.0

            self.target_pupil = (
                normalized - 0.5
            ) * 40

        else:

            self.target_pupil = 0

        self.pupil += (
            self.target_pupil - self.pupil
        ) * 0.08

        self.screen.fill(
            (220, 235, 255)
        )

        self.draw_face()

        self.draw_eye(
            350,
            280
        )

        self.draw_eye(
            650,
            280
        )

        self.draw_mouth()

        font = pygame.font.SysFont(
            None,
            42
        )

        if self.person_count == 0:

            text = "Waiting for visitor"

        elif self.person_count == 1:

            text = "Hello Visitor!"

        else:

            text = "Hello Everyone!"

        surface = font.render(
            text,
            True,
            (50,80,150)
        )

        rect = surface.get_rect(
            center=(500,170)
        )

        self.screen.blit(
            surface,
            rect
        )

        pygame.display.flip()


def main(args=None):

    rclpy.init(args=args)

    node = RobotFace()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
