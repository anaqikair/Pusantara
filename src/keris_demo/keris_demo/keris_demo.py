import cv2
import mediapipe as mp
import numpy as np
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

# =====================================================
# CONFIG
# =====================================================

KERIS_FILE = os.path.join(
    os.path.dirname(__file__),
    "keris.png"
)

SHOW_LANDMARKS = False
SHOW_BOUNDING_BOX = False
SHOW_GRIP_POINT = True
SHOW_DEBUG_TEXT = True

# Only render the keris once the hand actually forms a grip/fist.
# Set to False to always render it attached to the hand (old behavior).
REQUIRE_FIST_TO_HOLD = False

# Overall size of the keris relative to palm width. Higher = bigger.
KERIS_SIZE_MULTIPLIER = 5

# Curl ratio = (fingertip-to-wrist distance) / (knuckle-to-wrist distance).
# Open hand ~1.5-2.0, closed fist usually drops below 1.0.
# Hysteresis avoids flicker right at the boundary.
GRIP_ENTER_THRESHOLD = 0.95
GRIP_EXIT_THRESHOLD = 1.15

# How much the grip point is pulled toward the wrist, away from the
# raw index/pinky knuckle midpoint, to sit closer to the center of a
# closed fist (0 = knuckle midpoint, 1 = wrist).
GRIP_BLEND = 0.15

SMOOTHING = 0.75

WINDOW_NAME = "PUSANTARA AR Keris"

# =====================================================
# LOAD KERIS
# =====================================================

keris_original = cv2.imread(
    KERIS_FILE,
    cv2.IMREAD_UNCHANGED
)

if keris_original is None:
    raise Exception(
        f"Could not load {KERIS_FILE}"
    )

if keris_original.shape[2] != 4:
    raise Exception(
        "Keris image must be RGBA PNG"
    )

ORIG_H, ORIG_W = keris_original.shape[:2]


def detect_handle_anchor(image):
    """
    Auto-detects where a hand should grip the keris, directly from the
    PNG's alpha channel. Works for any keris-shaped image: handle/pommel
    at the top, a flared guard, then a blade extending down.

    Strategy: the guard is the widest part of the object near the top.
    The grip point sits a little above the guard, inside the handle,
    which is where a closed fist naturally lands. We also return the
    blade direction (anchor -> tip) in the UN-rotated image, so later
    we know exactly how far to rotate to match the hand's direction.
    """

    alpha = image[:, :, 3]
    h, w = alpha.shape

    rows_with_content = np.where(alpha.max(axis=1) > 10)[0]
    top = int(rows_with_content.min())
    bottom = int(rows_with_content.max())

    widths = np.zeros(h, dtype=int)
    centers = np.zeros(h, dtype=float)

    for y in range(top, bottom + 1):
        xs = np.where(alpha[y] > 10)[0]
        widths[y] = xs.max() - xs.min() + 1
        centers[y] = (xs.min() + xs.max()) / 2.0

    # Guard is assumed to be the widest cross-section in the top 40%
    # of the object (handle/pommel + guard region).
    search_end = top + int(0.4 * (bottom - top))
    guard_row = top + int(np.argmax(widths[top:search_end + 1]))

    # Anchor sits 30% of the way up from the guard toward the top of
    # the handle -- inside the grip area, just above the guard.
    anchor_y = int(guard_row - 0.3 * (guard_row - top))
    anchor_x = float(centers[anchor_y])

    tip_x = float(centers[bottom])
    tip_y = float(bottom)

    blade_angle = math.degrees(
        math.atan2(tip_y - anchor_y, tip_x - anchor_x)
    )

    return anchor_x, float(anchor_y), blade_angle


ANCHOR_X, ANCHOR_Y, ORIG_BLADE_ANGLE = detect_handle_anchor(keris_original)

# =====================================================
# MEDIAPIPE
# =====================================================

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

# =====================================================
# HELPERS
# =====================================================

def rotate_image_and_point(image, angle, point):
    """
    Rotates `image` by `angle` degrees (same expanding-canvas behavior
    as before), and also carries a single (x, y) point through the
    exact same transform. This is what lets the grip anchor stay
    pinned under the hand at every rotation, instead of drifting.
    """

    h, w = image.shape[:2]

    center = (w / 2, h / 2)

    matrix = cv2.getRotationMatrix2D(
        center,
        angle,
        1.0
    )

    cos = abs(matrix[0, 0])
    sin = abs(matrix[0, 1])

    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))

    matrix[0, 2] += (new_w / 2) - center[0]
    matrix[1, 2] += (new_h / 2) - center[1]

    rotated = cv2.warpAffine(
        image,
        matrix,
        (new_w, new_h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0, 0)
    )

    px, py = point

    new_x = matrix[0, 0] * px + matrix[0, 1] * py + matrix[0, 2]
    new_y = matrix[1, 0] * px + matrix[1, 1] * py + matrix[1, 2]

    return rotated, (new_x, new_y)


def overlay_png(background, overlay, x, y):

    bg_h, bg_w = background.shape[:2]
    h, w = overlay.shape[:2]

    if x >= bg_w or y >= bg_h:
        return

    if x + w <= 0 or y + h <= 0:
        return

    x1 = max(0, x)
    y1 = max(0, y)

    x2 = min(bg_w, x + w)
    y2 = min(bg_h, y + h)

    overlay_x1 = x1 - x
    overlay_y1 = y1 - y

    overlay_x2 = overlay_x1 + (x2 - x1)
    overlay_y2 = overlay_y1 + (y2 - y1)

    overlay_crop = overlay[
        overlay_y1:overlay_y2,
        overlay_x1:overlay_x2
    ]

    alpha = overlay_crop[:, :, 3] / 255.0

    for c in range(3):

        background[y1:y2, x1:x2, c] = (
            alpha * overlay_crop[:, :, c]
            +
            (1 - alpha) *
            background[y1:y2, x1:x2, c]
        )


# =====================================================
# NODE
# =====================================================

class KerisNode(Node):

    def __init__(self):
        super().__init__('keris_node')

        self.bridge = CvBridge()

        # Per-frame smoothing / grip state now lives on the node
        # instance instead of as module globals, since the main loop
        # is no longer a single persistent `while True:` block but a
        # callback that fires once per incoming frame.
        self.smooth_x = None
        self.smooth_y = None
        self.smooth_angle = None
        self.smooth_ratio = None

        self.is_gripping = False

        self.prev_time = time.time()

        # GUI window -- opens automatically as soon as the node starts,
        # no separate viewer process needed.
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            '/keris/image',
            10
        )

        self.get_logger().info("Keris Node Started")

    # -------------------------------------------------
    # MAIN PROCESSING (was the body of `while True:`)
    # -------------------------------------------------

    def process_frame(self, frame):

        frame = cv2.flip(frame, 1)

        h, w, _ = frame.shape

        rgb = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        results = hands.process(rgb)

        if results.multi_hand_landmarks:

            for hand in results.multi_hand_landmarks:

                if SHOW_LANDMARKS:

                    mp_draw.draw_landmarks(
                        frame,
                        hand,
                        mp_hands.HAND_CONNECTIONS
                    )

                points = []

                for lm in hand.landmark:

                    px = int(lm.x * w)
                    py = int(lm.y * h)

                    points.append((px, py))

                xs = [p[0] for p in points]
                ys = [p[1] for p in points]

                if SHOW_BOUNDING_BOX:

                    cv2.rectangle(
                        frame,
                        (min(xs), min(ys)),
                        (max(xs), max(ys)),
                        (255, 0, 0),
                        2
                    )

                # ---------------------------------
                # LANDMARKS
                # ---------------------------------

                wrist = points[0]

                index_mcp = points[5]
                middle_mcp = points[9]
                ring_mcp = points[13]
                pinky_mcp = points[17]

                # ---------------------------------
                # GRIP / FIST DETECTION
                # ---------------------------------
                # Rotation-invariant: compares each fingertip's distance
                # from the wrist to its own knuckle's distance from the
                # wrist. Curled fingers pull the tip back toward the
                # wrist regardless of how the hand is angled on screen.

                finger_pairs = [
                    (8, 5),    # index tip, index mcp
                    (12, 9),   # middle tip, middle mcp
                    (16, 13),  # ring tip, ring mcp
                    (20, 17),  # pinky tip, pinky mcp
                ]

                wrist_np = np.array(wrist, dtype=float)

                ratios = []

                for tip_idx, mcp_idx in finger_pairs:

                    tip_np = np.array(points[tip_idx], dtype=float)
                    mcp_np = np.array(points[mcp_idx], dtype=float)

                    d_tip = np.linalg.norm(tip_np - wrist_np)
                    d_mcp = np.linalg.norm(mcp_np - wrist_np)

                    ratios.append(d_tip / max(d_mcp, 1e-3))

                curl_ratio = float(np.mean(ratios))

                if self.smooth_ratio is None:
                    self.smooth_ratio = curl_ratio
                else:
                    self.smooth_ratio = (
                        SMOOTHING * self.smooth_ratio
                        +
                        (1 - SMOOTHING) * curl_ratio
                    )

                if not self.is_gripping:
                    if self.smooth_ratio < GRIP_ENTER_THRESHOLD:
                        self.is_gripping = True
                else:
                    if self.smooth_ratio > GRIP_EXIT_THRESHOLD:
                        self.is_gripping = False

                render_keris = self.is_gripping or not REQUIRE_FIST_TO_HOLD

                # ---------------------------------
                # GRIP POINT
                # ---------------------------------
                # Blended slightly toward the wrist so it sits closer to
                # the center of a closed fist rather than the open-hand
                # knuckle line.

                raw_grip_x = (index_mcp[0] + pinky_mcp[0]) / 2
                raw_grip_y = (index_mcp[1] + pinky_mcp[1]) / 2

                grip_x = raw_grip_x * (1 - GRIP_BLEND) + wrist[0] * GRIP_BLEND
                grip_y = raw_grip_y * (1 - GRIP_BLEND) + wrist[1] * GRIP_BLEND

                # ---------------------------------
                # HAND ANGLE
                # ---------------------------------

                dx = middle_mcp[0] - wrist[0]
                dy = middle_mcp[1] - wrist[1]

                angle = math.degrees(
                    math.atan2(dy, dx)
                )

                # ---------------------------------
                # SMOOTHING
                # ---------------------------------

                if self.smooth_x is None:

                    self.smooth_x = grip_x
                    self.smooth_y = grip_y
                    self.smooth_angle = angle

                else:

                    self.smooth_x = (
                        SMOOTHING * self.smooth_x
                        +
                        (1 - SMOOTHING) * grip_x
                    )

                    self.smooth_y = (
                        SMOOTHING * self.smooth_y
                        +
                        (1 - SMOOTHING) * grip_y
                    )

                    self.smooth_angle = (
                        SMOOTHING * self.smooth_angle
                        +
                        (1 - SMOOTHING) * angle
                    )

                grip_x = int(self.smooth_x)
                grip_y = int(self.smooth_y)

                angle = self.smooth_angle

                # ---------------------------------
                # PALM WIDTH
                # ---------------------------------

                palm_width = int(
                    np.linalg.norm(
                        np.array(index_mcp)
                        -
                        np.array(pinky_mcp)
                    )
                )

                palm_width = max(
                    palm_width,
                    60
                )

                if render_keris:

                    # ---------------------------------
                    # SCALE KERIS (image + anchor together)
                    # ---------------------------------

                    keris_width = int(
                        palm_width * KERIS_SIZE_MULTIPLIER
                    )

                    aspect = ORIG_H / ORIG_W

                    keris_height = int(
                        keris_width * aspect
                    )

                    keris_scaled = cv2.resize(
                        keris_original,
                        (keris_width, keris_height),
                        interpolation=cv2.INTER_AREA
                    )

                    sx = keris_width / ORIG_W
                    sy = keris_height / ORIG_H

                    scaled_anchor = (ANCHOR_X * sx, ANCHOR_Y * sy)

                    # ---------------------------------
                    # ROTATE (image + anchor together)
                    # ---------------------------------
                    # theta is solved so that, after rotation, the
                    # anchor->tip direction exactly equals the hand's
                    # angle -- not an approximation.

                    theta = ORIG_BLADE_ANGLE - angle

                    keris_rotated, rotated_anchor = rotate_image_and_point(
                        keris_scaled,
                        theta,
                        scaled_anchor
                    )

                    # ---------------------------------
                    # PLACE: pin the anchor exactly under the grip point
                    # ---------------------------------

                    x = int(grip_x - rotated_anchor[0])
                    y = int(grip_y - rotated_anchor[1])

                    overlay_png(
                        frame,
                        keris_rotated,
                        x,
                        y
                    )

                # ---------------------------------
                # DEBUG
                # ---------------------------------

                if SHOW_GRIP_POINT:

                    grip_color = (0, 255, 0) if self.is_gripping else (0, 165, 255)

                    cv2.circle(
                        frame,
                        (grip_x, grip_y),
                        8,
                        grip_color,
                        -1
                    )

                if SHOW_DEBUG_TEXT:

                    cv2.putText(
                        frame,
                        f"Angle: {int(angle)}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        frame,
                        f"Palm Width: {palm_width}",
                        (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2
                    )

                    grip_label = "HOLDING" if self.is_gripping else "OPEN"

                    cv2.putText(
                        frame,
                        f"Grip: {grip_label} ({self.smooth_ratio:.2f})",
                        (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0) if self.is_gripping else (0, 165, 255),
                        2
                    )

        else:

            self.is_gripping = False
            self.smooth_ratio = None

        # =================================================
        # FPS
        # =================================================

        current = time.time()

        fps = 1 / max(
            current - self.prev_time,
            0.0001
        )

        self.prev_time = current

        cv2.putText(
            frame,
            f"FPS: {int(fps)}",
            (20, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        return frame

    # -------------------------------------------------
    # ROS CALLBACK
    # -------------------------------------------------

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        frame = self.process_frame(frame)

        out_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        out_msg.header = msg.header

        self.publisher.publish(out_msg)

        # ---------------------------------
        # GUI DISPLAY
        # ---------------------------------
        # Same window the original script showed via cv2.imshow inside
        # its while-True loop -- here it just gets called once per
        # incoming frame instead. waitKey both pumps the window's
        # event loop and lets ESC close everything down.

        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1)

        if key == 27:
            self.get_logger().info("ESC pressed, shutting down")
            cv2.destroyAllWindows()
            rclpy.shutdown()


# =====================================================
# MAIN
# =====================================================

def main(args=None):

    rclpy.init(args=args)

    node = KerisNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
