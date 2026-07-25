#!/usr/bin/env python3

import json
import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk

from PIL import Image, ImageTk

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge


class MoldDetectorGUI(Node):
    def __init__(self, root):
        super().__init__('mold_detector_gui')

        self.root = root
        self.root.title("PUSANTARA CaveCare - Green Mold Detector")
        self.root.geometry("1180x720")

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('min_area', 800)
        self.declare_parameter('warning_percent', 1.0)
        self.declare_parameter('danger_percent', 4.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.min_area_default = int(self.get_parameter('min_area').value)
        self.warning_percent = float(self.get_parameter('warning_percent').value)
        self.danger_percent = float(self.get_parameter('danger_percent').value)

        self.bridge = CvBridge()

        self.latest_debug_frame = None
        self.latest_mask = None
        self.latest_status = "WAITING"
        self.latest_green_percent = 0.0
        self.latest_patch_count = 0

        self.create_gui()

        self.sub = self.create_subscription(
            RosImage,
            self.image_topic,
            self.image_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            String,
            '/mold/detections',
            10
        )

        self.debug_pub = self.create_publisher(
            RosImage,
            '/mold/debug_image',
            10
        )

        self.get_logger().info(f"Green mold detector GUI started.")
        self.get_logger().info(f"Subscribing to: {self.image_topic}")

    def create_gui(self):
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        title = ttk.Label(
            main_frame,
            text="PUSANTARA CaveCare - Green Mold / Moss Detection",
            font=("Arial", 18, "bold")
        )
        title.pack(pady=5)

        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=5)

        self.status_label = tk.Label(
            status_frame,
            text="STATUS: WAITING",
            font=("Arial", 22, "bold"),
            bg="gray",
            fg="white",
            width=22
        )
        self.status_label.pack(side=tk.LEFT, padx=10)

        self.info_label = ttk.Label(
            status_frame,
            text="Green Area: 0.00% | Patches: 0",
            font=("Arial", 16)
        )
        self.info_label.pack(side=tk.LEFT, padx=20)

        image_frame = ttk.Frame(main_frame)
        image_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        left_frame = ttk.LabelFrame(image_frame, text="Live Camera + Detection Box")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        right_frame = ttk.LabelFrame(image_frame, text="Green Mask")
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        self.camera_label = ttk.Label(left_frame)
        self.camera_label.pack(fill=tk.BOTH, expand=True)

        self.mask_label = ttk.Label(right_frame)
        self.mask_label.pack(fill=tk.BOTH, expand=True)

        control_frame = ttk.LabelFrame(main_frame, text="Detection Tuning")
        control_frame.pack(fill=tk.X, pady=5)

        self.lower_h = tk.IntVar(value=35)
        self.upper_h = tk.IntVar(value=90)
        self.lower_s = tk.IntVar(value=40)
        self.lower_v = tk.IntVar(value=30)
        self.min_area = tk.IntVar(value=self.min_area_default)

        self.add_slider(control_frame, "Lower H", self.lower_h, 0, 179, 0)
        self.add_slider(control_frame, "Upper H", self.upper_h, 0, 179, 1)
        self.add_slider(control_frame, "Lower S", self.lower_s, 0, 255, 2)
        self.add_slider(control_frame, "Lower V", self.lower_v, 0, 255, 3)
        self.add_slider(control_frame, "Min Area", self.min_area, 50, 5000, 4)

    def add_slider(self, parent, label, variable, min_value, max_value, row):
        ttk.Label(parent, text=label, width=12).grid(row=row, column=0, padx=5, pady=2)

        slider = ttk.Scale(
            parent,
            from_=min_value,
            to=max_value,
            orient=tk.HORIZONTAL,
            variable=variable,
            length=350
        )
        slider.grid(row=row, column=1, padx=5, pady=2)

        value_label = ttk.Label(parent, textvariable=variable, width=8)
        value_label.grid(row=row, column=2, padx=5, pady=2)

    def classify_status(self, green_percent):
        if green_percent >= self.danger_percent:
            return "DANGER"
        elif green_percent >= self.warning_percent:
            return "WARNING"
        else:
            return "SAFE"

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        debug_frame, mask, result = self.detect_green(frame)

        self.latest_debug_frame = debug_frame
        self.latest_mask = mask
        self.latest_status = result["status"]
        self.latest_green_percent = result["green_percent"]
        self.latest_patch_count = result["patch_count"]

        self.detection_pub.publish(String(data=json.dumps(result)))

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Debug image publish error: {e}")

    def detect_green(self, frame):
        debug_frame = frame.copy()
        height, width = frame.shape[:2]
        total_pixels = width * height

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_green = np.array([
            int(self.lower_h.get()),
            int(self.lower_s.get()),
            int(self.lower_v.get())
        ])

        upper_green = np.array([
            int(self.upper_h.get()),
            255,
            255
        ])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detected_patches = []
        green_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < int(self.min_area.get()):
                continue

            x, y, w, h = cv2.boundingRect(contour)
            green_area += area

            cx = x + w // 2
            cy = y + h // 2

            detected_patches.append({
                "x": int(x),
                "y": int(y),
                "w": int(w),
                "h": int(h),
                "center_x": int(cx),
                "center_y": int(cy),
                "area": float(area)
            })

            # Green bounding box on detected mold/moss
            cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

            cv2.putText(
                debug_frame,
                "POSSIBLE MOLD/MOSS",
                (x, max(y - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            cv2.circle(debug_frame, (cx, cy), 5, (0, 255, 0), -1)

        green_percent = (green_area / total_pixels) * 100.0
        status = self.classify_status(green_percent)

        if status == "SAFE":
            status_color = (0, 255, 0)
        elif status == "WARNING":
            status_color = (0, 255, 255)
        else:
            status_color = (0, 0, 255)

        # Top information panel
        cv2.rectangle(debug_frame, (0, 0), (width, 95), (0, 0, 0), -1)

        cv2.putText(
            debug_frame,
            f"STATUS: {status}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            status_color,
            3
        )

        cv2.putText(
            debug_frame,
            f"Green Area: {green_percent:.2f}% | Patches: {len(detected_patches)}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2
        )

        result = {
            "status": status,
            "green_percent": round(green_percent, 2),
            "patch_count": len(detected_patches),
            "patches": detected_patches
        }

        return debug_frame, mask, result

    def update_gui(self):
        if self.latest_debug_frame is not None:
            camera_img = cv2.cvtColor(self.latest_debug_frame, cv2.COLOR_BGR2RGB)
            camera_img = cv2.resize(camera_img, (560, 420))

            pil_img = Image.fromarray(camera_img)
            tk_img = ImageTk.PhotoImage(image=pil_img)

            self.camera_label.configure(image=tk_img)
            self.camera_label.image = tk_img

        if self.latest_mask is not None:
            mask_rgb = cv2.cvtColor(self.latest_mask, cv2.COLOR_GRAY2RGB)
            mask_rgb = cv2.resize(mask_rgb, (560, 420))

            pil_mask = Image.fromarray(mask_rgb)
            tk_mask = ImageTk.PhotoImage(image=pil_mask)

            self.mask_label.configure(image=tk_mask)
            self.mask_label.image = tk_mask

        self.info_label.configure(
            text=f"Green Area: {self.latest_green_percent:.2f}% | Patches: {self.latest_patch_count}"
        )

        if self.latest_status == "SAFE":
            self.status_label.configure(text="STATUS: SAFE", bg="green", fg="white")
        elif self.latest_status == "WARNING":
            self.status_label.configure(text="STATUS: WARNING", bg="orange", fg="black")
        elif self.latest_status == "DANGER":
            self.status_label.configure(text="STATUS: DANGER", bg="red", fg="white")
        else:
            self.status_label.configure(text="STATUS: WAITING", bg="gray", fg="white")

        self.root.after(30, self.update_gui)


def main(args=None):
    rclpy.init(args=args)

    root = tk.Tk()
    node = MoldDetectorGUI(root)

    def ros_spin():
        rclpy.spin_once(node, timeout_sec=0.001)
        root.after(10, ros_spin)

    def on_close():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    root.after(10, ros_spin)
    root.after(30, node.update_gui)

    root.mainloop()


if __name__ == '__main__':
    main()
