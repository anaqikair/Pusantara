#!/usr/bin/env python3
"""
PUSANTARA Mission Control - Museum Demo Loop
==============================================
Rewritten control node for the pusantara_demo_loop package.

Only this file (plus setup.py/package.xml if needed) should be touched.
Nav2, SLAM, maps, behavior trees, nav2_params.yaml, robot.launch.py and
navigation.launch.py are NOT modified or recreated by this script.

Fixes vs previous version:
  1. Window close no longer freezes the GUI. Shutdown (stopping every
     launched process, which can each take up to ~1.5s) now happens on a
     background thread, with a hard watchdog fallback (os._exit) in case
     something refuses to die.
  2. New color palette (navy / ivory / gold) with real contrast between
     neutral, success, warning and danger states, applied consistently
     across every panel instead of the previous flat brown-on-black look.
  3. Proper flow: tabbed UI (Setup / Waypoints / Visitor Demo / Logs),
     an explicit state machine (OFFLINE -> ... -> APP_RUNNING -> READY),
     multi-app waypoints with auto-launch vs visitor-select modes, and
     always-on background systems (AprilTag explainer, robot face, TTS,
     camera, YOLO, visitor greeter) that keep running no matter what
     app/state the visitor flow is in.
  4. BUGFIX: destination "Go Here" buttons were previously stored inside
     the waypoint's own data dict (wp["_btn"] = btn). That dict is the
     same object passed all the way through send_goal() -> result_cb(),
     where it gets json.dumps()'d on arrival. A live tkinter Button is
     not JSON-serializable, so json.dumps() silently raised inside the
     rclpy callback thread on every single arrival, before the "arrived"
     UI event ever reached the GUI thread. Net effect: the robot arrived,
     spoke the arrival line, and then the app-selection buttons / auto
     launch never fired for ANY waypoint or app. Fixed by keeping button
     widgets in a separate self.dest_buttons dict keyed by waypoint id,
     never mixed into the waypoint data dict, plus a defensive filter +
     try/except around the arrival JSON payload as a second safety net.
"""

import os
import math
import json
import time
import signal
import subprocess
import threading
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter.scrolledtext import ScrolledText

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ============================================================
# PATHS
# ============================================================

HOME = Path.home()
WS = HOME / "vectra_ws"
FEATURE_BASE = HOME / "Desktop" / "feature"

ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "1")

DEFAULT_MAP = WS / "maps" / "pusantara_map1.yaml"
DEFAULT_NAV2_PARAMS = WS / "src" / "pusantara_navigation" / "config" / "nav2_params.yaml"
WAYPOINT_FILE = WS / "maps" / "pusantara_demo_waypoints.json"
HOME_POSITION_FILE = WS / "maps" / "pusantara_home_position.json"

LOG_DIR = Path("/tmp/pusantara_demo_loop")
LOG_DIR.mkdir(parents=True, exist_ok=True)

GREET_COOLDOWN_SEC = 30.0
AUTO_APP_LAUNCH_DELAY_MS = 1500
SHUTDOWN_WATCHDOG_SEC = 8.0

# Same default covariance RViz2's "2D Pose Estimate" tool uses, so publishing
# a home/initial pose from the GUI behaves the same as doing it by hand.
INITIAL_POSE_COVARIANCE = [
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787,
]


# ============================================================
# COLOR PALETTE
# ============================================================

BG = "#171522"          # app background - deep navy/plum, not flat black
PANEL_BG = "#211E30"     # card/panel background
PANEL_BORDER = "#332F49"
TEXT = "#F2ECDD"         # warm ivory text
MUTED_TEXT = "#A79FC0"
ACCENT = "#E3B04B"       # gold - primary action / branding
ACCENT_DARK = "#B8862F"
SUCCESS = "#4CAF7D"
WARNING = "#E3A23D"
DANGER = "#E0524B"
INFO_BG = "#262238"

STATE_COLORS = {
    "OFFLINE": MUTED_TEXT,
    "BASE_STARTED": WARNING,
    "NAV2_STARTED": WARNING,
    "LOCALIZED": WARNING,
    "READY": SUCCESS,
    "VISITOR_DETECTED": ACCENT,
    "GREETED": ACCENT,
    "DESTINATION_SELECTED": ACCENT,
    "NAVIGATING": WARNING,
    "ARRIVED": SUCCESS,
    "APP_SELECTION": ACCENT,
    "APP_RUNNING": SUCCESS,
    "ESTOP": DANGER,
}

FONT_FAMILY = "Helvetica"


# ============================================================
# APPLICATION REGISTRY
# ============================================================
# Selectable heritage experiences that can be assigned to waypoints.
# AprilTag Explainer / Robot Face are NOT here - they are always-on
# background systems, never tied to waypoint/app selection.

APP_REGISTRY = {
    "AR Keris": {
        "cmd": "ros2 run keris_demo keris_demo",
        "cwd": None,
        "launch_type": "ros2_run",
        "notes": "",
    },
    "Food Quiz": {
        "cmd": "ros2 run food_quiz food_quiz",
        "cwd": None,
        "launch_type": "ros2_run",
        "notes": "",
    },
    "Batik Designer": {
        "cmd": "ros2 run batik_designer batik_designer",
        "cwd": None,
        "launch_type": "ros2_run",
        "notes": "",
    },
    "Cultural Performance Panel": {
        "cmd": 'python3 "performance_panel.py"',
        "cwd": FEATURE_BASE / "cultural_performance" / "admin_console",
        "launch_type": "python_file",
        "notes": "",
    },
    "AR Kompang": {
        "cmd": 'python3 "ar_kompang.py"',
        "cwd": FEATURE_BASE / "ar_kompang-20260702T040102Z-3-001" / "ar_kompang",
        "launch_type": "python_file",
        "notes": "",
    },
    "Pusantara Food GUI": {
        "cmd": 'python3 "main.py"',
        "cwd": FEATURE_BASE / "pusantara_food_gui",
        "launch_type": "python_file",
        "notes": "",
    },
    "VR Museum": {
        "cmd": 'python3 "main.py"',
        "cwd": FEATURE_BASE / "pusantara_vr_museum_v2-20260702T040415Z-3-001" / "pusantara_vr_museum_v2",
        "launch_type": "python_file",
        "notes": "",
    },
    "Sumazau Tutorial": {
        "cmd": 'python3 "sumazau_tutorial.py"',
        "cwd": FEATURE_BASE / "Sumazau Tutorial",
        "launch_type": "python_file",
        "notes": "",
    },
}

SELECTABLE_APPS = list(APP_REGISTRY.keys())

# Core movement pipeline - must always be present for Nav2 to move the robot.
CORE_BASE_PROCESSES = [
    ("Core Robot Bringup", "ros2 launch pusantara_bringup robot.launch.py", None),
    ("cmd_vel Bridge", "ros2 run cmd_vel_bridge cmd_vel_to_robot_command", None),
]

# Always-on interaction systems - started once, never stopped/relaunched by
# app selection. AprilTag explainer + heritage_explainer live here.
ALWAYS_ON_PROCESSES = [
    ("Camera", "ros2 run camera_node webcam_pub", None),
    ("YOLO Detector", "ros2 run vectra_yolo yolo_detector --ros-args -p debug:=false -p confidence_threshold:=0.6", None),
    ("TTS", "ros2 run tts_node tts_node", None),
    ("Visitor Greeter", "ros2 run visitor_greeter visitor_greeter", None),
    ("Robot Face", "ros2 run robot_face face_node", None),
    ("AprilTag Detector", "ros2 run apriltag_detector apriltag_detector --ros-args -p show_ui:=true", None),
    ("Heritage Explainer", "ros2 run heritage_explainer heritage_explainer", None),
]


# ============================================================
# STATE MACHINE
# ============================================================

class S:
    OFFLINE = "OFFLINE"
    BASE_STARTED = "BASE_STARTED"
    NAV2_STARTED = "NAV2_STARTED"
    LOCALIZED = "LOCALIZED"
    READY = "READY"
    VISITOR_DETECTED = "VISITOR_DETECTED"
    GREETED = "GREETED"
    DESTINATION_SELECTED = "DESTINATION_SELECTED"
    NAVIGATING = "NAVIGATING"
    ARRIVED = "ARRIVED"
    APP_SELECTION = "APP_SELECTION"
    APP_RUNNING = "APP_RUNNING"
    ESTOP = "ESTOP"


# ============================================================
# PROCESS MANAGEMENT
# ============================================================

def source_prefix():
    return (
        "source /opt/ros/humble/setup.bash && "
        f"source {WS}/install/setup.bash && "
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
    )


def safe_name(name):
    return "".join(c if c.isalnum() else "_" for c in name.lower()).strip("_")


class ManagedProcess:
    def __init__(self, name, cmd, cwd=None):
        self.name = name
        self.cmd = cmd
        self.cwd = cwd
        self.proc = None
        self.log_file = LOG_DIR / f"{safe_name(name)}.log"
        self.error = None

    def start(self):
        if self.is_running():
            return

        shell_cmd = self.cmd
        if self.cwd:
            shell_cmd = f'cd "{self.cwd}" && {self.cmd}'

        full_cmd = source_prefix() + shell_cmd

        try:
            log = open(self.log_file, "a", buffering=1)
            log.write("\n\n==============================\n")
            log.write(f"START {self.name} at {time.ctime()}\n")
            log.write(f"CMD: {full_cmd}\n")
            log.write("==============================\n")

            self.proc = subprocess.Popen(
                ["bash", "-lc", full_cmd],
                stdout=log,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            self.error = None
        except Exception as e:
            self.error = str(e)

    def stop(self):
        """Stop the process. Safe to call from a background thread."""
        if not self.is_running():
            return

        try:
            pgid = os.getpgid(self.proc.pid)
        except Exception:
            return

        for sig, wait_s in [(signal.SIGINT, 0.6), (signal.SIGTERM, 0.4), (signal.SIGKILL, 0.0)]:
            try:
                os.killpg(pgid, sig)
            except Exception:
                return
            if wait_s:
                deadline = time.time() + wait_s
                while time.time() < deadline:
                    if self.proc.poll() is not None:
                        return
                    time.sleep(0.05)
            if self.proc.poll() is not None:
                return

    def is_running(self):
        return self.proc is not None and self.proc.poll() is None

    def status_text(self):
        if self.error:
            return "ERROR"
        if self.is_running():
            return "RUNNING"
        if self.proc is not None:
            return "STOPPED"
        return "NOT STARTED"


# ============================================================
# ROS NODE
# ============================================================

def yaw_to_quaternion(yaw_rad):
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DemoRosNode(Node):
    def __init__(self, ui_cb):
        super().__init__("pusantara_demo_loop_v3")

        self.ui_cb = ui_cb
        self.last_clicked_point = None
        self.current_pose = None
        self.current_goal_handle = None

        self.tts_pub = self.create_publisher(String, "/tts_text", 10)
        self.robot_cmd_pub = self.create_publisher(String, "/robot_command", 10)
        self.safe_cmd_pub = self.create_publisher(String, "/safe_robot_command", 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.create_subscription(PointStamped, "/clicked_point", self.clicked_point_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.create_subscription(String, "/robot_command", self.robot_command_cb, 10)
        self.create_subscription(String, "/safe_robot_command", self.safe_robot_command_cb, 10)
        self.create_subscription(String, "/tts_text", self.tts_text_cb, 10)
        self.create_subscription(String, "/yolo/detections", self.yolo_cb, 10)
        self.create_subscription(String, "/apriltag/detections", self.apriltag_cb, 10)

    def ui(self, kind, text):
        if self.ui_cb:
            self.ui_cb(kind, text)

    def clicked_point_cb(self, msg):
        self.last_clicked_point = {"x": msg.point.x, "y": msg.point.y, "z": msg.point.z}
        self.ui("clicked", f"x={msg.point.x:.2f}, y={msg.point.y:.2f}")
        self.ui("log", f"RViz clicked point received: x={msg.point.x:.2f}, y={msg.point.y:.2f}")

    def amcl_pose_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.degrees(quaternion_to_yaw(q))
        self.current_pose = {"x": p.x, "y": p.y, "yaw_deg": yaw}
        self.ui("pose", f"x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.1f}\u00b0")
        self.ui("localized", "")

    def cmd_vel_cb(self, msg):
        self.ui("cmdvel", f"linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")

    def robot_command_cb(self, msg):
        self.ui("robotcmd", msg.data)

    def safe_robot_command_cb(self, msg):
        self.ui("safecmd", msg.data)

    def tts_text_cb(self, msg):
        self.ui("tts", msg.data)

    def yolo_cb(self, msg):
        self.ui("yolo", msg.data)
        if "person" in msg.data.lower():
            self.ui("person", msg.data)

    def apriltag_cb(self, msg):
        self.ui("apriltag", msg.data)

    def speak(self, text):
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        self.ui("log", f"TTS -> {text}")

    def publish_initial_pose(self, x, y, yaw_deg):
        """Publish a PoseWithCovarianceStamped to /initialpose, equivalent to
        using RViz2's '2D Pose Estimate' tool at the given home coordinates."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(math.radians(float(yaw_deg)))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = list(INITIAL_POSE_COVARIANCE)

        self.initialpose_pub.publish(msg)
        self.ui("log", f"Published /initialpose (home position): x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}")

    def stop_robot(self):
        for pub in [self.robot_cmd_pub, self.safe_cmd_pub]:
            msg = String()
            msg.data = "STOP"
            pub.publish(msg)
        self.ui("log", "STOP sent to /robot_command and /safe_robot_command")

    def cancel_nav(self):
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.ui("log", "Nav2 cancel requested.")
        self.stop_robot()

    def send_goal(self, waypoint):
        threading.Thread(target=self._send_goal_thread, args=(waypoint,), daemon=True).start()

    def _send_goal_thread(self, waypoint):
        name = waypoint["name"]
        x = float(waypoint["x"])
        y = float(waypoint["y"])
        yaw_deg = float(waypoint["yaw_deg"])
        yaw_rad = math.radians(yaw_deg)

        self.ui("nav", "Waiting for Nav2 action server...")
        self.ui("log", f"Preparing Nav2 goal: {name} | x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}")

        if not self.nav_client.wait_for_server(timeout_sec=8.0):
            self.ui("nav", "Nav2 action server NOT available")
            self.ui("log", "ERROR: /navigate_to_pose is not available. Start Nav2 first.")
            self.ui("nav_failed", name)
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(yaw_rad)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.ui("nav", f"Sending goal: {name}")
        self.speak(f"Please follow me. I am going to {name}.")

        future = self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(lambda f: self.goal_response_cb(f, waypoint))

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        try:
            self.ui("nav", f"Navigating... remaining {fb.distance_remaining:.2f} m")
        except Exception:
            self.ui("nav", "Navigating...")

    def goal_response_cb(self, future, waypoint):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.ui("nav", "Goal rejected")
            self.ui("log", f"Nav2 rejected waypoint: {waypoint['name']}")
            self.ui("nav_failed", waypoint["name"])
            return

        self.current_goal_handle = goal_handle
        self.ui("nav", f"Goal accepted: {waypoint['name']}")
        self.ui("log", f"Nav2 accepted waypoint: {waypoint['name']}")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.result_cb(f, waypoint))

    def result_cb(self, future, waypoint):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.ui("nav", f"ARRIVED: {waypoint['name']}")
            # Defensive: waypoint dicts should only ever contain JSON-safe
            # values, but filter out any stray non-serializable keys (e.g.
            # a leftover "_btn" widget reference) so a bad key can never
            # again silently swallow the "arrived" event before it reaches
            # the GUI thread.
            try:
                clean_wp = {k: v for k, v in waypoint.items() if not k.startswith("_")}
                payload = json.dumps(clean_wp)
                self.ui("arrived", payload)
            except Exception as e:
                self.ui("log", f"ERROR building arrived payload for {waypoint.get('name')}: {e}")
            self.speak(f"We have arrived at {waypoint['name']}. Please choose the heritage experience.")
        else:
            self.ui("nav", f"Navigation ended. Status={status}")
            self.ui("log", f"Nav2 ended with status {status}")
            self.ui("nav_failed", waypoint["name"])
            self.speak("Navigation did not complete. Please check the robot position and map.")


# ============================================================
# WAYPOINT EDITOR DIALOG
# ============================================================

class WaypointDialog(tk.Toplevel):
    """Modal dialog for creating/editing a waypoint with multi-app assignment."""

    def __init__(self, parent, title, initial=None, prefill_xyz=None):
        super().__init__(parent)
        self.title(title)
        self.configure(bg=PANEL_BG)
        self.resizable(False, False)
        self.result = None
        self.transient(parent)
        self.grab_set()

        initial = initial or {}
        px, py, pyaw = (prefill_xyz or (initial.get("x", 0.0), initial.get("y", 0.0), initial.get("yaw_deg", 0.0)))

        pad = {"padx": 10, "pady": 4}

        def label(text, row):
            tk.Label(self, text=text, bg=PANEL_BG, fg=TEXT, font=(FONT_FAMILY, 10, "bold")).grid(
                row=row, column=0, sticky="w", **pad
            )

        self.name_var = tk.StringVar(value=initial.get("name", ""))
        self.desc_var = tk.StringVar(value=initial.get("description", ""))
        self.x_var = tk.DoubleVar(value=px)
        self.y_var = tk.DoubleVar(value=py)
        self.yaw_var = tk.DoubleVar(value=pyaw)
        self.mode_var = tk.StringVar(value=initial.get("app_mode", "visitor_select"))

        label("Waypoint name:", 0)
        tk.Entry(self, textvariable=self.name_var, width=36, bg=INFO_BG, fg=TEXT,
                 insertbackground=TEXT, relief="flat").grid(row=0, column=1, columnspan=2, **pad)

        label("Description:", 1)
        tk.Entry(self, textvariable=self.desc_var, width=36, bg=INFO_BG, fg=TEXT,
                 insertbackground=TEXT, relief="flat").grid(row=1, column=1, columnspan=2, **pad)

        label("X:", 2)
        tk.Entry(self, textvariable=self.x_var, width=12, bg=INFO_BG, fg=TEXT,
                 insertbackground=TEXT, relief="flat").grid(row=2, column=1, sticky="w", **pad)

        label("Y:", 3)
        tk.Entry(self, textvariable=self.y_var, width=12, bg=INFO_BG, fg=TEXT,
                 insertbackground=TEXT, relief="flat").grid(row=3, column=1, sticky="w", **pad)

        label("Yaw (deg):", 4)
        tk.Entry(self, textvariable=self.yaw_var, width=12, bg=INFO_BG, fg=TEXT,
                 insertbackground=TEXT, relief="flat").grid(row=4, column=1, sticky="w", **pad)

        label("Assigned apps:", 5)
        app_frame = tk.Frame(self, bg=PANEL_BG)
        app_frame.grid(row=5, column=1, columnspan=2, sticky="w", **pad)

        self.app_vars = {}
        existing_apps = set(initial.get("apps", []))
        for i, app_name in enumerate(SELECTABLE_APPS):
            var = tk.BooleanVar(value=app_name in existing_apps)
            self.app_vars[app_name] = var
            cb = tk.Checkbutton(
                app_frame, text=app_name, variable=var, bg=PANEL_BG, fg=TEXT,
                selectcolor=INFO_BG, activebackground=PANEL_BG, activeforeground=ACCENT,
                anchor="w", font=(FONT_FAMILY, 9),
            )
            cb.grid(row=i // 2, column=i % 2, sticky="w", padx=4, pady=2)

        label("App mode:", 6)
        mode_frame = tk.Frame(self, bg=PANEL_BG)
        mode_frame.grid(row=6, column=1, columnspan=2, sticky="w", **pad)
        tk.Radiobutton(mode_frame, text="Auto-launch (single app)", variable=self.mode_var,
                        value="auto", bg=PANEL_BG, fg=TEXT, selectcolor=INFO_BG,
                        activebackground=PANEL_BG, activeforeground=ACCENT).pack(anchor="w")
        tk.Radiobutton(mode_frame, text="Visitor selects (multiple apps)", variable=self.mode_var,
                        value="visitor_select", bg=PANEL_BG, fg=TEXT, selectcolor=INFO_BG,
                        activebackground=PANEL_BG, activeforeground=ACCENT).pack(anchor="w")

        btn_frame = tk.Frame(self, bg=PANEL_BG)
        btn_frame.grid(row=7, column=0, columnspan=3, pady=(10, 12))

        tk.Button(btn_frame, text="Save", command=self.on_save, bg=ACCENT, fg="#1B1B2F",
                  activebackground=ACCENT_DARK, relief="flat", font=(FONT_FAMILY, 10, "bold"),
                  padx=16, pady=4).pack(side="left", padx=6)
        tk.Button(btn_frame, text="Cancel", command=self.destroy, bg=INFO_BG, fg=TEXT,
                  activebackground=PANEL_BORDER, relief="flat", font=(FONT_FAMILY, 10),
                  padx=16, pady=4).pack(side="left", padx=6)

        self.wait_window(self)

    def on_save(self):
        name = self.name_var.get().strip()
        if not name:
            messagebox.showerror("Missing name", "Waypoint name is required.")
            return

        apps = [a for a, v in self.app_vars.items() if v.get()]
        if not apps:
            messagebox.showerror("No apps assigned", "Select at least one app for this waypoint.")
            return

        mode = self.mode_var.get()
        if mode == "auto" and len(apps) > 1:
            messagebox.showerror(
                "Too many apps for auto mode",
                "Auto-launch mode supports exactly one app. Either pick one app, or switch to "
                "'Visitor selects'.",
            )
            return

        try:
            x = float(self.x_var.get())
            y = float(self.y_var.get())
            yaw = float(self.yaw_var.get())
        except Exception:
            messagebox.showerror("Invalid numbers", "X, Y and yaw must be numeric.")
            return

        self.result = {
            "id": safe_name(name),
            "name": name,
            "description": self.desc_var.get().strip(),
            "x": x,
            "y": y,
            "yaw_deg": yaw,
            "apps": apps,
            "app_mode": mode,
        }
        self.destroy()


# ============================================================
# GUI
# ============================================================

class DemoGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PUSANTARA Mission Control")
        self.root.geometry("1320x820")
        self.root.minsize(1150, 720)
        self.root.configure(bg=BG)

        self.processes = {}          # name -> ManagedProcess (base + always-on + rviz)
        self.current_app_proc = None
        self.current_app_name = None
        self.waypoints = []
        self.dest_buttons = {}       # waypoint id -> tk.Button (kept OUT of waypoint dicts)
        self.last_arrived_waypoint = None
        self.last_greet_time = 0.0
        self.home_position = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}

        self._closing = False
        self._closed = False

        self.state = S.OFFLINE

        self.map_var = tk.StringVar(value=str(DEFAULT_MAP))
        self.yaw_var = tk.DoubleVar(value=0.0)
        self.home_x_var = tk.DoubleVar(value=0.0)
        self.home_y_var = tk.DoubleVar(value=0.0)
        self.home_yaw_var = tk.DoubleVar(value=0.0)

        self.build_style()
        self.build_ui()

        self.ros_node = DemoRosNode(self.ros_ui_callback)
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        self.load_waypoints_silent()
        self.load_home_position_silent()
        self.refresh_map_list()
        self.refresh_waypoint_table()
        self.refresh_destination_buttons()
        self.refresh_process_table()
        self.set_state(S.OFFLINE)
        self.log("Mission control ready.")

        self.root.after(1000, self.periodic_tick)

    # ---------------- Style ----------------

    def build_style(self):
        style = ttk.Style()
        style.theme_use("clam")

        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL_BG)

        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=PANEL_BG, foreground=MUTED_TEXT,
                         padding=(16, 8), font=(FONT_FAMILY, 10, "bold"))
        style.map("TNotebook.Tab",
                  background=[("selected", ACCENT)],
                  foreground=[("selected", "#1B1B2F")])

        style.configure("Panel.TLabelframe", background=PANEL_BG, foreground=ACCENT, borderwidth=1)
        style.configure("Panel.TLabelframe.Label", background=PANEL_BG, foreground=ACCENT,
                         font=(FONT_FAMILY, 11, "bold"))

        style.configure("Title.TLabel", background=BG, foreground=ACCENT, font=(FONT_FAMILY, 22, "bold"))
        style.configure("Sub.TLabel", background=BG, foreground=MUTED_TEXT, font=(FONT_FAMILY, 10))
        style.configure("Info.TLabel", background=PANEL_BG, foreground=TEXT, font=(FONT_FAMILY, 10))
        style.configure("InfoTitle.TLabel", background=PANEL_BG, foreground=MUTED_TEXT,
                         font=(FONT_FAMILY, 9, "bold"))

        style.configure("Big.TButton", font=(FONT_FAMILY, 11, "bold"), padding=8,
                         background=INFO_BG, foreground=TEXT)
        style.map("Big.TButton", background=[("active", PANEL_BORDER)])

        style.configure("Accent.TButton", font=(FONT_FAMILY, 11, "bold"), padding=8,
                         background=ACCENT, foreground="#1B1B2F")
        style.map("Accent.TButton", background=[("active", ACCENT_DARK)])

        style.configure("Danger.TButton", font=(FONT_FAMILY, 11, "bold"), padding=8,
                         background=DANGER, foreground="#1B1B2F")
        style.map("Danger.TButton", background=[("active", "#B33A34")])

        style.configure("Treeview", background=INFO_BG, fieldbackground=INFO_BG,
                         foreground=TEXT, rowheight=24, borderwidth=0)
        style.configure("Treeview.Heading", background=PANEL_BORDER, foreground=TEXT,
                         font=(FONT_FAMILY, 9, "bold"))
        style.map("Treeview", background=[("selected", ACCENT)], foreground=[("selected", "#1B1B2F")])

        style.configure("TCombobox", fieldbackground=INFO_BG, background=INFO_BG, foreground=TEXT)
        style.configure("TEntry", fieldbackground=INFO_BG, foreground=TEXT)

    # ---------------- ROS ----------------

    def spin_ros(self):
        try:
            while rclpy.ok() and not self._closing:
                rclpy.spin_once(self.ros_node, timeout_sec=0.2)
        except Exception as e:
            print(f"ROS spin error: {e}")

    def ros_ui_callback(self, kind, text):
        self.root.after(0, lambda: self.handle_ros_ui(kind, text))

    def handle_ros_ui(self, kind, text):
        if self._closing:
            return
        if kind == "log":
            self.log(text)
        elif kind == "pose":
            self.pose_value.config(text=text)
        elif kind == "localized":
            if self.state in (S.BASE_STARTED, S.NAV2_STARTED):
                self.set_state(S.LOCALIZED)
                self.check_ready()
        elif kind == "clicked":
            self.clicked_value.config(text=text)
        elif kind == "cmdvel":
            self.cmdvel_value.config(text=text)
        elif kind == "robotcmd":
            self.robotcmd_value.config(text=text)
        elif kind == "safecmd":
            self.safecmd_value.config(text=text)
        elif kind == "tts":
            self.tts_value.config(text=text[:80])
        elif kind == "yolo":
            self.yolo_value.config(text=text[:80])
        elif kind == "apriltag":
            self.apriltag_value.config(text=text[:80])
        elif kind == "person":
            self.on_person_detected()
        elif kind == "nav":
            self.nav_value.config(text=text)
            self.log(text)
        elif kind == "arrived":
            try:
                self.last_arrived_waypoint = json.loads(text)
                self.on_arrived()
            except Exception as e:
                self.log(f"ERROR parsing arrived payload: {e}")
        elif kind == "nav_failed":
            if self.state == S.NAVIGATING:
                self.set_state(S.READY)

    def periodic_tick(self):
        if self._closing:
            return
        self.refresh_process_table()
        self.root.after(1500, self.periodic_tick)

    # ---------------- UI BUILD ----------------

    def build_ui(self):
        header = ttk.Frame(self.root)
        header.pack(fill="x", padx=14, pady=(12, 6))

        header_top = ttk.Frame(header)
        header_top.pack(fill="x")

        title_col = ttk.Frame(header_top)
        title_col.pack(side="left", fill="x", expand=True)
        ttk.Label(title_col, text="PUSANTARA MISSION CONTROL", style="Title.TLabel").pack(anchor="w")
        ttk.Label(title_col, text="Preparation \u2192 Setup \u2192 Waypoints \u2192 Live Visitor Demo",
                  style="Sub.TLabel").pack(anchor="w")

        # Always visible on every tab - full teardown without closing the app.
        stop_everything_btn = tk.Button(
            header_top, text="\u26d4  STOP EVERYTHING", command=self.stop_everything,
            bg=DANGER, fg="#1B1B2F", activebackground="#B33A34", relief="flat",
            font=(FONT_FAMILY, 13, "bold"), padx=18, pady=10,
        )
        stop_everything_btn.pack(side="right", padx=(10, 0))

        # State banner
        self.state_banner = tk.Label(
            self.root, text="STATE: OFFLINE", font=(FONT_FAMILY, 13, "bold"),
            bg=MUTED_TEXT, fg="#1B1B2F", pady=6,
        )
        self.state_banner.pack(fill="x", padx=14, pady=(0, 8))

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=14, pady=(0, 6))

        self.tab_setup = ttk.Frame(notebook, style="TFrame")
        self.tab_waypoints = ttk.Frame(notebook, style="TFrame")
        self.tab_visitor = ttk.Frame(notebook, style="TFrame")
        self.tab_logs = ttk.Frame(notebook, style="TFrame")

        notebook.add(self.tab_setup, text="  Setup  ")
        notebook.add(self.tab_waypoints, text="  Waypoint Editor  ")
        notebook.add(self.tab_visitor, text="  Visitor Demo  ")
        notebook.add(self.tab_logs, text="  Logs  ")

        self.build_setup_tab()
        self.build_waypoints_tab()
        self.build_visitor_tab()
        self.build_logs_tab()

        # Persistent mini-log strip at the very bottom, always visible
        bottom = ttk.LabelFrame(self.root, text="Mission Log", style="Panel.TLabelframe")
        bottom.pack(fill="x", padx=14, pady=(0, 10))
        self.log_box = ScrolledText(bottom, height=6, bg="#0F0E17", fg=TEXT,
                                     insertbackground=TEXT, relief="flat", borderwidth=0)
        self.log_box.pack(fill="both", expand=True, padx=6, pady=6)

    # ---- Setup tab ----

    def build_setup_tab(self):
        t = self.tab_setup
        left = ttk.Frame(t)
        left.pack(side="left", fill="y", padx=8, pady=8)
        right = ttk.Frame(t)
        right.pack(side="left", fill="both", expand=True, padx=8, pady=8)

        map_panel = ttk.LabelFrame(left, text="1. Map Selection", style="Panel.TLabelframe")
        map_panel.pack(fill="x", pady=(0, 8))
        self.map_combo = ttk.Combobox(map_panel, textvariable=self.map_var, width=42)
        self.map_combo.pack(fill="x", padx=6, pady=(6, 3))
        row = ttk.Frame(map_panel)
        row.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Button(row, text="Browse", command=self.browse_map).pack(side="left", padx=2)
        ttk.Button(row, text="Refresh", command=self.refresh_map_list).pack(side="left", padx=2)

        sys_panel = ttk.LabelFrame(left, text="2. System Control", style="Panel.TLabelframe")
        sys_panel.pack(fill="x", pady=(0, 8))
        ttk.Button(sys_panel, text="Start Robot Base", style="Big.TButton",
                   command=self.start_robot_base).pack(fill="x", padx=6, pady=3)
        ttk.Button(sys_panel, text="Start Nav2 With Selected Map", style="Big.TButton",
                   command=self.start_nav2).pack(fill="x", padx=6, pady=3)
        ttk.Button(sys_panel, text="Open RViz2", style="Big.TButton",
                   command=self.open_rviz).pack(fill="x", padx=6, pady=3)
        ttk.Button(sys_panel, text="Start Always-On Systems", style="Accent.TButton",
                   command=self.start_always_on).pack(fill="x", padx=6, pady=3)

        home_panel = ttk.LabelFrame(left, text="3. Home / Initial Position", style="Panel.TLabelframe")
        home_panel.pack(fill="x", pady=(0, 8))

        home_grid = ttk.Frame(home_panel)
        home_grid.pack(fill="x", padx=6, pady=(6, 4))
        for i, (label, var) in enumerate([("X:", self.home_x_var), ("Y:", self.home_y_var), ("Yaw\u00b0:", self.home_yaw_var)]):
            ttk.Label(home_grid, text=label, style="Info.TLabel").grid(row=0, column=i * 2, padx=(0, 2), pady=2)
            tk.Entry(home_grid, textvariable=var, width=7, bg=INFO_BG, fg=TEXT,
                     insertbackground=TEXT, relief="flat").grid(row=0, column=i * 2 + 1, padx=(0, 6), pady=2)

        ttk.Button(home_panel, text="Set Home From Current Pose", style="Big.TButton",
                   command=self.set_home_from_current_pose).pack(fill="x", padx=6, pady=2)
        ttk.Button(home_panel, text="Save Home Position", style="Big.TButton",
                   command=self.save_home_position).pack(fill="x", padx=6, pady=2)
        ttk.Button(home_panel, text="Publish as Initial Pose (AMCL)", style="Accent.TButton",
                   command=self.publish_home_as_initial_pose).pack(fill="x", padx=6, pady=2)
        ttk.Button(home_panel, text="Go To Home", style="Big.TButton",
                   command=self.go_to_home).pack(fill="x", padx=6, pady=(2, 6))

        danger_panel = ttk.LabelFrame(left, text="Safety", style="Panel.TLabelframe")
        danger_panel.pack(fill="x")
        ttk.Button(danger_panel, text="Cancel Nav", style="Danger.TButton",
                   command=self.cancel_nav).pack(fill="x", padx=6, pady=3)
        ttk.Button(danger_panel, text="E-STOP", style="Danger.TButton",
                   command=self.e_stop).pack(fill="x", padx=6, pady=3)
        ttk.Button(danger_panel, text="Stop All Processes", style="Danger.TButton",
                   command=self.stop_all_processes).pack(fill="x", padx=6, pady=3)
        ttk.Button(danger_panel, text="Stop All ROS Nodes (System-Wide)", style="Danger.TButton",
                   command=self.stop_all_ros_nodes).pack(fill="x", padx=6, pady=3)

        # Right: readiness + process table
        ready_panel = ttk.LabelFrame(right, text="Readiness", style="Panel.TLabelframe")
        ready_panel.pack(fill="x", pady=(0, 8))
        grid = ttk.Frame(ready_panel)
        grid.pack(fill="x", padx=6, pady=6)
        self.chk_map = self.add_check_row(grid, 0, "Map selected")
        self.chk_base = self.add_check_row(grid, 1, "Robot base running")
        self.chk_nav2 = self.add_check_row(grid, 2, "Nav2 running")
        self.chk_localized = self.add_check_row(grid, 3, "Localized (/amcl_pose received)")
        self.chk_alwayson = self.add_check_row(grid, 4, "Always-on systems running")

        self.ready_label = tk.Label(ready_panel, text="NOT READY", font=(FONT_FAMILY, 14, "bold"),
                                     bg=WARNING, fg="#1B1B2F", pady=6)
        self.ready_label.pack(fill="x", padx=6, pady=(0, 6))

        proc_panel = ttk.LabelFrame(right, text="Process Status", style="Panel.TLabelframe")
        proc_panel.pack(fill="both", expand=True)
        self.proc_tree = ttk.Treeview(proc_panel, columns=("status", "log"), show="tree headings", height=12)
        self.proc_tree.heading("#0", text="PROCESS")
        self.proc_tree.heading("status", text="STATUS")
        self.proc_tree.heading("log", text="LOG FILE")
        self.proc_tree.column("#0", width=220)
        self.proc_tree.column("status", width=110)
        self.proc_tree.column("log", width=420)
        self.proc_tree.pack(fill="both", expand=True, padx=6, pady=6)

    def add_check_row(self, parent, row, text):
        tk.Label(parent, text="\u25cb", bg=PANEL_BG, fg=MUTED_TEXT, font=(FONT_FAMILY, 12)).grid(
            row=row, column=0, sticky="w", padx=(0, 6), pady=2
        )
        lbl = ttk.Label(parent, text=text, style="Info.TLabel")
        lbl.grid(row=row, column=1, sticky="w", pady=2)
        dot = parent.grid_slaves(row=row, column=0)[0]
        return dot

    # ---- Waypoints tab ----

    def build_waypoints_tab(self):
        t = self.tab_waypoints

        hint = ("RViz waypoint method: Open RViz2 \u2192 'Publish Point' tool \u2192 click on the map, "
                "then press 'Add Clicked Point'.")
        ttk.Label(t, text=hint, style="Sub.TLabel").pack(fill="x", padx=8, pady=(8, 4))

        clicked_row = ttk.Frame(t)
        clicked_row.pack(fill="x", padx=8, pady=(0, 6))
        ttk.Label(clicked_row, text="Last RViz click:", style="Sub.TLabel").pack(side="left")
        self.clicked_value = ttk.Label(clicked_row, text="none", style="Sub.TLabel")
        self.clicked_value.pack(side="left", padx=(6, 20))

        btn_row = ttk.Frame(t)
        btn_row.pack(fill="x", padx=8, pady=(0, 6))
        ttk.Button(btn_row, text="Add Clicked Point", style="Accent.TButton",
                   command=self.add_clicked_waypoint).pack(side="left", padx=3)
        ttk.Button(btn_row, text="Add Current Robot Pose", style="Big.TButton",
                   command=self.add_current_pose_waypoint).pack(side="left", padx=3)
        ttk.Button(btn_row, text="Manual Add", style="Big.TButton",
                   command=self.add_manual_waypoint).pack(side="left", padx=3)
        ttk.Button(btn_row, text="Edit Selected", style="Big.TButton",
                   command=self.edit_waypoint).pack(side="left", padx=3)
        ttk.Button(btn_row, text="Delete Selected", style="Danger.TButton",
                   command=self.delete_waypoint).pack(side="left", padx=3)

        table_panel = ttk.LabelFrame(t, text="Waypoints", style="Panel.TLabelframe")
        table_panel.pack(fill="both", expand=True, padx=8, pady=6)

        self.wp_tree = ttk.Treeview(
            table_panel, columns=("x", "y", "yaw", "apps", "mode", "desc"),
            show="headings", height=12,
        )
        for col, width, title in [
            ("x", 70, "X"), ("y", 70, "Y"), ("yaw", 70, "YAW"),
            ("apps", 280, "APPS"), ("mode", 100, "MODE"), ("desc", 260, "DESCRIPTION"),
        ]:
            self.wp_tree.heading(col, text=title)
            self.wp_tree.column(col, width=width)
        self.wp_tree.pack(fill="both", expand=True, padx=6, pady=6)

        save_row = ttk.Frame(t)
        save_row.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Button(save_row, text="Save Waypoints", style="Big.TButton",
                   command=self.save_waypoints).pack(side="left", padx=3)
        ttk.Button(save_row, text="Load Waypoints", style="Big.TButton",
                   command=self.load_waypoints).pack(side="left", padx=3)
        self.wp_file_label = ttk.Label(save_row, text=str(WAYPOINT_FILE), style="Sub.TLabel")
        self.wp_file_label.pack(side="left", padx=12)

    # ---- Visitor tab ----

    def build_visitor_tab(self):
        t = self.tab_visitor

        top = ttk.Frame(t)
        top.pack(fill="x", padx=8, pady=8)

        ttk.Button(top, text="Greet Visitor", style="Accent.TButton",
                   command=self.greet).pack(side="left", padx=3)
        ttk.Button(top, text="Cancel Navigation", style="Danger.TButton",
                   command=self.cancel_nav).pack(side="left", padx=3)
        ttk.Button(top, text="E-STOP", style="Danger.TButton",
                   command=self.e_stop).pack(side="left", padx=3)
        ttk.Button(top, text="Resume from E-STOP", style="Big.TButton",
                   command=self.resume_from_estop).pack(side="left", padx=3)
        ttk.Button(top, text="Stop Current App", style="Big.TButton",
                   command=self.stop_current_app_manual).pack(side="left", padx=3)
        ttk.Button(top, text="Go To Home", style="Big.TButton",
                   command=self.go_to_home).pack(side="left", padx=3)
        ttk.Button(top, text="Again / Next Visitor", style="Accent.TButton",
                   command=self.again).pack(side="right", padx=3)

        info_row = ttk.Frame(t)
        info_row.pack(fill="x", padx=8, pady=(0, 6))
        self.nav_value = self.add_status(info_row, 0, "Navigation", "idle")
        self.current_app_value = self.add_status(info_row, 1, "Current App", "none")
        self.person_value = self.add_status(info_row, 2, "Last Person Detection", "none")

        dest_panel = ttk.LabelFrame(t, text="Choose a Destination", style="Panel.TLabelframe")
        dest_panel.pack(fill="both", expand=True, padx=8, pady=6)
        self.dest_canvas_frame = ttk.Frame(dest_panel)
        self.dest_canvas_frame.pack(fill="both", expand=True, padx=6, pady=6)

        app_panel = ttk.LabelFrame(t, text="Choose an Experience (after arrival)", style="Panel.TLabelframe")
        app_panel.pack(fill="x", padx=8, pady=6)
        self.app_select_frame = ttk.Frame(app_panel)
        self.app_select_frame.pack(fill="x", padx=6, pady=6)

    def add_status(self, parent, col, title, value):
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=col, sticky="nsew", padx=6)
        parent.columnconfigure(col, weight=1)
        ttk.Label(frame, text=title, style="InfoTitle.TLabel").pack(fill="x")
        lbl = tk.Label(frame, text=value, bg=INFO_BG, fg=TEXT, anchor="w", padx=6, pady=3)
        lbl.pack(fill="x")
        return lbl

    # ---- Logs tab ----

    def build_logs_tab(self):
        t = self.tab_logs

        monitor = ttk.LabelFrame(t, text="Live ROS Topic Monitor", style="Panel.TLabelframe")
        monitor.pack(fill="x", padx=8, pady=8)
        grid = ttk.Frame(monitor)
        grid.pack(fill="x", padx=6, pady=6)

        self.pose_value = self.add_status(grid, 0, "/amcl_pose", "waiting...")
        self.cmdvel_value = self.add_status(grid, 1, "/cmd_vel", "waiting...")
        self.robotcmd_value = self.add_status(grid, 2, "/robot_command", "waiting...")
        self.safecmd_value = self.add_status(grid, 3, "/safe_robot_command", "waiting...")

        grid2 = ttk.Frame(monitor)
        grid2.pack(fill="x", padx=6, pady=(0, 6))
        self.tts_value = self.add_status(grid2, 0, "/tts_text", "waiting...")
        self.yolo_value = self.add_status(grid2, 1, "/yolo/detections", "waiting...")
        self.apriltag_value = self.add_status(grid2, 2, "/apriltag/detections", "waiting...")

        btn_row = ttk.Frame(t)
        btn_row.pack(fill="x", padx=8, pady=(0, 6))
        ttk.Button(btn_row, text="Open Log Folder", style="Big.TButton",
                   command=self.open_log_folder).pack(side="left", padx=3)
        ttk.Label(btn_row, text=str(LOG_DIR), style="Sub.TLabel").pack(side="left", padx=10)

        ttk.Label(t, text="Full mission log is shown at the bottom of every tab.",
                  style="Sub.TLabel").pack(anchor="w", padx=8, pady=(4, 0))

    # ---------------- Flow / state machine ----------------

    def set_state(self, new_state):
        self.state = new_state
        color = STATE_COLORS.get(new_state, MUTED_TEXT)
        self.state_banner.config(text=f"STATE: {new_state.replace('_', ' ')}", bg=color)
        self.log(f"State -> {new_state}")

    def check_ready(self):
        base_ok = "Core Robot Bringup" in self.processes and self.processes["Core Robot Bringup"].is_running()
        nav2_ok = "Navigation Stack" in self.processes and self.processes["Navigation Stack"].is_running()
        localized_ok = self.ros_node.current_pose is not None
        alwayson_ok = all(
            name in self.processes and self.processes[name].is_running()
            for name, _, _ in ALWAYS_ON_PROCESSES
        )
        map_ok = Path(self.map_var.get()).exists()

        self.set_check(self.chk_map, map_ok)
        self.set_check(self.chk_base, base_ok)
        self.set_check(self.chk_nav2, nav2_ok)
        self.set_check(self.chk_localized, localized_ok)
        self.set_check(self.chk_alwayson, alwayson_ok)

        all_ok = base_ok and nav2_ok and localized_ok and alwayson_ok and map_ok
        if all_ok:
            self.ready_label.config(text="READY FOR VISITOR", bg=SUCCESS)
            if self.state in (S.OFFLINE, S.BASE_STARTED, S.NAV2_STARTED, S.LOCALIZED):
                self.set_state(S.READY)
        else:
            self.ready_label.config(text="NOT READY", bg=WARNING)

    def set_check(self, dot_widget, ok):
        dot_widget.config(text="\u25cf" if ok else "\u25cb", fg=SUCCESS if ok else MUTED_TEXT)

    def log(self, text):
        stamp = time.strftime("%H:%M:%S")
        self.log_box.insert("end", f"[{stamp}] {text}\n")
        self.log_box.see("end")
        print(f"[{stamp}] {text}")

    # ---------------- Map ----------------

    def refresh_map_list(self):
        maps = []
        for folder in [WS / "maps", HOME, WS]:
            if folder.exists():
                maps.extend(str(p) for p in folder.glob("*.yaml"))
        maps = sorted(set(maps))
        self.map_combo["values"] = maps
        if str(DEFAULT_MAP) in maps:
            self.map_var.set(str(DEFAULT_MAP))
        self.check_ready()
        self.log("Map list refreshed.")

    def browse_map(self):
        path = filedialog.askopenfilename(
            title="Select Nav2 map YAML",
            initialdir=str(WS / "maps"),
            filetypes=[("YAML map", "*.yaml"), ("All files", "*.*")],
        )
        if path:
            self.map_var.set(path)
            self.check_ready()
            self.log(f"Selected map: {path}")

    # ---------------- Process commands ----------------

    def start_process(self, name, cmd, cwd=None):
        if name in self.processes and self.processes[name].is_running():
            self.log(f"{name} already running.")
            return
        proc = ManagedProcess(name, cmd, cwd)
        proc.start()
        self.processes[name] = proc
        if proc.error:
            self.log(f"FAILED to start {name}: {proc.error}")
        else:
            self.log(f"Started {name}. Log file: {proc.log_file}")
        self.refresh_process_table()

    def refresh_process_table(self):
        for item in self.proc_tree.get_children():
            self.proc_tree.delete(item)
        for name, proc in self.processes.items():
            status = proc.status_text()
            tag = "running" if status == "RUNNING" else ("error" if status == "ERROR" else "stopped")
            self.proc_tree.insert("", "end", text=name, values=(status, str(proc.log_file)), tags=(tag,))
        self.proc_tree.tag_configure("running", foreground=SUCCESS)
        self.proc_tree.tag_configure("error", foreground=DANGER)
        self.proc_tree.tag_configure("stopped", foreground=MUTED_TEXT)
        self.check_ready()

    def start_robot_base(self):
        self.log("Starting robot base (core movement pipeline)...")
        for name, cmd, cwd in CORE_BASE_PROCESSES:
            self.start_process(name, cmd, cwd)
            time.sleep(0.3)
        self.set_state(S.BASE_STARTED)
        self.ros_node.speak("Robot base system is starting.")

    def start_always_on(self):
        self.log("Starting always-on interaction systems...")
        for name, cmd, cwd in ALWAYS_ON_PROCESSES:
            self.start_process(name, cmd, cwd)
            time.sleep(0.3)
        self.log("Always-on systems requested. AprilTag explainer will run continuously "
                  "regardless of which app is open.")
        self.check_ready()

    def start_nav2(self):
        map_path = Path(self.map_var.get())
        if not map_path.exists():
            messagebox.showerror("Map missing", f"Map file does not exist:\n{map_path}")
            return

        if "Navigation Stack" in self.processes and self.processes["Navigation Stack"].is_running():
            if not messagebox.askyesno("Restart Nav2?", "Nav2 is already running. Restart it with the selected map?"):
                return
            self.processes["Navigation Stack"].stop()

        cmd = (
            f"ros2 launch pusantara_navigation navigation.launch.py "
            f"use_sim_time:=False autostart:=True "
            f"map:={map_path} "
            f"params_file:={DEFAULT_NAV2_PARAMS}"
        )
        self.start_process("Navigation Stack", cmd)
        self.set_state(S.NAV2_STARTED)
        self.log(f"Nav2 started with map: {map_path}")
        self.ros_node.speak("Navigation is starting. Please set my initial pose in RViz.")

    def open_rviz(self):
        self.start_process("RViz2", "rviz2")
        self.log("RViz2 opened. Use 2D Pose Estimate first, then Publish Point to choose waypoints.")

    # ---------------- Demo actions ----------------

    def greet(self):
        self.ros_node.speak(
            "Assalamualaikum and welcome. I am PUSANTARA, your cultural heritage service robot. "
            "Please select a waypoint and I will guide you to a heritage experience."
        )
        self.last_greet_time = time.time()
        if self.state in (S.READY, S.VISITOR_DETECTED):
            self.set_state(S.GREETED)
        self.log("Visitor greeting triggered.")

    def on_person_detected(self):
        self.person_value.config(text=time.strftime("%H:%M:%S"))
        if self.state != S.READY:
            return
        now = time.time()
        if now - self.last_greet_time < GREET_COOLDOWN_SEC:
            return
        self.set_state(S.VISITOR_DETECTED)
        self.greet()

    def cancel_nav(self):
        self.ros_node.cancel_nav()
        if self.state == S.NAVIGATING:
            self.set_state(S.READY)
        self.log("Navigation canceled.")

    def e_stop(self):
        self.ros_node.cancel_nav()
        self.ros_node.stop_robot()
        self.stop_current_app_manual()
        self.set_state(S.ESTOP)
        self.log("EMERGENCY STOP activated.")

    def stop_everything(self):
        """Full stop: cancels navigation, stops robot motion, stops the current
        app, and stops every process this GUI launched (base, Nav2, always-on
        systems, RViz2). Unlike E-STOP this also tears down processes, so use
        it when you actually want to end the demo, not just pause motion."""
        if not messagebox.askyesno(
            "Stop everything?",
            "This cancels navigation, stops the robot, stops the current app, "
            "and stops EVERY launched process (base, Nav2, always-on systems, RViz2).\n\n"
            "Continue?",
        ):
            return

        try:
            self.ros_node.cancel_nav()
        except Exception:
            pass

        self.set_destination_buttons_enabled(True)
        for w in self.app_select_frame.winfo_children():
            w.destroy()
        self.last_arrived_waypoint = None

        self.set_state(S.OFFLINE)
        self.log("STOP EVERYTHING pressed - tearing down all processes and motion.")
        self.stop_all_processes()

    def stop_all_ros_nodes(self):
        """System-wide kill switch: stops every process this GUI tracks AND
        pkills any other ros2/rviz2 node on the machine, even ones started
        outside this GUI (e.g. from another terminal). More aggressive than
        'Stop All Processes' / 'STOP EVERYTHING', which only touch processes
        this GUI itself launched."""
        if not messagebox.askyesno(
            "Stop ALL ROS nodes?",
            "This will stop every process launched by this GUI AND kill any other "
            "ros2/rviz2 node running on this machine, even ones started outside "
            "this GUI (e.g. from another terminal).\n\n"
            "Use this as a full reset between demo sessions. Continue?",
        ):
            return

        try:
            self.ros_node.cancel_nav()
        except Exception:
            pass
        self.set_destination_buttons_enabled(True)
        for w in self.app_select_frame.winfo_children():
            w.destroy()
        self.last_arrived_waypoint = None
        self.set_state(S.OFFLINE)

        threading.Thread(target=self._stop_all_ros_nodes_worker, daemon=True).start()
        self.log("Stopping ALL ROS nodes (GUI-tracked + system-wide) in background...")

    def _stop_all_ros_nodes_worker(self):
        # 1. Stop everything this GUI knows about first (clean SIGINT/TERM/KILL).
        try:
            self.stop_current_app_process()
        except Exception:
            pass
        try:
            self.ros_node.stop_robot()
        except Exception:
            pass
        for proc in list(self.processes.values()):
            try:
                proc.stop()
            except Exception:
                pass
        self.processes.clear()

        # 2. Best-effort listing, for the log, of what ros2 currently sees.
        try:
            result = subprocess.run(
                ["bash", "-lc", source_prefix() + "ros2 node list"],
                capture_output=True, text=True, timeout=4,
            )
            nodes = [n for n in result.stdout.splitlines() if n.strip()]
            if nodes:
                self.root.after(0, lambda: self.log(f"ros2 node list before cleanup: {', '.join(nodes)}"))
        except Exception:
            pass

        # 3. System-wide sweep for anything not started by this GUI
        #    (another terminal, a leftover launch, etc.). Only targets
        #    ROS/RViz process patterns, never unrelated system processes.
        patterns = ["ros2 run", "ros2 launch", "rviz2", "component_container"]
        for pattern in patterns:
            try:
                subprocess.run(["pkill", "-f", "-9", pattern], capture_output=True, timeout=3)
            except Exception:
                pass

        try:
            subprocess.run(["bash", "-lc", "ros2 daemon stop"], capture_output=True, timeout=3)
        except Exception:
            pass

        self.root.after(0, lambda: (
            self.refresh_process_table(),
            self.log("ALL ROS nodes stopped (GUI processes + system-wide sweep)."),
        ))

    def resume_from_estop(self):
        if self.state == S.ESTOP:
            self.set_state(S.READY)
            self.check_ready()
            self.log("Resumed from E-STOP.")

    # ---------------- Waypoints ----------------

    def add_waypoint(self, wp):
        self.waypoints.append(wp)
        self.refresh_waypoint_table()
        self.refresh_destination_buttons()
        self.log(f"Waypoint added: {wp['name']} (apps: {', '.join(wp['apps'])})")

    def add_clicked_waypoint(self):
        point = self.ros_node.last_clicked_point
        if not point:
            messagebox.showinfo(
                "No RViz point yet",
                "In RViz2, select the 'Publish Point' tool, then click on the map first.",
            )
            return
        dlg = WaypointDialog(self.root, "New Waypoint (from RViz click)",
                              prefill_xyz=(point["x"], point["y"], self.yaw_var.get()))
        if dlg.result:
            self.add_waypoint(dlg.result)

    def add_current_pose_waypoint(self):
        pose = self.ros_node.current_pose
        if not pose:
            messagebox.showinfo("No AMCL pose", "No /amcl_pose received yet. Set initial pose in RViz2 first.")
            return
        dlg = WaypointDialog(self.root, "New Waypoint (from robot pose)",
                              prefill_xyz=(pose["x"], pose["y"], pose["yaw_deg"]))
        if dlg.result:
            self.add_waypoint(dlg.result)

    def add_manual_waypoint(self):
        dlg = WaypointDialog(self.root, "New Waypoint (manual)")
        if dlg.result:
            self.add_waypoint(dlg.result)

    def edit_waypoint(self):
        wp = self.get_selected_waypoint()
        if wp is None:
            return
        index = self.waypoints.index(wp)
        dlg = WaypointDialog(self.root, "Edit Waypoint", initial=wp)
        if dlg.result:
            self.waypoints[index] = dlg.result
            self.refresh_waypoint_table()
            self.refresh_destination_buttons()
            self.log(f"Waypoint edited: {dlg.result['name']}")

    def refresh_waypoint_table(self):
        for item in self.wp_tree.get_children():
            self.wp_tree.delete(item)
        for i, wp in enumerate(self.waypoints):
            self.wp_tree.insert(
                "", "end", iid=str(i), text=wp["name"],
                values=(
                    f"{wp['x']:.2f}", f"{wp['y']:.2f}", f"{wp['yaw_deg']:.1f}",
                    ", ".join(wp.get("apps", [])), wp.get("app_mode", ""),
                    wp.get("description", ""),
                ),
            )
        # give the tree a name-visible first column
        self.wp_tree["show"] = "tree headings"
        self.wp_tree.heading("#0", text="NAME")
        self.wp_tree.column("#0", width=180)

    def get_selected_waypoint(self):
        sel = self.wp_tree.selection()
        if not sel:
            messagebox.showinfo("No waypoint selected", "Select a waypoint first.")
            return None
        index = int(sel[0])
        return self.waypoints[index]

    def delete_waypoint(self):
        sel = self.wp_tree.selection()
        if not sel:
            return
        index = int(sel[0])
        removed = self.waypoints.pop(index)
        self.dest_buttons.pop(removed.get("id"), None)
        self.refresh_waypoint_table()
        self.refresh_destination_buttons()
        self.log(f"Deleted waypoint: {removed['name']}")

    def save_waypoints(self):
        WAYPOINT_FILE.parent.mkdir(parents=True, exist_ok=True)
        payload = {"map": self.map_var.get(), "waypoints": self.waypoints}
        with open(WAYPOINT_FILE, "w") as f:
            json.dump(payload, f, indent=2)
        self.log(f"Waypoints saved to {WAYPOINT_FILE}")

    def load_waypoints_silent(self):
        if WAYPOINT_FILE.exists():
            try:
                with open(WAYPOINT_FILE, "r") as f:
                    data = json.load(f)
                self.waypoints = data.get("waypoints", data if isinstance(data, list) else [])
            except Exception:
                pass

    def load_waypoints(self):
        if not WAYPOINT_FILE.exists():
            messagebox.showinfo("No waypoint file", f"No file found:\n{WAYPOINT_FILE}")
            return
        with open(WAYPOINT_FILE, "r") as f:
            data = json.load(f)
        self.waypoints = data.get("waypoints", data if isinstance(data, list) else [])
        self.refresh_waypoint_table()
        self.refresh_destination_buttons()
        self.log(f"Waypoints loaded from {WAYPOINT_FILE}")

    # ---------------- Home / initial position ----------------

    def load_home_position_silent(self):
        if HOME_POSITION_FILE.exists():
            try:
                with open(HOME_POSITION_FILE, "r") as f:
                    data = json.load(f)
                self.home_position = {
                    "x": float(data.get("x", 0.0)),
                    "y": float(data.get("y", 0.0)),
                    "yaw_deg": float(data.get("yaw_deg", 0.0)),
                }
            except Exception:
                pass
        self.home_x_var.set(self.home_position["x"])
        self.home_y_var.set(self.home_position["y"])
        self.home_yaw_var.set(self.home_position["yaw_deg"])

    def save_home_position(self):
        try:
            self.home_position = {
                "x": float(self.home_x_var.get()),
                "y": float(self.home_y_var.get()),
                "yaw_deg": float(self.home_yaw_var.get()),
            }
        except Exception:
            messagebox.showerror("Invalid numbers", "Home X, Y and yaw must be numeric.")
            return
        HOME_POSITION_FILE.parent.mkdir(parents=True, exist_ok=True)
        with open(HOME_POSITION_FILE, "w") as f:
            json.dump(self.home_position, f, indent=2)
        self.log(f"Home position saved: x={self.home_position['x']:.2f}, "
                  f"y={self.home_position['y']:.2f}, yaw={self.home_position['yaw_deg']:.1f} "
                  f"-> {HOME_POSITION_FILE}")

    def set_home_from_current_pose(self):
        pose = self.ros_node.current_pose
        if not pose:
            messagebox.showinfo("No AMCL pose", "No /amcl_pose received yet. Set initial pose in RViz2 first.")
            return
        self.home_x_var.set(round(pose["x"], 3))
        self.home_y_var.set(round(pose["y"], 3))
        self.home_yaw_var.set(round(pose["yaw_deg"], 1))
        self.save_home_position()
        self.log("Home position set from current robot pose.")

    def publish_home_as_initial_pose(self):
        try:
            x = float(self.home_x_var.get())
            y = float(self.home_y_var.get())
            yaw = float(self.home_yaw_var.get())
        except Exception:
            messagebox.showerror("Invalid numbers", "Home X, Y and yaw must be numeric.")
            return
        self.ros_node.publish_initial_pose(x, y, yaw)
        self.log("Requested AMCL localization at saved home position (equivalent to RViz 2D Pose Estimate).")

    def go_to_home(self):
        if self.state == S.ESTOP:
            messagebox.showwarning("E-STOP active", "Resolve E-STOP before navigating.")
            return
        try:
            wp = {
                "name": "Home",
                "x": float(self.home_x_var.get()),
                "y": float(self.home_y_var.get()),
                "yaw_deg": float(self.home_yaw_var.get()),
            }
        except Exception:
            messagebox.showerror("Invalid numbers", "Home X, Y and yaw must be numeric.")
            return
        self.set_destination_buttons_enabled(False)
        self.set_state(S.NAVIGATING)
        self.log("Sending robot to home / initial position.")
        self.ros_node.send_goal(wp)

    # ---------------- Visitor destination buttons ----------------

    def refresh_destination_buttons(self):
        for w in self.dest_canvas_frame.winfo_children():
            w.destroy()
        # IMPORTANT: button widgets live ONLY in self.dest_buttons, keyed by
        # waypoint id - never stored inside the waypoint dict itself. The
        # waypoint dict is passed straight into json.dumps() on arrival, so
        # anything non-serializable put in there breaks the whole arrival
        # flow (see result_cb / on_arrived).
        self.dest_buttons.clear()

        if not self.waypoints:
            tk.Label(self.dest_canvas_frame, text="No waypoints yet. Add some in the Waypoint Editor tab.",
                     bg=PANEL_BG, fg=MUTED_TEXT, font=(FONT_FAMILY, 11)).pack(pady=20)
            return

        cols = 3
        for i, wp in enumerate(self.waypoints):
            r, c = divmod(i, cols)
            self.dest_canvas_frame.columnconfigure(c, weight=1)

            card = tk.Frame(self.dest_canvas_frame, bg=INFO_BG, highlightbackground=PANEL_BORDER,
                             highlightthickness=1)
            card.grid(row=r, column=c, sticky="nsew", padx=6, pady=6)

            tk.Label(card, text=wp["name"], bg=INFO_BG, fg=ACCENT,
                     font=(FONT_FAMILY, 13, "bold")).pack(anchor="w", padx=10, pady=(8, 0))
            if wp.get("description"):
                tk.Label(card, text=wp["description"], bg=INFO_BG, fg=TEXT, wraplength=260,
                         justify="left", font=(FONT_FAMILY, 9)).pack(anchor="w", padx=10, pady=(2, 4))
            tk.Label(card, text="Apps: " + ", ".join(wp.get("apps", [])), bg=INFO_BG, fg=MUTED_TEXT,
                     wraplength=260, justify="left", font=(FONT_FAMILY, 8)).pack(anchor="w", padx=10, pady=(0, 6))

            btn = tk.Button(card, text="\U0001F9ED  Go Here", bg=ACCENT, fg="#1B1B2F",
                             activebackground=ACCENT_DARK, relief="flat", font=(FONT_FAMILY, 11, "bold"),
                             command=lambda w=wp: self.navigate_to(w))
            btn.pack(fill="x", padx=10, pady=(0, 10))
            self.dest_buttons[wp["id"]] = btn

    def set_destination_buttons_enabled(self, enabled):
        for btn in self.dest_buttons.values():
            try:
                btn.config(state="normal" if enabled else "disabled")
            except Exception:
                pass

    def navigate_to(self, wp):
        if self.state == S.ESTOP:
            messagebox.showwarning("E-STOP active", "Resolve E-STOP before navigating.")
            return
        self.set_state(S.DESTINATION_SELECTED)
        self.set_destination_buttons_enabled(False)
        self.log(f"Destination selected: {wp['name']}")
        self.set_state(S.NAVIGATING)
        self.ros_node.send_goal(wp)

    # ---------------- Arrival / app selection ----------------

    def on_arrived(self):
        self.set_state(S.ARRIVED)
        self.set_destination_buttons_enabled(True)
        wp = self.last_arrived_waypoint
        if not wp:
            return

        apps = wp.get("apps", [])
        mode = wp.get("app_mode", "visitor_select")

        for w in self.app_select_frame.winfo_children():
            w.destroy()

        if mode == "auto" and len(apps) == 1:
            self.set_state(S.APP_SELECTION)
            self.log(f"Auto-launching {apps[0]} in {AUTO_APP_LAUNCH_DELAY_MS}ms...")
            self.root.after(AUTO_APP_LAUNCH_DELAY_MS, lambda: self.launch_app(apps[0]))
        else:
            self.set_state(S.APP_SELECTION)
            for app_name in apps:
                b = tk.Button(
                    self.app_select_frame, text=app_name, bg=ACCENT, fg="#1B1B2F",
                    activebackground=ACCENT_DARK, relief="flat", font=(FONT_FAMILY, 12, "bold"),
                    padx=16, pady=10, command=lambda a=app_name: self.launch_app(a),
                )
                b.pack(side="left", padx=6, pady=4)

    def launch_app(self, app_name):
        self.stop_current_app_process()

        feature = APP_REGISTRY.get(app_name)
        if not feature:
            messagebox.showerror("App missing", f"App not found in registry: {app_name}")
            self.log(f"ERROR: '{app_name}' is assigned to a waypoint but not present in APP_REGISTRY.")
            return

        cwd = feature.get("cwd")
        if cwd is not None and not Path(cwd).exists():
            messagebox.showerror("Folder missing", f"Folder does not exist (showing as MISSING):\n{cwd}")
            self.log(f"MISSING app folder for {app_name}: {cwd}")
            return

        proc = ManagedProcess(app_name, feature["cmd"], cwd)
        proc.start()
        self.current_app_proc = proc
        self.current_app_name = app_name
        self.current_app_value.config(text=app_name)

        for w in self.app_select_frame.winfo_children():
            w.destroy()

        self.set_state(S.APP_RUNNING)
        self.log(f"Launched heritage app: {app_name} (always-on systems keep running in background).")
        self.ros_node.speak(f"Launching {app_name}.")

    def stop_current_app_process(self):
        if self.current_app_proc and self.current_app_proc.is_running():
            self.current_app_proc.stop()
            self.log(f"Stopped app: {self.current_app_proc.name}")
        self.current_app_proc = None
        self.current_app_name = None
        self.current_app_value.config(text="none")

    def stop_current_app_manual(self):
        self.stop_current_app_process()
        if self.state == S.APP_RUNNING:
            self.set_state(S.APP_SELECTION)

    def again(self):
        self.stop_current_app_process()
        self.last_arrived_waypoint = None
        for w in self.app_select_frame.winfo_children():
            w.destroy()
        self.set_destination_buttons_enabled(True)
        self.set_state(S.READY)
        self.ros_node.speak("Thank you. Please select another destination for the next heritage experience.")
        self.log("Loop reset. Ready for next visitor.")

    # ---------------- Logs tab helper ----------------

    def open_log_folder(self):
        try:
            subprocess.Popen(["xdg-open", str(LOG_DIR)])
        except Exception as e:
            messagebox.showinfo("Log folder", f"Could not open a file browser automatically.\n"
                                               f"Logs are at:\n{LOG_DIR}\n\n({e})")

    # ---------------- Shutdown (fixed: non-blocking, watchdog) ----------------

    def stop_all_processes(self):
        threading.Thread(target=self._stop_all_worker, daemon=True).start()
        self.log("Stopping all processes in background...")

    def _stop_all_worker(self):
        self.stop_current_app_process()
        try:
            self.ros_node.stop_robot()
        except Exception:
            pass
        for proc in list(self.processes.values()):
            proc.stop()
        self.processes.clear()
        self.root.after(0, lambda: (self.refresh_process_table(), self.log("All launched processes stopped.")))

    def on_close(self):
        if self._closing:
            return
        if not messagebox.askyesno("Exit", "Stop all launched processes and exit?"):
            return

        self._closing = True
        self.root.title("PUSANTARA Mission Control \u2014 Closing...")
        self.log("Shutting down. This can take a few seconds while processes stop...")

        threading.Thread(target=self._shutdown_worker, daemon=True).start()
        self.root.after(int(SHUTDOWN_WATCHDOG_SEC * 1000), self._force_exit)

    def _shutdown_worker(self):
        try:
            self.stop_current_app_process()
        except Exception:
            pass
        try:
            self.ros_node.stop_robot()
        except Exception:
            pass
        for proc in list(self.processes.values()):
            try:
                proc.stop()
            except Exception:
                pass
        self.processes.clear()

        try:
            self.ros_node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

        self.root.after(0, self._finish_close)

    def _finish_close(self):
        self._closed = True
        try:
            self.root.destroy()
        except Exception:
            pass

    def _force_exit(self):
        if not self._closed:
            print("Shutdown watchdog fired - forcing exit.")
            os._exit(0)


def main():
    rclpy.init()
    root = tk.Tk()
    app = DemoGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
