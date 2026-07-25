#!/usr/bin/env python3

import os
import json
import math
import time
import signal
import random
import subprocess
import threading
from pathlib import Path
from collections import defaultdict, deque

try:
    import psutil
except ImportError:
    psutil = None

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter.scrolledtext import ScrolledText

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose


# ============================================================
# PATHS
# ============================================================

HOME = Path.home()
WS = HOME / "vectra_ws"

ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "1")

DEFAULT_MAP = WS / "maps" / "pusantara_map1.yaml"
DEFAULT_NAV2_PARAMS = WS / "src" / "pusantara_navigation" / "config" / "nav2_params.yaml"

WAYPOINT_FILE = HOME / "vectra_data" / "cavecare_waypoints.json"
LOG_DIR = Path("/tmp/pusantara_cavecare_panel")
LOG_DIR.mkdir(parents=True, exist_ok=True)

# Topic RViz2's "2D Nav Goal" tool publishes to in a standard Nav2 config.
# If your RViz config publishes elsewhere (e.g. /move_base_simple/goal),
# change this one constant.
RVIZ_GOAL_TOPIC = "/goal_pose"

# NOTE: /robot_command is reused here for the "arrived -> TAKEOUT" signal
# because it's the same channel STOP already uses in this file. This has
# NOT been verified against the ESP32 / robot-command firmware — confirm
# the expected string/topic with whoever owns that code.
TAKEOUT_COMMAND = "TAKEOUT"

INITIAL_POSE_COVARIANCE = [
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787,
]


# ============================================================
# COLORS / THEME
# ============================================================

COL_BG = "#f3f4f6"
COL_PANEL = "#ffffff"
COL_HEADER = "#111827"
COL_HEADER_FG = "#f9fafb"
COL_ACCENT = "#2563eb"
COL_ACCENT_DARK = "#1d4ed8"
COL_SUCCESS = "#16a34a"
COL_DANGER = "#dc2626"
COL_DANGER_DARK = "#b91c1c"
COL_WARNING = "#d97706"
COL_TEXT = "#111827"
COL_MUTED = "#6b7280"
COL_BORDER = "#e5e7eb"

FONT_BASE = ("Segoe UI", 7)
FONT_BOLD = ("Segoe UI", 7, "bold")
FONT_TITLE = ("Segoe UI", 11, "bold")
FONT_SECTION = ("Segoe UI", 8, "bold")
FONT_MONO = ("Consolas", 7)


# ============================================================
# PROCESS COMMANDS
# ============================================================

CORE_PROCESSES = [
    ("Robot Bringup", "ros2 launch pusantara_bringup robot.launch.py", None),
    ("cmd_vel Bridge", "ros2 run cmd_vel_bridge cmd_vel_to_robot_command", None),
]

VISION_PROCESSES = [
    ("Camera", "ros2 run camera_node webcam_pub", None),
    ("AprilTag Detector", "ros2 run apriltag_detector apriltag_detector --ros-args -p show_ui:=true", None),
]

CAVECARE_PROCESSES = [
    ("TTS", "ros2 run tts_node tts_node", None),
    ("ESP32 Receiver", "ros2 run esp32_wifi_receiver esp32_http_receiver", None),
    ("Mold Detector", "ros2 run mold_detector mold_detector_node --ros-args -p image_topic:=/image_raw", None),
    ("Cave Mission Manager", "ros2 run cavecare_mission cave_mission_manager", None),
]


# ============================================================
# HELPERS
# ============================================================

def source_prefix():
    return (
        "source /opt/ros/humble/setup.bash && "
        f"source {WS}/install/setup.bash && "
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
    )


def safe_name(name):
    return "".join(c if c.isalnum() else "_" for c in name.lower()).strip("_")


def yaw_to_quaternion(yaw_rad):
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return qz, qw


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def next_waypoint_name(existing):
    n = 1
    while f"wp_{n}" in existing:
        n += 1
    return f"wp_{n}"


# ============================================================
# MANAGED PROCESS
# ============================================================

class ManagedProcess:
    def __init__(self, name, cmd, cwd=None):
        self.name = name
        self.cmd = cmd
        self.cwd = cwd
        self.proc = None
        self.error = None
        self.log_file = LOG_DIR / f"{safe_name(name)}.log"

    def start(self):
        if self.is_running():
            return

        shell_cmd = self.cmd

        if self.cwd:
            shell_cmd = f'cd "{self.cwd}" && {self.cmd}'

        full_cmd = source_prefix() + shell_cmd

        try:
            log = open(self.log_file, "a", buffering=1)
            log.write("\n\n=================================\n")
            log.write(f"START {self.name} at {time.ctime()}\n")
            log.write(f"CMD: {full_cmd}\n")
            log.write("=================================\n")

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
        if not self.is_running():
            return

        pid = self.proc.pid

        try:
            pgid = os.getpgid(pid)
        except Exception:
            pgid = None

        # Collect descendants up front. Some launch tools (e.g. ros2 launch,
        # Nav2's launch tree) spawn children into their OWN process group,
        # which means a plain killpg() on our pgid can miss them entirely.
        child_pids = []
        if psutil is not None:
            try:
                child_pids = [c.pid for c in psutil.Process(pid).children(recursive=True)]
            except Exception:
                child_pids = []

        for sig, wait_s in [
            (signal.SIGINT, 1.0),
            (signal.SIGTERM, 1.0),
            (signal.SIGKILL, 0.0),
        ]:
            if pgid is not None:
                try:
                    os.killpg(pgid, sig)
                except Exception:
                    pass

            for cpid in child_pids:
                try:
                    os.kill(cpid, sig)
                except Exception:
                    pass

            try:
                os.kill(pid, sig)
            except Exception:
                pass

            if wait_s:
                deadline = time.time() + wait_s
                while time.time() < deadline:
                    if self.proc.poll() is not None:
                        break
                    time.sleep(0.05)

            if self.proc.poll() is not None:
                break

        # Sweep for any stragglers that spawned late or re-parented.
        if psutil is not None:
            try:
                if psutil.pid_exists(pid):
                    for c in psutil.Process(pid).children(recursive=True):
                        try:
                            c.kill()
                        except Exception:
                            pass
            except Exception:
                pass

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

class CaveCarePanelNode(Node):
    def __init__(self, ui_cb):
        super().__init__("cavecare_panel")

        self.ui_cb = ui_cb
        self.current_pose = None
        self.current_goal_handle = None
        self.current_goal_name = None

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.tts_pub = self.create_publisher(String, "/tts_text", 10)
        self.cave_pub = self.create_publisher(String, "/cave_node/status", 10)
        self.robot_cmd_pub = self.create_publisher(String, "/robot_command", 10)
        self.safe_cmd_pub = self.create_publisher(String, "/safe_robot_command", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_cb, 10)
        self.create_subscription(String, "/cave_node/status", self.cave_cb, 10)
        self.create_subscription(String, "/mold_detector/status", self.mold_cb, 10)
        self.create_subscription(String, "/apriltag/detections", self.tag_cb, 10)
        self.create_subscription(String, "/inspection_report", self.report_cb, 10)
        self.create_subscription(String, "/robot_command", self.robot_cmd_cb, 10)
        self.create_subscription(String, "/safe_robot_command", self.safe_cmd_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)

        # RViz2 "2D Nav Goal" -> capture as a candidate waypoint
        self.create_subscription(PoseStamped, RVIZ_GOAL_TOPIC, self.rviz_goal_cb, 10)

    def ui(self, kind, text):
        if self.ui_cb:
            self.ui_cb(kind, text)

    def amcl_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q)
        self.current_pose = {
            "x": p.x,
            "y": p.y,
            "yaw": yaw,
        }
        self.ui("pose", f"x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f} rad")

    def rviz_goal_cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = quaternion_to_yaw(q)
        self.ui("rviz_pose", {"x": p.x, "y": p.y, "yaw": yaw})
        self.ui("log", f"Captured pose from RViz2 ({RVIZ_GOAL_TOPIC}): x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f}")

    def cave_cb(self, msg):
        self.ui("cave", msg.data)

        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if "temperature" in data or "humidity" in data:
            self.ui("sensor_data", data)

    def mold_cb(self, msg):
        self.ui("mold", msg.data)

    def tag_cb(self, msg):
        self.ui("tag", msg.data)

    def report_cb(self, msg):
        self.ui("report", msg.data)

    def robot_cmd_cb(self, msg):
        self.ui("robot_cmd", msg.data)

    def safe_cmd_cb(self, msg):
        self.ui("safe_cmd", msg.data)

    def cmd_vel_cb(self, msg):
        self.ui("cmd_vel", f"linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")

    def speak(self, text):
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        self.ui("log", f"TTS: {text}")

    def publish_cave_alert(self, node_id, panel_id, tag_id, temp, humidity, status):
        data = {
            "node_id": str(node_id),
            "panel_id": str(panel_id),
            "tag_id": int(tag_id),
            "temperature": float(temp),
            "humidity": float(humidity),
            "status": str(status).upper(),
        }

        msg = String()
        msg.data = json.dumps(data)
        self.cave_pub.publish(msg)

        self.ui("log", f"Published cave alert: {msg.data}")
        self.speak(
            f"Cave node alert. {node_id} reports humidity {float(humidity):.1f} percent. "
            f"Status is {status}."
        )

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(float(yaw))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = list(INITIAL_POSE_COVARIANCE)

        self.initialpose_pub.publish(msg)
        self.ui("log", f"Published /initialpose: x={x}, y={y}, yaw={yaw}")

    def stop_robot(self):
        stop = String()
        stop.data = "STOP"

        self.robot_cmd_pub.publish(stop)
        self.safe_cmd_pub.publish(stop)

        zero = Twist()
        self.cmd_vel_pub.publish(zero)

        self.ui("log", "STOP sent to /robot_command, /safe_robot_command, and /cmd_vel")

    def send_takeout_command(self, name):
        # See TAKEOUT_COMMAND note at top of file re: unverified against
        # actual robot/ESP32 firmware.
        msg = String()
        msg.data = TAKEOUT_COMMAND
        self.robot_cmd_pub.publish(msg)
        self.ui("log", f"Arrived at '{name}': published '{TAKEOUT_COMMAND}' to /robot_command")

    def cancel_nav(self):
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
                self.ui("log", "Nav2 cancel requested.")
            except Exception as e:
                self.ui("log", f"Cancel failed: {e}")

        self.stop_robot()

    def send_goal(self, name, x, y, yaw):
        threading.Thread(
            target=self._send_goal_thread,
            args=(name, x, y, yaw),
            daemon=True,
        ).start()

    def _send_goal_thread(self, name, x, y, yaw):
        self.ui("nav", "Waiting for Nav2 action server...")

        if not self.nav_client.wait_for_server(timeout_sec=8.0):
            self.ui("nav", "Nav2 action server not available.")
            self.ui("log", "ERROR: /navigate_to_pose is not available.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(float(yaw))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.current_goal_name = name
        self.ui("nav", f"Sending goal: {name}")
        self.speak(f"Moving to {name}.")

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.goal_response_cb(f, name))

    def goal_response_cb(self, future, name):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.ui("nav", "Goal rejected.")
            self.speak("Navigation goal rejected.")
            return

        self.current_goal_handle = goal_handle
        self.ui("nav", f"Goal accepted: {name}")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.goal_result_cb(f, name))

    def goal_result_cb(self, future, name):
        self.ui("nav", f"Navigation finished: {name}")
        self.speak(f"Navigation to {name} finished.")

        # Arrived at an inspection waypoint (not just returning home) ->
        # signal the robot to take the sample/panel out.
        if name and name.strip().lower() != "home":
            self.send_takeout_command(name)


# ============================================================
# GUI
# ============================================================

class CaveCarePanel:
    def __init__(self, root):
        self.root = root
        self.root.title("PUSANTARA CaveCare Control Panel")
        self.root.geometry("800x480")
        self.root.minsize(760, 440)
        self.root.configure(bg=COL_BG)

        self.processes = {}
        self._closing = False

        self.map_var = tk.StringVar(value=str(DEFAULT_MAP))
        self.wp_name_var = tk.StringVar(value="node_1")
        self.node_id_var = tk.StringVar(value="node_1")
        self.panel_id_var = tk.StringVar(value="panel_a")
        self.tag_id_var = tk.StringVar(value="1")
        self.x_var = tk.StringVar(value="1.0")
        self.y_var = tk.StringVar(value="0.0")
        self.yaw_var = tk.StringVar(value="0.0")

        self.temp_var = tk.StringVar(value="28.5")
        self.humidity_var = tk.StringVar(value="82.0")
        self.status_var = tk.StringVar(value="DANGER")

        self.rviz_last_var = tk.StringVar(value="No pose captured from RViz2 yet.")

        self.waypoints = self.load_waypoints()

        # sensor history per node_id: deque of (t, temp, humidity)
        self.sensor_start_time = time.time()
        self.sensor_history = defaultdict(lambda: {
            "t": deque(maxlen=300),
            "temp": deque(maxlen=300),
            "hum": deque(maxlen=300),
        })
        self.sensor_latest_status = {}

        # simulated "extra node" data, derived from the real ESP32 reading
        self.fake_enabled = tk.BooleanVar(value=False)
        self.fake_count_var = tk.IntVar(value=2)
        self.fake_node_keys = set()
        self.fake_state = {}

        self.setup_style()
        self.build_ui()

        self.ros_node = CaveCarePanelNode(self.ros_ui_callback)
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        self.refresh_waypoint_list()
        self.refresh_process_table()
        self.log("CaveCare panel ready.")

        self.root.after(1000, self.periodic_update)
        self.root.after(2000, self.periodic_graph_update)

    # ============================================================
    # STYLE
    # ============================================================

    def setup_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except Exception:
            pass

        style.configure(".", font=FONT_BASE, background=COL_BG, foreground=COL_TEXT)
        style.configure("TFrame", background=COL_BG)
        style.configure("TLabel", background=COL_BG, foreground=COL_TEXT)
        style.configure("TLabelframe", background=COL_BG, foreground=COL_TEXT, font=FONT_SECTION)
        style.configure("TLabelframe.Label", background=COL_BG, foreground=COL_TEXT, font=FONT_SECTION)

        style.configure("TNotebook", background=COL_BG, borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            font=FONT_BOLD,
            padding=(6, 2),
            background=COL_BORDER,
            foreground=COL_MUTED,
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", COL_PANEL)],
            foreground=[("selected", COL_ACCENT_DARK)],
        )

        style.configure("TButton", font=FONT_BASE, padding=2)

        style.configure("Accent.TButton", font=FONT_BOLD, padding=3, background=COL_ACCENT, foreground="white")
        style.map("Accent.TButton", background=[("active", COL_ACCENT_DARK)])

        style.configure("Success.TButton", font=FONT_BOLD, padding=3, background=COL_SUCCESS, foreground="white")
        style.map("Success.TButton", background=[("active", "#15803d")])

        style.configure("Danger.TButton", font=FONT_BOLD, padding=3, background=COL_DANGER, foreground="white")
        style.map("Danger.TButton", background=[("active", COL_DANGER_DARK)])

        style.configure("Treeview", font=FONT_BASE, rowheight=15, background=COL_PANEL, fieldbackground=COL_PANEL)
        style.configure("Treeview.Heading", font=FONT_BOLD, background=COL_BORDER)

    # ============================================================
    # UI BUILD
    # ============================================================

    def build_ui(self):
        title_frame = tk.Frame(self.root, bg=COL_HEADER)
        title_frame.pack(fill="x")

        tk.Label(
            title_frame,
            text="PUSANTARA CaveCare Control Panel",
            font=FONT_TITLE,
            bg=COL_HEADER,
            fg=COL_HEADER_FG,
            pady=3,
            padx=6,
        ).pack(side="left")

        self.header_status_label = tk.Label(
            title_frame,
            text="● SYSTEM IDLE",
            font=FONT_BOLD,
            bg=COL_HEADER,
            fg="#9ca3af",
            padx=6,
        )
        self.header_status_label.pack(side="right")

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=2, pady=2)
        self.notebook = notebook

        self.setup_tab = ttk.Frame(notebook)
        self.waypoint_tab = ttk.Frame(notebook)
        self.mission_tab = ttk.Frame(notebook)
        self.graph_tab = ttk.Frame(notebook)
        self.log_tab = ttk.Frame(notebook)

        notebook.add(self.setup_tab, text="Launch / Setup")
        notebook.add(self.waypoint_tab, text="Waypoints")
        notebook.add(self.mission_tab, text="Mission Control")
        notebook.add(self.graph_tab, text="Sensor Graphs")
        notebook.add(self.log_tab, text="Logs / Live Data")

        self.build_setup_tab()
        self.build_waypoint_tab()
        self.build_mission_tab()
        self.build_graph_tab()
        self.build_log_tab()

    def build_setup_tab(self):
        left = ttk.LabelFrame(self.setup_tab, text="System Launch")
        left.pack(side="left", fill="y", padx=4, pady=4)

        right = ttk.LabelFrame(self.setup_tab, text="Process Status")
        right.pack(side="left", fill="both", expand=True, padx=4, pady=4)

        ttk.Label(left, text="Map YAML").pack(anchor="w", padx=6, pady=(4, 0))

        map_row = ttk.Frame(left)
        map_row.pack(fill="x", padx=6, pady=2)

        self.map_entry = ttk.Entry(map_row, textvariable=self.map_var, width=26)
        self.map_entry.pack(side="left", padx=(0, 4))

        ttk.Button(map_row, text="Browse", command=self.browse_map).pack(side="left")

        ttk.Separator(left).pack(fill="x", padx=6, pady=4)

        btn_grid = ttk.Frame(left)
        btn_grid.pack(fill="x", padx=6, pady=2)
        btn_grid.columnconfigure(0, weight=1)
        btn_grid.columnconfigure(1, weight=1)

        grid_buttons = [
            ("1. Bringup", self.start_robot_bringup),
            ("2. Nav2", self.start_nav2),
            ("3. RViz2", self.open_rviz),
            ("4. Vision", self.start_vision),
            ("5. CaveCare", self.start_cavecare_nodes),
        ]

        for i, (label, cmd) in enumerate(grid_buttons):
            r, c = divmod(i, 2)
            ttk.Button(btn_grid, text=label, command=cmd).grid(row=r, column=c, sticky="ew", padx=2, pady=2)

        ttk.Button(left, text="START ALL", style="Success.TButton", command=self.start_all).pack(fill="x", padx=6, pady=(6, 2))

        ttk.Separator(left).pack(fill="x", padx=6, pady=4)

        stop_grid = ttk.Frame(left)
        stop_grid.pack(fill="x", padx=6, pady=2)
        stop_grid.columnconfigure(0, weight=1)
        stop_grid.columnconfigure(1, weight=1)

        ttk.Button(stop_grid, text="E-STOP", style="Danger.TButton", command=self.e_stop).grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(stop_grid, text="Stop All", command=self.stop_all_processes).grid(row=0, column=1, sticky="ew", padx=2, pady=2)
        ttk.Button(left, text="KILL ALL ROS2 / RVIZ2", style="Danger.TButton", command=self.kill_all_ros2).pack(fill="x", padx=6, pady=2)

        self.proc_tree = ttk.Treeview(
            right,
            columns=("status", "log"),
            show="tree headings",
            height=8,
        )
        self.proc_tree.heading("#0", text="Process")
        self.proc_tree.heading("status", text="Status")
        self.proc_tree.heading("log", text="Log File")

        self.proc_tree.column("#0", width=140)
        self.proc_tree.column("status", width=80)
        self.proc_tree.column("log", width=320)

        self.proc_tree.tag_configure("RUNNING", foreground=COL_SUCCESS, font=FONT_BOLD)
        self.proc_tree.tag_configure("ERROR", foreground=COL_DANGER, font=FONT_BOLD)
        self.proc_tree.tag_configure("STOPPED", foreground=COL_MUTED)
        self.proc_tree.tag_configure("NOT STARTED", foreground=COL_MUTED)

        self.proc_tree.pack(fill="both", expand=True, padx=4, pady=4)

    def build_waypoint_tab(self):
        left = ttk.LabelFrame(self.waypoint_tab, text="Waypoint Editor")
        left.pack(side="left", fill="y", padx=4, pady=4)

        mid = ttk.LabelFrame(self.waypoint_tab, text="From RViz2 (\"2D Nav Goal\")")
        mid.pack(side="left", fill="y", padx=4, pady=4)

        right = ttk.LabelFrame(self.waypoint_tab, text="Saved Waypoints")
        right.pack(side="left", fill="both", expand=True, padx=4, pady=4)

        # ---- Editor ----
        ttk.Label(left, text="Waypoint Name").pack(anchor="w", padx=6, pady=(4, 0))
        self.wp_combo = ttk.Combobox(left, textvariable=self.wp_name_var, width=18)
        self.wp_combo.pack(fill="x", padx=6, pady=2)
        self.wp_combo.bind("<<ComboboxSelected>>", lambda e: self.load_selected_waypoint())

        fields = [
            ("Node ID", self.node_id_var),
            ("Panel ID", self.panel_id_var),
            ("AprilTag ID", self.tag_id_var),
            ("X map", self.x_var),
            ("Y map", self.y_var),
            ("Yaw rad", self.yaw_var),
        ]

        for label, var in fields:
            ttk.Label(left, text=label).pack(anchor="w", padx=6, pady=(2, 0))
            ttk.Entry(left, textvariable=var, width=20).pack(fill="x", padx=6, pady=1)

        ttk.Button(left, text="Use Current Robot Pose", command=self.use_current_pose).pack(fill="x", padx=6, pady=(6, 2))
        ttk.Button(left, text="Save / Update Waypoint", style="Accent.TButton", command=self.save_waypoint_from_fields).pack(fill="x", padx=6, pady=2)
        ttk.Button(left, text="Set HOME From Current Pose", command=self.set_home_from_current_pose).pack(fill="x", padx=6, pady=2)
        ttk.Button(left, text="Delete Selected Waypoint", command=self.delete_selected_waypoint).pack(fill="x", padx=6, pady=2)

        ttk.Separator(left).pack(fill="x", padx=6, pady=4)

        ttk.Button(left, text="Go To Selected Waypoint", style="Success.TButton", command=self.go_selected_waypoint).pack(fill="x", padx=6, pady=(2, 2))
        ttk.Button(left, text="Return HOME", command=self.return_home).pack(fill="x", padx=6, pady=2)
        ttk.Button(left, text="Publish HOME as Initial Pose", command=self.publish_home_initial_pose).pack(fill="x", padx=6, pady=2)

        # ---- RViz2 capture ----
        instr = tk.Label(
            mid,
            text=(
                "In RViz2: click \"2D Nav Goal\",\n"
                "click the map, drag for facing.\n"
                "Fields on the left auto-fill."
            ),
            justify="left",
            wraplength=170,
            bg=COL_BG,
            fg=COL_MUTED,
            font=FONT_BASE,
        )
        instr.pack(anchor="w", padx=6, pady=(4, 4))

        self.rviz_status_label = tk.Label(
            mid,
            textvariable=self.rviz_last_var,
            justify="left",
            wraplength=170,
            bg=COL_BG,
            fg=COL_TEXT,
            font=FONT_BOLD,
        )
        self.rviz_status_label.pack(anchor="w", padx=6, pady=(0, 4))

        ttk.Button(
            mid,
            text="Fill Fields With Last Pose",
            command=self.apply_last_rviz_pose,
        ).pack(fill="x", padx=6, pady=2)

        ttk.Label(mid, text="Then set name/IDs on\nthe left and Save.", justify="left", foreground=COL_MUTED, font=FONT_BASE).pack(anchor="w", padx=6, pady=(4, 4))

        # ---- Table ----
        self.wp_tree = ttk.Treeview(
            right,
            columns=("node", "panel", "tag", "x", "y", "yaw"),
            show="tree headings",
            height=9,
        )

        self.wp_tree.heading("#0", text="Name")
        self.wp_tree.heading("node", text="Node ID")
        self.wp_tree.heading("panel", text="Panel ID")
        self.wp_tree.heading("tag", text="Tag")
        self.wp_tree.heading("x", text="X")
        self.wp_tree.heading("y", text="Y")
        self.wp_tree.heading("yaw", text="Yaw")

        self.wp_tree.column("#0", width=90)
        self.wp_tree.column("node", width=80)
        self.wp_tree.column("panel", width=80)
        self.wp_tree.column("tag", width=45)
        self.wp_tree.column("x", width=55)
        self.wp_tree.column("y", width=55)
        self.wp_tree.column("yaw", width=55)

        self.wp_tree.pack(fill="both", expand=True, padx=4, pady=4)
        self.wp_tree.bind("<<TreeviewSelect>>", self.on_wp_tree_select)

    def build_mission_tab(self):
        left = ttk.LabelFrame(self.mission_tab, text="ESP32 / Cave Node Alert")
        left.pack(side="left", fill="y", padx=4, pady=4)

        right = ttk.LabelFrame(self.mission_tab, text="Mission Controls")
        right.pack(side="left", fill="both", expand=True, padx=4, pady=4)

        ttk.Label(left, text="Temperature").pack(anchor="w", padx=6, pady=(4, 0))
        ttk.Entry(left, textvariable=self.temp_var, width=18).pack(fill="x", padx=6, pady=2)

        ttk.Label(left, text="Humidity").pack(anchor="w", padx=6, pady=(2, 0))
        ttk.Entry(left, textvariable=self.humidity_var, width=18).pack(fill="x", padx=6, pady=2)

        ttk.Label(left, text="Risk Status").pack(anchor="w", padx=6, pady=(2, 0))
        ttk.Combobox(
            left,
            textvariable=self.status_var,
            values=["SAFE", "WARNING", "DANGER"],
            width=16,
        ).pack(fill="x", padx=6, pady=2)

        ttk.Button(left, text="Publish Fake ESP32 Alert", command=self.publish_fake_alert).pack(fill="x", padx=6, pady=(6, 2))
        ttk.Button(left, text="START FULL CAVE MISSION", style="Success.TButton", command=self.start_full_mission).pack(fill="x", padx=6, pady=2)
        ttk.Button(left, text="Speak Latest Report", command=self.speak_latest_report).pack(fill="x", padx=6, pady=2)

        ttk.Separator(left).pack(fill="x", padx=6, pady=4)

        btn_row = ttk.Frame(left)
        btn_row.pack(fill="x", padx=6, pady=2)
        btn_row.columnconfigure(0, weight=1)
        btn_row.columnconfigure(1, weight=1)
        ttk.Button(btn_row, text="Cancel Nav", command=self.cancel_nav).grid(row=0, column=0, sticky="ew", padx=2)
        ttk.Button(btn_row, text="E-STOP", style="Danger.TButton", command=self.e_stop).grid(row=0, column=1, sticky="ew", padx=2)

        self.pose_label = self.make_live_label(right, "Current /amcl_pose", "Waiting...")
        self.nav_label = self.make_live_label(right, "Nav2 Status", "Idle")
        self.cave_label = self.make_live_text(right, "/cave_node/status")
        self.mold_label = self.make_live_text(right, "/mold_detector/status")
        self.tag_label = self.make_live_text(right, "/apriltag/detections")
        self.report_label = self.make_live_text(right, "/inspection_report")

    def build_graph_tab(self):
        top_bar = ttk.Frame(self.graph_tab)
        top_bar.pack(fill="x", padx=4, pady=(4, 0))

        ttk.Label(top_bar, text="Real ESP32 data (solid) vs. simulated nodes (dashed)", font=FONT_BOLD).pack(side="left")
        ttk.Button(top_bar, text="Clear History", command=self.clear_sensor_history).pack(side="right")

        sim_bar = ttk.Frame(self.graph_tab)
        sim_bar.pack(fill="x", padx=4, pady=2)

        ttk.Checkbutton(
            sim_bar, text="Simulate extra nodes (based on real data)",
            variable=self.fake_enabled, command=self.on_fake_toggle,
        ).pack(side="left", padx=(2, 8))

        ttk.Label(sim_bar, text="Count:").pack(side="left")
        ttk.Spinbox(
            sim_bar, from_=0, to=4, width=3, textvariable=self.fake_count_var,
            command=self.on_fake_toggle,
        ).pack(side="left", padx=(2, 0))

        chart_frame = ttk.Frame(self.graph_tab)
        chart_frame.pack(fill="both", expand=True, padx=4, pady=4)

        self.fig = Figure(figsize=(5.4, 2.2), dpi=80)
        self.ax_temp = self.fig.add_subplot(211)
        self.ax_hum = self.fig.add_subplot(212)

        self.ax_temp.set_title("Temperature (°C)", fontsize=7)
        self.ax_hum.set_title("Humidity (%)", fontsize=7)
        self.ax_hum.set_xlabel("Seconds since start", fontsize=6)
        self.ax_temp.tick_params(labelsize=6)
        self.ax_hum.tick_params(labelsize=6)
        self.fig.tight_layout()

        self.graph_canvas = FigureCanvasTkAgg(self.fig, master=chart_frame)
        self.graph_canvas.draw()
        self.graph_canvas.get_tk_widget().pack(fill="both", expand=True)

        table_frame = ttk.LabelFrame(self.graph_tab, text="Latest Reading Per Node")
        table_frame.pack(fill="x", padx=4, pady=(0, 4))

        self.sensor_tree = ttk.Treeview(
            table_frame,
            columns=("type", "panel", "temp", "hum", "status", "updated"),
            show="tree headings",
            height=3,
        )
        self.sensor_tree.heading("#0", text="Node ID")
        self.sensor_tree.heading("type", text="Type")
        self.sensor_tree.heading("panel", text="Panel ID")
        self.sensor_tree.heading("temp", text="Temp (°C)")
        self.sensor_tree.heading("hum", text="Humidity (%)")
        self.sensor_tree.heading("status", text="Status")
        self.sensor_tree.heading("updated", text="Last Update")

        self.sensor_tree.column("#0", width=90)
        self.sensor_tree.column("type", width=50)
        self.sensor_tree.column("panel", width=80)
        self.sensor_tree.column("temp", width=70)
        self.sensor_tree.column("hum", width=80)
        self.sensor_tree.column("status", width=70)
        self.sensor_tree.column("updated", width=80)

        self.sensor_tree.tag_configure("SAFE", foreground=COL_SUCCESS)
        self.sensor_tree.tag_configure("WARNING", foreground=COL_WARNING)
        self.sensor_tree.tag_configure("DANGER", foreground=COL_DANGER)
        self.sensor_tree.tag_configure("SIM", foreground=COL_MUTED)

        self.sensor_tree.pack(fill="x", padx=4, pady=4)

    def build_log_tab(self):
        top = ttk.LabelFrame(self.log_tab, text="Live Commands")
        top.pack(fill="x", padx=4, pady=4)

        self.cmd_vel_label = self.make_live_label(top, "/cmd_vel", "Waiting...")
        self.robot_cmd_label = self.make_live_label(top, "/robot_command", "Waiting...")
        self.safe_cmd_label = self.make_live_label(top, "/safe_robot_command", "Waiting...")

        log_frame = ttk.LabelFrame(self.log_tab, text="Panel Log")
        log_frame.pack(fill="both", expand=True, padx=4, pady=4)

        self.log_box = ScrolledText(log_frame, height=7, font=FONT_MONO, bg=COL_PANEL, fg=COL_TEXT)
        self.log_box.pack(fill="both", expand=True, padx=4, pady=4)

    def make_live_label(self, parent, title, value):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", padx=6, pady=2)

        ttk.Label(frame, text=title, width=18, font=FONT_BOLD).pack(side="left")
        label = ttk.Label(frame, text=value)
        label.pack(side="left", fill="x", expand=True)

        return label

    def make_live_text(self, parent, title):
        ttk.Label(parent, text=title, font=FONT_BOLD).pack(anchor="w", padx=6, pady=(4, 0))
        box = tk.Text(parent, height=2, wrap="word", font=FONT_MONO, bg=COL_PANEL, fg=COL_TEXT)
        box.pack(fill="x", padx=6, pady=2)
        return box

    # ============================================================
    # ROS SPIN
    # ============================================================

    def spin_ros(self):
        while rclpy.ok() and not self._closing:
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            except Exception as e:
                print(f"ROS spin error: {e}")

    def ros_ui_callback(self, kind, text):
        self.root.after(0, lambda: self.handle_ros_ui(kind, text))

    def handle_ros_ui(self, kind, text):
        if kind == "log":
            self.log(text)
        elif kind == "pose":
            self.pose_label.config(text=text)
        elif kind == "nav":
            self.nav_label.config(text=text)
            self.log(text)
        elif kind == "cave":
            self.set_text(self.cave_label, text)
        elif kind == "mold":
            self.set_text(self.mold_label, text)
        elif kind == "tag":
            self.set_text(self.tag_label, text)
        elif kind == "report":
            self.set_text(self.report_label, text)
        elif kind == "cmd_vel":
            self.cmd_vel_label.config(text=text)
        elif kind == "robot_cmd":
            self.robot_cmd_label.config(text=text)
        elif kind == "safe_cmd":
            self.safe_cmd_label.config(text=text)
        elif kind == "rviz_pose":
            self.handle_rviz_pose(text)
        elif kind == "sensor_data":
            self.handle_sensor_data(text)

    def set_text(self, widget, text):
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, text)

    # ============================================================
    # RVIZ2 WAYPOINT CAPTURE
    # ============================================================

    def handle_rviz_pose(self, pose):
        self._last_rviz_pose = pose
        self.rviz_last_var.set(
            f"Captured: x={pose['x']:.2f}, y={pose['y']:.2f}, yaw={pose['yaw']:.2f} rad\n"
            f"(at {time.strftime('%H:%M:%S')})"
        )
        self.apply_last_rviz_pose(switch_tab=True, suggest_name=True)

    def apply_last_rviz_pose(self, switch_tab=False, suggest_name=False):
        pose = getattr(self, "_last_rviz_pose", None)

        if pose is None:
            messagebox.showinfo("No RViz2 pose", "No pose has been captured from RViz2 yet.")
            return

        self.x_var.set(f"{pose['x']:.3f}")
        self.y_var.set(f"{pose['y']:.3f}")
        self.yaw_var.set(f"{pose['yaw']:.3f}")

        if suggest_name and self.wp_name_var.get() in self.waypoints:
            new_name = next_waypoint_name(self.waypoints)
            self.wp_name_var.set(new_name)
            self.node_id_var.set(new_name)
            self.panel_id_var.set(new_name)

        if switch_tab:
            self.notebook.select(self.waypoint_tab)

        self.log("RViz2 pose applied to waypoint editor fields. Set name/IDs and click Save.")

    # ============================================================
    # PROCESS LAUNCH
    # ============================================================

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
            self.log(f"Started {name}. Log: {proc.log_file}")

        self.refresh_process_table()

    def start_robot_bringup(self):
        for name, cmd, cwd in CORE_PROCESSES:
            self.start_process(name, cmd, cwd)
            time.sleep(0.2)

        self.ros_node.speak("Robot bringup started.")
        self.update_header_status()

    def start_nav2(self):
        map_path = Path(self.map_var.get())

        if not map_path.exists():
            messagebox.showerror("Map Missing", f"Map file does not exist:\n{map_path}")
            return

        cmd = (
            f"ros2 launch pusantara_navigation navigation.launch.py "
            f"use_sim_time:=False autostart:=True "
            f"map:={map_path} "
            f"params_file:={DEFAULT_NAV2_PARAMS}"
        )

        self.start_process("Nav2 Navigation", cmd)
        self.ros_node.speak("Navigation is starting with selected map.")
        self.update_header_status()

    def open_rviz(self):
        self.start_process("RViz2", "rviz2")
        self.log("RViz2 opened. Map will appear after Nav2/map server is running.")
        self.update_header_status()

    def start_vision(self):
        for name, cmd, cwd in VISION_PROCESSES:
            self.start_process(name, cmd, cwd)
            time.sleep(0.2)

        self.ros_node.speak("Camera and AprilTag detector started.")
        self.update_header_status()

    def start_cavecare_nodes(self):
        for name, cmd, cwd in CAVECARE_PROCESSES:
            self.start_process(name, cmd, cwd)
            time.sleep(0.2)

        self.ros_node.speak("CaveCare mission nodes started.")
        self.update_header_status()

    def start_all(self):
        self.start_robot_bringup()
        time.sleep(0.5)
        self.start_nav2()
        time.sleep(0.5)
        self.open_rviz()
        time.sleep(0.5)
        self.start_vision()
        time.sleep(0.5)
        self.start_cavecare_nodes()

    def stop_all_processes(self):
        if not self.processes:
            self.log("No launched processes to stop.")
            return

        self.log(f"Stopping {len(self.processes)} process(es)...")
        threading.Thread(target=self._stop_all_worker, daemon=True).start()

    def _stop_all_worker(self):
        try:
            self.ros_node.stop_robot()
        except Exception:
            pass

        threads = []

        for name, proc in list(self.processes.items()):
            def _stop_one(name=name, proc=proc):
                try:
                    proc.stop()
                    self.root.after(0, lambda: self.log(f"Stopped: {name}"))
                except Exception as e:
                    self.root.after(0, lambda: self.log(f"Error stopping {name}: {e}"))
                self.root.after(0, self.refresh_process_table)

            t = threading.Thread(target=_stop_one, daemon=True)
            t.start()
            threads.append(t)

        # stop them concurrently so total wait is ~seconds, not
        # seconds-per-process
        for t in threads:
            t.join(timeout=5.0)

        self.processes.clear()
        self.root.after(0, self.refresh_process_table)
        self.root.after(0, lambda: self.log("All launched processes stopped."))
        self.root.after(0, self.update_header_status)

    def kill_all_ros2(self):
        if not messagebox.askyesno(
            "Kill all ROS2?",
            "This will kill ros2 and rviz2 processes, including ones opened from other terminals. Continue?",
        ):
            return

        self.e_stop()

        try:
            subprocess.run(["pkill", "-f", "-9", "ros2"], timeout=3)
        except Exception:
            pass

        try:
            subprocess.run(["pkill", "-f", "-9", "rviz2"], timeout=3)
        except Exception:
            pass

        try:
            subprocess.run(["bash", "-lc", "ros2 daemon stop"], timeout=3)
        except Exception:
            pass

        self.processes.clear()
        self.refresh_process_table()
        self.log("Killed ROS2/RViz2 processes.")
        self.update_header_status()

    def update_header_status(self):
        running = [n for n, p in self.processes.items() if p.is_running()]

        if running:
            self.header_status_label.config(text=f"● {len(running)} PROCESS(ES) RUNNING", fg=COL_SUCCESS)
        else:
            self.header_status_label.config(text="● SYSTEM IDLE", fg="#9ca3af")

    # ============================================================
    # WAYPOINTS
    # ============================================================

    def load_waypoints(self):
        WAYPOINT_FILE.parent.mkdir(parents=True, exist_ok=True)

        if not WAYPOINT_FILE.exists():
            data = {
                "home": {
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "node_id": "home",
                    "panel_id": "home",
                    "tag_id": 0,
                },
                "node_1": {
                    "x": 1.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "node_id": "node_1",
                    "panel_id": "panel_a",
                    "tag_id": 1,
                },
            }

            with open(WAYPOINT_FILE, "w") as f:
                json.dump(data, f, indent=2)

            return data

        try:
            with open(WAYPOINT_FILE, "r") as f:
                return json.load(f)
        except Exception:
            return {}

    def save_waypoints(self):
        WAYPOINT_FILE.parent.mkdir(parents=True, exist_ok=True)

        with open(WAYPOINT_FILE, "w") as f:
            json.dump(self.waypoints, f, indent=2)

    def refresh_waypoint_list(self):
        names = sorted(self.waypoints.keys())
        self.wp_combo["values"] = names

        if names and self.wp_name_var.get() not in names:
            self.wp_name_var.set(names[0])

        self.refresh_waypoint_table()
        self.load_selected_waypoint()

    def refresh_waypoint_table(self):
        for item in self.wp_tree.get_children():
            self.wp_tree.delete(item)

        for name, wp in self.waypoints.items():
            self.wp_tree.insert(
                "",
                "end",
                iid=name,
                text=name,
                values=(
                    wp.get("node_id", ""),
                    wp.get("panel_id", ""),
                    wp.get("tag_id", ""),
                    wp.get("x", ""),
                    wp.get("y", ""),
                    wp.get("yaw", ""),
                ),
            )

    def on_wp_tree_select(self, event):
        sel = self.wp_tree.selection()
        if sel:
            self.wp_name_var.set(sel[0])
            self.load_selected_waypoint()

    def load_selected_waypoint(self):
        name = self.wp_name_var.get()

        if name not in self.waypoints:
            return

        wp = self.waypoints[name]

        self.node_id_var.set(str(wp.get("node_id", name)))
        self.panel_id_var.set(str(wp.get("panel_id", name)))
        self.tag_id_var.set(str(wp.get("tag_id", 0)))
        self.x_var.set(str(wp.get("x", 0.0)))
        self.y_var.set(str(wp.get("y", 0.0)))
        self.yaw_var.set(str(wp.get("yaw", 0.0)))

    def save_waypoint_from_fields(self):
        name = self.wp_name_var.get().strip()

        if not name:
            messagebox.showerror("Error", "Waypoint name is empty.")
            return

        try:
            self.waypoints[name] = {
                "x": float(self.x_var.get()),
                "y": float(self.y_var.get()),
                "yaw": float(self.yaw_var.get()),
                "node_id": self.node_id_var.get().strip() or name,
                "panel_id": self.panel_id_var.get().strip() or name,
                "tag_id": int(self.tag_id_var.get()),
            }
        except ValueError:
            messagebox.showerror("Error", "X, Y, Yaw, and AprilTag ID must be numbers.")
            return

        self.save_waypoints()
        self.refresh_waypoint_list()
        self.log(f"Saved waypoint: {name}")

    def delete_selected_waypoint(self):
        name = self.wp_name_var.get().strip()

        if name not in self.waypoints:
            messagebox.showerror("Error", "Waypoint not found.")
            return

        if name == "home":
            messagebox.showerror("Error", "Cannot delete the HOME waypoint.")
            return

        if not messagebox.askyesno("Delete waypoint", f"Delete waypoint '{name}'?"):
            return

        del self.waypoints[name]
        self.save_waypoints()
        self.refresh_waypoint_list()
        self.log(f"Deleted waypoint: {name}")

    def use_current_pose(self):
        pose = self.ros_node.current_pose

        if pose is None:
            messagebox.showwarning("No pose", "No /amcl_pose received yet. Set initial pose first.")
            return

        self.x_var.set(f"{pose['x']:.3f}")
        self.y_var.set(f"{pose['y']:.3f}")
        self.yaw_var.set(f"{pose['yaw']:.3f}")

    def set_home_from_current_pose(self):
        pose = self.ros_node.current_pose

        if pose is None:
            messagebox.showwarning("No pose", "No /amcl_pose received yet. Set initial pose first.")
            return

        self.waypoints["home"] = {
            "x": pose["x"],
            "y": pose["y"],
            "yaw": pose["yaw"],
            "node_id": "home",
            "panel_id": "home",
            "tag_id": 0,
        }

        self.save_waypoints()
        self.refresh_waypoint_list()
        self.wp_name_var.set("home")
        self.load_selected_waypoint()
        self.log("HOME waypoint saved from current robot pose.")

    def selected_waypoint(self):
        name = self.wp_name_var.get()

        if name not in self.waypoints:
            messagebox.showerror("Error", "Waypoint not found.")
            return None, None

        return name, self.waypoints[name]

    def go_selected_waypoint(self):
        name, wp = self.selected_waypoint()

        if not wp:
            return

        self.ros_node.send_goal(name, wp["x"], wp["y"], wp["yaw"])

    def return_home(self):
        if "home" not in self.waypoints:
            messagebox.showerror("Error", "No home waypoint saved.")
            return

        wp = self.waypoints["home"]
        self.ros_node.send_goal("home", wp["x"], wp["y"], wp["yaw"])

    def publish_home_initial_pose(self):
        if "home" not in self.waypoints:
            messagebox.showerror("Error", "No home waypoint saved.")
            return

        wp = self.waypoints["home"]
        self.ros_node.publish_initial_pose(wp["x"], wp["y"], wp["yaw"])

    # ============================================================
    # MISSION
    # ============================================================

    def publish_fake_alert(self):
        name, wp = self.selected_waypoint()

        if not wp:
            return

        self.ros_node.publish_cave_alert(
            node_id=wp.get("node_id", name),
            panel_id=wp.get("panel_id", name),
            tag_id=wp.get("tag_id", 0),
            temp=self.temp_var.get(),
            humidity=self.humidity_var.get(),
            status=self.status_var.get(),
        )

    def start_full_mission(self):
        self.publish_fake_alert()
        self.log("Full mission triggered. cave_mission_manager should navigate, scan, report, and return home.")

    def speak_latest_report(self):
        text = self.report_label.get("1.0", tk.END).strip()

        if not text:
            text = "No inspection report is available yet."

        self.ros_node.speak(f"Latest inspection report. {text}")

    def cancel_nav(self):
        self.ros_node.cancel_nav()

    def e_stop(self):
        self.ros_node.cancel_nav()
        self.ros_node.stop_robot()
        self.log("E-STOP activated.")

    # ============================================================
    # SENSOR GRAPHS
    # ============================================================

    def handle_sensor_data(self, data):
        node_id = str(data.get("node_id", "unknown"))
        t = time.time() - self.sensor_start_time

        hist = self.sensor_history[node_id]

        if "temperature" in data:
            hist["t"].append(t)
            hist["temp"].append(float(data["temperature"]))

        if "humidity" in data:
            hist["hum"].append(float(data["humidity"]))

        self.sensor_latest_status[node_id] = {
            "panel_id": data.get("panel_id", ""),
            "temperature": data.get("temperature", ""),
            "humidity": data.get("humidity", ""),
            "status": data.get("status", ""),
            "updated": time.strftime("%H:%M:%S"),
            "is_sim": False,
        }

        self.refresh_sensor_table()

    def refresh_sensor_table(self):
        for item in self.sensor_tree.get_children():
            self.sensor_tree.delete(item)

        for node_id, info in self.sensor_latest_status.items():
            status = str(info.get("status", "")).upper()
            is_sim = info.get("is_sim", False)
            tags = []
            if status in ("SAFE", "WARNING", "DANGER"):
                tags.append(status)
            if is_sim:
                tags.append("SIM")

            self.sensor_tree.insert(
                "",
                "end",
                text=node_id,
                values=(
                    "SIM" if is_sim else "REAL",
                    info.get("panel_id", ""),
                    info.get("temperature", ""),
                    info.get("humidity", ""),
                    status,
                    info.get("updated", ""),
                ),
                tags=tuple(tags),
            )

    def get_real_latest(self):
        """Most recent (temp, hum) among real (non-simulated) nodes, or None."""
        latest = None
        latest_t = -1.0

        for node_id, hist in self.sensor_history.items():
            if node_id in self.fake_node_keys or not hist["t"]:
                continue

            t = hist["t"][-1]
            if t > latest_t:
                latest_t = t
                temp = hist["temp"][-1] if hist["temp"] else None
                hum = hist["hum"][-1] if hist["hum"] else None
                latest = (temp, hum)

        return latest

    def on_fake_toggle(self):
        if not self.fake_enabled.get():
            # remove simulated nodes/history so the graph goes back to real-only
            for key in list(self.fake_node_keys):
                self.sensor_history.pop(key, None)
                self.sensor_latest_status.pop(key, None)
            self.fake_node_keys.clear()
            self.fake_state.clear()
            self.refresh_sensor_table()
            self.redraw_graphs()

    def generate_fake_readings(self):
        if not self.fake_enabled.get():
            return

        base = self.get_real_latest()

        if base is None or base[0] is None:
            try:
                base_temp = float(self.temp_var.get())
            except ValueError:
                base_temp = 28.0
        else:
            base_temp = base[0]

        if base is None or base[1] is None:
            try:
                base_hum = float(self.humidity_var.get())
            except ValueError:
                base_hum = 80.0
        else:
            base_hum = base[1]

        count = max(0, min(4, self.fake_count_var.get()))
        desired_keys = [f"Sim Node {chr(65 + i)}" for i in range(count)]

        for key in list(self.fake_node_keys):
            if key not in desired_keys:
                self.fake_node_keys.discard(key)
                self.fake_state.pop(key, None)
                self.sensor_history.pop(key, None)
                self.sensor_latest_status.pop(key, None)

        t = time.time() - self.sensor_start_time

        for key in desired_keys:
            self.fake_node_keys.add(key)

            if key not in self.fake_state:
                self.fake_state[key] = {
                    "temp": base_temp + random.uniform(-1.5, 1.5),
                    "hum": base_hum + random.uniform(-4.0, 4.0),
                }

            st = self.fake_state[key]

            # random walk that gently drifts back toward the real reading,
            # so it tracks real conditions but still looks like its own sensor
            st["temp"] += (base_temp - st["temp"]) * 0.12 + random.uniform(-0.25, 0.25)
            st["hum"] += (base_hum - st["hum"]) * 0.12 + random.uniform(-0.8, 0.8)
            st["hum"] = max(0.0, min(100.0, st["hum"]))

            hist = self.sensor_history[key]
            hist["t"].append(t)
            hist["temp"].append(st["temp"])
            hist["hum"].append(st["hum"])

            if st["hum"] >= 90:
                status = "DANGER"
            elif st["hum"] >= 78:
                status = "WARNING"
            else:
                status = "SAFE"

            self.sensor_latest_status[key] = {
                "panel_id": "sim",
                "temperature": round(st["temp"], 1),
                "humidity": round(st["hum"], 1),
                "status": status,
                "updated": time.strftime("%H:%M:%S"),
                "is_sim": True,
            }

        self.refresh_sensor_table()

    def redraw_graphs(self):
        self.ax_temp.clear()
        self.ax_hum.clear()

        self.ax_temp.set_title("Temperature (°C)", fontsize=7)
        self.ax_hum.set_title("Humidity (%)", fontsize=7)
        self.ax_hum.set_xlabel("Seconds since start", fontsize=6)
        self.ax_temp.tick_params(labelsize=6)
        self.ax_hum.tick_params(labelsize=6)

        any_data = False

        for node_id, hist in self.sensor_history.items():
            if not hist["t"]:
                continue

            any_data = True
            is_sim = node_id in self.fake_node_keys
            label = f"{node_id} (sim)" if is_sim else node_id
            style = dict(linestyle="--", alpha=0.7, linewidth=1.2) if is_sim else dict(linestyle="-", linewidth=1.5)

            if hist["temp"]:
                self.ax_temp.plot(list(hist["t"])[-len(hist["temp"]):], list(hist["temp"]), label=label, markersize=2, **style)

            if hist["hum"]:
                self.ax_hum.plot(list(hist["t"])[-len(hist["hum"]):], list(hist["hum"]), label=label, markersize=2, **style)

        if any_data:
            self.ax_temp.legend(loc="upper left", fontsize=6)
            self.ax_hum.legend(loc="upper left", fontsize=6)

        self.fig.tight_layout()
        self.graph_canvas.draw_idle()

    def clear_sensor_history(self):
        if not messagebox.askyesno("Clear history", "Clear all recorded sensor history?"):
            return

        self.sensor_history.clear()
        self.sensor_latest_status.clear()
        self.fake_node_keys.clear()
        self.fake_state.clear()
        self.sensor_start_time = time.time()
        self.refresh_sensor_table()
        self.redraw_graphs()
        self.log("Sensor history cleared.")

    def periodic_graph_update(self):
        if not self._closing:
            self.generate_fake_readings()
            self.redraw_graphs()
            self.root.after(2000, self.periodic_graph_update)

    # ============================================================
    # GENERAL
    # ============================================================

    def browse_map(self):
        path = filedialog.askopenfilename(
            title="Select Nav2 Map YAML",
            initialdir=str(WS / "maps"),
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")],
        )

        if path:
            self.map_var.set(path)

    def refresh_process_table(self):
        for item in self.proc_tree.get_children():
            self.proc_tree.delete(item)

        for name, proc in self.processes.items():
            status = proc.status_text()
            self.proc_tree.insert(
                "",
                "end",
                text=name,
                values=(status, str(proc.log_file)),
                tags=(status,),
            )

        self.update_header_status()

    def periodic_update(self):
        if not self._closing:
            self.refresh_process_table()
            self.root.after(1500, self.periodic_update)

    def log(self, text):
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {text}\n"

        try:
            self.log_box.insert(tk.END, line)
            self.log_box.see(tk.END)
        except Exception:
            pass

        print(line, end="")

    def on_close(self):
        self._closing = True
        self.stop_all_processes()

        try:
            self.ros_node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)

    root = tk.Tk()
    app = CaveCarePanel(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)

    root.mainloop()


if __name__ == "__main__":
    main()
