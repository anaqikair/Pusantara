import sys
import os

# Prevent OpenCV's pre-compiled Qt plugins from conflicting with system PyQt5
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import time
import socket
import subprocess
import signal
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan

# Module imports
from pusantara_admin.system_monitor import get_system_stats
from pusantara_admin.camera_widget import CameraWidget
from pusantara_admin.lidar_widget import LidarWidget

# ═══════════════════════════════════════════════════════════════════════════
# Color system (Premium Malaysian Cultural Heritage Theme: Warm Wood & Gold)
# ═══════════════════════════════════════════════════════════════════════════
BG_MAIN = "#26211D"
BG_CARD = "#3A322C"
BG_ACCENT = "#C89B3C"
BG_ACCENT_HOVER = "#E2B555"
COLOR_TEXT = "#F3EFE9"
COLOR_MUTED = "#A4998E"
COLOR_SUCCESS = "#4ADE80"
COLOR_DANGER = "#EF4444"
COLOR_WARNING = "#F59E0B"
COLOR_INFO = "#E0B75E"

# ═══════════════════════════════════════════════════════════════════════════
# NOTE ON SIZING: this console targets a 7" / 1024x600 kiosk touch display.
# Every metric below (padding, min-height, font-size) was pulled down from
# the original "desktop" values specifically so nothing gets clipped by the
# available screen real estate. If you run this on a larger monitor it will
# simply look compact rather than cramped - that's the intended trade-off.
# ═══════════════════════════════════════════════════════════════════════════
STYLE_SHEET = f"""
QMainWindow {{
    background-color: {BG_MAIN};
}}
QWidget#loginPage, QWidget#mainPage {{
    background-color: {BG_MAIN};
}}
QLabel {{
    color: {COLOR_TEXT};
    font-family: 'Outfit', 'Inter', 'Segoe UI', sans-serif;
    font-size: 13px;
}}
QLabel#titleLabel {{
    font-size: 18px;
    font-weight: 900;
    color: {COLOR_INFO};
    padding: 2px 0px;
    letter-spacing: 0.5px;
}}
QLabel#sectionHeader {{
    font-size: 14px;
    font-weight: 800;
    color: {COLOR_INFO};
}}
/* Topbar styling */
QListWidget#topbar {{
    background-color: {BG_CARD};
    color: {COLOR_MUTED};
    border: none;
    border-bottom: 2px solid #1A1512;
    border-radius: 8px;
    font-size: 13px;
    font-weight: 800;
    outline: none;
}}
QListWidget#topbar::item {{
    padding: 0px 12px;
    border-bottom: 3px solid transparent;
    border-radius: 0px;
}}
QListWidget#topbar::item:selected {{
    background-color: {BG_MAIN};
    color: {COLOR_INFO};
    border-bottom: 3px solid {COLOR_INFO};
}}
QListWidget#topbar::item:hover {{
    background-color: #483E38;
    color: {COLOR_INFO};
}}
/* Views styling */
QStackedWidget {{
    background-color: {BG_MAIN};
}}
QPushButton {{
    background-color: {BG_CARD};
    color: {COLOR_TEXT};
    border: 1px solid #483E38;
    border-radius: 8px;
    padding: 8px 12px;
    font-weight: 800;
    font-size: 13px;
    min-height: 34px;
}}
QPushButton:hover {{
    background-color: #483E38;
    border: 1px solid {COLOR_INFO};
    color: {COLOR_INFO};
}}
QPushButton:pressed {{
    background-color: #26211D;
}}
QPushButton:disabled {{
    background-color: #2A2521;
    color: #5C544C;
    border: 1px solid #383028;
}}
QPushButton#btnEmergency {{
    background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 {COLOR_DANGER}, stop:1 #991B1B);
    color: white;
    font-size: 17px;
    font-weight: 900;
    border: none;
    border-radius: 10px;
    padding: 12px;
}}
QPushButton#btnEmergency:hover {{
    background-color: #DC2626;
}}
QPushButton#btnEmergency:pressed {{
    background-color: #7F1D1D;
}}
QPushButton#btnRecovery {{
    background-color: {BG_CARD};
    border: 1px solid {COLOR_WARNING};
    color: {COLOR_WARNING};
    font-size: 12px;
    min-height: 28px;
    padding: 4px 8px;
}}
QPushButton#btnRecovery:hover {{
    background-color: #4A3F2E;
}}
QLineEdit, QComboBox {{
    background-color: {BG_CARD};
    color: {COLOR_TEXT};
    border: 1px solid #483E38;
    border-radius: 6px;
    padding: 7px 10px;
    font-size: 13px;
}}
QLineEdit:focus, QComboBox:focus {{
    border: 1px solid {COLOR_INFO};
    background-color: #1A1512;
}}
QProgressBar {{
    border: 1px solid #483E38;
    border-radius: 5px;
    background-color: {BG_MAIN};
    text-align: center;
    color: {COLOR_TEXT};
    font-weight: 800;
    font-size: 12px;
    min-height: 18px;
    max-height: 18px;
}}
QProgressBar::chunk {{
    background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {COLOR_INFO}, stop:1 {COLOR_SUCCESS});
    border-radius: 4px;
}}
QListWidget, QTextEdit, QTableWidget {{
    background-color: {BG_CARD};
    color: {COLOR_TEXT};
    border: 1px solid #483E38;
    border-radius: 8px;
    padding: 6px;
    font-size: 12px;
}}
QHeaderView::section {{
    background-color: {BG_MAIN};
    color: {COLOR_MUTED};
    padding: 8px;
    border: none;
    border-bottom: 2px solid #483E38;
    font-weight: 800;
    font-size: 12px;
}}
QGroupBox {{
    font-weight: 800;
    font-size: 13px;
    border: 1px solid #483E38;
    background-color: {BG_CARD};
    border-radius: 10px;
    margin-top: 16px;
    padding-top: 4px;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 1px 8px;
    color: {COLOR_INFO};
    background-color: {BG_CARD};
    border-radius: 4px;
    left: 8px;
    top: 2px;
}}
QScrollArea {{
    border: none;
    background-color: transparent;
}}
QScrollBar:vertical {{
    border: none;
    background: {BG_MAIN};
    width: 10px;
    border-radius: 5px;
}}
QScrollBar::handle:vertical {{
    background: #483E38;
    min-height: 24px;
    border-radius: 5px;
}}
QScrollBar::handle:vertical:hover {{
    background: {COLOR_INFO};
}}
QScrollBar:horizontal {{
    border: none;
    background: {BG_MAIN};
    height: 10px;
    border-radius: 5px;
}}
QScrollBar::handle:horizontal {{
    background: #483E38;
    min-width: 24px;
    border-radius: 5px;
}}
"""

# Estimated blocking durations (ms) for each PRIZM firmware sequence. These
# mirror the delay() math in the Arduino sketch so the GUI can lock out
# controls for roughly as long as the microcontroller will actually be busy,
# instead of letting the operator flood the serial buffer with commands the
# firmware can't act on yet.
GRIPPER_COMMAND_DURATIONS_MS = {
    "OPEN": 1100,
    "CLOSE": 1100,
    "FORWARD": 1000,
    "BACK": 1000,
    "TAKEIN": 4600,
    "TAKEOUT": 4600,
}


class RosSignals(QObject):
    image_received = pyqtSignal(object)
    scan_received = pyqtSignal(object)
    cmd_received = pyqtSignal(str, bool)
    speech_received = pyqtSignal(str, str)
    apriltag_received = pyqtSignal(str)
    yolo_received = pyqtSignal(str)
    imu_cal_received = pyqtSignal(str)


class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)


class SystemMonitorThread(QThread):
    stats_updated = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._is_running = True

    def run(self):
        while self._is_running:
            stats = get_system_stats()
            self.stats_updated.emit(stats)
            for _ in range(10):
                if not self._is_running:
                    break
                time.sleep(0.1)

    def stop(self):
        self._is_running = False
        self.quit()
        self.wait()


class PusantaraRosNode(Node):
    def __init__(self):
        super().__init__('pusantara_admin_console')
        self.signals = RosSignals()

        # Publishers
        self.cmd_pub = self.create_publisher(String, '/robot_command', 10)
        self.safe_cmd_pub = self.create_publisher(String, '/safe_robot_command', 10)
        self.user_text_pub = self.create_publisher(String, '/user_text', 10)
        self.tts_text_pub = self.create_publisher(String, '/tts_text', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.cmd_sub = self.create_subscription(
            String, '/robot_command', self.cmd_callback, 10
        )
        self.safe_cmd_sub = self.create_subscription(
            String, '/safe_robot_command', self.safe_cmd_callback, 10
        )

        # Interaction & perception subscribers
        # NOTE: fixed - speech_to_text actually publishes on '/speech_text'
        # (see architecture doc section 6.5). The old '/speech/text' topic
        # had no publisher, so STT transcripts never reached this console.
        self.speech_sub = self.create_subscription(
            String, '/speech_text', self.speech_callback, 10
        )
        self.tts_sub = self.create_subscription(
            String, '/tts_text', self.tts_callback, 10
        )
        self.user_text_sub = self.create_subscription(
            String, '/user_text', self.user_text_callback, 10
        )
        self.apriltag_sub = self.create_subscription(
            String, '/apriltag/detections', self.apriltag_callback, 10
        )
        self.yolo_sub = self.create_subscription(
            String, '/yolo/detections', self.yolo_callback, 10
        )
        # NEW: IMU calibration status wasn't wired up even though the admin
        # console is documented as a subscriber for it.
        self.imu_cal_sub = self.create_subscription(
            String, '/imu/cal_status', self.imu_cal_callback, 10
        )

    def image_callback(self, msg):
        self.signals.image_received.emit(msg)

    def scan_callback(self, msg):
        self.signals.scan_received.emit(msg)

    def cmd_callback(self, msg):
        self.signals.cmd_received.emit(msg.data, False)

    def safe_cmd_callback(self, msg):
        self.signals.cmd_received.emit(msg.data, True)

    def speech_callback(self, msg):
        self.signals.speech_received.emit(msg.data, "Voice (STT)")

    def tts_callback(self, msg):
        self.signals.speech_received.emit(msg.data, "Robot (TTS)")

    def user_text_callback(self, msg):
        self.signals.speech_received.emit(msg.data, "Operator (Text)")

    def apriltag_callback(self, msg):
        self.signals.apriltag_received.emit(msg.data)

    def yolo_callback(self, msg):
        self.signals.yolo_received.emit(msg.data)

    def imu_cal_callback(self, msg):
        self.signals.imu_cal_received.emit(msg.data)


class AdminConsoleWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.processes = getattr(self, "processes", {})
        self.node_groups = {}
        self.group_status_labels = {}
        self.group_error_labels = {}
        self.group_starting_until = {}
        self.launch_log_dir = os.path.expanduser("~/.ros/pusantara_admin_launcher_logs")
        if not os.path.exists(self.launch_log_dir):
            os.makedirs(self.launch_log_dir, exist_ok=True)
            
        self.setup_node_groups()

        self.setWindowTitle("PUSANTARA ADMIN CONSOLE")
        self.setMinimumSize(1024, 600)
        self.setStyleSheet(STYLE_SHEET)

        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        self.current_user = "None"
        self.current_role = "None"

        # Safety state
        self.estop_active = False
        self.lockout_active = False
        self.gripper_state = "UNKNOWN"
        self.arm_busy = False
        self._gripper_unlock_timer = QTimer(self)
        self._gripper_unlock_timer.setSingleShot(True)
        self._gripper_unlock_timer.timeout.connect(self._gripper_unlock)

        # Build views
        self.build_login_page()
        self.build_main_page()

        self.stacked_widget.setCurrentIndex(0)

        # Keep the window matched to the current screen size. This matters
        # specifically for VNC: the virtual display's resolution can change
        # mid-session (e.g. TigerVNC/x11vnc "remote resizing" when a client
        # reconnects with a different viewport size). The fit is debounced
        # and deferred to the next event-loop tick (see setup_responsive_screen)
        # to avoid the double-paint/ghosting artifact that shows up when the
        # window geometry is changed mid-layout, before the first paint.
        self.setup_responsive_screen()

    def setup_responsive_screen(self):
        self._resize_debounce = QTimer(self)
        self._resize_debounce.setSingleShot(True)
        self._resize_debounce.setInterval(120)
        self._resize_debounce.timeout.connect(self._apply_screen_fit)

        screen = QApplication.primaryScreen()
        if screen:
            screen.geometryChanged.connect(self._queue_screen_fit)
            screen.availableGeometryChanged.connect(self._queue_screen_fit)

        app = QApplication.instance()
        app.screenAdded.connect(self._on_screen_added)
        app.screenRemoved.connect(self._on_screen_removed)

        # Defer the first fit to the next event-loop tick (after show()),
        # instead of resizing synchronously while the window is still being
        # constructed. Doing it early is what produced the overlapping /
        # "ghosted" text seen on the login card over VNC.
        QTimer.singleShot(0, self._apply_screen_fit)

    def _queue_screen_fit(self, *args):
        self._resize_debounce.start()

    def _apply_screen_fit(self, *args):
        screen = QApplication.primaryScreen()
        if screen:
            self.setGeometry(screen.availableGeometry())
            self.updateGeometry()
            # Force a clean repaint rather than relying on the partial
            # update some VNC servers send after a geometry change.
            self.repaint()

    def closeEvent(self, event):
        if hasattr(self, 'sys_monitor_thread') and self.sys_monitor_thread.isRunning():
            self.sys_monitor_thread.stop()
        super().closeEvent(event)

    def _on_screen_added(self, screen):
        screen.geometryChanged.connect(self._queue_screen_fit)
        screen.availableGeometryChanged.connect(self._queue_screen_fit)
        self._queue_screen_fit()

    def _on_screen_removed(self, screen):
        self._queue_screen_fit()

    def start_ros(self):
        self.ros_node = PusantaraRosNode()

        self.ros_node.signals.image_received.connect(self.camera_widget.handle_ros_image)
        self.ros_node.signals.scan_received.connect(self.lidar_widget.handle_ros_scan)
        self.ros_node.signals.cmd_received.connect(self.handle_command_history)
        self.ros_node.signals.speech_received.connect(self.handle_speech_log)
        self.ros_node.signals.apriltag_received.connect(self.handle_apriltag_detection)
        self.ros_node.signals.yolo_received.connect(self.handle_yolo_detection)
        self.ros_node.signals.imu_cal_received.connect(self.handle_imu_cal_status)

        self.ros_thread = RosThread(self.ros_node)
        self.ros_thread.start()

        self.sys_monitor_thread = SystemMonitorThread()
        self.sys_monitor_thread.stats_updated.connect(self.update_system_stats)
        self.sys_monitor_thread.start()

        self.ros_monitor_timer = QTimer(self)
        self.ros_monitor_timer.timeout.connect(self.update_ros_monitor)
        self.ros_monitor_timer.start(2000)

        self.node_poller_timer = QTimer(self)
        self.node_poller_timer.timeout.connect(self.poll_nodes_status)
        self.node_poller_timer.start(1000)

        self.group_status_timer = QTimer(self)
        self.group_status_timer.timeout.connect(self.update_group_statuses)
        self.group_status_timer.start(1000)


    # ═══════════════════════════════════════════════════════════════════
    # CENTRAL LAUNCHER SYSTEM
    # ═══════════════════════════════════════════════════════════════════
    def setup_node_groups(self):
        # ── Individual node registry (single source of truth) ─────────────
        # Each entry is a unique launchable node/process.
        self.all_nodes = {
            "webcam_pub":            {"node_hint": "webcam_pub",            "command": "ros2 run camera_node webcam_pub",                                                   "kill_pattern": "webcam_pub",              "description": "USB camera"},
            "yolo_detector":         {"node_hint": "yolo_detector",         "command": "ros2 run vectra_yolo yolo_detector --ros-args -p debug:=true -p confidence_threshold:=0.6", "kill_pattern": "yolo_detector",    "description": "YOLO person detection"},
            "apriltag_detector":     {"node_hint": "apriltag_detector",     "command": "ros2 run apriltag_detector apriltag_detector --ros-args -p show_ui:=true",             "kill_pattern": "apriltag_detector",   "description": "AprilTag detection"},
            "apriltag_follower":     {"node_hint": "apriltag_follower",     "command": "ros2 run apriltag_follower apriltag_follower",                                          "kill_pattern": "apriltag_follower",   "description": "AprilTag follower"},
            "person_follower":       {"node_hint": "person_follower",       "command": "ros2 run person_follower person_follower",                                              "kill_pattern": "person_follower",     "description": "Person follower"},
            "face_node":             {"node_hint": "face_node",             "command": "ros2 run robot_face face_node",                                                        "kill_pattern": "face_node",           "description": "Robot face animation"},
            "tts_node":              {"node_hint": "tts_node",              "command": "ros2 run tts_node tts_node",                                                           "kill_pattern": "tts_node",            "description": "TTS node"},
            "tts_sender":            {"node_hint": "tts_sender",            "command": "ros2 run tts_node tts_sender",                                                         "kill_pattern": "tts_sender",          "description": "TTS sender"},
            "bridge_node":           {"node_hint": "bridge_node",           "command": "ros2 run serial_bridge bridge_node",                                                   "kill_pattern": "bridge_node",         "description": "Serial bridge to PRIZM"},
            "command_sender":        {"node_hint": "command_sender",        "command": "ros2 run serial_bridge command_sender",                                                "kill_pattern": "command_sender",      "description": "Serial command sender"},
            "motion_manager":        {"node_hint": "motion_manager",        "command": "ros2 run motion_manager motion_manager",                                               "kill_pattern": "motion_manager",      "description": "Safety stop layer"},
            "hlds_laser_publisher":  {"node_hint": "hlds_laser_publisher",  "command": "ros2 launch lidar_launch hlds_laser.launch.py",                                        "kill_pattern": "hlds_laser",          "description": "LiDAR sensor"},
            "imu_node":              {"node_hint": "imu_node",              "command": "ros2 run bno055_driver imu_node",                                                      "kill_pattern": "imu_node",            "description": "IMU sensor"},
            "odometry":              {"node_hint": "odometry",              "command": "ros2 launch vectra_odometry odometry.launch.py",                                        "kill_pattern": "odometry",            "description": "Wheel odometry"},
            "cmd_vel_bridge":        {"node_hint": "cmd_vel_to_robot_command", "command": "ros2 run cmd_vel_bridge cmd_vel_to_robot_command",                                  "kill_pattern": "cmd_vel_to_robot_command", "description": "Translate Nav2 cmd_vel"},
            "robot_bringup":         {"node_hint": "hlds_laser_publisher",  "command": "ros2 launch pusantara_bringup robot.launch.py",                                        "kill_pattern": "robot.launch",        "description": "Full bringup (LiDAR+IMU+odom)"},
            "slam_toolbox":          {"node_hint": "async_slam_toolbox_node", "command": "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/orin_nano/vectra_ws/config/slam.yaml", "kill_pattern": "slam_toolbox", "description": "SLAM toolbox"},
            "nav2":                  {"node_hint": "amcl",                  "command": "ros2 launch pusantara_navigation navigation.launch.py use_sim_time:=False autostart:=True map:=$HOME/vectra_ws/maps/pusantara_map1.yaml params_file:=$HOME/vectra_ws/src/pusantara_navigation/config/nav2_params.yaml", "kill_pattern": "nav2", "description": "Nav2 navigation stack"},
            "heritage_explainer":    {"node_hint": "heritage_explainer",    "command": "ros2 run heritage_explainer heritage_explainer",                                        "kill_pattern": "heritage_explainer",  "description": "Heritage narration"},
            "visitor_greeter":       {"node_hint": "visitor_greeter",       "command": "ros2 run visitor_greeter visitor_greeter",                                              "kill_pattern": "visitor_greeter",     "description": "Visitor greeter"},
            "keris_demo":            {"node_hint": "keris_demo",            "command": "ros2 run keris_demo keris_demo",                                                        "kill_pattern": "keris_demo",          "description": "Keris AR overlay"},
            "food_quiz":             {"node_hint": "food_quiz",             "command": "ros2 run food_quiz food_quiz",                                                          "kill_pattern": "food_quiz",           "description": "Food quiz GUI"},
            "batik_designer":        {"node_hint": "batik_designer",        "command": "ros2 run batik_designer batik_designer",                                               "kill_pattern": "batik_designer",      "description": "Batik design studio"},
            "rqt":                   {"node_hint": "rqt_gui_cpp",           "command": "rqt",                                                                                  "kill_pattern": "rqt",                 "description": "RQt GUI"},
            "rviz2":                 {"node_hint": "rviz2",                 "command": "rviz2",                                                                                "kill_pattern": "rviz2",               "description": "RViz2 visualizer"},
        }

        # ── Node groups: define which nodes belong to each feature ─────────
        # Groups reference keys from self.all_nodes
        self.node_groups = {
            "face_speech":       ["face_node", "tts_node"],
            "vision":            ["webcam_pub", "apriltag_detector", "yolo_detector"],
            "vision_tags_only": ["webcam_pub", "apriltag_detector"],
            "greeting":          ["webcam_pub", "yolo_detector", "visitor_greeter", "tts_node"],
            "lidar":             ["hlds_laser_publisher"],
            "lidar_safety":      ["hlds_laser_publisher", "motion_manager"],
            "motion_core":       ["bridge_node", "command_sender", "motion_manager"],
            "nav_cmd_bridge":    ["cmd_vel_bridge"],
            "core_robot":        ["robot_bringup"],
            "arm_gripper":       ["bridge_node", "command_sender"],
            "heritage_explainer":["webcam_pub", "apriltag_detector", "heritage_explainer", "tts_node"],
            "food_quiz":         ["food_quiz", "tts_node"],
            "batik_designer":    ["batik_designer"],
            "keris_ar":          ["webcam_pub", "keris_demo"],
            "speech_ai":         ["tts_node"],
            "slam_mapping":      ["robot_bringup", "slam_toolbox"],
            "nav2_navigation":   ["robot_bringup", "nav2", "cmd_vel_bridge"],
            "rqt":               ["rqt"],
            "rviz2":             ["rviz2"],
        }

    def _spec_for(self, key):
        """Return a spec-like dict for a node key from all_nodes."""
        n = self.all_nodes.get(key, {})
        return {"key": key, "node_hint": n.get("node_hint", key),
                "command": n.get("command", ""), "kill_pattern": n.get("kill_pattern", key),
                "description": n.get("description", ""), "required": True}

    def _ros_cmd(self, command: str):
        return [
            "bash",
            "-lc",
            "source /opt/ros/humble/setup.bash && source ~/vectra_ws/install/setup.bash && " + command
        ]

    def _launch_process(self, key, command):
        os.makedirs(self.launch_log_dir, exist_ok=True)
        log_path = os.path.join(self.launch_log_dir, f"{key}.log")
        log_file = open(log_path, "a")
        proc = subprocess.Popen(
            self._ros_cmd(command),
            stdout=log_file,
            stderr=log_file,
            preexec_fn=os.setsid
        )
        self.processes[key] = proc
        return proc

    def _normalize_node_name(self, name):
        return name.strip("/")

    def _get_active_nodes_safe(self):
        try:
            return [f"{ns}/{n}".replace("//", "/") for n, ns in self.ros_node.get_node_names_and_namespaces()]
        except Exception:
            return []

    def _node_is_active(self, node_hint, active_nodes):
        node_hint = self._normalize_node_name(node_hint)
        for node in active_nodes:
            node_clean = self._normalize_node_name(node)
            # Match bare name or last segment after namespace
            bare = node_clean.split("/")[-1]
            if bare == node_hint or node_clean == node_hint:
                return True
        return False

    def start_node_group(self, group_name):
        try:
            if group_name not in self.node_groups:
                self.log_event("error", f"Group {group_name} not found.")
                return

            if group_name in self.group_status_labels:
                self.group_status_labels[group_name].setText("STARTING")
                self.group_status_labels[group_name].setStyleSheet(f"color: {COLOR_WARNING}; font-weight: bold;")
                
            self.group_starting_until[group_name] = time.time() + 5.0
            active_nodes = self._get_active_nodes_safe()
            
            for node_key in self.node_groups[group_name]:
                spec = self._spec_for(node_key)
                if self._node_is_active(spec["node_hint"], active_nodes):
                    self.log_event("info", f"Node {node_key} already active, skipping.")
                    continue
                
                proc = self.processes.get(node_key)
                if proc and proc.poll() is None:
                    self.log_event("info", f"Process {node_key} already launched, skipping.")
                    continue
                    
                self._launch_process(node_key, spec["command"])
                self.log_event("info", f"Launched {node_key}")
                
        except Exception as e:
            self.log_event("error", f"Error starting group {group_name}: {e}")
            if group_name in self.group_status_labels:
                self.group_status_labels[group_name].setText("ERROR")
                self.group_status_labels[group_name].setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")

    def stop_node_group(self, group_name):
        try:
            if group_name not in self.node_groups:
                return
            
            self.send_robot_stop()
            QTimer.singleShot(200, lambda: self._stop_node_group_processes(group_name))
        except Exception as e:
            self.log_event("error", f"Error stopping group {group_name}: {e}")

    def send_robot_stop(self):
        msg = String()
        msg.data = "M,0,0"
        try:
            self.ros_node.safe_cmd_pub.publish(msg)
        except Exception:
            pass

    def _stop_node_group_processes(self, group_name):
        for node_key in self.node_groups.get(group_name, []):
            spec = self._spec_for(node_key)
            key = node_key
            if key in self.processes:
                proc = self.processes[key]
                if proc.poll() is None:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    except Exception:
                        pass
                    QTimer.singleShot(500, lambda p=proc: os.killpg(os.getpgid(p.pid), signal.SIGKILL) if p.poll() is None else None)
                if proc.poll() is not None:
                    del self.processes[key]
            else:
                try:
                    subprocess.run(["pkill", "-f", spec["kill_pattern"]], timeout=1)
                except Exception:
                    pass

    def get_group_status(self, group_name):
        if group_name not in self.node_groups:
            return "ERROR"
            
        active_nodes = self._get_active_nodes_safe()
        node_keys = self.node_groups[group_name]
        required_count = len(node_keys)
        active_count = 0
        
        for node_key in node_keys:
            spec = self._spec_for(node_key)
            if self._node_is_active(spec["node_hint"], active_nodes):
                active_count += 1
                
        if active_count == 0:
            status = "OFFLINE"
        elif active_count < required_count:
            status = "PARTIAL"
        else:
            status = "RUNNING"
            
        if time.time() < self.group_starting_until.get(group_name, 0) and status in ["OFFLINE", "PARTIAL"]:
            return "STARTING"
            
        return status

    def update_group_statuses(self):
        for group_name, label in self.group_status_labels.items():
            # Skip if it is not a real group
            if group_name not in self.node_groups:
                continue
            status = self.get_group_status(group_name)
            label.setText(status)
            if status == "OFFLINE":
                label.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")
            elif status in ["STARTING", "PARTIAL"]:
                label.setStyleSheet(f"color: {COLOR_WARNING}; font-weight: bold;")
            elif status == "RUNNING":
                label.setStyleSheet(f"color: {COLOR_SUCCESS}; font-weight: bold;")
            else:
                label.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")
                
        # Also update individual nodes in table
        if hasattr(self, 'node_status_labels'):
            active_nodes = self._get_active_nodes_safe()
            for key, (spec, label) in self.node_status_labels.items():
                if self._node_is_active(spec["node_hint"], active_nodes):
                    label.setText("RUNNING")
                    label.setStyleSheet(f"color: {COLOR_SUCCESS}; font-weight: bold;")
                else:
                    label.setText("OFFLINE")
                    label.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")

    def log_event(self, level, text):
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] [{level.upper()}] {text}"
        if hasattr(self, 'log_text_edit'):
            self.log_text_edit.append(log_entry)
        elif hasattr(self, 'log_list'):
            self.log_list.addItem(log_entry)
            self.log_list.scrollToBottom()

    # ═══════════════════════════════════════════════════════════════════
    # LOGIN SYSTEM
    # ═══════════════════════════════════════════════════════════════════
    def build_login_page(self):
        login_widget = QWidget()
        login_widget.setObjectName("loginPage")
        outer = QVBoxLayout(login_widget)
        outer.setContentsMargins(0, 0, 0, 0)

        # Wrapped in a scroll area so that on very short displays the card
        # scrolls instead of its rows visually overlapping each other.
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)

        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setAlignment(Qt.AlignCenter)
        layout.setContentsMargins(20, 20, 20, 20)

        card = QFrame()
        card.setObjectName("loginCard")
        card.setStyleSheet(
            f"#loginCard {{ background-color: {BG_CARD}; border: 1px solid #483E38; "
            f"border-radius: 12px; padding: 18px; }}"
        )
        card.setFixedWidth(380)
        card_layout = QVBoxLayout(card)
        card_layout.setSpacing(8)

        title = QLabel("PUSANTARA")
        title.setStyleSheet(
            f"font-size: 26px; font-weight: bold; color: {COLOR_INFO}; "
            f"font-family: 'Outfit', sans-serif;"
        )
        title.setAlignment(Qt.AlignCenter)
        subtitle = QLabel("Cultural Heritage Robot Console")
        subtitle.setStyleSheet(f"color: {COLOR_MUTED}; font-size: 12px;")
        subtitle.setAlignment(Qt.AlignCenter)

        lbl_role = QLabel("Select Security Role")
        lbl_role.setStyleSheet("font-weight: bold;")
        self.combo_role = QComboBox()
        self.combo_role.addItems(["Guest", "Operator", "Admin"])

        lbl_pass = QLabel("Access PIN / Password")
        lbl_pass.setStyleSheet("font-weight: bold; margin-top: 4px;")
        self.edit_password = QLineEdit()
        self.edit_password.setEchoMode(QLineEdit.Password)
        self.edit_password.setPlaceholderText("Enter credentials (hint: admin, operator, or blank)")
        self.edit_password.returnPressed.connect(self.attempt_login)

        btn_login = QPushButton("AUTHENTICATE")
        btn_login.setStyleSheet(
            f"background-color: {BG_ACCENT}; color: white; padding: 10px; "
            f"font-size: 14px; margin-top: 10px;"
        )
        btn_login.clicked.connect(self.attempt_login)

        card_layout.addWidget(title)
        card_layout.addWidget(subtitle)
        card_layout.addSpacing(6)
        card_layout.addWidget(lbl_role)
        card_layout.addWidget(self.combo_role)
        card_layout.addWidget(lbl_pass)
        card_layout.addWidget(self.edit_password)
        card_layout.addWidget(btn_login)

        layout.addWidget(card)
        scroll.setWidget(inner)
        outer.addWidget(scroll)
        self.stacked_widget.addWidget(login_widget)

    def attempt_login(self):
        role = self.combo_role.currentText()
        password = self.edit_password.text()

        authenticated = False
        if role == "Guest":
            authenticated = True
        elif role == "Operator" and password == "operator":
            authenticated = True
        elif role == "Admin" and password == "admin":
            authenticated = True
        elif password == "":
            authenticated = True

        if authenticated:
            self.current_role = role
            self.current_user = f"{role}_User"
            self.apply_role_permissions()
            self.stacked_widget.setCurrentIndex(1)
            self.log_event("info", f"User logged in as {role}")
        else:
            QMessageBox.critical(self, "Access Denied", "Incorrect password or security privileges.")

    def apply_role_permissions(self):
        self.lbl_dash_user.setText(self.current_user)
        self.lbl_dash_role.setText(self.current_role)

        is_guest = (self.current_role == "Guest")
        is_operator = (self.current_role == "Operator")
        is_admin = (self.current_role == "Admin")

        def safe_enable(attr_name, enable_bool):
            if hasattr(self, attr_name):
                # Respect an active maintenance lockout: it overrides role
                # permissions for anything movement-related.
                if self.lockout_active and attr_name in self._movement_attr_names():
                    getattr(self, attr_name).setEnabled(False)
                else:
                    getattr(self, attr_name).setEnabled(enable_bool)

        # Node Launcher Tab
        safe_enable("btn_start_selected", is_operator or is_admin)
        safe_enable("btn_stop_selected", is_operator or is_admin)
        safe_enable("btn_start_all", is_operator or is_admin)
        safe_enable("btn_stop_all", is_operator or is_admin)
        safe_enable("btn_start_robot", is_operator or is_admin)

        if hasattr(self, "node_table"):
            for row in range(self.node_table.rowCount()):
                btn_start = self.node_table.cellWidget(row, 2)
                btn_stop = self.node_table.cellWidget(row, 3)
                if btn_start:
                    btn_start.setEnabled(is_operator or is_admin)
                if btn_stop:
                    btn_stop.setEnabled(is_operator or is_admin)

        # Camera Tab
        safe_enable("btn_cam_start", is_operator or is_admin)
        safe_enable("btn_cam_stop", is_operator or is_admin)

        # Motion controls
        safe_enable("btn_up", is_operator or is_admin)
        safe_enable("btn_down", is_operator or is_admin)
        safe_enable("btn_left", is_operator or is_admin)
        safe_enable("btn_right", is_operator or is_admin)
        safe_enable("btn_stop", is_operator or is_admin)

        # Arm & Gripper controls
        safe_enable("btn_arm_fwd", is_operator or is_admin)
        safe_enable("btn_arm_back", is_operator or is_admin)
        safe_enable("btn_grip_open", is_operator or is_admin)
        safe_enable("btn_grip_close", is_operator or is_admin)
        safe_enable("btn_seq_takein", is_operator or is_admin)
        safe_enable("btn_seq_takeout", is_operator or is_admin)

        # Applications Tab
        safe_enable("btn_app_food_quiz", is_admin)
        safe_enable("btn_app_batik_designer", is_admin)
        safe_enable("btn_app_robot_face", is_admin)
        safe_enable("btn_app_visitor_greeter", is_admin)
        safe_enable("btn_app_heritage_explainer", is_admin)
        safe_enable("btn_app_keris_demo", is_admin)
        safe_enable("btn_app_qwen_assistant", is_admin)
        safe_enable("btn_app_speech_to_text", is_admin)

        # AI Interaction Tab
        safe_enable("btn_send_qwen", is_operator or is_admin)
        safe_enable("btn_send_tts", is_operator or is_admin)
        safe_enable("edit_chat_input", is_operator or is_admin)

        # Cultural Exhibition Tab
        safe_enable("btn_trigger_keris_ar", is_operator or is_admin)
        if hasattr(self, "narration_table"):
            for row in range(self.narration_table.rowCount()):
                btn_speak = self.narration_table.cellWidget(row, 2)
                if btn_speak:
                    btn_speak.setEnabled(is_operator or is_admin)

        # Emergency / recovery controls
        safe_enable("btn_estop_main", is_operator or is_admin)
        safe_enable("btn_estop_clear", is_admin)
        safe_enable("btn_estop_restart", is_admin)
        safe_enable("btn_estop_disable", is_admin)

        # If e-stop is currently latched, movement stays locked regardless
        # of role until an admin clears it.
        if self.estop_active:
            self._set_movement_enabled(False)

    def _movement_attr_names(self):
        return [
            "btn_up", "btn_down", "btn_left", "btn_right",
            "btn_arm_fwd", "btn_arm_back", "btn_grip_open", "btn_grip_close",
            "btn_seq_takein", "btn_seq_takeout", "btn_start_robot",
        ]

    def _set_movement_enabled(self, enabled):
        for name in self._movement_attr_names():
            if hasattr(self, name):
                getattr(self, name).setEnabled(enabled)

    # ═══════════════════════════════════════════════════════════════════
    # MAIN CONSOLE VIEW
    # ═══════════════════════════════════════════════════════════════════
    def build_main_page(self):
        main_widget = QWidget()
        main_widget.setObjectName("mainPage")
        layout = QVBoxLayout(main_widget)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(6)

        # Top Dashboard bar
        top_bar = QHBoxLayout()
        title = QLabel("PUSANTARA CONSOLE")
        title.setObjectName("titleLabel")

        self.lbl_time = QLabel()
        self.lbl_time.setStyleSheet("font-size: 13px; font-weight: bold; color: #00D4FF;")

        btn_logout = QPushButton("LOGOUT")
        btn_logout.setFixedWidth(90)
        btn_logout.clicked.connect(self.logout)

        top_bar.addWidget(title)
        top_bar.addStretch()
        top_bar.addWidget(self.lbl_time)
        top_bar.addWidget(btn_logout)
        layout.addLayout(top_bar)

        # Top Navigation Bar (compact, so it never pushes content off-screen)
        self.topbar = QListWidget()
        self.topbar.setObjectName("topbar")
        self.topbar.setFlow(QListView.LeftToRight)
        self.topbar.setSpacing(0)
        self.topbar.setFixedHeight(38)
        self.topbar.setFocusPolicy(Qt.NoFocus)
        self.topbar.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.topbar.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.topbar.addItems([
            "Dashboard",
            "Perception",
            "Operation & AI",
            "Exhibition Apps",
            "System & Logs"
        ])

        self.views_stack = QStackedWidget()

        layout.addWidget(self.topbar)
        layout.addWidget(self.views_stack, 1)

        self.camera_widget = CameraWidget()
        self.lidar_widget = LidarWidget()

        self.create_dashboard_view()
        self.create_perception_view()
        self.create_operation_view()
        self.create_exhibition_view()
        self.create_system_view()

        self.topbar.currentRowChanged.connect(self.views_stack.setCurrentIndex)
        if self.topbar.count() > 0:
            self.topbar.setCurrentRow(0)

        self.stacked_widget.addWidget(main_widget)

    def logout(self):
        self.stacked_widget.setCurrentIndex(0)
        self.edit_password.clear()
        self.log_event("info", f"User {self.current_user} logged out")
        self.current_user = "None"
        self.current_role = "None"


    def build_feature_launcher(self, parent, group_name, title, description, include_stop=True, extra_buttons=None):
        card = QFrame(parent)
        card.setStyleSheet(f"background-color: {BG_CARD}; border: 1px solid #483E38; border-radius: 8px;")
        card.setFixedHeight(65)
        
        layout = QHBoxLayout(card)
        layout.setContentsMargins(10, 5, 10, 5)
        
        text_layout = QVBoxLayout()
        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(f"color: {COLOR_INFO}; font-weight: 800; font-size: 14px; border: none;")
        desc_lbl = QLabel(description)
        desc_lbl.setStyleSheet(f"color: {COLOR_MUTED}; font-size: 11px; border: none;")
        text_layout.addWidget(title_lbl)
        text_layout.addWidget(desc_lbl)
        layout.addLayout(text_layout, 1)
        
        layout.addStretch()
        
        status_lbl = QLabel("OFFLINE")
        status_lbl.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold; border: none;")
        self.group_status_labels[group_name] = status_lbl
        layout.addWidget(status_lbl)
        
        layout.addSpacing(10)
        
        btn_start = QPushButton("START")
        btn_start.setStyleSheet("background-color: #10b981; color: white; min-height: 28px;")
        btn_start.clicked.connect(lambda: self.start_node_group(group_name))
        layout.addWidget(btn_start)
        
        if include_stop:
            btn_stop = QPushButton("STOP")
            btn_stop.setStyleSheet("background-color: #ef4444; color: white; min-height: 28px;")
            btn_stop.clicked.connect(lambda: self.stop_node_group(group_name))
            layout.addWidget(btn_stop)
            
        if extra_buttons:
            for btn_text, group_target in extra_buttons:
                btn = QPushButton(btn_text)
                btn.setStyleSheet(f"background-color: {BG_MAIN}; color: {COLOR_INFO}; border: 1px solid {COLOR_INFO}; min-height: 28px;")
                btn.clicked.connect(lambda checked, g=group_target: self.start_node_group(g))
                layout.addWidget(btn)
                
        return card

    # ═══════════════════════════════════════════════════════════════════
    # 1. Dashboard View
    # ═══════════════════════════════════════════════════════════════════
    def create_dashboard_view(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)
        view = QWidget()
        layout = QHBoxLayout(view)
        layout.setSpacing(8)

        left_panel = QVBoxLayout()
        left_panel.setSpacing(8)

        group_system = QGroupBox("System Status")
        sys_layout = QGridLayout(group_system)
        sys_layout.setVerticalSpacing(4)
        sys_layout.addWidget(QLabel("Robot Name:"), 0, 0)
        sys_layout.addWidget(QLabel("PUSANTARA"), 0, 1)
        sys_layout.addWidget(QLabel("Status:"), 1, 0)
        self.lbl_dash_status = QLabel("ONLINE")
        self.lbl_dash_status.setStyleSheet("color: #00E676; font-weight: bold;")
        sys_layout.addWidget(self.lbl_dash_status, 1, 1)

        sys_layout.addWidget(QLabel("Current User:"), 2, 0)
        self.lbl_dash_user = QLabel("--")
        sys_layout.addWidget(self.lbl_dash_user, 2, 1)
        sys_layout.addWidget(QLabel("Privilege:"), 3, 0)
        self.lbl_dash_role = QLabel("--")
        self.lbl_dash_role.setStyleSheet("color: #FF9100; font-weight: bold;")
        sys_layout.addWidget(self.lbl_dash_role, 3, 1)

        sys_layout.addWidget(QLabel("WiFi Network:"), 4, 0)
        self.lbl_dash_wifi = QLabel("--")
        sys_layout.addWidget(self.lbl_dash_wifi, 4, 1)
        sys_layout.addWidget(QLabel("IP Address:"), 5, 0)
        self.lbl_dash_ip = QLabel("--")
        sys_layout.addWidget(self.lbl_dash_ip, 5, 1)

        # NEW: IMU calibration readout (topic existed, nothing displayed it)
        sys_layout.addWidget(QLabel("IMU Cal Status:"), 6, 0)
        self.lbl_dash_imu = QLabel("--")
        self.lbl_dash_imu.setStyleSheet(f"color: {COLOR_MUTED}; font-weight: bold;")
        sys_layout.addWidget(self.lbl_dash_imu, 6, 1)

        left_panel.addWidget(group_system)

        group_metrics = QGroupBox("Hardware Performance")
        metrics_layout = QVBoxLayout(group_metrics)
        metrics_layout.setSpacing(3)
        metrics_layout.addWidget(QLabel("CPU Load"))
        self.bar_cpu = QProgressBar()
        metrics_layout.addWidget(self.bar_cpu)
        metrics_layout.addWidget(QLabel("RAM Allocation"))
        self.bar_ram = QProgressBar()
        metrics_layout.addWidget(self.bar_ram)
        self.lbl_cpu_temp = QLabel("Temp: -- \u00b0C")
        metrics_layout.addWidget(self.lbl_cpu_temp)

        self.bar_disk = QProgressBar()
        self.lbl_dash_host = QLabel()
        self.lbl_dash_uptime = QLabel()

        left_panel.addWidget(group_metrics)
        left_panel.addStretch()

        right_panel = QVBoxLayout()
        right_panel.setSpacing(8)

        
        launcher_core = self.build_feature_launcher(view, "core_robot", "Core Robot Bringup", "Starts LiDAR, IMU, serial bridge, odometry")
        right_panel.addWidget(launcher_core)
        
        launcher_face = self.build_feature_launcher(view, "face_speech", "Face / Speech", "Starts robot face and TTS")
        right_panel.addWidget(launcher_face)
        
        right_panel.addStretch()

        self.btn_estop_main = QPushButton("EMERGENCY STOP")
        self.btn_estop_main.setObjectName("btnEmergency")
        self.btn_estop_main.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.btn_estop_main.clicked.connect(self.trigger_emergency_stop)
        right_panel.addWidget(self.btn_estop_main, 1)

        # NEW: recovery controls - previously referenced by apply_role_permissions
        # (btn_estop_clear / btn_estop_restart / btn_estop_disable) but never
        # actually created, so admins had no way to recover from an e-stop
        # short of restarting the whole console.
        recovery_row = QHBoxLayout()
        recovery_row.setSpacing(6)

        self.btn_estop_clear = QPushButton("Clear E-Stop")
        self.btn_estop_clear.setObjectName("btnRecovery")
        self.btn_estop_clear.clicked.connect(self.clear_emergency_stop)

        self.btn_estop_restart = QPushButton("Restart Safety Nodes")
        self.btn_estop_restart.setObjectName("btnRecovery")
        self.btn_estop_restart.clicked.connect(self.restart_safety_nodes)

        self.btn_estop_disable = QPushButton("Maintenance Lockout")
        self.btn_estop_disable.setObjectName("btnRecovery")
        self.btn_estop_disable.clicked.connect(self.toggle_lockout)

        recovery_row.addWidget(self.btn_estop_clear)
        recovery_row.addWidget(self.btn_estop_restart)
        recovery_row.addWidget(self.btn_estop_disable)
        right_panel.addLayout(recovery_row)

        layout.addLayout(left_panel, 1)
        layout.addLayout(right_panel, 1)

        scroll.setWidget(view)
        self.views_stack.addWidget(scroll)


    def trigger_emergency_stop(self):
        self.estop_active = True
        self.log_event("danger", "EMERGENCY STOP TRIGGERED!")
        msg = String()
        msg.data = "M,0,0"
        self.ros_node.safe_cmd_pub.publish(msg)
        if hasattr(self, 'lbl_mot_status'):
            self.lbl_mot_status.setText("EMERGENCY STOPPED")
            self.lbl_mot_status.setStyleSheet("color: #FF1744; font-weight: bold;")
        self.lbl_dash_status.setText("EMERGENCY STOP")
        self.lbl_dash_status.setStyleSheet("color: #FF1744; font-weight: bold;")
        self._set_movement_enabled(False)
        self.gripper_state = "UNKNOWN"
        self._update_gripper_status_label("LOCKED (E-STOP)", COLOR_DANGER)
        QMessageBox.critical(self, "EMERGENCY ESTOP", "Emergency Stop command published.")

    def clear_emergency_stop(self):
        if not self.estop_active:
            self.log_event("info", "Clear E-Stop pressed but no active E-Stop.")
            return
        self.estop_active = False
        self.lbl_dash_status.setText("ONLINE")
        self.lbl_dash_status.setStyleSheet("color: #00E676; font-weight: bold;")
        if hasattr(self, 'lbl_mot_status'):
            self.lbl_mot_status.setText("READY")
            self.lbl_mot_status.setStyleSheet("color: #00E676; font-weight: bold;")
        self._update_gripper_status_label("READY", COLOR_SUCCESS)
        self.apply_role_permissions()
        self.log_event("info", f"E-Stop cleared by {self.current_user}")

    def restart_safety_nodes(self):
        reply = QMessageBox.question(
            self, "Restart Safety Nodes",
            "This restarts Motion Manager and Serial Bridge. The robot will "
            "briefly stop responding to drive commands. Continue?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return
        for node in ["Motion Manager", "Serial Bridge"]:
            self.node_manager.stop_node(node)
            self.log_event("warning", f"Stopped {node} for restart")
        QApplication.processEvents()
        time.sleep(0.5)
        for node in ["Motion Manager", "Serial Bridge"]:
            self.node_manager.start_node(node)
            self.log_event("info", f"Restarted {node}")

    def toggle_lockout(self):
        self.lockout_active = not self.lockout_active
        if self.lockout_active:
            self._set_movement_enabled(False)
            self.btn_estop_disable.setText("LOCKED \u2013 tap to release")
            self.btn_estop_disable.setStyleSheet(
                f"border: 1px solid {COLOR_DANGER}; color: {COLOR_DANGER};"
            )
            self.log_event("warning", f"Maintenance lockout engaged by {self.current_user}")
        else:
            self.btn_estop_disable.setText("Maintenance Lockout")
            self.btn_estop_disable.setStyleSheet("")
            self.apply_role_permissions()
            self.log_event("info", f"Maintenance lockout released by {self.current_user}")

    def handle_imu_cal_status(self, status_str):
        self.lbl_dash_imu.setText(status_str)
        # Heuristic: BNO055 calibration levels are 0-3 per sensor; treat a
        # string containing no "0" as fully calibrated.
        if status_str and "0" not in status_str:
            self.lbl_dash_imu.setStyleSheet(f"color: {COLOR_SUCCESS}; font-weight: bold;")
        else:
            self.lbl_dash_imu.setStyleSheet(f"color: {COLOR_WARNING}; font-weight: bold;")

    # ═══════════════════════════════════════════════════════════════════
    # 2. Perception View (Camera + LiDAR)
    # ═══════════════════════════════════════════════════════════════════
    def create_perception_view(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)
        view = QWidget()
        layout = QVBoxLayout(view)
        layout.setSpacing(6)

        # Grid instead of a single HBox row - a single row of buttons plus
        # camera/lidar status text was what ran off the right edge of the
        # 1024px display in the "Zoom" / status cut-off screenshot.
        ctrl_layout = QGridLayout()
        ctrl_layout.setHorizontalSpacing(8)
        ctrl_layout.setVerticalSpacing(4)

        
        launcher_vision = self.build_feature_launcher(
            view, "vision", "Camera + Vision", "Starts camera, AprilTag, YOLO",
            extra_buttons=[("Tags Only", "vision_tags_only"), ("Greeting Mode", "greeting"), ("Keris AR", "keris_ar")]
        )
        layout.addWidget(launcher_vision)

        launcher_lidar = self.build_feature_launcher(view, "lidar", "LiDAR Sensor", "Starts LiDAR node")
        layout.addWidget(launcher_lidar)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.camera_widget)
        splitter.addWidget(self.lidar_widget)
        splitter.setSizes([1, 1])  # keep camera/lidar panels evenly split
        layout.addWidget(splitter, 1)
        scroll.setWidget(view)
        self.views_stack.addWidget(scroll)

    # ═══════════════════════════════════════════════════════════════════
    # 3. Operation & AI View
    # ═══════════════════════════════════════════════════════════════════
    def create_operation_view(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)
        view = QWidget()
        layout = QHBoxLayout(view)
        layout.setSpacing(8)

        left_layout = QVBoxLayout()
        left_layout.setSpacing(6)
        
        launcher_motion = self.build_feature_launcher(view, "motion_core", "Drive / Motion Core", "Starts serial bridge and motion manager")
        left_layout.addWidget(launcher_motion)
        
        launcher_nav = self.build_feature_launcher(view, "nav2_navigation", "Navigation", "Starts Nav2 Stack and cmd bridge")
        left_layout.addWidget(launcher_nav)
        
        launcher_rqt = self.build_feature_launcher(view, "rqt", "RQt GUI", "ROS 2 debugging tool")
        left_layout.addWidget(launcher_rqt)
        
        launcher_rviz = self.build_feature_launcher(view, "rviz2", "RViz2", "ROS 2 visualizer")
        left_layout.addWidget(launcher_rviz)

        tel_group = QGroupBox("Manual Teleoperation")
        tel_layout = QGridLayout(tel_group)
        tel_layout.setSpacing(4)
        self.btn_up = QPushButton("FORWARD")
        self.btn_up.clicked.connect(lambda: self.send_motion("M,50,50"))
        self.btn_left = QPushButton("LEFT")
        self.btn_left.clicked.connect(lambda: self.send_motion("M,-30,30"))
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setStyleSheet("background-color: #FF1744; color: white;")
        self.btn_stop.clicked.connect(lambda: self.send_motion("M,0,0"))
        self.btn_right = QPushButton("RIGHT")
        self.btn_right.clicked.connect(lambda: self.send_motion("M,30,-30"))
        self.btn_down = QPushButton("REVERSE")
        self.btn_down.clicked.connect(lambda: self.send_motion("M,-50,-50"))
        tel_layout.addWidget(self.btn_up, 0, 1)
        tel_layout.addWidget(self.btn_left, 1, 0)
        tel_layout.addWidget(self.btn_stop, 1, 1)
        tel_layout.addWidget(self.btn_right, 1, 2)
        tel_layout.addWidget(self.btn_down, 2, 1)
        left_layout.addWidget(tel_group)

        # ── Arm & Gripper Control ────────────────────────────────────
        # Enhanced: every action now goes through send_gripper_command()
        # which (a) shows a live BUSY/READY status with the known gripper
        # state, (b) disables the panel for the sequence's real blocking
        # duration on the Arduino side so operators can't stack commands
        # the firmware can't process yet, and (c) asks for confirmation
        # before the longer TAKEIN/TAKEOUT pick-and-place sequences.
        arm_group = QGroupBox("Arm & Gripper Control")
        arm_layout = QGridLayout(arm_group)
        arm_layout.setSpacing(4)

        self.btn_arm_fwd = QPushButton("ARM FWD")
        self.btn_arm_fwd.clicked.connect(lambda: self.send_gripper_command("FORWARD"))
        self.btn_arm_back = QPushButton("ARM BACK")
        self.btn_arm_back.clicked.connect(lambda: self.send_gripper_command("BACK"))

        self.btn_grip_open = QPushButton("GRIP OPEN")
        self.btn_grip_open.clicked.connect(lambda: self.send_gripper_command("OPEN"))
        self.btn_grip_close = QPushButton("GRIP CLOSE")
        self.btn_grip_close.clicked.connect(lambda: self.send_gripper_command("CLOSE"))

        self.btn_seq_takein = QPushButton("SEQ TAKE IN")
        self.btn_seq_takein.clicked.connect(
            lambda: self.confirm_gripper_sequence("TAKEIN", "Run the TAKE IN pick-up sequence?")
        )
        self.btn_seq_takeout = QPushButton("SEQ TAKE OUT")
        self.btn_seq_takeout.clicked.connect(
            lambda: self.confirm_gripper_sequence("TAKEOUT", "Run the TAKE OUT place-down sequence?")
        )

        arm_layout.addWidget(self.btn_arm_fwd, 0, 0)
        arm_layout.addWidget(self.btn_arm_back, 0, 1)
        arm_layout.addWidget(self.btn_grip_open, 1, 0)
        arm_layout.addWidget(self.btn_grip_close, 1, 1)
        arm_layout.addWidget(self.btn_seq_takein, 2, 0)
        arm_layout.addWidget(self.btn_seq_takeout, 2, 1)

        gripper_status_row = QHBoxLayout()
        gripper_status_row.addWidget(QLabel("Gripper:"))
        self.lbl_gripper_state = QLabel("UNKNOWN")
        self.lbl_gripper_state.setStyleSheet(f"color: {COLOR_MUTED}; font-weight: bold;")
        gripper_status_row.addWidget(self.lbl_gripper_state)
        gripper_status_row.addStretch()
        gripper_status_row.addWidget(QLabel("Panel:"))
        self.lbl_gripper_status = QLabel("READY")
        self.lbl_gripper_status.setStyleSheet(f"color: {COLOR_SUCCESS}; font-weight: bold;")
        gripper_status_row.addWidget(self.lbl_gripper_status)
        arm_layout.addLayout(gripper_status_row, 3, 0, 1, 2)

        left_layout.addWidget(arm_group)

        stat_group = QGroupBox("Motor Status")
        stat_layout = QFormLayout(stat_group)
        self.lbl_mot_status = QLabel("READY")
        self.lbl_mot_status.setStyleSheet("color: #00E676; font-weight: bold;")
        self.lbl_mot_cmd = QLabel("M,0,0")
        stat_layout.addRow("Status:", self.lbl_mot_status)
        stat_layout.addRow("Command:", self.lbl_mot_cmd)
        left_layout.addWidget(stat_group)
        left_layout.addStretch()

        right_layout = QVBoxLayout()
        launcher_ai = self.build_feature_launcher(view, "speech_ai", "Speech + AI", "Starts STT, Qwen assistant, and TTS")
        right_layout.addWidget(launcher_ai)
        
        header = QLabel("Qwen AI Dialogue & Chat")
        header.setObjectName("sectionHeader")
        right_layout.addWidget(header)
        self.chat_display = QTextEdit()
        self.chat_display.setReadOnly(True)
        self.chat_display.setHtml("<div style='color: #94a3b8;'>No conversation yet.</div>")
        right_layout.addWidget(self.chat_display, 1)

        input_layout = QHBoxLayout()
        self.edit_chat_input = QLineEdit()
        self.edit_chat_input.setPlaceholderText("Type prompt...")
        self.edit_chat_input.returnPressed.connect(self.send_qwen_text)
        self.btn_send_qwen = QPushButton("Send")
        self.btn_send_qwen.clicked.connect(self.send_qwen_text)

        self.btn_send_tts = QPushButton("TTS")
        self.btn_send_tts.clicked.connect(self.send_tts_text)

        input_layout.addWidget(self.edit_chat_input)
        input_layout.addWidget(self.btn_send_qwen)
        input_layout.addWidget(self.btn_send_tts)
        right_layout.addLayout(input_layout)

        layout.addLayout(left_layout, 1)
        layout.addLayout(right_layout, 1)

        scroll.setWidget(view)
        self.views_stack.addWidget(scroll)

    def send_motion(self, command_str):
        self.log_event("info", f"Teleop Command: {command_str}")
        msg = String()
        msg.data = command_str
        self.ros_node.cmd_pub.publish(msg)
        self.lbl_mot_cmd.setText(command_str)

    def confirm_gripper_sequence(self, command_str, question):
        reply = QMessageBox.question(
            self, "Confirm Sequence", question,
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.send_gripper_command(command_str)

    def send_gripper_command(self, command_str):
        if self.arm_busy:
            self.log_event("warning", f"Ignored {command_str}: gripper still busy.")
            return
        if self.estop_active or self.lockout_active:
            self.log_event("warning", f"Ignored {command_str}: robot is locked out.")
            return

        self.log_event("info", f"Gripper/Arm Command: {command_str}")
        msg = String()
        msg.data = command_str
        self.ros_node.cmd_pub.publish(msg)

        # Track the best-known state from the last command issued (the
        # firmware doesn't report gripper position back over serial).
        if command_str == "OPEN":
            self.gripper_state = "OPEN"
        elif command_str == "CLOSE":
            self.gripper_state = "CLOSED"
        elif command_str == "TAKEIN":
            self.gripper_state = "STOWED (post pick-up)"
        elif command_str == "TAKEOUT":
            self.gripper_state = "OPEN (post place-down)"
        self.lbl_gripper_state.setText(self.gripper_state)
        self.lbl_gripper_state.setStyleSheet(f"color: {COLOR_INFO}; font-weight: bold;")

        duration_ms = GRIPPER_COMMAND_DURATIONS_MS.get(command_str, 1000)
        self.arm_busy = True
        self._set_gripper_controls_enabled(False)
        self._update_gripper_status_label(f"BUSY ({command_str})", COLOR_WARNING)
        self._gripper_unlock_timer.start(duration_ms)

    def _gripper_unlock(self):
        self.arm_busy = False
        self._set_gripper_controls_enabled(True)
        self._update_gripper_status_label("READY", COLOR_SUCCESS)

    def _set_gripper_controls_enabled(self, enabled):
        for name in ("btn_arm_fwd", "btn_arm_back", "btn_grip_open",
                      "btn_grip_close", "btn_seq_takein", "btn_seq_takeout"):
            if hasattr(self, name):
                getattr(self, name).setEnabled(enabled)

    def _update_gripper_status_label(self, text, color):
        if hasattr(self, "lbl_gripper_status"):
            self.lbl_gripper_status.setText(text)
            self.lbl_gripper_status.setStyleSheet(f"color: {color}; font-weight: bold;")

    def send_qwen_text(self):
        text = self.edit_chat_input.text().strip()
        if not text:
            return
        self.log_event("info", f"Sending to Qwen: {text}")
        msg = String()
        msg.data = text
        self.ros_node.user_text_pub.publish(msg)
        self.edit_chat_input.clear()

    def send_tts_text(self):
        text = self.edit_chat_input.text().strip()
        if not text:
            return
        msg = String()
        msg.data = text
        self.ros_node.tts_text_pub.publish(msg)
        self.edit_chat_input.clear()

    # ═══════════════════════════════════════════════════════════════════
    # 4. Exhibition Apps View
    # ═══════════════════════════════════════════════════════════════════
    def create_exhibition_view(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)
        view = QWidget()
        layout = QVBoxLayout(view)
        layout.setSpacing(8)

        apps_layout = QVBoxLayout()
        apps_layout.addWidget(self.build_feature_launcher(view, "heritage_explainer", "Heritage Explainer", "Starts camera, AprilTag, TTS, narration"))
        apps_layout.addWidget(self.build_feature_launcher(view, "food_quiz", "Food Quiz", "Starts Malaysian food heritage quiz"))
        apps_layout.addWidget(self.build_feature_launcher(view, "batik_designer", "Batik Designer", "Starts interactive batik design studio"))
        apps_layout.addWidget(self.build_feature_launcher(view, "keris_ar", "Keris AR Demo", "Starts camera and Keris AR overlay"))
        apps_layout.addWidget(self.build_feature_launcher(view, "greeting", "Visitor Greeter", "Starts YOLO visitor greeting and TTS"))
        layout.addLayout(apps_layout)

        visitor_group = QGroupBox("YOLO Visitor & AprilTag Tracking")
        visitor_layout = QFormLayout(visitor_group)
        self.lbl_visitor_detected = QLabel("NO VISITOR")
        self.lbl_active_tag_name = QLabel("No active marker")
        visitor_layout.addRow("Visitor:", self.lbl_visitor_detected)
        visitor_layout.addRow("AprilTag:", self.lbl_active_tag_name)

        self.lbl_visitor_info = QLabel("Searching...")
        self.lbl_last_seen = QLabel("Never")
        self.lbl_active_tag_id = QLabel("None")
        self.lbl_active_tag_desc = QLabel("Place marker in front of camera")
        self.lbl_active_tag_desc.setWordWrap(True)
        visitor_layout.addRow("Last Seen:", self.lbl_last_seen)
        visitor_layout.addRow("Tag ID:", self.lbl_active_tag_id)
        visitor_layout.addRow("Narration:", self.lbl_active_tag_desc)

        layout.addWidget(visitor_group)
        layout.addStretch()

        self.btn_trigger_keris_ar = QPushButton("Toggle Keris AR")
        self.heritage_narratives = {
            "1": ("Wau Bulan", "This is Wau Bulan. Wau Bulan is a traditional kite from Kelantan. It is famous for its crescent moon shape and beautiful traditional artwork."),
            "2": ("Keris", "This is a Keris. The Keris is a traditional Malay dagger that represents bravery, heritage, and leadership.")
        }

        scroll.setWidget(view)
        self.views_stack.addWidget(scroll)

    # ═══════════════════════════════════════════════════════════════════
    # 5. System & Logs View
    # ═══════════════════════════════════════════════════════════════════
    def create_system_view(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)
        view = QWidget()
        layout = QVBoxLayout(view)
        layout.setSpacing(6)
        
        top_bar = QHBoxLayout()
        self.btn_toggle_logs = QPushButton("Toggle Logs & Echo")
        self.btn_toggle_logs.setCheckable(True)
        self.btn_toggle_logs.setChecked(True)
        self.btn_toggle_logs.clicked.connect(self.toggle_system_logs_panel)
        top_bar.addWidget(self.btn_toggle_logs)
        top_bar.addStretch()
        layout.addLayout(top_bar)
        
        self.sys_splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(self.sys_splitter, 1)

        node_group = QGroupBox("Node Process Manager")
        node_layout = QVBoxLayout(node_group)
        node_layout.setSpacing(4)

        batch_layout = QHBoxLayout()
        self.btn_start_selected = QPushButton("Start Selected")
        self.btn_stop_selected = QPushButton("Stop Selected")
        self.btn_start_all = QPushButton("START ALL")
        self.btn_start_all.clicked.connect(self.confirm_start_all_nodes)
        self.btn_stop_all = QPushButton("STOP ALL")
        self.btn_stop_all.clicked.connect(self.stop_all_nodes)
        batch_layout.addWidget(self.btn_start_all)
        batch_layout.addWidget(self.btn_stop_all)
        node_layout.addLayout(batch_layout)

        self.node_table = QTableWidget()
        self.node_table.setColumnCount(4)
        self.node_table.setHorizontalHeaderLabels(["Node", "Description", "Status", "Action"])
        self.node_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.node_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.node_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.node_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.node_table.verticalHeader().setVisible(False)
        self.node_table.setAlternatingRowColors(True)

        # Show each unique node exactly once (from all_nodes registry)
        self.node_rows = list(self.all_nodes.keys())
        self.node_table.setRowCount(len(self.node_rows))

        self.node_status_labels = {}
        for row, node_key in enumerate(self.node_rows):
            node_info = self.all_nodes[node_key]
            self.node_table.setCellWidget(row, 0, QLabel(node_key))
            desc_lbl = QLabel(node_info["description"])
            desc_lbl.setStyleSheet(f"color: {COLOR_MUTED};")
            self.node_table.setCellWidget(row, 1, desc_lbl)

            status_label = QLabel("OFFLINE")
            status_label.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")
            self.node_status_labels[node_key] = (node_info, status_label)
            self.node_table.setCellWidget(row, 2, status_label)

            spec = self._spec_for(node_key)
            btn_start = QPushButton("▶")
            btn_start.setFixedWidth(32)
            btn_start.setToolTip(f"Launch {node_key}")
            btn_start.clicked.connect(lambda checked, k=node_key, s=spec: self._launch_single_node(k, s))
            self.node_table.setCellWidget(row, 3, btn_start)

        node_layout.addWidget(self.node_table)
        self.sys_splitter.addWidget(node_group)
        
        self.sys_right_panel = QWidget()
        right_layout = QVBoxLayout(self.sys_right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        echo_group = QGroupBox("ROS 2 Topic Echo")
        echo_layout = QVBoxLayout(echo_group)
        echo_input_layout = QHBoxLayout()
        self.edit_topic_name = QLineEdit()
        self.edit_topic_name.setPlaceholderText("/topic_name")
        self.btn_echo_topic = QPushButton("Echo")
        self.btn_echo_topic.clicked.connect(self.toggle_topic_echo)
        echo_input_layout.addWidget(self.edit_topic_name)
        echo_input_layout.addWidget(self.btn_echo_topic)
        
        self.text_topic_echo = QTextEdit()
        self.text_topic_echo.setReadOnly(True)
        self.text_topic_echo.setMaximumHeight(80)
        
        echo_layout.addLayout(echo_input_layout)
        echo_layout.addWidget(self.text_topic_echo)
        right_layout.addWidget(echo_group, 1)

        log_group = QGroupBox("Terminal Logs")
        log_layout = QVBoxLayout(log_group)
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.setStyleSheet("background-color: #1A1512; color: #F3EFE9; font-family: monospace;")
        log_layout.addWidget(self.log_text_edit)

        log_btn_row = QHBoxLayout()
        btn_clear_log = QPushButton("Clear Log")
        btn_clear_log.clicked.connect(self.log_text_edit.clear)
        log_btn_row.addStretch()
        log_btn_row.addWidget(btn_clear_log)
        log_layout.addLayout(log_btn_row)

        right_layout.addWidget(log_group, 2)
        self.sys_splitter.addWidget(self.sys_right_panel)
        self.sys_splitter.setSizes([600, 400])
        
        self.log_reader_timer = QTimer(self)
        self.log_reader_timer.timeout.connect(self.read_terminal_logs)
        self.log_reader_timer.start(500)
        self.log_file_positions = {}
        self.echo_process = None

        scroll.setWidget(view)
        self.views_stack.addWidget(scroll)


    def toggle_system_logs_panel(self, checked):
        self.sys_right_panel.setVisible(checked)

    def _launch_single_node(self, key, spec):
        """Launch a single node directly (not via group)."""
        active_nodes = self._get_active_nodes_safe()
        if self._node_is_active(spec.get("node_hint", key), active_nodes):
            self.log_event("info", f"Node {key} already active.")
            return
        proc = self.processes.get(key)
        if proc and proc.poll() is None:
            self.log_event("info", f"Process {key} already running.")
            return
        self._launch_process(key, spec["command"])
        self.log_event("info", f"Launched {key}")


    def trigger_start_node_row(self, row):
        name = self.node_rows[row]
        self.start_node_group(name)

    def trigger_stop_node_row(self, row):
        name = self.node_rows[row]
        self.stop_node_group(name)

    def confirm_start_all_nodes(self):
        reply = QMessageBox.question(
            self, "Start All Nodes",
            f"Start all {self.node_table.rowCount()} registered nodes?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.start_all_nodes()

    def start_all_nodes(self):
        groups = [
            "core_robot",
            "face_speech",
            "vision",
            "heritage_explainer",
            "greeting",
            "nav_cmd_bridge"
        ]
        for g in groups:
            self.start_node_group(g)

    def stop_all_nodes(self):
        self.send_robot_stop()
        QTimer.singleShot(200, self.send_robot_stop)
        QTimer.singleShot(1000, self._execute_kills)
        
    def _execute_kills(self):
        for key, proc in list(self.processes.items()):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except:
                pass
        
        # Kill patterns
        kill_patterns = set()
        for group in self.node_groups.values():
            for spec in group:
                kill_patterns.add(spec["kill_pattern"])
                
        for pattern in kill_patterns:
            try:
                subprocess.run(["pkill", "-f", pattern], timeout=1)
            except:
                pass
                
        self.processes.clear()


    def toggle_topic_echo(self):
        if self.echo_process and self.echo_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.echo_process.pid), signal.SIGTERM)
            except:
                pass
            self.echo_process = None
            self.btn_echo_topic.setText("Echo")
        else:
            topic = self.edit_topic_name.text().strip()
            if topic:
                self.text_topic_echo.clear()
                self.text_topic_echo.append(f"Starting echo for {topic}...")
                self.btn_echo_topic.setText("Stop")
                self.echo_process = subprocess.Popen(
                    self._ros_cmd(f"ros2 topic echo {topic}"),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    preexec_fn=os.setsid
                )

    def read_terminal_logs(self):
        if self.echo_process and self.echo_process.poll() is None:
            import fcntl
            fd = self.echo_process.stdout.fileno()
            fl = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
            try:
                out = self.echo_process.stdout.read()
                if out:
                    self.text_topic_echo.append(out.strip())
                    self.text_topic_echo.verticalScrollBar().setValue(self.text_topic_echo.verticalScrollBar().maximum())
            except:
                pass
                
        if not hasattr(self, 'log_text_edit') or not os.path.exists(self.launch_log_dir):
            return
            
        for file in os.listdir(self.launch_log_dir):
            if not file.endswith('.log'): continue
            path = os.path.join(self.launch_log_dir, file)
            
            if file not in self.log_file_positions:
                self.log_file_positions[file] = 0
                
            try:
                with open(path, 'r') as f:
                    f.seek(self.log_file_positions[file])
                    new_data = f.read()
                    if new_data:
                        self.log_file_positions[file] = f.tell()
                        for line in new_data.strip().split('\n'):
                            if line:
                                self.log_text_edit.append(f"[{file}] {line}")
                        self.log_text_edit.verticalScrollBar().setValue(self.log_text_edit.verticalScrollBar().maximum())
            except Exception:
                pass

    def handle_command_history(self, cmd_data, is_safe):
        pass

    def handle_speech_log(self, text, source):
        timestamp = time.strftime("%H:%M:%S")
        bubble = f"<div style='margin-bottom:8px;'><b>[{timestamp}] {source}:</b> {text}</div>"
        if "No conversation" in self.chat_display.toHtml():
            self.chat_display.setHtml(bubble)
        else:
            self.chat_display.append(bubble)

    def update_system_stats(self, stats):
        self.lbl_time.setText(time.strftime("%a, %b %d %Y | %H:%M:%S"))
        self.bar_cpu.setValue(int(stats["cpu_percent"]))
        self.bar_ram.setValue(int(stats["ram_percent"]))
        self.lbl_cpu_temp.setText(f"Temp: {stats['cpu_temp']:.1f} \u00b0C")

        if hasattr(self, 'lbl_dash_host'):
            self.lbl_dash_host.setText(stats["hostname"])
            self.lbl_dash_ip.setText(stats["ip_address"])
            self.lbl_dash_wifi.setText(f"{stats['wifi_status']} ({stats['wifi_name']})")
            self.lbl_dash_uptime.setText(stats["uptime"])

    def handle_yolo_detection(self, data_str):
        self.lbl_visitor_info.setText(data_str)
        now_str = time.strftime("%H:%M:%S")
        if "person" in data_str.lower():
            self.lbl_visitor_detected.setText("VISITOR PRESENT")
            self.lbl_visitor_detected.setStyleSheet(f"color: {COLOR_SUCCESS}; font-weight: bold;")
            self.lbl_last_seen.setText(now_str)
        else:
            self.lbl_visitor_detected.setText("NO VISITOR")
            self.lbl_visitor_detected.setStyleSheet(f"color: {COLOR_MUTED}; font-weight: bold;")

    def handle_apriltag_detection(self, data_str):
        try:
            import json
            data = json.loads(data_str)
            tag_id = str(data.get("id"))
            area = data.get("area", 0)
            if area > 1000 and tag_id in self.heritage_narratives:
                name, text = self.heritage_narratives[tag_id]
                self.lbl_active_tag_id.setText(tag_id)
                self.lbl_active_tag_name.setText(name)
                self.lbl_active_tag_name.setStyleSheet(f"color: {COLOR_INFO}; font-weight: bold;")
                self.lbl_active_tag_desc.setText(text)
        except Exception:
            pass

    def update_ros_monitor(self):
        pass

    def poll_nodes_status(self):
        # We handle group statuses using update_group_statuses now
        # Also sync table labels
        if hasattr(self, 'node_table'):
            for row in range(self.node_table.rowCount()):
                name_widget = self.node_table.cellWidget(row, 0)
                if name_widget:
                    name = name_widget.text()
                    status = self.get_group_status(name)
                    status_label = self.node_table.cellWidget(row, 1)
                    if status_label:
                        status_label.setText(status)
                        if status == "RUNNING":
                            status_label.setStyleSheet(f"color: {COLOR_SUCCESS}; font-weight: bold;")
                        elif status in ["STARTING", "PARTIAL"]:
                            status_label.setStyleSheet(f"color: {COLOR_WARNING}; font-weight: bold;")
                        elif status == "OFFLINE":
                            status_label.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")
                        else:
                            status_label.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")


def main(args=None):
    rclpy.init(args=args)

    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)

    window = AdminConsoleWindow()
    window.start_ros()
    window.show()

    exit_code = app.exec_()

    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()