import os

FILE_PATH = "/home/orin_nano/vectra_ws/src/pusantara_admin/pusantara_admin/admin_console.py"

with open(FILE_PATH, "r") as f:
    content = f.read()

# 1. Imports
content = content.replace("import subprocess", "import subprocess\nimport signal")
content = content.replace("from pusantara_admin.node_manager import NodeManager\n", "")

# 2. Init
old_init = """    def __init__(self):
        super().__init__()

        self.node_manager = NodeManager()

        self.setWindowTitle("PUSANTARA ADMIN CONSOLE")"""

new_init = """    def __init__(self):
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

        self.setWindowTitle("PUSANTARA ADMIN CONSOLE")"""
content = content.replace(old_init, new_init)


# 3. New Launcher Methods
launcher_methods = """
    # ═══════════════════════════════════════════════════════════════════
    # CENTRAL LAUNCHER SYSTEM
    # ═══════════════════════════════════════════════════════════════════
    def setup_node_groups(self):
        self.node_groups = {
            "face_speech": [
                {"key": "robot_face", "node_hint": "face_node", "command": "ros2 run robot_face face_node", "kill_pattern": "face_node", "required": True, "description": "Robot face animation"},
                {"key": "tts_node", "node_hint": "tts_node", "command": "ros2 run tts_node tts_node", "kill_pattern": "tts_node", "required": True, "description": "TTS node"}
            ],
            "vision": [
                {"key": "webcam_publisher", "node_hint": "webcam_pub", "command": "ros2 run camera_node webcam_pub", "kill_pattern": "webcam_pub", "required": True, "description": "USB camera publisher"},
                {"key": "apriltag_detector", "node_hint": "apriltag_detector", "command": "ros2 run apriltag_detector apriltag_detector --ros-args -p show_ui:=false", "kill_pattern": "apriltag_detector", "required": True, "description": "AprilTag detection"},
                {"key": "yolo_detector", "node_hint": "yolo_detector", "command": "ros2 run vectra_yolo yolo_detector", "kill_pattern": "yolo_detector", "required": True, "description": "YOLO person detection"}
            ],
            "vision_tags_only": [
                {"key": "webcam_publisher", "node_hint": "webcam_pub", "command": "ros2 run camera_node webcam_pub", "kill_pattern": "webcam_pub", "required": True, "description": "USB camera publisher"},
                {"key": "apriltag_detector", "node_hint": "apriltag_detector", "command": "ros2 run apriltag_detector apriltag_detector --ros-args -p show_ui:=false", "kill_pattern": "apriltag_detector", "required": True, "description": "AprilTag detection"}
            ],
            "greeting": [
                {"key": "webcam_publisher", "node_hint": "webcam_pub", "command": "ros2 run camera_node webcam_pub", "kill_pattern": "webcam_pub", "required": True, "description": "USB camera publisher"},
                {"key": "yolo_detector", "node_hint": "yolo_detector", "command": "ros2 run vectra_yolo yolo_detector", "kill_pattern": "yolo_detector", "required": True, "description": "YOLO person detection"},
                {"key": "visitor_greeter", "node_hint": "visitor_greeter", "command": "ros2 run visitor_greeter visitor_greeter", "kill_pattern": "visitor_greeter", "required": True, "description": "Visitor greeting logic"},
                {"key": "tts_node", "node_hint": "tts_node", "command": "ros2 run tts_node tts_node", "kill_pattern": "tts_node", "required": True, "description": "TTS node"}
            ],
            "lidar": [
                {"key": "hlds_laser_publisher", "node_hint": "hlds_laser_publisher", "command": "ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py", "kill_pattern": "hlds_laser_publisher", "required": True, "description": "LiDAR node"}
            ],
            "lidar_safety": [
                {"key": "hlds_laser_publisher", "node_hint": "hlds_laser_publisher", "command": "ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py", "kill_pattern": "hlds_laser_publisher", "required": True, "description": "LiDAR node"},
                {"key": "motion_manager", "node_hint": "motion_manager", "command": "ros2 run motion_manager motion_manager", "kill_pattern": "motion_manager", "required": True, "description": "Safety stop layer"}
            ],
            "motion_core": [
                {"key": "serial_bridge", "node_hint": "bridge_node", "command": "ros2 run serial_bridge bridge_node", "kill_pattern": "bridge_node", "required": True, "description": "Serial bridge to PRIZM"},
                {"key": "motion_manager", "node_hint": "motion_manager", "command": "ros2 run motion_manager motion_manager", "kill_pattern": "motion_manager", "required": True, "description": "Safety stop layer"}
            ],
            "nav_cmd_bridge": [
                {"key": "cmd_vel_to_robot_command", "node_hint": "cmd_vel_to_robot_command", "command": "ros2 run cmd_vel_bridge cmd_vel_to_robot_command", "kill_pattern": "cmd_vel_to_robot_command", "required": True, "description": "Translate Nav2 cmd_vel"}
            ],
            "core_robot": [
                {"key": "robot_bringup", "node_hint": "hlds_laser_publisher", "command": "ros2 launch pusantara_bringup robot.launch.py", "kill_pattern": "robot_bringup", "required": True, "description": "Core robot hardware"}
            ],
            "arm_gripper": [
                {"key": "serial_bridge", "node_hint": "bridge_node", "command": "ros2 run serial_bridge bridge_node", "kill_pattern": "bridge_node", "required": True, "description": "Serial bridge to PRIZM"}
            ],
            "heritage_explainer": [
                {"key": "webcam_publisher", "node_hint": "webcam_pub", "command": "ros2 run camera_node webcam_pub", "kill_pattern": "webcam_pub", "required": True, "description": "USB camera"},
                {"key": "apriltag_detector", "node_hint": "apriltag_detector", "command": "ros2 run apriltag_detector apriltag_detector --ros-args -p show_ui:=false", "kill_pattern": "apriltag_detector", "required": True, "description": "AprilTag detection"},
                {"key": "heritage_explainer", "node_hint": "heritage_explainer", "command": "ros2 run heritage_explainer heritage_explainer", "kill_pattern": "heritage_explainer", "required": True, "description": "Heritage narration"},
                {"key": "tts_node", "node_hint": "tts_node", "command": "ros2 run tts_node tts_node", "kill_pattern": "tts_node", "required": True, "description": "TTS node"}
            ],
            "food_quiz": [
                {"key": "tts_node", "node_hint": "tts_node", "command": "ros2 run tts_node tts_node", "kill_pattern": "tts_node", "required": True, "description": "TTS node"},
                {"key": "food_quiz", "node_hint": "food_quiz", "command": "ros2 run food_quiz food_quiz", "kill_pattern": "food_quiz", "required": True, "description": "Food quiz GUI"}
            ],
            "batik_designer": [
                {"key": "batik_designer", "node_hint": "batik_designer", "command": "ros2 run batik_designer batik_designer", "kill_pattern": "batik_designer", "required": True, "description": "Batik designer studio"}
            ],
            "keris_ar": [
                {"key": "webcam_publisher", "node_hint": "webcam_pub", "command": "ros2 run camera_node webcam_pub", "kill_pattern": "webcam_pub", "required": True, "description": "USB camera"},
                {"key": "keris_node", "node_hint": "keris_demo", "command": "ros2 run keris_demo keris_demo", "kill_pattern": "keris_demo", "required": True, "description": "Keris AR overlay"}
            ],
            "speech_ai": [
                {"key": "speech_to_text", "node_hint": "stt_node", "command": "ros2 run speech_to_text stt_node", "kill_pattern": "stt_node", "required": True, "description": "Speech recognition"},
                {"key": "qwen_node", "node_hint": "qwen_node", "command": "ros2 run qwen_assistant qwen_node", "kill_pattern": "qwen_node", "required": True, "description": "Qwen assistant"},
                {"key": "tts_node", "node_hint": "tts_node", "command": "ros2 run tts_node tts_node", "kill_pattern": "tts_node", "required": True, "description": "TTS node"}
            ],
            "slam_mapping": [
                {"key": "robot_bringup", "node_hint": "hlds_laser_publisher", "command": "ros2 launch pusantara_bringup robot.launch.py", "kill_pattern": "robot_bringup", "required": True, "description": "Core robot hardware"},
                {"key": "slam_toolbox", "node_hint": "slam_toolbox", "command": "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/orin_nano/vectra_ws/config/slam.yaml", "kill_pattern": "slam_toolbox", "required": True, "description": "SLAM toolbox"}
            ],
            "nav2_navigation": [
                {"key": "robot_bringup", "node_hint": "hlds_laser_publisher", "command": "ros2 launch pusantara_bringup robot.launch.py", "kill_pattern": "robot_bringup", "required": True, "description": "Core robot hardware"},
                {"key": "nav2", "node_hint": "amcl", "command": "ros2 launch pusantara_navigation navigation.launch.py use_sim_time:=False autostart:=True", "kill_pattern": "nav2", "required": True, "description": "Nav2 stack"},
                {"key": "cmd_vel_to_robot_command", "node_hint": "cmd_vel_to_robot_command", "command": "ros2 run cmd_vel_bridge cmd_vel_to_robot_command", "kill_pattern": "cmd_vel_to_robot_command", "required": True, "description": "Translate Nav2 cmd_vel"}
            ]
        }

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
            return self.ros_node.get_node_names()
        except Exception:
            return []

    def _node_is_active(self, node_hint, active_nodes):
        node_hint = self._normalize_node_name(node_hint)
        for node in active_nodes:
            node = self._normalize_node_name(node)
            if node == node_hint or node.endswith("/" + node_hint):
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
                
            self.group_starting_until[group_name] = time.time() + 3.0
            active_nodes = self._get_active_nodes_safe()
            
            for spec in self.node_groups[group_name]:
                if self._node_is_active(spec["node_hint"], active_nodes):
                    self.log_event("info", f"Node {spec['key']} already active, skipping.")
                    continue
                
                proc = self.processes.get(spec["key"])
                if proc and proc.poll() is None:
                    self.log_event("info", f"Process {spec['key']} already launched, skipping.")
                    continue
                    
                self._launch_process(spec["key"], spec["command"])
                self.log_event("info", f"Launched {spec['key']}")
                
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
        for spec in self.node_groups.get(group_name, []):
            key = spec["key"]
            if key in self.processes:
                proc = self.processes[key]
                if proc.poll() is None:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    except Exception:
                        pass
                    # Use a timer to wait and check if it's dead, then SIGKILL if needed
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
        required_count = len(self.node_groups[group_name])
        active_count = 0
        
        for spec in self.node_groups[group_name]:
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
"""
# Insert after QTimer import or before class? Actually put it inside the AdminConsoleWindow class.
# I will append it to `start_ros` function's end or somewhere similar.
content = content.replace("        self.node_poller_timer.start(1000)", "        self.node_poller_timer.start(1000)\n\n        self.group_status_timer = QTimer(self)\n        self.group_status_timer.timeout.connect(self.update_group_statuses)\n        self.group_status_timer.start(1000)")
content = content.replace("    def log_event(self, level, text):", launcher_methods + "\n    def log_event(self, level, text):")


# 4. build_feature_launcher
feature_launcher = """
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
"""
content = content.replace("    # ═══════════════════════════════════════════════════════════════════\n    # 1. Dashboard View", feature_launcher + "\n    # ═══════════════════════════════════════════════════════════════════\n    # 1. Dashboard View")


# 5. Dashboard View Changes
old_dash_buttons = """        self.btn_start_robot = QPushButton("START FULL ROBOT")
        self.btn_start_robot.setStyleSheet(
            "background-color: #10b981; color: white; font-weight: bold; "
            "font-size: 15px; padding: 12px;"
        )
        self.btn_start_robot.clicked.connect(self.confirm_start_full_robot)
        right_panel.addWidget(self.btn_start_robot)

        right_panel.addStretch()"""

new_dash_buttons = """        
        launcher_core = self.build_feature_launcher(view, "core_robot", "Core Robot Bringup", "Starts LiDAR, IMU, serial bridge, odometry")
        right_panel.addWidget(launcher_core)
        
        launcher_face = self.build_feature_launcher(view, "face_speech", "Face / Speech", "Starts robot face and TTS")
        right_panel.addWidget(launcher_face)
        
        right_panel.addStretch()"""
content = content.replace(old_dash_buttons, new_dash_buttons)


# 6. Perception View Changes
old_perc_ctrl = """        self.btn_cam_start = QPushButton("Start Camera")
        self.btn_cam_start.setStyleSheet("border-color: #00E676;")
        self.btn_cam_start.clicked.connect(lambda: self.node_manager.start_node("Camera"))
        self.btn_cam_stop = QPushButton("Stop Camera")
        self.btn_cam_stop.setStyleSheet("border-color: #FF1744;")
        self.btn_cam_stop.clicked.connect(lambda: self.node_manager.stop_node("Camera"))

        ctrl_layout.addWidget(self.btn_cam_start, 0, 0)
        ctrl_layout.addWidget(self.btn_cam_stop, 0, 1)
        ctrl_layout.setColumnStretch(2, 1)
        layout.addLayout(ctrl_layout)"""

new_perc_ctrl = """        
        launcher_vision = self.build_feature_launcher(
            view, "vision", "Camera + Vision", "Starts camera, AprilTag, YOLO",
            extra_buttons=[("Tags Only", "vision_tags_only"), ("Greeting Mode", "greeting"), ("Keris AR", "keris_ar")]
        )
        layout.addWidget(launcher_vision)
"""
content = content.replace(old_perc_ctrl, new_perc_ctrl)


# 7. Operation & AI View Changes
old_op_top = """        left_layout = QVBoxLayout()
        left_layout.setSpacing(6)

        tel_group = QGroupBox("Manual Teleoperation")"""

new_op_top = """        left_layout = QVBoxLayout()
        left_layout.setSpacing(6)
        
        launcher_motion = self.build_feature_launcher(view, "motion_core", "Drive / Motion Core", "Starts serial bridge and motion manager")
        left_layout.addWidget(launcher_motion)
        
        launcher_nav = self.build_feature_launcher(view, "nav_cmd_bridge", "Nav2 Command Bridge", "Converts cmd_vel to robot_command")
        left_layout.addWidget(launcher_nav)

        tel_group = QGroupBox("Manual Teleoperation")"""
content = content.replace(old_op_top, new_op_top)

old_ai_top = """        right_layout = QVBoxLayout()
        header = QLabel("Qwen AI Dialogue & Chat")"""

new_ai_top = """        right_layout = QVBoxLayout()
        launcher_ai = self.build_feature_launcher(view, "speech_ai", "Speech + AI", "Starts STT, Qwen assistant, and TTS")
        right_layout.addWidget(launcher_ai)
        
        header = QLabel("Qwen AI Dialogue & Chat")"""
content = content.replace(old_ai_top, new_ai_top)


# 8. Exhibition Apps View
old_exhib = """        apps_group = QGroupBox("Exhibition Launchers")
        grid = QGridLayout(apps_group)
        grid.setSpacing(6)
        apps = ["Food Quiz", "Batik Designer", "Robot Face", "Visitor Greeter",
                "Heritage Explainer", "Keris Demo", "Qwen Assistant",
                "Speech to Text", "History Quiz", "RViz2", "RQt"]
        for i, app in enumerate(apps):
            btn = QPushButton(f"START {app.upper()}")
            btn.clicked.connect(lambda checked, a=app: self.node_manager.start_node(a))
            setattr(self, f"btn_app_{app.lower().replace(' ', '_')}", btn)
            grid.addWidget(btn, i // 3, i % 3)
        layout.addWidget(apps_group)"""

new_exhib = """        apps_layout = QVBoxLayout()
        apps_layout.addWidget(self.build_feature_launcher(view, "heritage_explainer", "Heritage Explainer", "Starts camera, AprilTag, TTS, narration"))
        apps_layout.addWidget(self.build_feature_launcher(view, "food_quiz", "Food Quiz", "Starts Malaysian food heritage quiz"))
        apps_layout.addWidget(self.build_feature_launcher(view, "batik_designer", "Batik Designer", "Starts interactive batik design studio"))
        apps_layout.addWidget(self.build_feature_launcher(view, "keris_ar", "Keris AR Demo", "Starts camera and Keris AR overlay"))
        layout.addLayout(apps_layout)"""
content = content.replace(old_exhib, new_exhib)


# 9. System & Logs View
old_sys_logs = """        self.node_table = QTableWidget()
        self.node_table.setColumnCount(4)
        self.node_table.setHorizontalHeaderLabels(["Node Name", "Status", "Start", "Stop"])
        self.node_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.node_table.verticalHeader().setVisible(False)
        self.node_rows = list(self.node_manager.commands.keys())
        self.node_table.setRowCount(len(self.node_rows))

        for row, name in enumerate(self.node_rows):
            self.node_table.setCellWidget(row, 0, QLabel(name))
            status_label = QLabel("STOPPED")
            self.node_table.setCellWidget(row, 1, status_label)
            btn_start = QPushButton("START")
            btn_start.clicked.connect(lambda checked, r=row: self.trigger_start_node_row(r))
            self.node_table.setCellWidget(row, 2, btn_start)
            btn_stop = QPushButton("STOP")
            btn_stop.clicked.connect(lambda checked, r=row: self.trigger_stop_node_row(r))
            self.node_table.setCellWidget(row, 3, btn_stop)"""

new_sys_logs = """        self.node_table = QTableWidget()
        self.node_table.setColumnCount(4)
        self.node_table.setHorizontalHeaderLabels(["Node Group", "Status", "Start", "Stop"])
        self.node_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.node_table.verticalHeader().setVisible(False)
        self.node_rows = list(self.node_groups.keys())
        self.node_table.setRowCount(len(self.node_rows))

        for row, name in enumerate(self.node_rows):
            self.node_table.setCellWidget(row, 0, QLabel(name))
            status_label = QLabel("OFFLINE")
            self.group_status_labels[name + "_table"] = status_label  # link to status update
            self.node_table.setCellWidget(row, 1, status_label)
            btn_start = QPushButton("START")
            btn_start.clicked.connect(lambda checked, n=name: self.start_node_group(n))
            self.node_table.setCellWidget(row, 2, btn_start)
            btn_stop = QPushButton("STOP")
            btn_stop.clicked.connect(lambda checked, n=name: self.stop_node_group(n))
            self.node_table.setCellWidget(row, 3, btn_stop)"""
content = content.replace(old_sys_logs, new_sys_logs)

content = content.replace("self.node_manager.start_node(name)", "self.start_node_group(name)")
content = content.replace("self.node_manager.stop_node(name)", "self.stop_node_group(name)")


# 10. Start All / Stop All Batch
old_start_all = """    def start_all_nodes(self):
        for row in range(self.node_table.rowCount()):
            self.trigger_start_node_row(row)

    def stop_all_nodes(self):
        for row in range(self.node_table.rowCount()):
            self.trigger_stop_node_row(row)"""

new_start_all = """    def start_all_nodes(self):
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
                
        self.processes.clear()"""
content = content.replace(old_start_all, new_start_all)


# 11. Replace old poll nodes
old_poll = """    def poll_nodes_status(self):
        statuses = self.node_manager.update_all_statuses()
        if hasattr(self, 'node_table'):
            for row in range(self.node_table.rowCount()):
                name_widget = self.node_table.cellWidget(row, 0)
                if name_widget:
                    name = name_widget.text()
                    status = statuses.get(name, "UNKNOWN")
                    status_label = self.node_table.cellWidget(row, 1)
                    if status_label:
                        status_label.setText(status)
                        if status == "RUNNING":
                            status_label.setStyleSheet(f"color: {COLOR_SUCCESS}; font-weight: bold;")
                        elif status == "CRASHED":
                            status_label.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")
                        else:
                            status_label.setStyleSheet(f"color: {COLOR_MUTED}; font-weight: bold;")"""
new_poll = """    def poll_nodes_status(self):
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
                            status_label.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")"""
content = content.replace(old_poll, new_poll)


# 12. Delete unneeded methods
content = content.replace("""    def confirm_start_full_robot(self):
        # Touch screens + a big green "start everything" button are an
        # accidental-tap risk, so confirm first.
        reply = QMessageBox.question(
            self, "Start Full Robot",
            "This will launch all 15 core/perception/interaction nodes. Continue?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.start_full_robot()

    def start_full_robot(self):
        startup_order = [
            "Camera", "LiDAR", "YOLO Detector", "AprilTag Detector",
            "Motion Manager", "Serial Bridge", "Person Follower",
            "AprilTag Follower", "Visitor Greeter", "Heritage Explainer",
            "TTS Engine", "TTS Sender", "Speech to Text", "Qwen Assistant", "Robot Face"
        ]
        for node in startup_order:
            self.node_manager.start_node(node)
            self.log_event("info", f"Started {node}")
            QApplication.processEvents()
            time.sleep(0.3)\n""", "")


with open(FILE_PATH, "w") as f:
    f.write(content)

print("Patch applied successfully.")
