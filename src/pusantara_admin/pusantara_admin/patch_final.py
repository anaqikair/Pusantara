import re
import os

FILE_PATH = "/home/orin_nano/vectra_ws/src/pusantara_admin/pusantara_admin/admin_console.py"

with open(FILE_PATH, "r") as f:
    content = f.read()

# 1. Add rviz2 and rqt
old_nav = """            "nav2_navigation": [
                {"key": "robot_bringup", "node_hint": "hlds_laser_publisher", "command": "ros2 launch pusantara_bringup robot.launch.py", "kill_pattern": "robot_bringup", "required": True, "description": "Core robot hardware"},
                {"key": "nav2", "node_hint": "amcl", "command": "ros2 launch pusantara_navigation navigation.launch.py use_sim_time:=False autostart:=True", "kill_pattern": "nav2", "required": True, "description": "Nav2 stack"},
                {"key": "cmd_vel_to_robot_command", "node_hint": "cmd_vel_to_robot_command", "command": "ros2 run cmd_vel_bridge cmd_vel_to_robot_command", "kill_pattern": "cmd_vel_to_robot_command", "required": True, "description": "Translate Nav2 cmd_vel"}
            ]"""
new_nav = """            "nav2_navigation": [
                {"key": "robot_bringup", "node_hint": "hlds_laser_publisher", "command": "ros2 launch pusantara_bringup robot.launch.py", "kill_pattern": "robot_bringup", "required": True, "description": "Core robot hardware"},
                {"key": "nav2", "node_hint": "amcl", "command": "ros2 launch pusantara_navigation navigation.launch.py use_sim_time:=False autostart:=True", "kill_pattern": "nav2", "required": True, "description": "Nav2 stack"},
                {"key": "cmd_vel_to_robot_command", "node_hint": "cmd_vel_to_robot_command", "command": "ros2 run cmd_vel_bridge cmd_vel_to_robot_command", "kill_pattern": "cmd_vel_to_robot_command", "required": True, "description": "Translate Nav2 cmd_vel"}
            ],
            "rqt": [
                {"key": "rqt_gui", "node_hint": "rqt_gui_cpp", "command": "rqt", "kill_pattern": "rqt", "required": True, "description": "RQt GUI"}
            ],
            "rviz2": [
                {"key": "rviz2", "node_hint": "rviz2", "command": "rviz2", "kill_pattern": "rviz2", "required": True, "description": "RViz2 Visualizer"}
            ]"""
content = content.replace(old_nav, new_nav)

# 2. Fix active nodes safe
old_active = """    def _get_active_nodes_safe(self):
        try:
            return self.ros_node.get_node_names()
        except Exception:
            return []"""
new_active = """    def _get_active_nodes_safe(self):
        try:
            return [f"{ns}/{n}".replace("//", "/") for n, ns in self.ros_node.get_node_names_and_namespaces()]
        except Exception:
            return []"""
content = content.replace(old_active, new_active)

# 3. Log event update
old_log_event = """    def log_event(self, level, text):
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] [{level.upper()}] {text}"
        self.log_list.addItem(log_entry)
        self.log_list.scrollToBottom()"""
new_log_event = """    def log_event(self, level, text):
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] [{level.upper()}] {text}"
        if hasattr(self, 'log_text_edit'):
            self.log_text_edit.append(log_entry)
        elif hasattr(self, 'log_list'):
            self.log_list.addItem(log_entry)
            self.log_list.scrollToBottom()"""
content = content.replace(old_log_event, new_log_event)

# 4. Perception view lidar
old_perc = """        launcher_vision = self.build_feature_launcher(
            view, "vision", "Camera + Vision", "Starts camera, AprilTag, YOLO",
            extra_buttons=[("Tags Only", "vision_tags_only"), ("Greeting Mode", "greeting"), ("Keris AR", "keris_ar")]
        )
        layout.addWidget(launcher_vision)


        splitter = QSplitter(Qt.Horizontal)"""
new_perc = """        launcher_vision = self.build_feature_launcher(
            view, "vision", "Camera + Vision", "Starts camera, AprilTag, YOLO",
            extra_buttons=[("Tags Only", "vision_tags_only"), ("Greeting Mode", "greeting"), ("Keris AR", "keris_ar")]
        )
        layout.addWidget(launcher_vision)

        launcher_lidar = self.build_feature_launcher(view, "lidar", "LiDAR Sensor", "Starts LiDAR node")
        layout.addWidget(launcher_lidar)

        splitter = QSplitter(Qt.Horizontal)"""
content = content.replace(old_perc, new_perc)

# 5. Operation view
old_op = """        launcher_nav = self.build_feature_launcher(view, "nav_cmd_bridge", "Nav2 Command Bridge", "Converts cmd_vel to robot_command")
        left_layout.addWidget(launcher_nav)"""
new_op = """        launcher_nav = self.build_feature_launcher(view, "nav2_navigation", "Navigation", "Starts Nav2 Stack and cmd bridge")
        left_layout.addWidget(launcher_nav)
        
        launcher_rqt = self.build_feature_launcher(view, "rqt", "RQt GUI", "ROS 2 debugging tool")
        left_layout.addWidget(launcher_rqt)
        
        launcher_rviz = self.build_feature_launcher(view, "rviz2", "RViz2", "ROS 2 visualizer")
        left_layout.addWidget(launcher_rviz)"""
content = content.replace(old_op, new_op)

# 6. Exhibition view
old_exhib = """        apps_layout.addWidget(self.build_feature_launcher(view, "heritage_explainer", "Heritage Explainer", "Starts camera, AprilTag, TTS, narration"))
        apps_layout.addWidget(self.build_feature_launcher(view, "food_quiz", "Food Quiz", "Starts Malaysian food heritage quiz"))
        apps_layout.addWidget(self.build_feature_launcher(view, "batik_designer", "Batik Designer", "Starts interactive batik design studio"))
        apps_layout.addWidget(self.build_feature_launcher(view, "keris_ar", "Keris AR Demo", "Starts camera and Keris AR overlay"))
        layout.addLayout(apps_layout)"""
new_exhib = """        apps_layout.addWidget(self.build_feature_launcher(view, "heritage_explainer", "Heritage Explainer", "Starts camera, AprilTag, TTS, narration"))
        apps_layout.addWidget(self.build_feature_launcher(view, "food_quiz", "Food Quiz", "Starts Malaysian food heritage quiz"))
        apps_layout.addWidget(self.build_feature_launcher(view, "batik_designer", "Batik Designer", "Starts interactive batik design studio"))
        apps_layout.addWidget(self.build_feature_launcher(view, "keris_ar", "Keris AR Demo", "Starts camera and Keris AR overlay"))
        apps_layout.addWidget(self.build_feature_launcher(view, "greeting", "Visitor Greeter", "Starts YOLO visitor greeting and TTS"))
        layout.addLayout(apps_layout)"""
content = content.replace(old_exhib, new_exhib)

# 7. System view
old_sys = """        self.node_table = QTableWidget()
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
            self.node_table.setCellWidget(row, 3, btn_stop)

        node_layout.addWidget(self.node_table)
        layout.addWidget(node_group, 2)

        log_group = QGroupBox("System Logs")
        log_layout = QVBoxLayout(log_group)
        self.log_list = QListWidget()
        log_layout.addWidget(self.log_list)

        log_btn_row = QHBoxLayout()
        btn_clear_log = QPushButton("Clear Log")
        btn_clear_log.clicked.connect(self.log_list.clear)
        log_btn_row.addStretch()
        log_btn_row.addWidget(btn_clear_log)
        log_layout.addLayout(log_btn_row)

        layout.addWidget(log_group, 1)"""

new_sys = """        self.node_table = QTableWidget()
        self.node_table.setColumnCount(4)
        self.node_table.setHorizontalHeaderLabels(["Group", "Node", "Status", "Action"])
        self.node_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.node_table.verticalHeader().setVisible(False)
        
        self.node_rows = []
        for g_name, specs in self.node_groups.items():
            for spec in specs:
                self.node_rows.append((g_name, spec))
                
        self.node_table.setRowCount(len(self.node_rows))

        self.node_status_labels = {}
        for row, (g_name, spec) in enumerate(self.node_rows):
            self.node_table.setCellWidget(row, 0, QLabel(g_name))
            self.node_table.setCellWidget(row, 1, QLabel(spec["key"]))
            
            status_label = QLabel("OFFLINE")
            self.node_status_labels[spec["key"]] = (spec, status_label)
            self.node_table.setCellWidget(row, 2, status_label)
            
            btn_start = QPushButton("START GRP")
            btn_start.clicked.connect(lambda checked, g=g_name: self.start_node_group(g))
            self.node_table.setCellWidget(row, 3, btn_start)

        node_layout.addWidget(self.node_table)
        layout.addWidget(node_group, 2)
        
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
        layout.addWidget(echo_group, 1)

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

        layout.addWidget(log_group, 2)
        
        self.log_reader_timer = QTimer(self)
        self.log_reader_timer.timeout.connect(self.read_terminal_logs)
        self.log_reader_timer.start(500)
        self.log_file_positions = {}
        self.echo_process = None"""
content = content.replace(old_sys, new_sys)

# 8. Add update_group_statuses modifications to also update individual node status
old_update = """    def update_group_statuses(self):
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
                label.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")"""
new_update = """    def update_group_statuses(self):
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
                    label.setStyleSheet(f"color: {COLOR_DANGER}; font-weight: bold;")"""
content = content.replace(old_update, new_update)

# 9. Append topic echo and log reading logic
new_methods = """
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
                        for line in new_data.strip().split('\\n'):
                            if line:
                                self.log_text_edit.append(f"[{file}] {line}")
                        self.log_text_edit.verticalScrollBar().setValue(self.log_text_edit.verticalScrollBar().maximum())
            except Exception:
                pass
"""
# insert near the end, just before `def handle_command_history`
content = content.replace("    def handle_command_history(self, cmd_data, is_safe):", new_methods + "\n    def handle_command_history(self, cmd_data, is_safe):")

with open(FILE_PATH, "w") as f:
    f.write(content)

print("Patching complete!")
