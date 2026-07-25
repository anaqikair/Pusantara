import os

FILE_PATH = "/home/orin_nano/vectra_ws/src/pusantara_admin/pusantara_admin/admin_console.py"

with open(FILE_PATH, "r") as f:
    content = f.read()

old_layout_start = """    def create_system_view(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)
        view = QWidget()
        layout = QVBoxLayout(view)
        layout.setSpacing(6)

        node_group = QGroupBox("Node Process Manager")"""

new_layout_start = """    def create_system_view(self):
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

        node_group = QGroupBox("Node Process Manager")"""

content = content.replace(old_layout_start, new_layout_start)

# Replace the layout.addWidget(node_group, 2)
old_node_add = """        node_layout.addWidget(self.node_table)
        layout.addWidget(node_group, 2)
        
        echo_group = QGroupBox("ROS 2 Topic Echo")"""

new_node_add = """        node_layout.addWidget(self.node_table)
        self.sys_splitter.addWidget(node_group)
        
        self.sys_right_panel = QWidget()
        right_layout = QVBoxLayout(self.sys_right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        echo_group = QGroupBox("ROS 2 Topic Echo")"""

content = content.replace(old_node_add, new_node_add)

# Replace the echo_group and log_group adds to go into right_layout instead of layout
content = content.replace("layout.addWidget(echo_group, 1)", "right_layout.addWidget(echo_group, 1)")
content = content.replace("layout.addWidget(log_group, 2)", "right_layout.addWidget(log_group, 2)\n        self.sys_splitter.addWidget(self.sys_right_panel)\n        self.sys_splitter.setSizes([600, 400])")


# Add the toggle function
toggle_func = """
    def toggle_system_logs_panel(self, checked):
        self.sys_right_panel.setVisible(checked)
"""

content = content.replace("    def trigger_start_node_row(self, row):", toggle_func + "\n    def trigger_start_node_row(self, row):")

with open(FILE_PATH, "w") as f:
    f.write(content)

print("Patch applied")
