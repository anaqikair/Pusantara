import math
import time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class LidarWidget(QWidget):
    # Signals for thread-safety
    scan_received = pyqtSignal(list, float, float, float, float) # ranges, angle_min, angle_increment, range_min, range_max
    status_changed = pyqtSignal(str, int, float) # status, count, rate

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.range_min = 0.0
        self.range_max = 10.0 # Default max range for drawing scale
        
        # State variables
        self.last_scan_time = 0.0
        self.scan_count = 0
        self.update_count_for_hz = 0
        self.hz = 0.0
        
        self.is_connected = False
        
        # UI layout
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        # Top status bar
        self.status_bar = QHBoxLayout()
        self.lbl_status = QLabel("Status: DISCONNECTED")
        self.lbl_status.setStyleSheet("font-weight: bold; color: #ff5555;")
        self.lbl_count = QLabel("Scans: 0")
        self.lbl_hz = QLabel("Rate: 0.0 Hz")
        self.lbl_max_range = QLabel("Scale: 5.0 m")
        
        self.status_bar.addWidget(self.lbl_status)
        self.status_bar.addWidget(self.lbl_count)
        self.status_bar.addWidget(self.lbl_hz)
        self.status_bar.addWidget(self.lbl_max_range)
        self.status_bar.addStretch()
        
        # Range adjustment slider/buttons
        self.btn_zoom_in = QPushButton("+ Zoom")
        self.btn_zoom_in.setFixedWidth(70)
        self.btn_zoom_in.clicked.connect(self.zoom_in)
        self.btn_zoom_out = QPushButton("- Zoom")
        self.btn_zoom_out.setFixedWidth(70)
        self.btn_zoom_out.clicked.connect(self.zoom_out)
        self.status_bar.addWidget(self.btn_zoom_in)
        self.status_bar.addWidget(self.btn_zoom_out)
        
        self.layout.addLayout(self.status_bar)
        
        # Rendering container
        self.render_area = QWidget()
        self.render_area.setStyleSheet("background-color: #0d0d0d; border: 2px solid #2d2d2d; border-radius: 8px;")
        self.render_area.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # Override paint event of render area
        self.render_area.paintEvent = self.paint_lidar_data
        self.layout.addWidget(self.render_area)
        
        # Timers
        self.hz_timer = QTimer(self)
        self.hz_timer.timeout.connect(self.calculate_hz)
        self.hz_timer.start(1000)
        
        self.timeout_timer = QTimer(self)
        self.timeout_timer.timeout.connect(self.check_timeout)
        self.timeout_timer.start(500)
        
        # Connect signals
        self.scan_received.connect(self.on_scan_received)
        self.status_changed.connect(self.on_status_changed)
        
        # Max drawing range (can be zoomed)
        self.view_max_range = 5.0 

    def zoom_in(self):
        if self.view_max_range > 1.0:
            self.view_max_range -= 1.0
            self.lbl_max_range.setText(f"Scale: {self.view_max_range} m")
            self.render_area.update()

    def zoom_out(self):
        if self.view_max_range < 25.0:
            self.view_max_range += 2.0
            self.lbl_max_range.setText(f"Scale: {self.view_max_range} m")
            self.render_area.update()

    def calculate_hz(self):
        if self.is_connected:
            self.hz = float(self.update_count_for_hz)
            self.update_count_for_hz = 0
            self.lbl_hz.setText(f"Rate: {self.hz:.1f} Hz")
        else:
            self.lbl_hz.setText("Rate: 0.0 Hz")

    def check_timeout(self):
        if self.is_connected and (time.time() - self.last_scan_time > 2.0):
            self.is_connected = False
            self.status_changed.emit("DISCONNECTED", self.scan_count, 0.0)
            self.render_area.update()

    def handle_ros_scan(self, msg):
        self.last_scan_time = time.time()
        self.scan_count += 1
        self.update_count_for_hz += 1
        
        if not self.is_connected:
            self.is_connected = True
            self.status_changed.emit("CONNECTED", self.scan_count, self.hz)
            
        # Extract fields to pass to PyQt thread-safely
        ranges_list = list(msg.ranges)
        self.scan_received.emit(
            ranges_list,
            float(msg.angle_min),
            float(msg.angle_increment),
            float(msg.range_min),
            float(msg.range_max)
        )

    @pyqtSlot(list, float, float, float, float)
    def on_scan_received(self, ranges, angle_min, angle_increment, range_min, range_max):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max
        self.lbl_count.setText(f"Scans: {self.scan_count}")
        self.render_area.update()

    @pyqtSlot(str, int, float)
    def on_status_changed(self, status, count, hz):
        self.lbl_count.setText(f"Scans: {count}")
        if status == "CONNECTED":
            self.lbl_status.setText("Status: CONNECTED")
            self.lbl_status.setStyleSheet("font-weight: bold; color: #55ff55;")
            self.render_area.setStyleSheet("background-color: #051105; border: 2px solid #55ff55; border-radius: 8px;")
        else:
            self.lbl_status.setText("Status: DISCONNECTED")
            self.lbl_status.setStyleSheet("font-weight: bold; color: #ff5555;")
            self.render_area.setStyleSheet("background-color: #0d0d0d; border: 2px solid #2d2d2d; border-radius: 8px;")

    def paint_lidar_data(self, event):
        painter = QPainter(self.render_area)
        painter.setRenderHint(QPainter.Antialiasing)
        
        w = self.render_area.width()
        h = self.render_area.height()
        cx = w / 2
        cy = h / 2
        
        # Find scale factor: pixels per meter
        # We leave 10px padding
        scale = (min(w, h) / 2 - 20) / self.view_max_range
        
        # 1. Draw polar grids (concentric circles)
        grid_pen = QPen(QColor(40, 40, 40), 1, Qt.DashLine)
        painter.setPen(grid_pen)
        
        # Draw circles every meter
        step = 1.0 if self.view_max_range <= 5.0 else (2.0 if self.view_max_range <= 10.0 else 5.0)
        r_meter = step
        while r_meter <= self.view_max_range:
            r_pixels = r_meter * scale
            painter.drawEllipse(QPointF(cx, cy), r_pixels, r_pixels)
            # Label
            painter.setPen(QPen(QColor(100, 100, 100)))
            painter.setFont(QFont("Arial", 8))
            painter.drawText(int(cx + r_pixels + 2), int(cy - 2), f"{r_meter}m")
            painter.setPen(grid_pen)
            r_meter += step
            
        # Draw radial lines (every 45 degrees)
        for angle_deg in range(0, 360, 45):
            angle_rad = math.radians(angle_deg)
            dx = math.cos(angle_rad) * (min(w, h) / 2 - 20)
            dy = math.sin(angle_rad) * (min(w, h) / 2 - 20)
            painter.drawLine(QPointF(cx, cy), QPointF(cx + dx, cy + dy))
            
        # 2. Draw robot chassis at center (simple differential drive drawing)
        painter.setBrush(QBrush(QColor(80, 80, 250, 150)))
        painter.setPen(QPen(QColor(100, 100, 255), 2))
        robot_radius = 12
        painter.drawEllipse(QPointF(cx, cy), robot_radius, robot_radius)
        # Draw robot heading indicator (facing up/forward, which is usually -Y in Qt or +X depending on convention.
        # Let's draw it pointing upward).
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(QPointF(cx, cy), QPointF(cx, cy - robot_radius - 8))
        
        # 3. Draw LaserScan points
        if self.is_connected and self.ranges:
            point_pen = QPen(QColor(0, 255, 200, 220), 3)
            painter.setPen(point_pen)
            
            for i, r in enumerate(self.ranges):
                if math.isnan(r) or math.isinf(r) or r < self.range_min or r > self.range_max:
                    continue
                
                # Polar angle: angle_min + i * angle_increment
                # ROS 0 rad faces forward (+X). In ROS, Y faces left.
                # In QPainter, +X is right, +Y is down.
                # To align ROS (+X forward) with QPainter (Up):
                # ROS angle = 0 -> Forward -> Up in UI (dx = 0, dy = -r)
                # ROS angle = pi/2 -> Left -> Left in UI (dx = -r, dy = 0)
                # Hence:
                # Screen X = cx - r * sin(angle)
                # Screen Y = cy - r * cos(angle)
                angle = self.angle_min + i * self.angle_increment
                
                # Compute coordinates
                rx = - r * math.sin(angle)
                ry = - r * math.cos(angle)
                
                # Scale to screen coordinates
                screen_x = cx + rx * scale
                screen_y = cy + ry * scale
                
                # Only draw if within viewport
                if 0 <= screen_x <= w and 0 <= screen_y <= h:
                    painter.drawPoint(QPointF(screen_x, screen_y))
        else:
            # Display warning text if disconnected
            painter.setPen(QPen(QColor(150, 150, 150)))
            painter.setFont(QFont("Arial", 12, QFont.Bold))
            painter.drawText(self.render_area.rect(), Qt.AlignCenter, "Waiting for LiDAR scan data (/scan)...")
