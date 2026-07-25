import os
import time

# Prevent OpenCV's pre-compiled Qt plugins from conflicting with system PyQt5
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# Optional imports with fallbacks
try:
    import cv2
    # Ensure it's cleared if importing cv2 set it
    os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

class CameraWidget(QWidget):
    # Signals to pass data from ROS thread safely to GUI
    frame_received = pyqtSignal(QImage, int, int) # image, width, height
    status_changed = pyqtSignal(str, str) # status, resolution_str

    def __init__(self, parent=None):
        super().__init__(parent)
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        
        # UI Setup
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        # Top status bar
        self.status_bar = QHBoxLayout()
        self.lbl_status = QLabel("Status: DISCONNECTED")
        self.lbl_status.setStyleSheet("font-weight: bold; color: #ff5555;")
        self.lbl_fps = QLabel("FPS: 0")
        self.lbl_res = QLabel("Resolution: --")
        
        self.status_bar.addWidget(self.lbl_status)
        self.status_bar.addWidget(self.lbl_fps)
        self.status_bar.addWidget(self.lbl_res)
        self.status_bar.addStretch()
        
        self.btn_snapshot = QPushButton("Take Snapshot")
        self.btn_snapshot.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.btn_snapshot.clicked.connect(self.take_snapshot)
        self.btn_snapshot.setEnabled(False)
        self.status_bar.addWidget(self.btn_snapshot)
        
        self.layout.addLayout(self.status_bar)
        
        # Image Display Area
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: #121212; border: 2px solid #2d2d2d; border-radius: 8px;")
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image_label.setMinimumSize(1, 1)
        self.layout.addWidget(self.image_label)
        
        # State variables
        self.last_frame_time = 0
        self.frame_count = 0
        self.fps = 0
        self.fps_timer = QTimer(self)
        self.fps_timer.timeout.connect(self.update_fps_counter)
        self.fps_timer.start(1000)
        
        # Timeout timer to detect if camera went offline
        self.timeout_timer = QTimer(self)
        self.timeout_timer.timeout.connect(self.check_connection_timeout)
        self.timeout_timer.start(500)
        
        # Connecting signals
        self.frame_received.connect(self.on_frame_received)
        self.status_changed.connect(self.on_status_changed)
        
        self.current_qimage = None
        self.is_connected = False
        
    def check_connection_timeout(self):
        if self.is_connected and (time.time() - self.last_frame_time > 2.0):
            self.is_connected = False
            self.status_changed.emit("DISCONNECTED", "--")
            self.btn_snapshot.setEnabled(False)
            self.image_label.clear()
            self.image_label.setText("No Camera Feed Active")
            self.image_label.setStyleSheet("background-color: #121212; border: 2px solid #ff5555; border-radius: 8px; font-size: 16px; color: #ff5555;")
            self.lbl_fps.setText("FPS: 0")

    def update_fps_counter(self):
        if self.is_connected:
            self.fps = self.frame_count
            self.frame_count = 0
            self.lbl_fps.setText(f"FPS: {self.fps}")
        else:
            self.lbl_fps.setText("FPS: 0")

    def handle_ros_image(self, msg):
        self.last_frame_time = time.time()
        self.frame_count += 1
        
        if not self.is_connected:
            self.is_connected = True
            self.status_changed.emit("CONNECTED", f"{msg.width}x{msg.height}")
            
        qimage = None
        if HAS_CV_BRIDGE and self.bridge:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                height, width, channel = cv_img.shape
                bytesPerLine = 3 * width
                qimage = QImage(cv_img.data, width, height, bytesPerLine, QImage.Format_RGB888)
            except Exception as e:
                # Fallback to direct memory reading if cv_bridge errors
                qimage = self._fallback_image_convert(msg)
        else:
            qimage = self._fallback_image_convert(msg)
            
        if qimage:
            # We copy the image because the underlying data might be garbage collected
            self.frame_received.emit(qimage.copy(), msg.width, msg.height)

    def _fallback_image_convert(self, msg):
        # Basic fallback for sensor_msgs/Image
        if not HAS_CV2:
            return None
        try:
            # Handle standard raw formats
            if msg.encoding in ['rgb8', 'bgr8', 'rgb', 'bgr']:
                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                if 'bgr' in msg.encoding:
                    np_arr = cv2.cvtColor(np_arr, cv2.COLOR_BGR2RGB)
                height, width, channel = np_arr.shape
                bytesPerLine = 3 * width
                return QImage(np_arr.data, width, height, bytesPerLine, QImage.Format_RGB888)
        except Exception:
            pass
        return None

    @pyqtSlot(QImage, int, int)
    def on_frame_received(self, qimage, width, height):
        self.current_qimage = qimage
        self.btn_snapshot.setEnabled(True)
        # Scaled pixmap keeping aspect ratio
        pixmap = QPixmap.fromImage(qimage)
        scaled_pixmap = pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image_label.setPixmap(scaled_pixmap)

    @pyqtSlot(str, str)
    def on_status_changed(self, status, resolution_str):
        self.lbl_res.setText(f"Resolution: {resolution_str}")
        if status == "CONNECTED":
            self.lbl_status.setText("Status: CONNECTED")
            self.lbl_status.setStyleSheet("font-weight: bold; color: #55ff55;")
            self.image_label.setStyleSheet("background-color: #121212; border: 2px solid #55ff55; border-radius: 8px;")
        else:
            self.lbl_status.setText("Status: DISCONNECTED")
            self.lbl_status.setStyleSheet("font-weight: bold; color: #ff5555;")
            self.image_label.setStyleSheet("background-color: #121212; border: 2px solid #2d2d2d; border-radius: 8px;")

    def take_snapshot(self):
        if self.current_qimage is None:
            return
            
        # Try to find a standard path
        home = os.path.expanduser("~")
        pictures_dir = os.path.join(home, "Pictures")
        if not os.path.exists(pictures_dir):
            os.makedirs(pictures_dir, exist_ok=True)
            
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        default_name = os.path.join(pictures_dir, f"pusantara_snapshot_{timestamp}.png")
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Snapshot", default_name, "Images (*.png *.jpg)"
        )
        
        if file_path:
            if self.current_qimage.save(file_path):
                QMessageBox.information(self, "Success", f"Snapshot saved successfully to:\n{file_path}")
            else:
                QMessageBox.warning(self, "Error", "Failed to save the snapshot image.")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Redraw current frame with correct aspect ratio on resize
        if self.current_qimage and self.is_connected:
            pixmap = QPixmap.fromImage(self.current_qimage)
            scaled_pixmap = pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.image_label.setPixmap(scaled_pixmap)
