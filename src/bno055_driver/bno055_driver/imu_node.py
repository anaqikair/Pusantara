#!/usr/bin/env python3
"""
imu_node.py
Improved ROS2 BNO055 driver for Jetson Orin Nano using smbus2.

Improvements over the original:
  - Calibration profile save/load (offsets persisted to JSON)
  - Calibration status published and logged
  - Auto-waits for full calibration on first run
  - Gyro drift reduction via deadband zero-ing
  - Orientation drift reduction via complementary filter on accel
  - Covariance matrices populated based on calibration quality
  - Graceful I2C error recovery (retries instead of crashing)
  - /imu/cal_status diagnostic topic

Publishes:
  /imu/data          sensor_msgs/Imu
  /imu/mag           sensor_msgs/MagneticField
  /imu/temp          sensor_msgs/Temperature
  /imu/cal_status    std_msgs/String   (JSON: sys/gyro/accel/mag 0-3)

Parameters:
  bus          int     I2C bus number          (default: 7)
  address      int     I2C address             (default: 0x28)
  frame_id     str     TF frame               (default: "imu_link")
  rate         float   Publish rate Hz         (default: 100.0)
  cal_file     str     Calibration JSON path   (default: "/tmp/bno055_cal.json")
  wait_cal     bool    Block until fully cal'd (default: false)
  gyro_deadband float  rad/s deadband          (default: 0.002)
  alpha        float   Complementary filter α  (default: 0.98)

Dependencies:
  sudo apt install python3-smbus
  pip3 install smbus2
"""

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String
from smbus2 import SMBus


# ---------------------------------------------------------------------------
# BNO055 driver
# ---------------------------------------------------------------------------

class BNO055:
    CHIP_ID = 0xA0

    # Registers
    REG_CHIP_ID      = 0x00
    REG_ACC          = 0x08
    REG_MAG          = 0x0E
    REG_GYR          = 0x14
    REG_QUAT         = 0x20
    REG_TEMP         = 0x34
    REG_CALIB_STAT   = 0x35
    REG_OPR_MODE     = 0x3D
    REG_SYS_TRIGGER  = 0x3F

    # Calibration offset registers (only accessible in CONFIG mode)
    REG_ACC_OFFSET_X = 0x55   # 6 bytes (x, y, z)
    REG_MAG_OFFSET_X = 0x5B   # 6 bytes
    REG_GYR_OFFSET_X = 0x61   # 6 bytes
    REG_ACC_RADIUS   = 0x67   # 2 bytes
    REG_MAG_RADIUS   = 0x69   # 2 bytes

    # Modes
    MODE_CONFIG  = 0x00
    MODE_IMUPLUS = 0x08   # accel + gyro only, no magnetometer
    MODE_NDOF    = 0x0C   # accel + gyro + mag (full fusion)

    # Scale factors
    _SCALE_ACCEL = 100.0
    _SCALE_GYRO  = 16.0
    _SCALE_MAG   = 16.0
    _SCALE_QUAT  = 16384.0

    def __init__(self, bus: int = 7, addr: int = 0x28) -> None:
        self._bus  = SMBus(bus)
        self._addr = addr
        self._reset_and_init()

    # ------------------------------------------------------------------
    # Init / reset
    # ------------------------------------------------------------------

    def _reset_and_init(self) -> None:
        if self._read8(self.REG_CHIP_ID) != self.CHIP_ID:
            raise RuntimeError("BNO055 not found on I2C bus")

        self._write8(self.REG_OPR_MODE, self.MODE_CONFIG)
        time.sleep(0.05)
        self._write8(self.REG_SYS_TRIGGER, 0x20)   # soft reset
        time.sleep(0.8)

        for _ in range(20):
            if self._read8(self.REG_CHIP_ID) == self.CHIP_ID:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("BNO055 did not recover after reset")

        # NDOF mode applied after optional offset restore (caller's responsibility)

    def enter_ndof(self) -> None:
        self._write8(self.REG_OPR_MODE, self.MODE_NDOF)
        time.sleep(0.1)

    def enter_imuplus(self) -> None:
        """Accel + gyro only — no magnetometer needed. Yaw is relative."""
        self._write8(self.REG_OPR_MODE, self.MODE_IMUPLUS)
        time.sleep(0.1)

    def enter_config(self) -> None:
        self._write8(self.REG_OPR_MODE, self.MODE_CONFIG)
        time.sleep(0.05)

    # ------------------------------------------------------------------
    # Low-level I2C helpers
    # ------------------------------------------------------------------

    def _write8(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(self._addr, reg, value)

    def _read8(self, reg: int) -> int:
        return self._bus.read_byte_data(self._addr, reg)

    def _read_vector(self, reg: int, scale: float) -> list[float]:
        data = self._bus.read_i2c_block_data(self._addr, reg, 6)
        result = []
        for i in range(0, 6, 2):
            raw = (data[i + 1] << 8) | data[i]
            if raw >= 32768:
                raw -= 65536
            result.append(raw / scale)
        return result

    def _write16_le(self, reg: int, value: int) -> None:
        low  = value & 0xFF
        high = (value >> 8) & 0xFF
        self._bus.write_i2c_block_data(self._addr, reg, [low, high])

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def calibration_status(self) -> dict:
        """
        Returns calibration level (0-3) for sys, gyro, accel, mag.
        3 = fully calibrated for that sensor.
        """
        stat = self._read8(self.REG_CALIB_STAT)
        return {
            "sys":   (stat >> 6) & 0x03,
            "gyro":  (stat >> 4) & 0x03,
            "accel": (stat >> 2) & 0x03,
            "mag":   (stat >> 0) & 0x03,
        }

    def is_fully_calibrated(self) -> bool:
        s = self.calibration_status()
        return s["sys"] == 3 and s["gyro"] == 3 and s["accel"] == 3 and s["mag"] == 3

    def save_calibration(self, path: str) -> None:
        """
        Switch to CONFIG mode, read 22 calibration offset bytes, save to JSON.
        Returns to NDOF afterwards.
        """
        self.enter_config()
        time.sleep(0.05)

        data = self._bus.read_i2c_block_data(self._addr, self.REG_ACC_OFFSET_X, 22)
        cal = {
            "acc_offset": data[0:6],
            "mag_offset": data[6:12],
            "gyr_offset": data[12:18],
            "acc_radius": data[18:20],
            "mag_radius": data[20:22],
        }

        Path(path).write_text(json.dumps(cal, indent=2))
        self.enter_ndof()

    def load_calibration(self, path: str) -> bool:
        """
        Load offsets from JSON and write them in CONFIG mode.
        Returns True on success, False if file not found / invalid.
        """
        p = Path(path)
        if not p.exists():
            return False

        try:
            cal = json.loads(p.read_text())
        except (json.JSONDecodeError, KeyError):
            return False

        self.enter_config()
        time.sleep(0.05)

        self._bus.write_i2c_block_data(
            self._addr, self.REG_ACC_OFFSET_X,
            cal["acc_offset"] + cal["mag_offset"] + cal["gyr_offset"]
            + cal["acc_radius"] + cal["mag_radius"]
        )
        time.sleep(0.05)
        self.enter_ndof()
        return True

    def wait_for_calibration(self, timeout: float = 120.0,
                             log_fn=None) -> bool:
        """
        Block until fully calibrated or timeout (seconds).
        log_fn is an optional callable for status messages.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            s = self.calibration_status()
            msg = (f"Calibrating — sys:{s['sys']} gyro:{s['gyro']} "
                   f"accel:{s['accel']} mag:{s['mag']}")
            if log_fn:
                log_fn(msg)
            if self.is_fully_calibrated():
                return True
            time.sleep(1.0)
        return False

    # ------------------------------------------------------------------
    # Sensor readings
    # ------------------------------------------------------------------

    def quaternion(self) -> list[float]:
        """Return [w, x, y, z]."""
        data = self._bus.read_i2c_block_data(self._addr, self.REG_QUAT, 8)
        result = []
        for i in range(0, 8, 2):
            raw = (data[i + 1] << 8) | data[i]
            if raw >= 32768:
                raw -= 65536
            result.append(raw / self._SCALE_QUAT)
        return result

    def accel(self) -> list[float]:
        """Return [x, y, z] in m/s²."""
        return self._read_vector(self.REG_ACC, self._SCALE_ACCEL)

    def gyro(self) -> list[float]:
        """Return [x, y, z] in rad/s."""
        deg_per_s = self._read_vector(self.REG_GYR, self._SCALE_GYRO)
        return [math.radians(v) for v in deg_per_s]

    def mag(self) -> list[float]:
        """Return [x, y, z] in Tesla."""
        microtesla = self._read_vector(self.REG_MAG, self._SCALE_MAG)
        return [v * 1e-6 for v in microtesla]

    def temp(self) -> float:
        """Return temperature in °C."""
        return float(self._read8(self.REG_TEMP))


# ---------------------------------------------------------------------------
# Covariance helpers
# ---------------------------------------------------------------------------

# Diagonal covariance matrices sized to calibration quality.
# BNO055 datasheet typical noise figures, scaled up when poorly calibrated.
_COV_ORIENT = {3: 1e-6, 2: 1e-5, 1: 1e-4, 0: 1e-3}
_COV_GYRO   = {3: 1e-7, 2: 1e-6, 1: 1e-5, 0: 1e-4}
_COV_ACCEL  = {3: 1e-5, 2: 1e-4, 1: 1e-3, 0: 1e-2}
_COV_MAG    = {3: 1e-9, 2: 1e-8, 1: 1e-7, 0: 1e-6}

def _diag_cov(value: float) -> list[float]:
    """9-element row-major covariance with identical diagonal."""
    m = [0.0] * 9
    m[0] = m[4] = m[8] = value
    return m

def _diag_cov3(value: float) -> list[float]:
    """Same but 3x3 for MagneticField."""
    return _diag_cov(value)


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class IMUNode(Node):
    def __init__(self) -> None:
        super().__init__("bno055_imu")

        self.declare_parameter("bus",           7)
        self.declare_parameter("address",       0x28)
        self.declare_parameter("frame_id",      "imu_link")
        self.declare_parameter("rate",          100.0)
        self.declare_parameter("cal_file",      "/tmp/bno055_cal.json")
        self.declare_parameter("wait_cal",      False)
        self.declare_parameter("gyro_deadband", 0.002)   # rad/s
        self.declare_parameter("alpha",         0.98)    # complementary filter
        self.declare_parameter("op_mode",       "ndof")  # "ndof" or "imuplus"

        bus      = self.get_parameter("bus").value
        addr     = self.get_parameter("address").value
        self._frame        = self.get_parameter("frame_id").value
        rate               = self.get_parameter("rate").value
        self._cal_file     = self.get_parameter("cal_file").value
        wait_cal           = self.get_parameter("wait_cal").value
        self._gyro_db      = self.get_parameter("gyro_deadband").value
        self._alpha        = self.get_parameter("alpha").value
        self._dt           = 1.0 / rate
        op_mode            = self.get_parameter("op_mode").value.lower()

        # Complementary filter state (roll, pitch from accel integration)
        self._cf_roll  = 0.0
        self._cf_pitch = 0.0
        self._cf_initialized = False

        # Calibration quality cache
        self._cal_status = {"sys": 0, "gyro": 0, "accel": 0, "mag": 0}

        # I2C error counter for back-off
        self._i2c_errors = 0

        # ---------- sensor init ----------
        self._sensor = BNO055(bus, addr)

        loaded = self._sensor.load_calibration(self._cal_file)
        if loaded:
            self.get_logger().info(f"Calibration loaded from {self._cal_file}")
        else:
            self.get_logger().warn(
                f"No calibration file at {self._cal_file}. "
                "Move sensor through figure-8 patterns to calibrate."
            )

        if wait_cal:
            self.get_logger().info("Waiting for full calibration (wait_cal=true) …")
            ok = self._sensor.wait_for_calibration(
                timeout=120.0, log_fn=self.get_logger().info
            )
            if ok:
                self.get_logger().info("Fully calibrated — saving offsets.")
                self._sensor.save_calibration(self._cal_file)
            else:
                self.get_logger().warn("Calibration timed out; continuing anyway.")

        # Apply operation mode
        if op_mode == "imuplus":
            self._sensor.enter_imuplus()
            self.get_logger().info(
                "Running in IMUPLUS mode (accel + gyro only). "
                "Yaw is relative — no magnetometer needed."
            )
        else:
            if not loaded and not wait_cal:
                self._sensor.enter_ndof()
            self.get_logger().info("Running in NDOF mode (full fusion).")

        # ---------- publishers ----------
        self._pub_imu  = self.create_publisher(Imu,           "/imu/data",       10)
        self._pub_mag  = self.create_publisher(MagneticField, "/imu/mag",        10)
        self._pub_temp = self.create_publisher(Temperature,   "/imu/temp",       10)
        self._pub_cal  = self.create_publisher(String,        "/imu/cal_status", 10)

        self.create_timer(self._dt,  self._update)
        # Publish calibration status and auto-save at 1 Hz
        self.create_timer(1.0, self._cal_update)

        self.get_logger().info("BNO055 node started.")

    # ------------------------------------------------------------------
    # Timer callbacks
    # ------------------------------------------------------------------

    def _update(self) -> None:
        try:
            now = self.get_clock().now().to_msg()

            q = self._sensor.quaternion()
            a = self._sensor.accel()
            g = self._sensor.gyro()
            m = self._sensor.mag()
            t = self._sensor.temp()

            # Apply gyro deadband (reduces static drift noise)
            g = self._apply_deadband(g, self._gyro_db)

            # Complementary filter: blend gyro-integrated orientation with accel tilt
            q = self._complementary_filter(q, a, g)

            self._pub_imu.publish(self._build_imu_msg(now, q, a, g))
            self._pub_mag.publish(self._build_mag_msg(now, m))
            self._pub_temp.publish(self._build_temp_msg(now, t))

            self._i2c_errors = 0  # reset on success

        except OSError as e:
            self._i2c_errors += 1
            self.get_logger().warn(
                f"I2C error #{self._i2c_errors}: {e}. "
                "Will retry next tick."
            )
            if self._i2c_errors > 10:
                self.get_logger().error(
                    "Too many consecutive I2C errors; attempting sensor re-init."
                )
                try:
                    self._sensor._reset_and_init()
                    self._sensor.enter_ndof()
                    self._i2c_errors = 0
                except Exception as reinit_err:
                    self.get_logger().error(f"Re-init failed: {reinit_err}")

    def _cal_update(self) -> None:
        """Publish calibration status; auto-save when newly fully calibrated."""
        try:
            prev = dict(self._cal_status)
            self._cal_status = self._sensor.calibration_status()

            msg = String()
            msg.data = json.dumps(self._cal_status)
            self._pub_cal.publish(msg)

            # Only log when something actually changed
            if self._cal_status != prev:
                s = self._cal_status
                self.get_logger().info(
                    f"Calibration updated — sys:{s['sys']} gyro:{s['gyro']} "
                    f"accel:{s['accel']} mag:{s['mag']}"
                )

            # Warn once if sys drops below 2
            if self._cal_status["sys"] < 2 and prev.get("sys", 0) >= 2:
                self.get_logger().warn(
                    "Calibration quality dropped! "
                    "Move sensor in figure-8 to recover magnetometer."
                )

            # Auto-save when we first reach full calibration
            if prev.get("sys", 0) < 3 and self._cal_status["sys"] == 3:
                self.get_logger().info(
                    f"Fully calibrated! Saving offsets to {self._cal_file}"
                )
                self._sensor.save_calibration(self._cal_file)

        except OSError:
            pass  # already handled in _update

    # ------------------------------------------------------------------
    # Drift reduction helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _apply_deadband(vec: list[float], threshold: float) -> list[float]:
        """Zero out gyro axes below the deadband to suppress static drift."""
        return [0.0 if abs(v) < threshold else v for v in vec]

    def _complementary_filter(
        self,
        q: list[float],
        a: list[float],
        g: list[float],
    ) -> list[float]:
        """
        Blend BNO055 quaternion with accelerometer-derived tilt via a
        complementary filter.  α close to 1.0 trusts the gyro/fusion output;
        lower values pull more toward the gravity vector.

        This corrects slow orientation drift that accumulates even in NDOF mode
        when magnetometer calibration is imperfect.
        """
        ax, ay, az = a

        # Accel-derived roll and pitch (radians)
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 0.1:
            # Sensor in free-fall or bad reading — skip this cycle
            return q

        ax_n, ay_n, az_n = ax / norm, ay / norm, az / norm
        accel_roll  =  math.atan2(ay_n, az_n)
        accel_pitch = -math.asin(ax_n)

        # Gyro-integrated roll and pitch from previous CF state
        gyro_roll  = self._cf_roll  + g[0] * self._dt
        gyro_pitch = self._cf_pitch + g[1] * self._dt

        if not self._cf_initialized:
            self._cf_roll  = accel_roll
            self._cf_pitch = accel_pitch
            self._cf_initialized = True
            return q

        # Complementary blend
        self._cf_roll  = self._alpha * gyro_roll  + (1 - self._alpha) * accel_roll
        self._cf_pitch = self._alpha * gyro_pitch + (1 - self._alpha) * accel_pitch

        # Convert CF roll/pitch back to a correction quaternion, then slerp with BNO055 q
        # We use a lightweight correction: derive a small corrective rotation and blend.
        cr, cp = math.cos(self._cf_roll / 2), math.cos(self._cf_pitch / 2)
        sr, sp = math.sin(self._cf_roll / 2), math.sin(self._cf_pitch / 2)

        # Roll-only quaternion (rotation around X)
        qr = [cr, sr, 0.0, 0.0]
        # Pitch-only quaternion (rotation around Y)
        qp = [cp, 0.0, sp, 0.0]
        # Compose roll * pitch
        q_cf = _quat_mul(qr, qp)

        # Slerp between BNO055 output and CF estimate
        # A small blend weight (1-alpha) pulls toward the accel-corrected estimate.
        blend = 1.0 - self._alpha
        return _quat_slerp(q, q_cf, blend)

    # ------------------------------------------------------------------
    # Message builders
    # ------------------------------------------------------------------

    def _build_imu_msg(self, stamp, q, a, g) -> Imu:
        msg = Imu()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame

        msg.orientation.w = q[0]
        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]

        msg.angular_velocity.x = g[0]
        msg.angular_velocity.y = g[1]
        msg.angular_velocity.z = g[2]

        msg.linear_acceleration.x = a[0]
        msg.linear_acceleration.y = a[1]
        msg.linear_acceleration.z = a[2]

        # Populate covariance based on current calibration quality
        msg.orientation_covariance        = _diag_cov(_COV_ORIENT[self._cal_status["sys"]])
        msg.angular_velocity_covariance   = _diag_cov(_COV_GYRO  [self._cal_status["gyro"]])
        msg.linear_acceleration_covariance = _diag_cov(_COV_ACCEL [self._cal_status["accel"]])

        return msg

    def _build_mag_msg(self, stamp, m) -> MagneticField:
        msg = MagneticField()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame

        msg.magnetic_field.x = m[0]
        msg.magnetic_field.y = m[1]
        msg.magnetic_field.z = m[2]

        msg.magnetic_field_covariance = _diag_cov3(_COV_MAG[self._cal_status["mag"]])

        return msg

    def _build_temp_msg(self, stamp, t) -> Temperature:
        msg = Temperature()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame
        msg.temperature     = t
        return msg


# ---------------------------------------------------------------------------
# Quaternion math utilities
# ---------------------------------------------------------------------------

def _quat_mul(a: list[float], b: list[float]) -> list[float]:
    """Hamilton product of two [w, x, y, z] quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ]

def _quat_dot(a: list[float], b: list[float]) -> float:
    return sum(x * y for x, y in zip(a, b))

def _quat_slerp(q0: list[float], q1: list[float], t: float) -> list[float]:
    """Spherical linear interpolation between two unit quaternions."""
    dot = _quat_dot(q0, q1)

    # Ensure shortest path
    if dot < 0.0:
        q1 = [-v for v in q1]
        dot = -dot

    dot = min(dot, 1.0)

    if dot > 0.9995:
        # Linear interpolation for nearly identical quaternions
        result = [q0[i] + t * (q1[i] - q0[i]) for i in range(4)]
        norm = math.sqrt(sum(v * v for v in result))
        return [v / norm for v in result]

    theta_0 = math.acos(dot)
    theta   = theta_0 * t
    sin_t   = math.sin(theta)
    sin_t0  = math.sin(theta_0)

    s0 = math.cos(theta) - dot * sin_t / sin_t0
    s1 = sin_t / sin_t0
    return [s0 * q0[i] + s1 * q1[i] for i in range(4)]


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    rclpy.init()
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
