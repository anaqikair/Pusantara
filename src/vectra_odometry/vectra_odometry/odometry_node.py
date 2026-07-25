#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odometry_node.py
================
Production-grade wheel odometry node for a differential drive robot.

Fuses wheel encoder ticks with BNO055 IMU orientation to produce:
  - nav_msgs/msg/Odometry on /odom
  - TF broadcast: odom → base_link

Heading is sourced exclusively from the IMU quaternion (yaw).
Linear displacement is sourced exclusively from wheel encoders.

Compatible with: RViz2, SLAM Toolbox, Nav2, robot_localization, tf2.

All quaternion/Euler math is implemented with pure Python (math stdlib).
No dependency on tf_transformations, transforms3d, or numpy is required,
avoiding the np.float removal in NumPy ≥ 1.24 that breaks tf_transformations
on ROS 2 Humble + Jetson (aarch64).

Author  : Production Robotics Team
Platform: ROS 2 Humble | Ubuntu 22.04 | Jetson Orin Nano (aarch64)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from rcl_interfaces.msg import (
    ParameterDescriptor,
    FloatingPointRange,
    SetParametersResult,
)

import tf2_ros

from geometry_msgs.msg import (
    TransformStamped,
    Quaternion,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

# ---------------------------------------------------------------------------
# Module-level constants  (no magic numbers anywhere else in the file)
# ---------------------------------------------------------------------------

TWO_PI: float = 2.0 * math.pi

# Maximum believable encoder jump per *callback* invocation (ticks).
# Physical ceiling at ~1 m/s, 30 Hz, r=0.05 m, 1425 ticks/rev:
#   max_ticks/cycle ≈ 1.0 / ((2π×0.05/1425) × 30) ≈ 152 ticks
# ×10 safety margin → 1500 ticks.
MAX_ENCODER_JUMP_TICKS: int = 1500

# Integration cycle guards (seconds).
MAX_DT_SECONDS: float = 1.0   # ROS time jump / stall threshold
MIN_DT_SECONDS: float = 1e-6  # Numerically unsafe floor

# Watchdog – warn if a source goes silent for this long (wall-clock seconds).
ENCODER_WATCHDOG_TIMEOUT: float = 2.0
IMU_WATCHDOG_TIMEOUT: float = 2.0

# Periodic diagnostic timer period (seconds).
DIAGNOSTIC_PERIOD: float = 5.0

# ---------------------------------------------------------------------------
# Drift-reduction constants
# ---------------------------------------------------------------------------

# Low-pass filter alpha for velocity smoothing (0 = heavy smoothing, 1 = no filter).
# At 30 Hz a value of 0.3 gives a time constant of ~(1/30)/(1-0.3) ≈ 48 ms.
VELOCITY_LPF_ALPHA: float = 0.3

# Zero-velocity deadband: encoder delta (ticks) below which we treat the robot
# as stationary and suppress the integration entirely.  Eliminates accumulation
# from quantisation noise while the robot is still.
ZERO_VELOCITY_TICK_THRESHOLD: int = 1

# Wheel-slip detection: if |left_dist - right_dist| / max(|avg_dist|, ε) exceeds
# this ratio while both wheels are moving, the cycle is flagged as suspect.
# A differential drive going straight should have a near-zero ratio; a hard slip
# produces a ratio >> 1.  We warn but still integrate (Nav2 can handle it).
SLIP_RATIO_THRESHOLD: float = 3.0

# BNO055 orientation calibration minimum (0–3 scale, 3 = fully calibrated).
# Messages below this level are accepted but logged as warnings.
IMU_MIN_ORIENTATION_CAL: int = 1

# Covariance growth rate per metre of travel (position uncertainty grows with
# distance).  Applied additively so that robot_localization / EKF correctly
# deweights long dead-reckoning runs.
POSE_COVARIANCE_GROWTH_PER_METRE: float = 1e-4  # m² / m travelled

# Realistic odometry covariance matrices (6×6, row-major, 36 elements).
# Axes: [x, y, z, roll, pitch, yaw]
# Planar robot: z / roll / pitch are unobservable → 1e6 (effectively ∞).
# Realistic odometry covariance matrices (6×6, row-major, 36 elements).
# Axes: [x, y, z, roll, pitch, yaw]
# Planar robot: z / roll / pitch are unobservable → 1e6 (effectively ∞).
# These are the *baseline* values; pose covariance grows with distance travelled
# (see POSE_COVARIANCE_GROWTH_PER_METRE) so that EKF / robot_localization
# correctly deweights long dead-reckoning segments.
POSE_COVARIANCE_BASE: Tuple[float, ...] = (
    1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,   # x
    0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,   # y
    0.0,  0.0,  1e6,  0.0,  0.0,  0.0,   # z
    0.0,  0.0,  0.0,  1e6,  0.0,  0.0,   # roll
    0.0,  0.0,  0.0,  0.0,  1e6,  0.0,   # pitch
    0.0,  0.0,  0.0,  0.0,  0.0,  1e-2,  # yaw
)

TWIST_COVARIANCE: Tuple[float, ...] = (
    1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,   # vx
    0.0,  1e6,  0.0,  0.0,  0.0,  0.0,   # vy  (non-holonomic → ∞)
    0.0,  0.0,  1e6,  0.0,  0.0,  0.0,   # vz
    0.0,  0.0,  0.0,  1e6,  0.0,  0.0,   # wx
    0.0,  0.0,  0.0,  0.0,  1e6,  0.0,   # wy
    0.0,  0.0,  0.0,  0.0,  0.0,  1e-2,  # wz
)


# ---------------------------------------------------------------------------
# Pure-Python quaternion helpers  (no external dependencies)
# ---------------------------------------------------------------------------

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> Optional[float]:
    """
    Extract yaw (rotation around Z-axis, ZYX convention) from a quaternion.

    The quaternion is normalised internally to tolerate minor sensor drift.

    Returns
    -------
    float
        Yaw angle in radians, in the range (-π, π].
    None
        If the quaternion is degenerate (near-zero norm) and cannot be used.
    """
    norm_sq: float = qx * qx + qy * qy + qz * qz + qw * qw
    if norm_sq < 1e-10:
        return None

    inv_norm: float = 1.0 / math.sqrt(norm_sq)
    qx *= inv_norm
    qy *= inv_norm
    qz *= inv_norm
    qw *= inv_norm

    # Standard ZYX yaw extraction.
    siny_cosp: float = 2.0 * (qw * qz + qx * qy)
    cosy_cosp: float = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert a planar yaw angle to a unit quaternion (roll=pitch=0).

    Parameters
    ----------
    yaw : float
        Heading in radians.

    Returns
    -------
    Tuple[float, float, float, float]
        (qx, qy, qz, qw)
    """
    half: float = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def build_quaternion_msg(qx: float, qy: float, qz: float, qw: float) -> Quaternion:
    """Construct a geometry_msgs/Quaternion from scalar components."""
    q = Quaternion()
    q.x = float(qx)
    q.y = float(qy)
    q.z = float(qz)
    q.w = float(qw)
    return q


def normalize_angle(angle: float) -> float:
    """
    Wrap an angle to the range (-π, π].

    Used to prevent yaw discontinuities when the IMU crosses the ±π boundary
    from causing large, erroneous position deltas.
    """
    while angle > math.pi:
        angle -= TWO_PI
    while angle <= -math.pi:
        angle += TWO_PI
    return angle


def is_finite(*values: float) -> bool:
    """Return True only if every value is finite (not NaN, not ±Inf)."""
    return all(math.isfinite(v) for v in values)


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------

@dataclass
class EncoderState:
    """Validated cumulative encoder tick counts."""
    left_ticks: int = 0
    right_ticks: int = 0
    initialized: bool = False
    last_stamp: Optional[Time] = None


@dataclass
class ImuState:
    """Validated IMU orientation from BNO055 fused output."""
    yaw: float = 0.0
    quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # (x,y,z,w)
    initialized: bool = False
    last_stamp: Optional[Time] = None


@dataclass
class RobotPose:
    """Current 2-D robot pose in the odom frame."""
    x: float = 0.0    # metres
    y: float = 0.0    # metres
    yaw: float = 0.0  # radians — always mirrors ImuState.yaw
    distance_travelled: float = 0.0  # cumulative metres; used for covariance growth


@dataclass
class RobotVelocity:
    """Body-frame robot velocities for the most recent integration cycle."""
    linear_x: float = 0.0   # m/s   (forward positive)
    angular_z: float = 0.0  # rad/s (CCW positive)


@dataclass
class NodeParameters:
    """Live-updatable copy of the node's ROS 2 parameters."""
    wheel_radius: float = 0.05
    wheel_base: float = 0.33
    ticks_per_revolution: int = 1425
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    publish_rate: float = 30.0

    @property
    def distance_per_tick(self) -> float:
        """Metres of travel per encoder tick."""
        return (TWO_PI * self.wheel_radius) / float(self.ticks_per_revolution)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class OdometryNode(Node):
    """
    Differential-drive odometry node.

    Subscribes
    ----------
    /wheel_encoder  std_msgs/Int32MultiArray  – cumulative [left, right] ticks
    /imu/data       sensor_msgs/Imu           – BNO055 fused orientation

    Publishes
    ---------
    /odom           nav_msgs/Odometry

    TF broadcast
    ------------
    odom → base_link  (every publish cycle, unconditionally)
    """

    def __init__(self) -> None:
        super().__init__("odometry_node")

        # ── Parameters ────────────────────────────────────────────────────
        self._params: NodeParameters = NodeParameters()
        self._declare_parameters()
        self._read_parameters()

        # ── Sensor state ──────────────────────────────────────────────────
        self._encoder: EncoderState = EncoderState()
        self._imu: ImuState = ImuState()

        # ── Robot state ───────────────────────────────────────────────────
        self._pose: RobotPose = RobotPose()
        self._velocity: RobotVelocity = RobotVelocity()

        # Integration bookkeeping.
        # _last_odom_time  : ROS Time of the previous publish cycle.
        # _prev_left_ticks : left  encoder count at the previous cycle.
        # _prev_right_ticks: right encoder count at the previous cycle.
        # All three are None until both sensors are ready.
        self._last_odom_time: Optional[Time] = None
        self._prev_left_ticks: Optional[int] = None
        self._prev_right_ticks: Optional[int] = None

        # Smoothed (low-pass filtered) velocities — initialised to zero and
        # updated each cycle with VELOCITY_LPF_ALPHA.  Publishing the filtered
        # rather than raw velocity suppresses encoder quantisation noise in
        # /odom twist, which prevents EKF from over-reacting to per-tick spikes.
        self._smooth_vx: float = 0.0
        self._smooth_wz: float = 0.0

        # Previous IMU yaw — used to detect ±π wraparound discontinuities so
        # that the heading update never introduces a >π jump into the pose.
        self._prev_imu_yaw: Optional[float] = None

        # Wall-clock timestamps for watchdog diagnostics.
        self._last_encoder_wall: float = time.monotonic()
        self._last_imu_wall: float = time.monotonic()

        # ── TF broadcaster ────────────────────────────────────────────────
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── QoS profiles ─────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self._encoder_sub = self.create_subscription(
            Int32MultiArray,
            "/wheel_encoder",
            self._encoder_callback,
            sensor_qos,
        )
        self._imu_sub = self.create_subscription(
            Imu,
            "/imu/data",
            self._imu_callback,
            sensor_qos,
        )

        # ── Publisher ─────────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, "/odom", reliable_qos)

        # ── Timers ───────────────────────────────────────────────────────
        self._publish_timer = self.create_timer(
            1.0 / self._params.publish_rate,
            self._publish_odometry,
        )
        self._diagnostic_timer = self.create_timer(
            DIAGNOSTIC_PERIOD,
            self._run_diagnostics,
        )

        # ── Parameter update callback ─────────────────────────────────────
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        # ── Startup log ───────────────────────────────────────────────────
        self._log_startup_summary()

    # ------------------------------------------------------------------ #
    # Parameter management                                                #
    # ------------------------------------------------------------------ #

    def _declare_parameters(self) -> None:
        """Declare all node parameters with descriptions and range constraints."""
        self.declare_parameter(
            "wheel_radius",
            value=0.05,
            descriptor=ParameterDescriptor(
                description="Wheel radius in metres.",
                floating_point_range=[
                    FloatingPointRange(from_value=0.001, to_value=1.0, step=0.0)
                ],
            ),
        )
        self.declare_parameter(
            "wheel_base",
            value=0.33,
            descriptor=ParameterDescriptor(
                description="Distance between the two drive wheels (metres).",
                floating_point_range=[
                    FloatingPointRange(from_value=0.01, to_value=5.0, step=0.0)
                ],
            ),
        )
        self.declare_parameter(
            "ticks_per_revolution",
            value=1425,
            descriptor=ParameterDescriptor(
                description="Encoder ticks per full wheel revolution.",
            ),
        )
        self.declare_parameter(
            "odom_frame",
            value="odom",
            descriptor=ParameterDescriptor(
                description="TF frame ID for the odometry parent frame.",
            ),
        )
        self.declare_parameter(
            "base_frame",
            value="base_link",
            descriptor=ParameterDescriptor(
                description="TF frame ID for the robot base.",
            ),
        )
        self.declare_parameter(
            "publish_rate",
            value=30.0,
            descriptor=ParameterDescriptor(
                description="Odometry publish rate in Hz.",
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=200.0, step=0.0)
                ],
            ),
        )

    def _read_parameters(self) -> None:
        """Read all declared parameters into the local cache."""
        self._params.wheel_radius = (
            self.get_parameter("wheel_radius").get_parameter_value().double_value
        )
        self._params.wheel_base = (
            self.get_parameter("wheel_base").get_parameter_value().double_value
        )
        self._params.ticks_per_revolution = (
            self.get_parameter("ticks_per_revolution").get_parameter_value().integer_value
        )
        self._params.odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self._params.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self._params.publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

    def _on_parameters_changed(self, params: list) -> SetParametersResult:
        """
        Live parameter update callback.

        Validates every incoming change and applies all atomically only if
        the entire set passes validation.
        """
        # Work on a temporary copy so we either accept all or none.
        candidate = NodeParameters(
            wheel_radius=self._params.wheel_radius,
            wheel_base=self._params.wheel_base,
            ticks_per_revolution=self._params.ticks_per_revolution,
            odom_frame=self._params.odom_frame,
            base_frame=self._params.base_frame,
            publish_rate=self._params.publish_rate,
        )

        for param in params:
            name = param.name
            val = param.value

            if name == "wheel_radius":
                if not isinstance(val, float) or val <= 0.0 or not math.isfinite(val):
                    return SetParametersResult(
                        successful=False,
                        reason="wheel_radius must be a positive finite float.",
                    )
                candidate.wheel_radius = val

            elif name == "wheel_base":
                if not isinstance(val, float) or val <= 0.0 or not math.isfinite(val):
                    return SetParametersResult(
                        successful=False,
                        reason="wheel_base must be a positive finite float.",
                    )
                candidate.wheel_base = val

            elif name == "ticks_per_revolution":
                if not isinstance(val, int) or val <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason="ticks_per_revolution must be a positive integer.",
                    )
                candidate.ticks_per_revolution = val

            elif name == "odom_frame":
                if not isinstance(val, str) or not val.strip():
                    return SetParametersResult(
                        successful=False,
                        reason="odom_frame must be a non-empty string.",
                    )
                candidate.odom_frame = val

            elif name == "base_frame":
                if not isinstance(val, str) or not val.strip():
                    return SetParametersResult(
                        successful=False,
                        reason="base_frame must be a non-empty string.",
                    )
                candidate.base_frame = val

            elif name == "publish_rate":
                if not isinstance(val, float) or val <= 0.0 or not math.isfinite(val):
                    return SetParametersResult(
                        successful=False,
                        reason="publish_rate must be a positive finite float.",
                    )
                candidate.publish_rate = val

        # All checks passed — apply atomically.
        self._params = candidate
        self.get_logger().info(
            "Parameters updated live: "
            f"wheel_radius={self._params.wheel_radius:.5f} m  "
            f"wheel_base={self._params.wheel_base:.5f} m  "
            f"ticks_per_rev={self._params.ticks_per_revolution}  "
            f"publish_rate={self._params.publish_rate:.1f} Hz"
        )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ #
    # Subscriber callbacks                                                #
    # ------------------------------------------------------------------ #

    def _encoder_callback(self, msg: Int32MultiArray) -> None:
        """
        Receive and validate a cumulative encoder tick message.

        The message must contain exactly two integers:
          data[0] = left  wheel cumulative ticks
          data[1] = right wheel cumulative ticks

        Corrupted or implausible messages are discarded with a warning.
        """
        # ── Structural validation ─────────────────────────────────────
        if len(msg.data) < 2:
            self.get_logger().error(
                f"Encoder message has {len(msg.data)} element(s); expected ≥ 2. "
                "Discarding."
            )
            return

        left_raw: int = int(msg.data[0])
        right_raw: int = int(msg.data[1])

        # ── Overflow / corruption guard ───────────────────────────────
        # Reject values that cannot be real encoder counts (e.g. wrap-around
        # from a 16-bit register being cast to 32-bit with sign extension).
        overflow_limit: int = 2 ** 30
        for label, value in (("left", left_raw), ("right", right_raw)):
            if abs(value) > overflow_limit:
                self.get_logger().error(
                    f"Encoder {label} tick value {value} exceeds overflow limit "
                    f"(±{overflow_limit}). Possible hardware fault. Discarding."
                )
                return

        # ── First message: initialise without computing a delta ───────
        if not self._encoder.initialized:
            self._encoder.left_ticks = left_raw
            self._encoder.right_ticks = right_raw
            self._encoder.initialized = True
            self._encoder.last_stamp = self.get_clock().now()
            self._last_encoder_wall = time.monotonic()
            self.get_logger().info(
                f"Encoder initialised — left: {left_raw}  right: {right_raw}"
            )
            return

        # ── Per-message jump detection ────────────────────────────────
        # Reject a single callback that moves more than MAX_ENCODER_JUMP_TICKS.
        left_delta: int = left_raw - self._encoder.left_ticks
        right_delta: int = right_raw - self._encoder.right_ticks

        if abs(left_delta) > MAX_ENCODER_JUMP_TICKS:
            self.get_logger().warning(
                f"Left encoder jumped {left_delta} ticks in one message "
                f"(limit ±{MAX_ENCODER_JUMP_TICKS}). "
                f"prev={self._encoder.left_ticks}  new={left_raw}. Discarding."
            )
            return

        if abs(right_delta) > MAX_ENCODER_JUMP_TICKS:
            self.get_logger().warning(
                f"Right encoder jumped {right_delta} ticks in one message "
                f"(limit ±{MAX_ENCODER_JUMP_TICKS}). "
                f"prev={self._encoder.right_ticks}  new={right_raw}. Discarding."
            )
            return

        # ── Store validated ticks ─────────────────────────────────────
        self._encoder.left_ticks = left_raw
        self._encoder.right_ticks = right_raw
        self._encoder.last_stamp = self.get_clock().now()
        self._last_encoder_wall = time.monotonic()

    def _imu_callback(self, msg: Imu) -> None:
        """
        Receive and validate a BNO055 IMU message.

        Extracts fused yaw from the orientation quaternion.
        Non-finite or degenerate quaternions are discarded with a warning.
        """
        q = msg.orientation
        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        qw = float(q.w)

        # ── Non-finite component guard ────────────────────────────────
        if not is_finite(qx, qy, qz, qw):
            self.get_logger().warning(
                f"IMU quaternion contains non-finite values "
                f"({qx}, {qy}, {qz}, {qw}). Discarding."
            )
            return

        # ── Extract yaw (handles norm check and normalisation) ────────
        yaw = quaternion_to_yaw(qx, qy, qz, qw)
        if yaw is None:
            self.get_logger().warning(
                f"IMU quaternion has near-zero norm "
                f"({math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw):.8f}). "
                "Discarding — BNO055 may not yet have a valid fix."
            )
            return

        if not math.isfinite(yaw):
            self.get_logger().warning(
                f"Extracted yaw is not finite ({yaw}). Discarding IMU message."
            )
            return

        # ── Store validated state ─────────────────────────────────────
        self._imu.yaw = yaw
        self._imu.quaternion = (qx, qy, qz, qw)
        self._imu.initialized = True
        self._imu.last_stamp = self.get_clock().now()
        self._last_imu_wall = time.monotonic()

        # ── BNO055 orientation calibration check ──────────────────────
        # The BNO055 stores its orientation calibration status in
        # linear_acceleration_covariance[0] as an integer 0–3.
        # Values below IMU_MIN_ORIENTATION_CAL indicate poor sensor fusion
        # quality and should be flagged so the operator can wait for the
        # sensor to converge before trusting the heading output.
        cal_raw = msg.linear_acceleration_covariance[0]
        cal_status = int(round(cal_raw))
        if cal_status < IMU_MIN_ORIENTATION_CAL:
            self.get_logger().warning(
                f"BNO055 orientation calibration status={cal_status}/3 "
                f"(minimum acceptable={IMU_MIN_ORIENTATION_CAL}). "
                "Heading accuracy may be degraded; rotate the robot to calibrate."
            )

        # ── Yaw discontinuity logging ─────────────────────────────────
        # A ±π wraparound (e.g. 179° → -179°) produces a raw delta of ~2π
        # which is correct after normalize_angle, but worth logging so that
        # operators know the robot crossed the ±180° boundary.
        if self._prev_imu_yaw is not None:
            raw_delta = normalize_angle(yaw - self._prev_imu_yaw)
            if abs(raw_delta) > math.radians(90.0):
                self.get_logger().warning(
                    f"IMU yaw changed {math.degrees(raw_delta):.1f}° in one message "
                    f"(prev={math.degrees(self._prev_imu_yaw):.1f}°  "
                    f"new={math.degrees(yaw):.1f}°). "
                    "Possible ±π wraparound or aggressive manoeuvre."
                )
        self._prev_imu_yaw = yaw

        # ── Unlock odometry integration once both sources are ready ───
        if self._encoder.initialized and self._last_odom_time is None:
            self._last_odom_time = self.get_clock().now()
            self._pose.yaw = yaw
            self.get_logger().info(
                "Both encoder and IMU ready. "
                f"Odometry integration started. Initial yaw: {math.degrees(yaw):.2f}°"
            )

    # ------------------------------------------------------------------ #
    # Odometry publish timer                                              #
    # ------------------------------------------------------------------ #

    def _publish_odometry(self) -> None:
        """
        Timer callback executed at `publish_rate` Hz.

        Steps
        -----
        1. Guard: both sensors must be ready.
        2. Compute dt; reject degenerate intervals.
        3. Snapshot encoder counts; compute delta ticks.
        4. Validate delta ticks against dt-scaled plausibility limit.
        5. Convert to distances; read IMU yaw.
        6. Integrate pose (x, y from encoders; yaw from IMU).
        7. Compute body-frame velocities.
        8. Publish /odom.
        9. Broadcast odom → base_link TF.
        """
        now: Time = self.get_clock().now()

        # ── Guard: sensors not ready ──────────────────────────────────
        if not self._encoder.initialized:
            self.get_logger().info(
                "Waiting for first /wheel_encoder message …",
            )
            return

        if not self._imu.initialized:
            self.get_logger().info(
                "Waiting for first /imu/data message …",
            )
            return

        if self._last_odom_time is None:
            # Sensors became ready during this tick; _imu_callback sets
            # _last_odom_time.  Nothing to integrate yet.
            return

        # ── Compute dt ────────────────────────────────────────────────
        dt_ns: int = (now - self._last_odom_time).nanoseconds
        dt: float = dt_ns * 1e-9

        if dt < MIN_DT_SECONDS:
            # Timer fired again before a full nanosecond elapsed — skip.
            return

        if dt < 0.0:
            self.get_logger().warning(
                f"Negative dt ({dt:.9f} s): ROS clock went backwards. "
                "Resetting time reference."
            )
            self._last_odom_time = now
            return

        if dt > MAX_DT_SECONDS:
            self.get_logger().warning(
                f"dt={dt:.3f} s exceeds limit ({MAX_DT_SECONDS} s). "
                "Possible timer stall or sim time jump. Resetting time reference."
            )
            self._last_odom_time = now
            return

        # ── Snapshot and initialise integration trackers ──────────────
        left_now: int = self._encoder.left_ticks
        right_now: int = self._encoder.right_ticks

        if self._prev_left_ticks is None or self._prev_right_ticks is None:
            # First integration cycle after unlock: seed previous values and
            # return without integrating to avoid a stale-data spike.
            self._prev_left_ticks = left_now
            self._prev_right_ticks = right_now
            self._last_odom_time = now
            return

        # ── Delta ticks ───────────────────────────────────────────────
        delta_left: int = left_now - self._prev_left_ticks
        delta_right: int = right_now - self._prev_right_ticks

        # dt-scaled plausibility guard: the maximum number of ticks that
        # could legitimately accumulate over the actual elapsed time.
        # Uses the nominal period as the baseline so that a slightly slow
        # timer fire is not incorrectly rejected.
        nominal_period: float = 1.0 / self._params.publish_rate
        # Allow up to 2× the nominal period worth of ticks as headroom.
        max_ticks_this_cycle: int = int(
            MAX_ENCODER_JUMP_TICKS * max(dt / nominal_period, 1.0) * 2.0
        )

        if abs(delta_left) > max_ticks_this_cycle:
            self.get_logger().warning(
                f"Left encoder integration delta {delta_left} ticks over {dt:.4f} s "
                f"exceeds scaled limit ±{max_ticks_this_cycle}. Skipping cycle."
            )
            self._prev_left_ticks = left_now
            self._prev_right_ticks = right_now
            self._last_odom_time = now
            return

        if abs(delta_right) > max_ticks_this_cycle:
            self.get_logger().warning(
                f"Right encoder integration delta {delta_right} ticks over {dt:.4f} s "
                f"exceeds scaled limit ±{max_ticks_this_cycle}. Skipping cycle."
            )
            self._prev_left_ticks = left_now
            self._prev_right_ticks = right_now
            self._last_odom_time = now
            return

        # ── Advance integration trackers ──────────────────────────────
        self._prev_left_ticks = left_now
        self._prev_right_ticks = right_now

        # ── Ticks → distances ─────────────────────────────────────────
        dpt: float = self._params.distance_per_tick
        left_dist: float = float(delta_left) * dpt
        right_dist: float = float(delta_right) * dpt

        if not is_finite(left_dist, right_dist):
            self.get_logger().error(
                f"Non-finite wheel distance: left={left_dist}, right={right_dist}. "
                f"delta_left={delta_left}, delta_right={delta_right}, dpt={dpt:.8f}. "
                "Skipping cycle."
            )
            self._last_odom_time = now
            return

        linear_dist: float = (left_dist + right_dist) * 0.5

        # ── Zero-velocity deadband ────────────────────────────────────
        # If both wheels report zero ticks (or are within the noise floor),
        # treat the robot as stationary and skip the integration entirely.
        # This prevents floating-point quantisation noise from accumulating
        # into a slow positional drift while the robot is at rest.
        if (abs(delta_left) <= ZERO_VELOCITY_TICK_THRESHOLD and
                abs(delta_right) <= ZERO_VELOCITY_TICK_THRESHOLD):
            # Still update time reference and publish zero velocity.
            self._smooth_vx = self._smooth_vx * VELOCITY_LPF_ALPHA  # decay to 0
            self._smooth_wz = self._smooth_wz * VELOCITY_LPF_ALPHA
            self._velocity.linear_x = self._smooth_vx
            self._velocity.angular_z = self._smooth_wz
            self._odom_pub.publish(self._build_odometry_msg(now))
            self._broadcast_tf(now)
            self._last_odom_time = now
            return

        # ── Wheel-slip detection ──────────────────────────────────────
        # A differential drive robot going straight should have nearly equal
        # wheel distances.  When they diverge wildly relative to the average,
        # it usually means a wheel is slipping or the encoder is glitching.
        # We log a warning but still integrate; the IMU heading bounds the
        # angular error, so a slip on a single wheel mostly contaminates
        # linear distance only.
        avg_abs_dist = max(abs(linear_dist), 1e-9)
        slip_ratio = abs(left_dist - right_dist) / avg_abs_dist
        if slip_ratio > SLIP_RATIO_THRESHOLD and abs(linear_dist) > 1e-4:
            self.get_logger().warning(
                f"Possible wheel slip: left={left_dist*1000:.2f} mm  "
                f"right={right_dist*1000:.2f} mm  "
                f"ratio={slip_ratio:.2f} (threshold {SLIP_RATIO_THRESHOLD}). "
                "Integrating anyway; heading anchored by IMU."
            )

        # ── IMU heading (with wraparound protection) ──────────────────
        # Use normalize_angle to ensure the yaw used for projection is always
        # in (-π, π], preventing cos/sin from receiving out-of-range inputs
        # after many full rotations (though math.cos/sin handle this fine,
        # normalising keeps diagnostics readable).
        yaw: float = normalize_angle(self._imu.yaw)

        # ── Pose integration ──────────────────────────────────────────
        # Use the midpoint (average) heading between previous and current yaw
        # for the direction of travel.  This is the standard second-order
        # Runge-Kutta correction for differential drive and reduces the
        # lateral drift that accumulates from applying end-of-cycle heading
        # to the whole arc, especially during turning manoeuvres.
        prev_yaw: float = normalize_angle(self._pose.yaw)
        mid_yaw: float = prev_yaw + 0.5 * normalize_angle(yaw - prev_yaw)

        delta_x: float = linear_dist * math.cos(mid_yaw)
        delta_y: float = linear_dist * math.sin(mid_yaw)

        if not is_finite(delta_x, delta_y):
            self.get_logger().error(
                f"Non-finite pose delta: dx={delta_x}, dy={delta_y}. "
                f"linear_dist={linear_dist:.6f}, mid_yaw={mid_yaw:.6f}. Skipping cycle."
            )
            self._last_odom_time = now
            return

        self._pose.x += delta_x
        self._pose.y += delta_y
        self._pose.yaw = yaw   # Always from IMU; never integrated from encoders.
        self._pose.distance_travelled += abs(linear_dist)

        # ── Velocities (low-pass filtered) ───────────────────────────
        # Raw tick-based velocity is quantised and noisy at low speeds.
        # A single-pole IIR (exponential moving average) smooths the output
        # without introducing the phase lag of a moving-window average.
        # angular_z comes from the wheel differential (not IMU derivative)
        # to avoid differentiating already-noisy orientation data.
        raw_vx: float = linear_dist / dt
        raw_wz: float = (right_dist - left_dist) / (self._params.wheel_base * dt)

        if not is_finite(raw_vx, raw_wz):
            self.get_logger().error(
                f"Non-finite velocity: vx={raw_vx}, wz={raw_wz}. Zeroing for this cycle."
            )
            raw_vx = 0.0
            raw_wz = 0.0

        self._smooth_vx = (VELOCITY_LPF_ALPHA * raw_vx +
                           (1.0 - VELOCITY_LPF_ALPHA) * self._smooth_vx)
        self._smooth_wz = (VELOCITY_LPF_ALPHA * raw_wz +
                           (1.0 - VELOCITY_LPF_ALPHA) * self._smooth_wz)

        self._velocity.linear_x = self._smooth_vx
        self._velocity.angular_z = self._smooth_wz

        # ── Publish /odom ─────────────────────────────────────────────
        self._odom_pub.publish(self._build_odometry_msg(now))

        # ── Broadcast TF ─────────────────────────────────────────────
        self._broadcast_tf(now)

        # ── Advance time reference ────────────────────────────────────
        self._last_odom_time = now

    # ------------------------------------------------------------------ #
    # Message builders                                                    #
    # ------------------------------------------------------------------ #

    def _build_odometry_msg(self, stamp: Time) -> Odometry:
        """
        Construct a fully-populated nav_msgs/Odometry message.

        Parameters
        ----------
        stamp : Time
            ROS clock timestamp for the header.

        Returns
        -------
        Odometry
            Ready-to-publish message with all fields set.
        """
        msg = Odometry()

        # Header.
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._params.odom_frame
        msg.child_frame_id = self._params.base_frame

        # Pose: position.
        msg.pose.pose.position.x = self._pose.x
        msg.pose.pose.position.y = self._pose.y
        msg.pose.pose.position.z = 0.0

        # Pose: orientation (yaw-only quaternion from pose state).
        qx, qy, qz, qw = yaw_to_quaternion(self._pose.yaw)
        msg.pose.pose.orientation = build_quaternion_msg(qx, qy, qz, qw)

        # Pose covariance (36-element, row-major 6×6).
        # The x and y diagonal elements grow with cumulative distance travelled
        # so that downstream EKF (robot_localization) correctly deweights
        # odometry that has accumulated over a long dead-reckoning segment.
        pose_cov = list(POSE_COVARIANCE_BASE)
        extra_xy = self._pose.distance_travelled * POSE_COVARIANCE_GROWTH_PER_METRE
        pose_cov[0]  += extra_xy   # x–x
        pose_cov[7]  += extra_xy   # y–y
        msg.pose.covariance = pose_cov

        # Twist: linear (body frame).
        msg.twist.twist.linear.x = self._velocity.linear_x
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        # Twist: angular.
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self._velocity.angular_z

        # Twist covariance.
        msg.twist.covariance = list(TWIST_COVARIANCE)

        return msg

    def _broadcast_tf(self, stamp: Time) -> None:
        """
        Broadcast the odom → base_link transform.

        Must be called every publish cycle without exception so that
        SLAM Toolbox, Nav2, and RViz2 never experience a TF timeout.
        """
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self._params.odom_frame
        t.child_frame_id = self._params.base_frame

        t.transform.translation.x = self._pose.x
        t.transform.translation.y = self._pose.y
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self._pose.yaw)
        t.transform.rotation = build_quaternion_msg(qx, qy, qz, qw)

        self._tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------ #
    # Diagnostics                                                         #
    # ------------------------------------------------------------------ #

    def _run_diagnostics(self) -> None:
        """
        Periodic watchdog — called every DIAGNOSTIC_PERIOD seconds.

        Checks for:
          • Missing or stale encoder data
          • Missing or stale IMU data
          • NaN / Inf contamination in pose and velocity
        """
        now_wall: float = time.monotonic()

        # ── Encoder watchdog ──────────────────────────────────────────
        if not self._encoder.initialized:
            self.get_logger().warning(
                "DIAGNOSTIC: No encoder message received yet on /wheel_encoder."
            )
        else:
            age = now_wall - self._last_encoder_wall
            if age > ENCODER_WATCHDOG_TIMEOUT:
                self.get_logger().warning(
                    f"DIAGNOSTIC: Encoder silent for {age:.1f} s "
                    f"(timeout {ENCODER_WATCHDOG_TIMEOUT} s). "
                    "Check /wheel_encoder publisher."
                )

        # ── IMU watchdog ──────────────────────────────────────────────
        if not self._imu.initialized:
            self.get_logger().warning(
                "DIAGNOSTIC: No IMU message received yet on /imu/data."
            )
        else:
            age = now_wall - self._last_imu_wall
            if age > IMU_WATCHDOG_TIMEOUT:
                self.get_logger().warning(
                    f"DIAGNOSTIC: IMU silent for {age:.1f} s "
                    f"(timeout {IMU_WATCHDOG_TIMEOUT} s). "
                    "Check /imu/data publisher."
                )

        # ── Pose sanity ───────────────────────────────────────────────
        if not is_finite(self._pose.x, self._pose.y, self._pose.yaw):
            self.get_logger().error(
                "DIAGNOSTIC: Pose contains non-finite values — "
                f"x={self._pose.x}  y={self._pose.y}  yaw={self._pose.yaw}. "
                "Odometry has diverged."
            )

        # ── Velocity sanity ───────────────────────────────────────────
        if not is_finite(self._velocity.linear_x, self._velocity.angular_z):
            self.get_logger().error(
                "DIAGNOSTIC: Velocity contains non-finite values — "
                f"vx={self._velocity.linear_x}  wz={self._velocity.angular_z}."
            )

        # ── Status summary (debug level, suppressed in default config) ─
        self.get_logger().debug(
            "DIAGNOSTIC STATUS | "
            f"pose=({self._pose.x:.3f} m, {self._pose.y:.3f} m, "
            f"{math.degrees(self._pose.yaw):.1f}°) | "
            f"dist_travelled={self._pose.distance_travelled:.2f} m | "
            f"vx={self._velocity.linear_x:.3f} m/s (smoothed)  "
            f"wz={self._velocity.angular_z:.3f} rad/s (smoothed) | "
            f"encoder={'OK' if self._encoder.initialized else 'WAITING'} | "
            f"imu={'OK' if self._imu.initialized else 'WAITING'}"
        )

    # ------------------------------------------------------------------ #
    # Startup logging                                                     #
    # ------------------------------------------------------------------ #

    def _log_startup_summary(self) -> None:
        """Emit a full parameter and configuration summary at startup."""
        dpt = self._params.distance_per_tick
        circ = TWO_PI * self._params.wheel_radius

        self.get_logger().info("=" * 62)
        self.get_logger().info("   odometry_node  —  starting")
        self.get_logger().info("=" * 62)
        self.get_logger().info("   Platform : ROS 2 Humble | Ubuntu 22.04 | aarch64")
        self.get_logger().info("   Robot    : Differential drive")
        self.get_logger().info("-" * 62)
        self.get_logger().info("   PARAMETERS")
        self.get_logger().info(f"     wheel_radius         : {self._params.wheel_radius:.6f} m")
        self.get_logger().info(f"     wheel_circumference  : {circ:.6f} m")
        self.get_logger().info(f"     wheel_base           : {self._params.wheel_base:.6f} m")
        self.get_logger().info(f"     ticks_per_revolution : {self._params.ticks_per_revolution}")
        self.get_logger().info(f"     distance_per_tick    : {dpt:.8f} m/tick")
        self.get_logger().info(f"     odom_frame           : {self._params.odom_frame}")
        self.get_logger().info(f"     base_frame           : {self._params.base_frame}")
        self.get_logger().info(f"     publish_rate         : {self._params.publish_rate:.1f} Hz")
        self.get_logger().info("-" * 62)
        self.get_logger().info("   SUBSCRIPTIONS")
        self.get_logger().info("     /wheel_encoder  →  std_msgs/Int32MultiArray")
        self.get_logger().info("     /imu/data       →  sensor_msgs/Imu")
        self.get_logger().info("   PUBLICATIONS")
        self.get_logger().info("     /odom           →  nav_msgs/Odometry")
        self.get_logger().info("   TF")
        self.get_logger().info(
            f"     {self._params.odom_frame} → {self._params.base_frame}"
        )
        self.get_logger().info("-" * 62)
        self.get_logger().info("   Heading  : IMU quaternion (BNO055 fused, no integration)")
        self.get_logger().info("   Distance : Wheel encoders (cumulative tick delta)")
        self.get_logger().info("   Drift reduction features:")
        self.get_logger().info(f"     Zero-vel deadband    : ±{ZERO_VELOCITY_TICK_THRESHOLD} tick(s)")
        self.get_logger().info(f"     Velocity LPF alpha   : {VELOCITY_LPF_ALPHA}")
        self.get_logger().info(f"     RK2 heading midpoint : enabled")
        self.get_logger().info(f"     Wheel-slip threshold : ratio > {SLIP_RATIO_THRESHOLD}")
        self.get_logger().info(f"     Cov growth rate      : {POSE_COVARIANCE_GROWTH_PER_METRE} m²/m")
        self.get_logger().info(f"     IMU cal min level    : {IMU_MIN_ORIENTATION_CAL}/3")
        self.get_logger().info("-" * 62)
        self.get_logger().info("   Waiting for /wheel_encoder and /imu/data …")
        self.get_logger().info("=" * 62)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    """ROS 2 entry point registered in setup.py console_scripts."""
    rclpy.init(args=args)

    node: Optional[OdometryNode] = None
    try:
        node = OdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("KeyboardInterrupt — shutting down.")
    except Exception as exc:
        # Ensure the exception appears in the ROS 2 / journald log before
        # the process exits so that it is visible in `ros2 launch` output.
        if node is not None:
            node.get_logger().fatal(
                f"Unhandled exception in odometry_node: "
                f"{type(exc).__name__}: {exc}"
            )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
