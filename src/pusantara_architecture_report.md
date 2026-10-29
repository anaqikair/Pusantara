# 🤖 PUSANTARA — ROS2 Architecture Report
> **Project:** PUSANTARA (Malaysian Cultural Heritage Robot)
> **Platform:** NVIDIA Jetson Orin Nano · ROS2 Humble · Ubuntu 22.04 (aarch64)
> **Total Packages:** 30 ROS2 packages + 1 PDF + misc files
> **Report Date:** 2026-10-22

---

## Table of Contents
1. [System Overview](#1-system-overview)
2. [Layered Architecture](#2-layered-architecture)
3. [Detailed Node Analysis (Every Node)](#3-detailed-node-analysis)
4. [Topic / Message Flow Map](#4-topic--message-flow-map)
5. [Issues Found](#5-issues-found)
6. [Architecture Cleanup Recommendations](#6-architecture-cleanup-recommendations)

---

## 1. System Overview

PUSANTARA is a **differential-drive museum / exhibition robot** that:
- Navigates autonomously using **Nav2 + AMCL + LiDAR**
- Detects visitors using **YOLOv8** and follows/greets them
- Can follow **AprilTag** fiducial markers (guided tours)
- Offers **cultural heritage interactive apps** (quizzes, Batik designer, AI guide)
- Has a **face expression display** (Pygame) and **text-to-speech** output
- Runs a **PyQt5 admin console** for operator control
- Has an **environmental monitoring** subsystem (DHT11 humidity/temp sensor)
- Has a **CaveCare mission** subsystem (mold + environmental alert)

---

## 2. Layered Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         OPERATOR / ADMIN LAYER                          │
│  pusantara_admin (PyQt5 console)   pusantara_demo_loop (Tkinter GUI)    │
└─────────────────────────────┬───────────────────────────────────────────┘
                              │ Nav2 Actions / /tts_text / /robot_command
┌─────────────────────────────▼───────────────────────────────────────────┐
│                      MISSION / BEHAVIOUR LAYER                           │
│  visitor_greeter   apriltag_follower   person_follower   cavecare_mission│
└─────────────────────────────┬───────────────────────────────────────────┘
                              │ /robot_command
┌─────────────────────────────▼───────────────────────────────────────────┐
│                        SAFETY / MOTION LAYER                             │
│              motion_manager   cmd_vel_bridge                             │
└──────────┬─────────────────────────────────────────────────┬────────────┘
           │ /safe_robot_command                             │ /cmd_vel
┌──────────▼──────────┐                          ┌───────────▼────────────┐
│   HARDWARE BRIDGE   │                          │     NAV2 STACK         │
│   serial_bridge     │                          │  pusantara_navigation  │
│   (PRIZM/Arduino)   │                          │  (AMCL+Planner+BT)    │
└──────────┬──────────┘                          └───────────┬────────────┘
           │ /wheel_encoder                                   │
┌──────────▼──────────────────────────────────────────────────────────────┐
│                          SENSING / ESTIMATION LAYER                      │
│  hls_lfcd_lds_driver (/scan)                                             │
│  bno055_driver       (/imu/data)                                         │
│  vectra_odometry     (/odom, TF: odom→base_link)                        │
│  camera_node         (/camera/image_raw)                                 │
│  dht11_gpio          (/dht11/temperature, /dht11/humidity)               │
│  esp32_wifi_receiver (/esp32/sensor_data)                                │
└─────────────────────────────────────────────────────────────────────────┘
                              │ /camera/image_raw
┌─────────────────────────────▼───────────────────────────────────────────┐
│                          PERCEPTION LAYER                                │
│  vectra_yolo      (/yolo/detections  — person CSV string)               │
│  apriltag_detector(/apriltag/detections — JSON)                         │
│  mold_detector    (/mold/status, /mold/mask_image)                      │
└─────────────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────▼───────────────────────────────────────────┐
│                    HUMAN-ROBOT INTERACTION LAYER                         │
│  tts_node        (/tts_text → espeak/pyttsx3 audio output)              │
│  speech_to_text  (mic input → /stt_result)                              │
│  robot_face      (/yolo/detections → Pygame face animation)             │
│  qwen_assistant  (/stt_result → Qwen2 LLM → /tts_text)                 │
└─────────────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────▼───────────────────────────────────────────┐
│                   CULTURAL HERITAGE APPS LAYER                           │
│  (standalone GUIs launched by pusantara_demo_loop / pusantara_admin)     │
│  batik_designer · food_quiz · heritage_explainer · keris_demo           │
│  malaysia_history_quiz · visitor_greeter                                 │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Detailed Node Analysis

### 🔌 HARDWARE DRIVERS

---

#### `hls_lfcd_lds_driver` — LiDAR Driver
| Attribute | Detail |
|---|---|
| **Node** | `hlds_laser_publisher` |
| **Type** | 3rd-party C++ driver |
| **Hardware** | Silicon Labs CP2102 USB-to-UART (LDS-01 LiDAR) |
| **Port** | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` |
| **Publishes** | `/scan` → `sensor_msgs/LaserScan` |
| **Subscribes** | — |
| **TF** | Relies on static TF `base_link → laser` (published in bringup) |
| **Role** | Provides 360° distance scan data used by Nav2 and motion_manager |
| **Issues** | Port path is hardcoded in launch file — fragile if port enumeration changes |

---

#### `bno055_driver` — IMU Driver
| Attribute | Detail |
|---|---|
| **Node** | `imu_node` |
| **Hardware** | BNO055 9-DoF IMU |
| **Publishes** | `/imu/data` → `sensor_msgs/Imu` |
| **Subscribes** | — |
| **Role** | Provides fused orientation quaternion to odometry node. BNO055 handles sensor fusion internally (gyro + accel + mag). |
| **Issues** | No calibration status published to ROS. `vectra_odometry` checks IMU calibration level internally but it can't be monitored externally. |

---

#### `serial_bridge` — PRIZM/Motor Controller Bridge
| Attribute | Detail |
|---|---|
| **Node** | `serial_bridge` |
| **File** | `serial_bridge/bridge_node.py` |
| **Hardware** | FTDI FT232R (PRIZM Arduino-compatible motor controller) |
| **Port** | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A105QM2R-if00-port0` |
| **Subscribes** | `/safe_robot_command` → `std_msgs/String` |
| **Publishes** | `/wheel_encoder` → `std_msgs/Int32MultiArray` |
| **Protocol** | Text-based: **TX** = `M,<right>,<left>\n` / **RX** = `E,<left_ticks>,<right_ticks>` |
| **Features** | Reconnect-on-failure loop, threading lock for serial access, 5s status timer |
| **Role** | Only physical actuator interface — all motor commands must go through here |
| **Issues** | Uses `String` for robot commands (error-prone CSV protocol). No acknowledgement / feedback from PRIZM beyond encoder ticks. Status timer logs every 5s (can be noisy). |

---

#### `camera_node` — Webcam Publisher
| Attribute | Detail |
|---|---|
| **Node** | `webcam_publisher` |
| **File** | `camera_node/camera_node.py` |
| **Hardware** | Logitech C270 HD Webcam (`/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_E01FE110-video-index0`) |
| **Publishes** | `/camera/image_raw` → `sensor_msgs/Image` (640×480, 30 FPS, bgr8) |
| **Subscribes** | — |
| **Role** | Raw camera stream source for YOLO, AprilTag, and Mold Detector |
| **Issues** | Camera path is hardcoded. If camera changes, this breaks. No QoS profile set (default best-effort). 30 FPS on Jetson is fine but could saturate the bus if multiple consumers subscribe at full rate. |

---

#### `dht11_gpio` — DHT11 Temperature/Humidity Sensor
| Attribute | Detail |
|---|---|
| **Node** | `dht11_gpio_node` |
| **File** | `dht11_gpio/dht11_gpio_node.py` |
| **Hardware** | DHT11 on Jetson GPIO physical pin 7 (BOARD mode) |
| **Publishes** | `/dht11/temperature` → `sensor_msgs/Temperature` |
|  | `/dht11/humidity` → `sensor_msgs/RelativeHumidity` |
|  | `/dht11/status` → `std_msgs/String` (SAFE / WARNING / DANGER) |
|  | `/dht11/data` → `std_msgs/String` (JSON blob) |
| **Read Rate** | Every 2 seconds (configurable) |
| **Logic** | Humidity < 70% → SAFE, 70–80% → WARNING, >80% → DANGER |
| **Role** | Environmental monitoring for mold risk and cave-care mission alerting |
| **Issues** | Bit-banging DHT11 in Python is fragile (timing-sensitive). Python `Jetson.GPIO` timing can be inaccurate under CPU load. Duplicate data in both `sensor_msgs` and JSON string — redundant. No subscriber to `/dht11/status` found in any other node. |

---

#### `esp32_wifi_receiver` — ESP32 WiFi→ROS Bridge
| Attribute | Detail |
|---|---|
| **Node** | `esp32_http_receiver` |
| **File** | `esp32_wifi_receiver/esp32_receiver.py` |
| **Transport** | Flask HTTP server on port 5000 (runs in a daemon thread) |
| **Publishes** | `/esp32/sensor_data` → `std_msgs/String` (JSON) |
| **Subscribes** | — |
| **Role** | Receives POST requests from an ESP32 sensor board over WiFi and bridges to ROS |
| **Issues** | **Global mutable `ros_node`** — poor design. Flask server errors don't propagate to ROS. No authentication on the HTTP endpoint. `/esp32/sensor_data` not consumed by any node in the workspace. |

---

### ⚙️ ESTIMATION & ODOMETRY

---

#### `vectra_odometry` — Differential Drive Odometry
| Attribute | Detail |
|---|---|
| **Node** | `odometry_node` |
| **File** | `vectra_odometry/odometry_node.py` |
| **Subscribes** | `/wheel_encoder` → `std_msgs/Int32MultiArray` |
|  | `/imu/data` → `sensor_msgs/Imu` |
| **Publishes** | `/odom` → `nav_msgs/Odometry` |
| **TF Broadcast** | `odom → base_link` (dynamic) |
| **Algorithm** | IMU-fused differential drive: uses **BNO055 quaternion for heading** (no gyro integration drift), **encoder ticks for linear distance**. RK2 heading midpoint, velocity LPF, zero-velocity deadband, wheel-slip detection |
| **Diagnostics** | Watchdog for encoder and IMU staleness, NaN/Inf pose check |
| **Parameters** | `wheel_radius`, `wheel_base`, `ticks_per_revolution`, `odom_frame`, `base_frame`, `publish_rate` |
| **Quality** | ✅ Most mature node in the workspace — well-structured, defensive coding, full diagnostics |
| **Issues** | `transforms3d` dependency not standard in ROS2 — needs explicit system install |

---

### 🛡️ SAFETY & MOTION

---

#### `motion_manager` — Safety / Motion Arbiter
| Attribute | Detail |
|---|---|
| **Node** | `motion_manager` |
| **File** | `motion_manager/motion_manager.py` |
| **Subscribes** | `/robot_command` → `std_msgs/String` (raw motor command from any behaviour node) |
|  | `/scan` → `sensor_msgs/LaserScan` (LiDAR, QoS: sensor_data) |
| **Publishes** | `/safe_robot_command` → `std_msgs/String` |
| **Modes** | `SAFE` (obstacle stop), `ASSIST` (slow near obstacle), `OVERRIDE` (bypass) |
| **Logic** | 10 Hz safety loop. If forward command AND front obstacle < `stop_distance` → override to `M,0,0`. If ASSIST mode and < `warning_distance` → slow to `M,10,10` |
| **Role** | **Critical safety gateway** — the ONLY layer between behaviour commands and actuators |
| **Issues** | Mode is re-read from parameter every loop (minor perf issue). Relies on `moving_forward()` parsing `M,<r>,<l>` format — if command format changes this breaks. Motor command parsing: **parts[1] is `right`, parts[2] is `left`** — but command is assembled as `M,right,left` — verify this matches the PRIZM protocol. |

---

#### `cmd_vel_bridge` — Nav2 cmd_vel → robot_command Bridge
| Attribute | Detail |
|---|---|
| **Package** | `cmd_vel_bridge` |
| **Purpose** | Translates `geometry_msgs/Twist` from Nav2's `/cmd_vel` into the robot's `M,<r>,<l>` string protocol |
| **Subscribes** | `/cmd_vel` → `geometry_msgs/Twist` |
| **Publishes** | `/robot_command` → `std_msgs/String` |
| **Note** | Source file not found during scan (likely missing the main `.py` node file — only `package.xml` exists) |
| **Issues** | ⚠️ **No main node Python file found** — this may be an empty placeholder or the node file is missing. This is a critical gap if Nav2 navigation is to work properly. |

---

### 👁️ PERCEPTION

---

#### `vectra_yolo` — YOLOv8 Person Detector
| Attribute | Detail |
|---|---|
| **Node** | `yolo_detector` |
| **File** | `vectra_yolo/yolo_detector.py` |
| **Model** | YOLOv8n (`yolov8n.pt` at workspace root, 6.5 MB) |
| **Subscribes** | `/camera/image_raw` → `sensor_msgs/Image` |
| **Publishes** | `/yolo/detections` → `std_msgs/String` (CSV: `person,<conf>,<cx>,<cy>,<w>,<h>;...`) |
|  | `/yolo/debug_image` → `sensor_msgs/Image` (if debug=True) |
| **Logic** | Filters for `person` class only. Draws bounding boxes and publishes CSV string |
| **Role** | Core person detection — feeds `visitor_greeter`, `person_follower`, `robot_face` |
| **Issues** | **Custom CSV string protocol** is fragile. No message type for detections (should use custom msg or `vision_msgs`). Debug window uses `cv2.imshow` which requires a display. Model path hardcoded. Only detects persons — no multi-class support. |

---

#### `apriltag_detector` — AprilTag Fiducial Detector
| Attribute | Detail |
|---|---|
| **Node** | `apriltag_detector` |
| **File** | `apriltag_detector/apriltag_detector.py` |
| **Subscribes** | `/camera/image_raw` → `sensor_msgs/Image` |
| **Publishes** | `/apriltag/detections` → `std_msgs/String` (JSON: `{tag_id, normalized_x, area, ...}`) |
| **Logic** | Uses OpenCV `aruco` module. Publishes best tag's normalized position and area. Optional `show_ui` for debug window |
| **Role** | Fiducial navigation aid — used by `apriltag_follower` for guided following |
| **Issues** | JSON over `std_msgs/String` is fragile. No tag family configuration parameter. Single-tag output (first detected). |

---

#### `mold_detector` — Mold/Green Patch Detector
| Attribute | Detail |
|---|---|
| **Node** | `mold_detector_node` (embedded in a `MoldDetectorGUI` Tkinter class) |
| **File** | `mold_detector/mold_detector_node.py` |
| **Subscribes** | `/camera/image_raw` → `sensor_msgs/Image` |
| **Publishes** | `/mold/status` → `std_msgs/String` (SAFE/WARNING/DANGER) |
|  | `/mold/mask_image` → `sensor_msgs/Image` |
| **Logic** | HSV color segmentation for green areas. Morphological cleanup. Patch count and coverage percentage. GUI display with Tkinter |
| **Role** | CaveCare mission — detects biological growth in a cave/enclosed space |
| **Issues** | **ROS node embedded inside Tkinter GUI class** — terrible separation of concerns. `rclpy.spin_once` called from `root.after()` — fragile event loop integration. Tkinter has GIL issues with ROS threading. |

---

### 🧭 NAVIGATION

---

#### `pusantara_navigation` — Nav2 Stack Launch
| Attribute | Detail |
|---|---|
| **Type** | Launch file only (no custom nodes) |
| **Nodes Launched** | `map_server`, `amcl`, `planner_server`, `controller_server`, `bt_navigator`, `behavior_server`, `lifecycle_manager` |
| **Map** | `maps/museum.yaml` |
| **Config** | `config/nav2_params.yaml` |
| **TF Required** | `map → odom → base_link → laser` |
| **Issues** | Map and config files not verified to exist. No `costmap_filters` configured for dynamic obstacles. |

---

#### `pusantara_description` — Robot URDF/Model
| Attribute | Detail |
|---|---|
| **Type** | URDF description package |
| **Launch** | `display.launch.py` (for RViz visualization) |
| **Issues** | No custom nodes. URDF file not directly inspected — TF consistency with `base_link`/`laser` frames not verified. |

---

### 🗣️ HUMAN-ROBOT INTERACTION

---

#### `tts_node` — Text-to-Speech
| Attribute | Detail |
|---|---|
| **Node** | `tts_node` |
| **File** | `tts_node/tts_node.py` |
| **Subscribes** | `/tts_text` → `std_msgs/String` |
| **Publishes** | (none — audio output) |
| **Role** | Receives text strings and speaks them using espeak/pyttsx3 |
| **Issues** | Source file truncated in scan — full implementation not visible. If TTS is blocking, it will block the ROS callback queue. Queue management not visible. |
| **Helper** | `tts_sender.py` — CLI tool for publishing to `/tts_text` manually |

---

#### `speech_to_text` — STT Node
| Attribute | Detail |
|---|---|
| **Node** | `stt_node` |
| **File** | `speech_to_text/stt_node.py` |
| **Publishes** | `/stt_result` → `std_msgs/String` |
| **Subscribes** | — |
| **Role** | Microphone input → Whisper/Vosk/similar → text published to ROS |
| **Issues** | Source file content not captured in scan. Connects to `qwen_assistant` via `/stt_result`. |

---

#### `robot_face` — Animated Face Display
| Attribute | Detail |
|---|---|
| **Node** | `face_node` (embedded in `RobotFace` class) |
| **File** | `robot_face/face_node.py` |
| **Subscribes** | `/yolo/detections` → `std_msgs/String` |
|  | `/tts_text` → `std_msgs/String` |
| **Logic** | Pygame window. Eyes track detected person horizontally. Mouth animates on speech. Blink randomly every 3–6s. Different mouth shape for 0/1/many visitors |
| **Role** | Expressive robot face on secondary display |
| **Issues** | `rclpy.spin(node)` blocks main thread — Pygame update loop must be called differently. Actually: `update()` is likely called from a timer, but `rclpy.spin` is the main loop. **Pygame requires `update()` to be called in the main thread** — this node architecture is fundamentally broken unless using a threading trick. |

---

#### `qwen_assistant` — AI Language Model Node
| Attribute | Detail |
|---|---|
| **Node** | `qwen_node` |
| **File** | `qwen_assistant/qwen_node.py` |
| **Subscribes** | `/stt_result` → `std_msgs/String` |
| **Publishes** | `/tts_text` → `std_msgs/String` |
| **Model** | Qwen2 (local inference, likely via ollama or transformers) |
| **Role** | Conversational AI — visitor asks questions verbally, Qwen answers via TTS |
| **Issues** | Inference could block the callback — needs background threading. Model loading time at startup. |

---

### 🤖 BEHAVIOUR / MISSION

---

#### `visitor_greeter` — Visitor Detection & Greeting
| Attribute | Detail |
|---|---|
| **Node** | `visitor_greeter` |
| **File** | `visitor_greeter/visitor_greeter.py` |
| **Subscribes** | `/yolo/detections` → `std_msgs/String` |
| **Publishes** | `/tts_text` → `std_msgs/String` |
| **Logic** | Detects person presence from YOLO string. State machine: `not_present → present → goodbye`. Greeting cooldown 15s. Engagement messages every 30–60s. Goodbye on 5s timeout |
| **Role** | Passive greeter — welcomes visitors and periodically engages them |
| **Issues** | Uses `time.time()` for all timing (not ROS time). Hardcoded greeting/goodbye strings — should be data-driven. Parsing YOLO string with `"person" in detections.lower()` — very fragile. |

---

#### `person_follower` — Person Following Behaviour
| Attribute | Detail |
|---|---|
| **Node** | `person_follower` |
| **File** | `person_follower/person_follower.py` |
| **Subscribes** | `/yolo/detections` → `std_msgs/String` |
| **Publishes** | `/robot_command` → `std_msgs/String` |
| **Logic** | Finds widest/largest detected person. Width > 250px → stop. CenterX < 260 → turn left. CenterX > 380 → turn right. Otherwise → forward |
| **Role** | Active person-following mode |
| **Issues** | Pixel thresholds (260, 380, 250) are hardcoded — not parameters. Assumes 640px wide frame but no verification. Sends no stop command when no person detected — robot keeps last command indefinitely (dangerous). |

---

#### `apriltag_follower` — AprilTag Following Behaviour
| Attribute | Detail |
|---|---|
| **Node** | `apriltag_follower` |
| **File** | `apriltag_follower/apriltag_follower.py` |
| **Subscribes** | `/apriltag/detections` → `std_msgs/String` (JSON) |
| **Publishes** | `/robot_command` → `std_msgs/String` |
| **Logic** | Proportional control (kP=80) on normalized_x error. Area-based distance control: stop_area=6000, slow_area=3000. Lost timeout → stop. Search mode with alternating rotation |
| **Role** | Guided following — robot follows a person carrying an AprilTag |
| **Features** | Exponential smoothing on X (α=0.3), search mode with oscillating rotation, parameterized |
| **Issues** | JSON parsing from `std_msgs/String` is fragile. Search mode has no timeout — could rotate forever. |

---

#### `cavecare_mission` — Cave/Environmental Mission
| Attribute | Detail |
|---|---|
| **Package** | `cavecare_mission` |
| **Dependencies** | `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav2_msgs`, `cv_bridge` |
| **Main file** | Not found in scan (package exists, dependencies declared, but main node .py not found) |
| **Role** | Presumably coordinates mold detection + DHT11 alerts + navigation for cave inspection |
| **Issues** | ⚠️ **Main node file appears missing** — only package.xml exists with declared deps. Either unimplemented or in an unexpected location. |

---

### 🎭 CULTURAL HERITAGE APPS (Standalone GUIs)

These are **not ROS nodes** — they are standalone Python GUI applications launched as subprocesses by `pusantara_admin` or `pusantara_demo_loop`.

---

#### `batik_designer` — Interactive Batik Art Studio
| Attribute | Detail |
|---|---|
| **Type** | Standalone Tkinter GUI (no ROS) |
| **File** | `batik_designer/batik_studio.py` |
| **Features** | Canvas drawing, symmetry modes (none/horizontal/vertical/four), 6 brush styles, 6 motif stamps (Flower, Lotus, Leaf, Bunga Raya, Awan Larat, Pucuk Rebung), save to `~/batik_designs/` |
| **Issues** | Saves as PostScript (`.ps`) format only — not PNG despite using PIL. Gallery shows filenames only, no thumbnail preview. No ROS integration (cannot receive TTS feedback or notify other nodes when done). |

---

#### `food_quiz` — Malaysian Food Quiz
| Attribute | Detail |
|---|---|
| **Type** | Standalone GUI (Tkinter/PyGame) |
| **File** | Source not fully captured |
| **Role** | Interactive quiz about Malaysian food for museum visitors |
| **Issues** | No ROS integration (isolated subprocess). |

---

#### `heritage_explainer` — Heritage Explainer App
| Attribute | Detail |
|---|---|
| **Type** | Standalone GUI |
| **Subscribes** | `/tts_text` (likely — subscribes based on `depend` on `rclpy` and `std_msgs`) |
| **Issues** | Node file content not fully captured. |

---

#### `keris_demo` — Keris (Traditional Dagger) Demo
| Attribute | Detail |
|---|---|
| **Type** | Standalone GUI |
| **File** | `keris_demo/keris_demo.py` |
| **Role** | Interactive demonstration about the Malaysian Keris artifact |
| **Issues** | No ROS deps in package.xml — fully standalone. |

---

#### `malaysia_history_quiz` — Malaysian History Quiz
| Attribute | Detail |
|---|---|
| **Type** | Standalone GUI |
| **File** | `malaysia_history_quiz/history_quiz.py` |
| **Role** | Quiz about Malaysian history |
| **Issues** | No ROS deps in package.xml — fully standalone. |

---

### 🖥️ ADMIN / CONTROL

---

#### `pusantara_admin` — Operator Admin Console (PyQt5)
| Attribute | Detail |
|---|---|
| **Node** | Embeds `rclpy` node within PyQt5 app |
| **Files** | `admin_console.py` (main), `system_monitor.py`, `node_manager.py`, `camera_widget.py`, `lidar_widget.py` |
| **Features** | Node start/stop, waypoint management (JSON file), Nav2 goal sending, E-STOP, camera live feed, LiDAR visualization, system stats (CPU/RAM/GPU), process table, mission alerts |
| **Subscribes** | `/camera/image_raw`, `/scan`, `/amcl_pose`, Nav2 action feedback |
| **Publishes** | `/tts_text`, `/robot_command`, Nav2 goals, `/initialpose` |
| **Target Display** | 7" kiosk touch display (1024×600) |
| **Theme** | Malaysian Cultural Heritage — warm wood & gold (PyQt5 stylesheet) |
| **Issues** | `system_monitor.py` uses `psutil` for stats — Jetson GPU stats may not work correctly. Waypoint format in `pusantara_admin` and `pusantara_demo_loop` may be **incompatible** (different JSON schemas). ROS spin runs in a `QTimer` — OK design but must not block. |

---

#### `pusantara_demo_loop` — Demo Loop GUI (Tkinter)
| Attribute | Detail |
|---|---|
| **Node** | Embeds `rclpy` in Tkinter event loop |
| **File** | `pusantara_demo_loop/demo_loop.py` |
| **Features** | Destination cards, waypoint editor (x/y/yaw/node_id/panel_id/tag_id/apps/app_mode), app launcher, Nav2 action client, state machine (READY→NAVIGATING→ARRIVED→APP_SELECTION→APP_RUNNING) |
| **States** | `READY, DESTINATION_SELECTED, NAVIGATING, ARRIVED, APP_SELECTION, APP_RUNNING, ESTOP` |
| **App Registry** | `APP_REGISTRY` dict maps app names to `{cmd, cwd}` — runs heritage apps as subprocesses |
| **Waypoint Schema** | `{name, description, x, y, yaw, apps[], app_mode, id}` |
| **Issues** | Waypoint schema is **different** from `pusantara_admin` schema (`node_id`, `panel_id`, `tag_id` vs `name`, `apps`, `app_mode`). Both write to different JSON files. `ManagedProcess` uses `subprocess.Popen` — no stdout capture. Shutdown watchdog (`_force_exit`) calls `os._exit(0)` — bypasses cleanup. |

---

#### `pusantara_bringup` — System Bringup Launch
| Attribute | Detail |
|---|---|
| **Type** | Launch file only |
| **Launches** | `hls_lfcd_lds_driver`, `bno055_driver`, `motion_manager`, `serial_bridge`, `vectra_odometry` (via include), static TF `base_link→laser` |
| **Issues** | Does NOT launch camera, YOLO, or any HRI nodes — those are launched separately. `camera_node` not in bringup — easy to forget. |

---

## 4. Topic / Message Flow Map

```
SENSORS ──────────────────────────────────────────────────────────────
  /scan              [LaserScan]  → motion_manager, Nav2
  /imu/data          [Imu]        → vectra_odometry
  /wheel_encoder     [Int32MA]    → vectra_odometry
  /camera/image_raw  [Image]      → vectra_yolo, apriltag_detector, mold_detector
  /dht11/temperature [Temp]       → (no subscriber found)
  /dht11/humidity    [RelHum]     → (no subscriber found)
  /dht11/status      [String]     → (no subscriber found)
  /esp32/sensor_data [String]     → (no subscriber found)

ESTIMATION ───────────────────────────────────────────────────────────
  /odom              [Odometry]   → Nav2 (amcl, costmap)
  TF: odom→base_link              → Nav2, RViz

PERCEPTION ───────────────────────────────────────────────────────────
  /yolo/detections   [String]     → visitor_greeter, person_follower, robot_face
  /apriltag/detections [String]   → apriltag_follower
  /mold/status       [String]     → (no subscriber found)
  /amcl_pose         [PoseWithCov]→ pusantara_admin

MOTION PIPELINE ──────────────────────────────────────────────────────
  /robot_command     [String]     → motion_manager          (from behaviours)
  /safe_robot_command[String]     → serial_bridge           (from motion_manager)
  /cmd_vel           [Twist]      → cmd_vel_bridge           (from Nav2)
  (cmd_vel_bridge)   [String]     → /robot_command          (bridge output)

HRI ──────────────────────────────────────────────────────────────────
  /tts_text          [String]     → tts_node
  /stt_result        [String]     → qwen_assistant
  (Microphone)                   → speech_to_text → /stt_result

NAVIGATION ───────────────────────────────────────────────────────────
  Nav2 Action: NavigateToPose    ← pusantara_admin, pusantara_demo_loop
  /initialpose       [PoseWithCov]← pusantara_admin
```

---

## 5. Issues Found

### 🔴 Critical Issues

| # | Issue | Location | Impact |
|---|---|---|---|
| C1 | **`cmd_vel_bridge` node file missing** — no Python source found, only `package.xml` | `cmd_vel_bridge/` | Nav2 cannot drive the robot (Nav2 → cmd_vel not connected to motors) |
| C2 | **`cavecare_mission` node file missing** — package declared but no main `.py` | `cavecare_mission/` | CaveCare mission unimplemented or broken |
| C3 | **`person_follower` sends no stop when no person detected** — robot keeps last speed command indefinitely | `person_follower.py` | Safety risk — robot could drive into obstacle if person disappears |
| C4 | **`robot_face` node uses `rclpy.spin()` in main thread with Pygame** — Pygame requires `pygame.display.update()` in main thread; `rclpy.spin` blocks | `face_node.py` | Pygame window may freeze / not render |
| C5 | **`mold_detector` runs ROS node inside Tkinter GUI class** with `spin_once` in `after()` — GIL + threading issues | `mold_detector_node.py` | Potential deadlocks, missed callbacks |

### 🟡 Significant Issues

| # | Issue | Location |
|---|---|---|
| S1 | **Dual waypoint JSON schemas** — `pusantara_admin` and `pusantara_demo_loop` use incompatible schemas | Both GUIs |
| S2 | **All sensor/environmental topics have no subscribers** — `/dht11/*`, `/esp32/sensor_data`, `/mold/status` are published but never consumed | Multiple |
| S3 | **Custom CSV/JSON string protocols** for detections — fragile, no type safety | `vectra_yolo`, `apriltag_detector` |
| S4 | **Hardcoded device paths** — LiDAR, camera, serial ports all hardcoded in launch/node files | bringup, camera_node |
| S5 | **`esp32_wifi_receiver` uses global mutable `ros_node`** — anti-pattern | `esp32_receiver.py` |
| S6 | **Motion command parsing fragility** — if command format `M,r,l` changes, safety layer breaks | `motion_manager.py` |
| S7 | **`pusantara_demo_loop` uses `os._exit(0)` as shutdown watchdog** | `demo_loop.py` |

### 🟠 Architecture Smells

| # | Issue | Location |
|---|---|---|
| A1 | **Two separate GUI controllers** (`pusantara_admin` + `pusantara_demo_loop`) with overlapping responsibilities | Both packages |
| A2 | **30 packages for a single robot** — many are thin wrappers; excessive fragmentation | Workspace |
| A3 | **`std_msgs/String` overused** — used as a transport for structured data (CSV, JSON) | System-wide |
| A4 | **`package.xml` missing ROS deps in many packages** — `food_quiz`, `keris_demo`, `batik_designer`, `camera_node` have no ROS dependencies declared | Multiple |
| A5 | **Heritage apps have no ROS integration** — cannot receive signals from robot state machine | All cultural apps |
| A6 | **No custom message types** despite structured data (detections, commands) | System-wide |
| A7 | **DHT11 Python bit-bang** unreliable under CPU load | `dht11_gpio` |

---

## 6. Architecture Cleanup Recommendations

### 🚀 Priority 1 — Fix Critical Bugs

#### 1.1 Implement `cmd_vel_bridge`
```python
# cmd_vel_bridge/cmd_vel_bridge_node.py
class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter('max_speed', 50)
        self.declare_parameter('wheel_base', 0.25)
        
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(String, '/robot_command', 10)
    
    def cb(self, msg: Twist):
        v = msg.linear.x   # m/s
        w = msg.angular.z  # rad/s
        # Differential drive conversion:
        # v_r = v + w*L/2,  v_l = v - w*L/2
        # Map m/s to motor speed units
        ...
```

#### 1.2 Fix `person_follower` — send stop when no person
```python
def detection_callback(self, msg):
    ...
    if largest_person is None:
        self.send_command("M,0,0")  # ADD THIS
        return
```

#### 1.3 Fix `robot_face` — use timer instead of rclpy.spin
```python
def main():
    rclpy.init()
    node = RobotFace()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok():
        executor.spin_once(timeout_sec=0)
        node.update()          # pygame update in main thread
        clock.tick(30)
```

---

### 🏗️ Priority 2 — Create Custom Message Types

Replace all CSV/JSON-over-String protocols with typed messages:

```
# pusantara_msgs/msg/Detection.msg
string label
float32 confidence
int32 center_x
int32 center_y
int32 width
int32 height

# pusantara_msgs/msg/DetectionArray.msg
Detection[] detections
std_msgs/Header header

# pusantara_msgs/msg/RobotCommand.msg  (or use existing geometry_msgs)
int32 right_speed   # -100 to 100
int32 left_speed    # -100 to 100
string command_type # "MOTOR", "STOP", "SPECIAL"
```

**Benefits:** Type safety, tab-completion, rosbag introspection, no parsing bugs.

---

### 🔗 Priority 3 — Unify the Two GUI Controllers

Currently you have **two overlapping GUIs**:
- `pusantara_admin` (PyQt5) — operator console, node manager, waypoints
- `pusantara_demo_loop` (Tkinter) — demo state machine, app launcher

**Recommendation:** Merge into one, or clearly separate responsibilities:

```
Option A: Single PyQt5 console (pusantara_admin)
  → Add a "Demo" tab with destination cards and app launcher
  → Remove pusantara_demo_loop entirely

Option B: Clear separation
  pusantara_admin    = operator tool (start/stop nodes, waypoint DB, e-stop)
  pusantara_demo_loop = kiosk-facing GUI (visitors don't see admin controls)
  → Unified waypoint JSON schema shared via a config package
```

---

### 📦 Priority 4 — Consolidate Packages

Merge thin single-file packages into logical groups:

```
BEFORE (30 packages):                    AFTER (suggested ~15 packages):
  apriltag_detector  ─┐                   pusantara_vision
  apriltag_follower  ─┤ → merge →           (yolo, apriltag detector, mold)
  vectra_yolo        ─┘
  
  visitor_greeter    ─┐                   pusantara_behaviours
  person_follower    ─┤ → merge →           (greeter, person_follower, 
  apriltag_follower  ─┘                      apriltag_follower, cavecare)

  tts_node           ─┐                   pusantara_hri
  speech_to_text     ─┤ → merge →           (tts, stt, robot_face, qwen)
  robot_face         ─┤
  qwen_assistant     ─┘

  batik_designer     ─┐                   pusantara_apps
  food_quiz          ─┤ → merge →           (all cultural apps in one pkg)
  heritage_explainer ─┤
  keris_demo         ─┘
  malaysia_history_quiz

  serial_bridge      ─┐                   pusantara_hardware
  camera_node        ─┤ → merge →           (serial, camera, dht11, esp32)
  dht11_gpio         ─┤
  esp32_wifi_receiver─┘
```

---

### 🔧 Priority 5 — Standardize Device Configuration

Move all hardcoded paths to a shared YAML config:

```yaml
# config/hardware.yaml
hardware:
  lidar_port: "/dev/serial/by-id/usb-Silicon_Labs_CP2102..."
  motor_port: "/dev/serial/by-id/usb-FTDI_FT232R..."
  camera_id:  "/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM..."
  dht11_pin:  7
  baudrate:   115200
```

Load in all nodes via `declare_parameter`.

---

### 📡 Priority 6 — Connect Dangling Topics

Currently these topics are published but have no subscribers:

| Topic | Published By | Suggested Consumer |
|---|---|---|
| `/dht11/temperature` | `dht11_gpio` | `cavecare_mission` or admin dashboard |
| `/dht11/humidity` | `dht11_gpio` | `cavecare_mission` (alerts on high humidity) |
| `/dht11/status` | `dht11_gpio` | `cavecare_mission`, admin console display |
| `/esp32/sensor_data` | `esp32_wifi_receiver` | `cavecare_mission` or admin console |
| `/mold/status` | `mold_detector` | `cavecare_mission`, admin console, TTS alert |

**Quick fix** — add a simple alert bridge in `cavecare_mission`:
```python
# Subscribe to /dht11/status and /mold/status
# If DANGER: publish to /tts_text + log to admin
```

---

### 🛡️ Priority 7 — Strengthen the Motion Safety Layer

#### 7.1 Define a command interface contract
```python
# Standardize: M,<right>,<left> where right/left ∈ [-100, 100]
# Document motor direction convention explicitly
```

#### 7.2 Add command watchdog in serial_bridge
```python
# If no command received for > 0.5s → auto-send M,0,0
# Prevents robot driving if motion_manager crashes
```

#### 7.3 Add mode service to motion_manager
```python
# Replace parameter polling with a proper ROS service:
# /set_motion_mode (request: mode: string)
# Modes: SAFE, ASSIST, OVERRIDE
```

---

### ✅ Summary — Recommended Action Order

| Priority | Action | Effort | Impact |
|---|---|---|---|
| 🔴 P1 | Implement `cmd_vel_bridge` node | Low | Critical — enables Nav2 |
| 🔴 P1 | Fix person_follower stop-on-no-detection | Trivial | Safety |
| 🔴 P1 | Fix robot_face threading | Low | Correctness |
| 🟡 P2 | Create `pusantara_msgs` custom message package | Medium | Robustness |
| 🟡 P2 | Implement `cavecare_mission` node | Medium | Feature complete |
| 🟡 P2 | Connect DHT11/mold topics to consumers | Low | Feature complete |
| 🟠 P3 | Unify waypoint JSON schema between GUIs | Low | Consistency |
| 🟠 P3 | Add command watchdog in serial_bridge | Low | Safety |
| 🔵 P4 | Consolidate packages into logical groups | High | Maintainability |
| 🔵 P4 | Externalize all device paths to config YAML | Medium | Portability |
| 🔵 P5 | Merge/clarify dual GUI strategy | High | Long-term clarity |

---

*Report generated by architecture analysis of all 30 ROS2 packages in `/home/orin_nano/vectra_ws/src`.*
