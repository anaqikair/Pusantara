# Pusantara Rule Engine — ROS 2 Humble

Event-driven, YAML-configurable reactive automation node for ROS 2. Bridges perception, speech, and motion triggers with dynamic node lifecycle management.

## Features
- **Dynamic Topic Discovery**: Detects topics appearing at runtime without restarting.
- **Selective Subscription**: Subscribes only to topics needed by active rules.
- **JSON Unpacking**: Deserializes JSON payloads carried in `std_msgs/msg/String`.
- **Condition Matching**: Supports `gt`, `gte`, `lt`, `lte`, `eq`, `ne`, `contains`, `in`, `exists`.
- **Trigger Modes**: `rising_edge` (fires false -> true transitions) and `continuous` (throttled by cooldown).
- **Node & Launch Lifecycle Management**:
  - `start_node`: Dynamically runs a ROS 2 node (`ros2 run`) in the background. Logs to `~/.ros/rule_engine_nodes/<id>.log`.
  - `stop_node`: Gracefully terminates a running node group (`SIGINT` -> `SIGTERM`).
  - `start_launch`: Dynamically starts a launch file (`ros2 launch`).
- **Text-to-Speech (TTS)**: Built-in `speak` action targeting `tts_node` (`/tts_text`).
- **Motion Safety**: Allowed topic filtering, timed Twist publishing, auto-stop on expiration, and automatic cleanup of child nodes on exit.

---

## Action Types in `then:`

### 1. `start_node`
Starts a ROS 2 executable in the background.
```yaml
- type: start_node
  id: apriltag_follower          # Identifier used to reference or stop it later
  package: apriltag_follower     # ROS 2 package
  executable: apriltag_follower  # Executable name
  # args: ["--ros-args", "-p", "target_x:=0.5"] # Optional
```

### 2. `stop_node`
Stops a previously started node gracefully.
```yaml
- type: stop_node
  id: apriltag_follower
```

### 3. `speak` (TTS)
Speaks text using `tts_node` over `/tts_text`.
```yaml
- type: speak
  text: "Hello! I am following AprilTag now."
```

### 4. `publish`
Publishes a ROS 2 message to an allow-listed topic.
```yaml
- type: publish
  topic: /cmd_vel
  message_type: geometry_msgs/msg/Twist
  message:
    linear:
      x: 0.15
  duration: 2.0
  auto_stop: true
```

### 5. `delay`
Pauses execution before the next step.
```yaml
- type: delay
  seconds: 1.5
```

---

## Build & Run

```bash
cd ~/vectra_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pusantara_rule_engine
source install/setup.bash

ros2 run pusantara_rule_engine engine --ros-args \
  -p config:=/home/orin_nano/vectra_ws/src/pusantara_rule_engine/config/rules.yaml
```
