#!/usr/bin/env python3

import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
import yaml

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields


def path_value(obj, path):
    """Read a nested dictionary/list field using dot notation."""
    cur = obj

    for part in path.split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        elif isinstance(cur, (list, tuple)):
            try:
                cur = cur[int(part)]
            except (ValueError, IndexError):
                return False, None
        else:
            return False, None

    return True, cur


def matches(actual, expected):
    """Evaluate a value against a condition."""
    if not isinstance(expected, dict):
        return actual == expected

    operators = {
        "gt": lambda a, b: a > b,
        "gte": lambda a, b: a >= b,
        "lt": lambda a, b: a < b,
        "lte": lambda a, b: a <= b,
        "eq": lambda a, b: a == b,
        "ne": lambda a, b: a != b,
        "contains": lambda a, b: b in a,
        "in": lambda a, b: a in b,
        "exists": lambda a, b: bool(a) == bool(b),
    }

    try:
        for operator, value in expected.items():
            if operator not in operators:
                return False

            if not operators[operator](actual, value):
                return False

        return True

    except (TypeError, ValueError):
        return False


class RuleEngine(Node):

    def __init__(self):
        super().__init__("pusantara_rule_engine")

        # Configuration parameter
        self.declare_parameter("config", "")
        config_path = self.get_parameter("config").value

        if not config_path:
            raise RuntimeError(
                "Set parameter config:=/absolute/path/rules.yaml"
            )

        # Load YAML
        with Path(config_path).expanduser().open(encoding="utf-8") as file:
            document = yaml.safe_load(file) or {}

        self.settings = document.get("settings", {})
        self.rules = document.get("rules", [])

        # Default allowed publish topics include cmd_vel and tts_text
        default_allowed = ["/cmd_vel", "/tts_text", "/robot_command"]
        configured_allowed = self.settings.get("allowed_publish_topics", ["/cmd_vel"])
        self.allowed_topics = set(configured_allowed) | set(default_allowed)

        self.subscribe_all = bool(
            self.settings.get("subscribe_all", False)
        )

        self.excluded_types = set(
            self.settings.get(
                "exclude_message_types",
                [
                    "sensor_msgs/msg/Image",
                    "sensor_msgs/msg/CompressedImage",
                    "sensor_msgs/msg/PointCloud2",
                ],
            )
        )

        # Internal state
        self.topic_subscriptions = {}
        self.topic_publishers = {}
        self.known_topics = set()

        self.previous = {rule["name"]: False for rule in self.rules}
        self.last_trigger = {rule["name"]: 0.0 for rule in self.rules}

        self.execution_lock = threading.Lock()

        # Managed subprocesses (nodes/launch files started by rules)
        # Structure: {proc_id: {"process": subprocess.Popen, "log_file": file_handle}}
        self.managed_processes = {}
        self.process_lock = threading.Lock()
        self.process_log_dir = Path.home() / ".ros" / "rule_engine_nodes"
        self.process_log_dir.mkdir(parents=True, exist_ok=True)

        # Topic discovery in background thread
        self.discovery_period = max(
            0.5, float(self.settings.get("discovery_period", 2.0))
        )
        self.discovery_stop = threading.Event()

        # Discover initial topics immediately
        self.discover_topics()

        # Background discovery thread
        self.discovery_thread = threading.Thread(
            target=self.discovery_loop, daemon=True
        )
        self.discovery_thread.start()

        self.get_logger().info(
            f"Rule engine loaded {len(self.rules)} rules. Allowed publish: {list(self.allowed_topics)}"
        )

    # --------------------------------------------------
    # TOPIC DISCOVERY
    # --------------------------------------------------

    def discovery_loop(self):
        while not self.discovery_stop.wait(self.discovery_period):
            try:
                self.discover_topics()
            except Exception as error:
                self.get_logger().error(f"Topic discovery error: {error}")

    def discover_topics(self):
        for topic, types in self.get_topic_names_and_types():
            if not types:
                continue

            message_type = types[0]

            if topic not in self.known_topics:
                self.known_topics.add(topic)
                self.get_logger().info(f"Discovered {topic} [{message_type}]")

            # Subscribe to topics used by rules
            wanted = any(rule.get("topic") == topic for rule in self.rules)

            if self.subscribe_all and message_type not in self.excluded_types:
                wanted = True

            if not wanted:
                continue

            if topic in self.topic_subscriptions:
                continue

            try:
                message_class = get_message(message_type)

                subscription = self.create_subscription(
                    message_class,
                    topic,
                    lambda msg, t=topic: self.on_message(t, msg),
                    QoSProfile(depth=10),
                )

                self.topic_subscriptions[topic] = subscription
                self.get_logger().info(f"Subscribed to {topic} [{message_type}]")

            except Exception as error:
                self.get_logger().warning(f"Cannot subscribe to {topic}: {error}")

    # --------------------------------------------------
    # MESSAGE CONVERSION
    # --------------------------------------------------

    def message_context(self, message):
        data = dict(message_to_ordereddict(message))

        # Automatically decode JSON inside std_msgs/String
        if set(data) == {"data"} and isinstance(data["data"], str):
            try:
                data["data"] = json.loads(data["data"])
            except json.JSONDecodeError:
                pass

        # String or basic messages: data.xyz
        if set(data) == {"data"}:
            return {"data": data["data"]}

        return {"data": data}

    # --------------------------------------------------
    # RULE EVALUATION
    # --------------------------------------------------

    def on_message(self, topic, message):
        context = self.message_context(message)
        now = time.monotonic()

        for rule in self.rules:
            if rule.get("topic") != topic:
                continue

            condition_matched = True

            for field, expected in rule.get("if", {}).items():
                exists, actual = path_value(context, field)

                if not exists or not matches(actual, expected):
                    condition_matched = False
                    break

            name = rule["name"]

            previous_state = self.previous[name]
            self.previous[name] = condition_matched

            trigger_mode = rule.get("trigger", "rising_edge")

            should_trigger = condition_matched and (
                trigger_mode == "continuous" or not previous_state
            )

            cooldown = float(
                rule.get("cooldown", self.settings.get("cooldown", 2.0))
            )

            if not should_trigger:
                continue

            if now - self.last_trigger[name] < cooldown:
                continue

            self.last_trigger[name] = now

            # Only one rule sequence executes at a time
            if self.execution_lock.acquire(blocking=False):
                worker = threading.Thread(
                    target=self.run_rule, args=(rule,), daemon=True
                )
                worker.start()
            else:
                self.get_logger().warning(
                    f"Skipped {name}: another rule action sequence is executing"
                )

    # --------------------------------------------------
    # GENERIC ROS PUBLISHER
    # --------------------------------------------------

    def publisher_for(self, topic, message_type):
        if topic not in self.allowed_topics:
            raise RuntimeError(f"Output topic {topic} is not allow-listed")

        key = (topic, message_type)

        if key not in self.topic_publishers:
            message_class = get_message(message_type)
            publisher = self.create_publisher(message_class, topic, 10)
            self.topic_publishers[key] = (publisher, message_class)

        return self.topic_publishers[key]

    def publish_message(self, topic, message_type, values):
        publisher, message_class = self.publisher_for(topic, message_type)
        message = message_class()

        if values:
            set_message_fields(message, values)

        publisher.publish(message)

    # --------------------------------------------------
    # NODE & PROCESS LIFECYCLE MANAGEMENT
    # --------------------------------------------------

    def start_node(self, proc_id, package, executable, args=None):
        """Start a ROS 2 node via 'ros2 run' as a managed background process."""
        with self.process_lock:
            if proc_id in self.managed_processes:
                existing = self.managed_processes[proc_id]
                proc = existing.get("process")
                if proc and proc.poll() is None:
                    self.get_logger().info(
                        f"Node '{proc_id}' is already running (PID {proc.pid})"
                    )
                    return
                # Clean up finished handle
                if existing.get("log_file"):
                    existing["log_file"].close()
                del self.managed_processes[proc_id]

            cmd = ["ros2", "run", package, executable]
            if args:
                if isinstance(args, list):
                    cmd.extend([str(a) for a in args])
                else:
                    cmd.extend(str(args).split())

            log_path = self.process_log_dir / f"{proc_id}.log"
            log_file = open(log_path, "a", encoding="utf-8")
            log_file.write(
                f"\n=== [{time.strftime('%Y-%m-%d %H:%M:%S')}] Launching: {' '.join(cmd)} ===\n"
            )
            log_file.flush()

            self.get_logger().info(f"Starting node '{proc_id}': {' '.join(cmd)}")
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,  # Isolate process group for clean signal handling
                    env=os.environ.copy(),
                )
                self.managed_processes[proc_id] = {
                    "process": proc,
                    "log_file": log_file,
                    "cmd": cmd,
                }
                self.get_logger().info(
                    f"Node '{proc_id}' started with PID {proc.pid} (logging to {log_path})"
                )
            except Exception as e:
                log_file.close()
                self.get_logger().error(f"Failed to start node '{proc_id}': {e}")

    def start_launch(self, proc_id, package, launch_file, args=None):
        """Start a ROS 2 launch file via 'ros2 launch' as a managed background process."""
        with self.process_lock:
            if proc_id in self.managed_processes:
                existing = self.managed_processes[proc_id]
                proc = existing.get("process")
                if proc and proc.poll() is None:
                    self.get_logger().info(
                        f"Launch '{proc_id}' is already running (PID {proc.pid})"
                    )
                    return
                if existing.get("log_file"):
                    existing["log_file"].close()
                del self.managed_processes[proc_id]

            cmd = ["ros2", "launch", package, launch_file]
            if args:
                if isinstance(args, list):
                    cmd.extend([str(a) for a in args])
                else:
                    cmd.extend(str(args).split())

            log_path = self.process_log_dir / f"{proc_id}.log"
            log_file = open(log_path, "a", encoding="utf-8")
            log_file.write(
                f"\n=== [{time.strftime('%Y-%m-%d %H:%M:%S')}] Launching: {' '.join(cmd)} ===\n"
            )
            log_file.flush()

            self.get_logger().info(f"Starting launch '{proc_id}': {' '.join(cmd)}")
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=os.environ.copy(),
                )
                self.managed_processes[proc_id] = {
                    "process": proc,
                    "log_file": log_file,
                    "cmd": cmd,
                }
                self.get_logger().info(
                    f"Launch '{proc_id}' started with PID {proc.pid} (logging to {log_path})"
                )
            except Exception as e:
                log_file.close()
                self.get_logger().error(f"Failed to start launch '{proc_id}': {e}")

    def stop_node(self, proc_id):
        """Gracefully terminate a managed background process group."""
        with self.process_lock:
            if proc_id not in self.managed_processes:
                self.get_logger().warning(
                    f"Cannot stop '{proc_id}': not currently tracked"
                )
                return

            item = self.managed_processes[proc_id]
            proc = item.get("process")
            log_file = item.get("log_file")

            if not proc or proc.poll() is not None:
                self.get_logger().info(
                    f"Node '{proc_id}' has already stopped (exit code {proc.returncode if proc else 'N/A'})"
                )
                if log_file:
                    log_file.close()
                del self.managed_processes[proc_id]
                return

            self.get_logger().info(f"Stopping node '{proc_id}' (PID {proc.pid})...")

            try:
                # Send SIGINT to process group so ROS 2 nodes shut down cleanly
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                try:
                    proc.wait(timeout=3.0)
                    self.get_logger().info(f"Node '{proc_id}' exited cleanly")
                except subprocess.TimeoutExpired:
                    self.get_logger().warning(
                        f"Node '{proc_id}' did not exit after SIGINT, sending SIGTERM..."
                    )
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    try:
                        proc.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        self.get_logger().error(
                            f"Node '{proc_id}' did not exit after SIGTERM, sending SIGKILL..."
                        )
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                        proc.wait(timeout=1.0)
            except ProcessLookupError:
                pass
            except Exception as e:
                self.get_logger().error(f"Error stopping node '{proc_id}': {e}")
            finally:
                if log_file:
                    try:
                        log_file.close()
                    except Exception:
                        pass
                self.managed_processes.pop(proc_id, None)

    def speak(self, text, topic="/tts_text"):
        """Publish text to speech topic for tts_node."""
        self.get_logger().info(f"SPEAK: \"{text}\" -> {topic}")
        self.publish_message(
            topic,
            "std_msgs/msg/String",
            {"data": str(text)},
        )

    # --------------------------------------------------
    # RULE EXECUTION
    # --------------------------------------------------

    def run_rule(self, rule):
        name = rule["name"]
        self.get_logger().info(f"TRIGGERED: {name}")

        try:
            for step in rule.get("then", []):
                step_type = step.get("type")

                # Action: Publish
                if step_type == "publish":
                    topic = step["topic"]
                    message_type = step["message_type"]
                    values = step.get("message", {})
                    duration = float(step.get("duration", 0.0))

                    if duration <= 0:
                        self.publish_message(topic, message_type, values)
                        continue

                    # Repeated publish for timed commands
                    rate = max(
                        1.0, float(self.settings.get("command_rate_hz", 10.0))
                    )
                    period = 1.0 / rate
                    end_time = time.monotonic() + duration

                    while time.monotonic() < end_time:
                        self.publish_message(topic, message_type, values)
                        remaining = end_time - time.monotonic()
                        time.sleep(min(period, max(0.0, remaining)))

                    # Auto stop for Twist
                    if (
                        message_type == "geometry_msgs/msg/Twist"
                        and step.get("auto_stop", True)
                    ):
                        self.publish_message(topic, message_type, {})

                # Action: Start Node
                elif step_type in ("start_node", "launch_node"):
                    proc_id = step.get("id") or step.get("executable")
                    package = step["package"]
                    executable = step["executable"]
                    args = step.get("args", [])
                    self.start_node(proc_id, package, executable, args)

                # Action: Stop Node
                elif step_type in ("stop_node", "kill_node"):
                    proc_id = step.get("id") or step.get("executable")
                    self.stop_node(proc_id)

                # Action: Start Launch File
                elif step_type == "start_launch":
                    proc_id = step.get("id") or step.get("launch_file")
                    package = step["package"]
                    launch_file = step["launch_file"]
                    args = step.get("args", [])
                    self.start_launch(proc_id, package, launch_file, args)

                # Action: Speak / TTS
                elif step_type in ("speak", "tts"):
                    text = step.get("text", "")
                    topic = step.get("topic", "/tts_text")
                    self.speak(text, topic)

                # Action: Delay / Pause
                elif step_type in ("delay", "sleep"):
                    seconds = float(step.get("seconds", 1.0))
                    time.sleep(seconds)

                else:
                    raise RuntimeError(f"Unsupported action type: {step_type}")

            self.get_logger().info(f"COMPLETED: {name}")

        except Exception as error:
            self.get_logger().error(f"Rule {name} failed: {error}")

        finally:
            self.execution_lock.release()

    # --------------------------------------------------
    # CLEAN SHUTDOWN
    # --------------------------------------------------

    def destroy_node(self):
        self.discovery_stop.set()

        if hasattr(self, "discovery_thread"):
            self.discovery_thread.join(timeout=2.0)

        # Cleanly stop any nodes launched by the rule engine
        with self.process_lock:
            for proc_id in list(self.managed_processes.keys()):
                self.stop_node(proc_id)

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = RuleEngine()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        import traceback

        print(f"Rule engine error: {error}")
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
