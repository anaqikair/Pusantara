#!/usr/bin/env python3

import json
import math
import os
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


WAYPOINT_FILE = os.path.expanduser('~/vectra_data/cavecare_waypoints.json')


class MissionState(Enum):
    IDLE = 0
    NAVIGATING_TO_PANEL = 1
    VERIFYING_TAG = 2
    SCANNING_MOLD = 3
    REPORTING = 4
    RETURNING_HOME = 5
    ERROR = 6


class CaveMissionManager(Node):
    def __init__(self):
        super().__init__('cave_mission_manager')

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers
        self.cave_node_sub = self.create_subscription(
            String,
            '/cave_node/status',
            self.cave_node_callback,
            10
        )

        self.mold_sub = self.create_subscription(
            String,
            '/mold_detector/status',
            self.mold_callback,
            10
        )

        self.apriltag_sub = self.create_subscription(
            String,
            '/apriltag/detections',
            self.apriltag_callback,
            10
        )

        # Publishers
        self.tts_pub = self.create_publisher(String, '/tts_text', 10)
        self.report_pub = self.create_publisher(String, '/inspection_report', 10)
        self.robot_cmd_pub = self.create_publisher(String, '/robot_command', 10)
        self.safe_cmd_pub = self.create_publisher(String, '/safe_robot_command', 10)

        # Mission variables
        self.state = MissionState.IDLE
        self.current_target_name = None
        self.current_waypoint = None
        self.current_cave_data = None

        self.last_seen_tag_id = None
        self.last_seen_tag_time = 0.0

        self.latest_mold_data = None
        self.mold_detected = False
        self.max_green_percent = 0.0

        self.scan_start_time = None
        self.scan_duration = 6.0

        self.active_goal_handle = None

        self.waypoints = self.load_waypoints()

        # Main loop
        self.timer = self.create_timer(0.5, self.state_loop)

        self.get_logger().info('PUSANTARA CaveCare Mission Manager started.')
        self.get_logger().info(f'Waypoint file: {WAYPOINT_FILE}')
        self.speak('CaveCare mission manager is ready.')

    # ============================================================
    # LOAD WAYPOINTS
    # ============================================================

    def load_waypoints(self):
        os.makedirs(os.path.dirname(WAYPOINT_FILE), exist_ok=True)

        if not os.path.exists(WAYPOINT_FILE):
            default_waypoints = {
                "home": {
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "node_id": "home",
                    "panel_id": "home",
                    "tag_id": 0
                },
                "node_1": {
                    "x": 1.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "node_id": "node_1",
                    "panel_id": "panel_a",
                    "tag_id": 1
                }
            }

            with open(WAYPOINT_FILE, 'w') as f:
                json.dump(default_waypoints, f, indent=2)

            self.get_logger().warn('Waypoint file not found. Created default waypoint file.')
            return default_waypoints

        try:
            with open(WAYPOINT_FILE, 'r') as f:
                data = json.load(f)

            self.get_logger().info(f'Loaded {len(data)} waypoints.')
            return data

        except Exception as e:
            self.get_logger().error(f'Failed to load waypoint file: {e}')
            return {}

    # ============================================================
    # CALLBACKS
    # ============================================================

    def cave_node_callback(self, msg):
        """
        Expected /cave_node/status format:

        {
            "node_id": "node_1",
            "panel_id": "panel_a",
            "tag_id": 1,
            "temperature": 28.5,
            "humidity": 82.0,
            "status": "DANGER"
        }
        """

        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warn(f'Invalid cave node JSON: {msg.data}')
            return

        node_id = str(data.get('node_id', '')).strip()
        humidity = float(data.get('humidity', 0.0))
        status = str(data.get('status', 'SAFE')).upper()

        self.get_logger().info(
            f'Cave node update: node={node_id}, humidity={humidity:.1f}, status={status}'
        )

        if self.state != MissionState.IDLE:
            self.get_logger().info('Mission already running. New cave alert ignored.')
            return

        if not node_id:
            self.get_logger().warn('Cave node data missing node_id.')
            return

        if humidity >= 70.0 or status in ['WARNING', 'DANGER']:
            self.start_mission(data)
        else:
            self.get_logger().info('Humidity is safe. No mission started.')

    def mold_callback(self, msg):
        """
        Expected /mold_detector/status format:

        {
            "mold_detected": true,
            "green_percent": 4.2,
            "risk": "WARNING"
        }
        """

        try:
            data = json.loads(msg.data)
        except Exception:
            return

        self.latest_mold_data = data

        green_percent = float(data.get('green_percent', 0.0))
        mold_detected = bool(data.get('mold_detected', False))

        if self.state == MissionState.SCANNING_MOLD:
            self.max_green_percent = max(self.max_green_percent, green_percent)

            if mold_detected:
                self.mold_detected = True

    def apriltag_callback(self, msg):
        """
        Supports both formats:

        Old:
        "1,120,200"

        New JSON:
        {
            "id": 1,
            "center_x": 320,
            "center_y": 240
        }
        """

        try:
            text = msg.data.strip()

            if text.startswith('{'):
                data = json.loads(text)
                self.last_seen_tag_id = int(data.get('id', -1))
            else:
                self.last_seen_tag_id = int(text.split(',')[0])

            self.last_seen_tag_time = time.time()

        except Exception:
            pass

    # ============================================================
    # MISSION FLOW
    # ============================================================

    def start_mission(self, cave_data):
        self.waypoints = self.load_waypoints()

        node_id = str(cave_data.get('node_id', '')).strip()

        if node_id not in self.waypoints:
            self.get_logger().error(f'No waypoint found for node_id: {node_id}')
            self.speak(f'Warning. No waypoint is set for {node_id}. Mission cannot start.')
            return

        self.current_target_name = node_id
        self.current_waypoint = self.waypoints[node_id]
        self.current_cave_data = cave_data

        humidity = float(cave_data.get('humidity', 0.0))

        self.mold_detected = False
        self.max_green_percent = 0.0
        self.latest_mold_data = None

        self.speak(
            f'Warning. High humidity detected at {node_id}. '
            f'Humidity is {humidity:.1f} percent. '
            f'PUSANTARA is moving to inspect the rock art panel.'
        )

        self.get_logger().info(f'Starting CaveCare mission to {node_id}')
        self.navigate_to_waypoint(node_id, self.current_waypoint)
        self.state = MissionState.NAVIGATING_TO_PANEL

    def state_loop(self):
        if self.state == MissionState.IDLE:
            return

        if self.state == MissionState.SCANNING_MOLD:
            elapsed = time.time() - self.scan_start_time

            if elapsed >= self.scan_duration:
                self.get_logger().info('Mold scan completed.')
                self.state = MissionState.REPORTING
                self.make_report()
                self.return_home()

    # ============================================================
    # NAV2
    # ============================================================

    def navigate_to_waypoint(self, name, waypoint):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(waypoint.get('x', 0.0))
        goal_msg.pose.pose.position.y = float(waypoint.get('y', 0.0))
        goal_msg.pose.pose.position.z = 0.0

        yaw = float(waypoint.get('yaw', 0.0))
        qz, qw = self.yaw_to_quaternion(yaw)

        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending Nav2 goal to {name}: '
            f'x={waypoint.get("x")}, y={waypoint.get("y")}, yaw={yaw}'
        )

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 navigate_to_pose action server not available.')
            self.speak('Navigation server is not available.')
            self.state = MissionState.ERROR
            return

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected.')
            self.speak('Navigation goal rejected.')
            self.state = MissionState.ERROR
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info('Nav2 goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()

        if self.state == MissionState.NAVIGATING_TO_PANEL:
            self.get_logger().info('Arrived at rock art panel waypoint.')
            self.on_arrived_at_panel()

        elif self.state == MissionState.RETURNING_HOME:
            self.get_logger().info('Returned home.')
            self.speak('Inspection complete. PUSANTARA has returned to home position.')
            self.reset_mission()

    def on_arrived_at_panel(self):
        self.stop_robot()

        self.speak('PUSANTARA has arrived at the inspection point.')

        self.state = MissionState.VERIFYING_TAG
        self.verify_apriltag()

        self.start_mold_scan()

    def return_home(self):
        self.waypoints = self.load_waypoints()

        if 'home' not in self.waypoints:
            self.get_logger().error('No home waypoint found.')
            self.speak('Home waypoint is not set. Mission cannot return home automatically.')
            self.reset_mission()
            return

        self.speak('Returning to home position.')
        self.state = MissionState.RETURNING_HOME
        self.navigate_to_waypoint('home', self.waypoints['home'])

    # ============================================================
    # APRILTAG CONFIRMATION
    # ============================================================

    def verify_apriltag(self):
        expected_tag = int(self.current_waypoint.get('tag_id', 0))

        if expected_tag == 0:
            self.get_logger().warn('No expected AprilTag ID set for this waypoint.')
            self.speak('No AprilTag ID is set for this panel. Continuing inspection.')
            return

        tag_is_recent = (time.time() - self.last_seen_tag_time) <= 3.0

        if tag_is_recent and self.last_seen_tag_id == expected_tag:
            self.get_logger().info(f'AprilTag confirmed: ID {expected_tag}')
            self.speak(f'Rock art panel confirmed. AprilTag ID {expected_tag} detected.')
        else:
            self.get_logger().warn(
                f'AprilTag not confirmed. Expected {expected_tag}, '
                f'last seen {self.last_seen_tag_id}'
            )
            self.speak(
                f'Panel marker is not clearly confirmed. '
                f'Expected AprilTag ID {expected_tag}. Continuing inspection for demonstration.'
            )

    # ============================================================
    # MOLD SCAN
    # ============================================================

    def start_mold_scan(self):
        self.state = MissionState.SCANNING_MOLD

        self.scan_start_time = time.time()
        self.mold_detected = False
        self.max_green_percent = 0.0

        self.speak('Starting mold and moss visual scan.')
        self.get_logger().info(f'Mold scan started for {self.scan_duration} seconds.')

    # ============================================================
    # REPORT
    # ============================================================

    def make_report(self):
        cave = self.current_cave_data or {}
        waypoint = self.current_waypoint or {}

        node_id = str(cave.get('node_id', self.current_target_name))
        panel_id = str(cave.get('panel_id', waypoint.get('panel_id', 'unknown')))
        tag_id = int(cave.get('tag_id', waypoint.get('tag_id', 0)))

        temperature = float(cave.get('temperature', 0.0))
        humidity = float(cave.get('humidity', 0.0))

        final_status = self.classify_status(
            humidity=humidity,
            mold_detected=self.mold_detected,
            green_percent=self.max_green_percent
        )

        report = {
            "node_id": node_id,
            "panel_id": panel_id,
            "tag_id": tag_id,
            "temperature": round(temperature, 2),
            "humidity": round(humidity, 2),
            "mold_detected": self.mold_detected,
            "green_percent": round(self.max_green_percent, 2),
            "final_status": final_status,
            "time": time.strftime('%Y-%m-%d %H:%M:%S')
        }

        msg = String()
        msg.data = json.dumps(report)
        self.report_pub.publish(msg)

        self.get_logger().info(f'Inspection report published: {msg.data}')

        self.speak_report(report)

    def classify_status(self, humidity, mold_detected, green_percent):
        if humidity >= 80.0 and mold_detected:
            return 'DANGER'

        if humidity >= 80.0 and green_percent >= 2.0:
            return 'DANGER'

        if humidity >= 70.0 or mold_detected or green_percent >= 2.0:
            return 'WARNING'

        return 'SAFE'

    def speak_report(self, report):
        panel_id = report.get('panel_id', 'unknown')
        humidity = float(report.get('humidity', 0.0))
        green_percent = float(report.get('green_percent', 0.0))
        status = report.get('final_status', 'SAFE')

        if status == 'SAFE':
            text = (
                f'Panel {panel_id} is safe. '
                f'Humidity is {humidity:.1f} percent. '
                f'No mold or moss risk is detected.'
            )

        elif status == 'WARNING':
            text = (
                f'Warning. Panel {panel_id} has humidity of {humidity:.1f} percent. '
                f'Possible mold or moss risk detected. '
                f'Green patch area is {green_percent:.1f} percent.'
            )

        else:
            text = (
                f'Danger. Panel {panel_id} has high humidity of {humidity:.1f} percent '
                f'and possible biological growth. '
                f'Human conservation inspection is required.'
            )

        self.speak(text)

    # ============================================================
    # STOP / RESET / HELPERS
    # ============================================================

    def stop_robot(self):
        stop = String()
        stop.data = 'STOP'

        self.robot_cmd_pub.publish(stop)
        self.safe_cmd_pub.publish(stop)

        self.get_logger().info('STOP command published.')

    def reset_mission(self):
        self.stop_robot()

        self.state = MissionState.IDLE
        self.current_target_name = None
        self.current_waypoint = None
        self.current_cave_data = None

        self.latest_mold_data = None
        self.mold_detected = False
        self.max_green_percent = 0.0

        self.scan_start_time = None
        self.active_goal_handle = None

        self.get_logger().info('Mission reset. Waiting for next cave node alert.')

    def speak(self, text):
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw


def main(args=None):
    rclpy.init(args=args)

    node = CaveMissionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
