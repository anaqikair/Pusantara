import os
import signal
import subprocess
import time

class NodeManager:
    def __init__(self):
        # Maps node key -> dict containing:
        # 'command': str
        # 'proc': subprocess.Popen or None
        # 'status': 'RUNNING' | 'STOPPED' | 'STARTING' | 'CRASHED'
        # 'launched_by_us': bool (to distinguish manual stop vs crash)
        self.nodes = {}
        
        # Pre-defined node commands
        
        self.commands = {
            # Vision
            "Camera": "ros2 run camera_node webcam_pub",
            "YOLO Detector": "ros2 run vectra_yolo yolo_detector --ros-args -p debug:=true -p confidence_threshold:=0.6",
            "AprilTag Detector": "ros2 run apriltag_detector apriltag_detector --ros-args -p show_ui:=true",
            "Keris Demo": "ros2 run keris_demo keris_demo",
            
            # Motion
            "AprilTag Follower": "ros2 run apriltag_follower apriltag_follower",
            "Person Follower": "ros2 run person_follower person_follower",
            "Motion Manager": "ros2 run motion_manager motion_manager",
            
            # Motor & Core
            "Robot Bringup": "ros2 launch pusantara_bringup robot.launch.py",
            "Serial Bridge": "ros2 run serial_bridge bridge_node",
            "Command Sender": "ros2 run serial_bridge command_sender",
            "LiDAR": "ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py",
            "Odometry": "ros2 launch vectra_odometry odometry.launch.py",
            "IMU": "ros2 run bno055_driver imu_node",
            
            # Autonomy
            "SLAM Toolbox": "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/orin_nano/vectra_ws/config/slam.yaml",
            "Navigation Stack": "ros2 launch pusantara_navigation navigation.launch.py use_sim_time:=False autostart:=True map:=/home/orin_nano/vectra_ws/src/pusantara_navigation/maps/museum.yaml params_file:=/home/orin_nano/vectra_ws/src/pusantara_navigation/config/nav2_params.yaml",

            # Interaction & Dialogue
            "Visitor Greeter": "ros2 run visitor_greeter visitor_greeter",
            "Heritage Explainer": "ros2 run heritage_explainer heritage_explainer",
            "TTS Engine": "ros2 run tts_node tts_node",
            "TTS Sender": "ros2 run tts_node tts_sender",
            "Speech to Text": "ros2 run speech_to_text stt_node",
            "Qwen Assistant": "ros2 run qwen_assistant qwen_node",
            
            # Heritage Apps
            "Food Quiz": "ros2 run food_quiz food_quiz",
            "Batik Designer": "ros2 run batik_designer batik_designer",
            "Admin Console": "ros2 run pusantara_admin admin_console",
            "Robot Face": "ros2 run robot_face face_node",
            "History Quiz": "ros2 launch malaysia_history_quiz history_quiz.launch.py",

            # Tools
            "RViz2": "rviz2",
            "RQt": "rqt"
        }
        
        # Initialize nodes state
        for name in self.commands:
            self.nodes[name] = {
                "proc": None,
                "status": "STOPPED",
                "launched_by_us": False
            }

    def start_node(self, name):
        if name not in self.commands:
            return False, f"Unknown node: {name}"
            
        node_info = self.nodes[name]
        
        # If already running/starting, prevent duplicate
        self.update_status(name)
        if node_info["status"] in ["RUNNING", "STARTING"]:
            return False, f"{name} is already {node_info['status']}"
            
        node_info["status"] = "STARTING"
        node_info["launched_by_us"] = True
        
        cmd = self.commands[name]
        try:
            # Use DEVNULL to prevent processes from freezing when the OS pipe buffer fills up
            import shlex
            proc = subprocess.Popen(
                shlex.split(cmd),
                shell=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            node_info["proc"] = proc
            # Give it a tiny bit of time to start, then mark RUNNING if it hasn't died
            time.sleep(0.2)
            if proc.poll() is None:
                node_info["status"] = "RUNNING"
            else:
                node_info["status"] = "CRASHED"
                node_info["proc"] = None
                return False, f"{name} failed to start (exited immediately)"
                
            return True, f"Started {name}"
        except Exception as e:
            node_info["status"] = "CRASHED"
            node_info["proc"] = None
            return False, str(e)

    def stop_node(self, name):
        if name not in self.nodes:
            return False, f"Unknown node: {name}"
            
        node_info = self.nodes[name]
        proc = node_info["proc"]
        
        if not proc:
            node_info["status"] = "STOPPED"
            return True, f"{name} is already stopped"
            
        node_info["launched_by_us"] = False
        try:
            # Kill the entire process group (including any child processes of the shell)
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            # Wait briefly for termination
            for _ in range(10):
                if proc.poll() is not None:
                    break
                time.sleep(0.1)
                
            if proc.poll() is None:
                # Force kill if SIGINT didn't work
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait()
                
            node_info["status"] = "STOPPED"
            node_info["proc"] = None
            return True, f"Stopped {name}"
        except Exception as e:
            node_info["status"] = "STOPPED"
            node_info["proc"] = None
            return False, str(e)

    def update_status(self, name):
        node_info = self.nodes[name]
        proc = node_info["proc"]
        
        if proc:
            exit_code = proc.poll()
            if exit_code is not None:
                # Process terminated
                if node_info["launched_by_us"]:
                    node_info["status"] = "CRASHED"
                else:
                    node_info["status"] = "STOPPED"
                node_info["proc"] = None
        else:
            if node_info["status"] != "CRASHED":
                node_info["status"] = "STOPPED"
        return node_info["status"]

    def update_all_statuses(self):
        status_dict = {}
        for name in self.nodes:
            status_dict[name] = self.update_status(name)
        return status_dict
