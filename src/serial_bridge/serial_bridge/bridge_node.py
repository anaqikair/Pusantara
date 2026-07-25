import threading
import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray


PARAM_PORT     = 'port'
PARAM_BAUDRATE = 'baudrate'
PARAM_TIMEOUT  = 'timeout'

DEFAULT_PORT     = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A105QM2R-if00-port0'
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT  = 0.1

RECONNECT_DELAY  = 2.0  # seconds between reconnect attempts


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        self._init_parameters()
        self._init_state()
        self._init_serial()
        self._init_ros_interfaces()
        self._start_reader_thread()

        self.get_logger().info("----------------------------------------")
        self.get_logger().info("Serial Bridge Started")
        self.get_logger().info(f"Port : {self.port}")
        self.get_logger().info(f"Baud : {self.baudrate}")
        self.get_logger().info("----------------------------------------")

    # ------------------------------------------------------------------ #
    # Initialisation                                                       #
    # ------------------------------------------------------------------ #

    def _init_parameters(self):
        self.declare_parameter(PARAM_PORT,     DEFAULT_PORT)
        self.declare_parameter(PARAM_BAUDRATE, DEFAULT_BAUDRATE)
        self.declare_parameter(PARAM_TIMEOUT,  DEFAULT_TIMEOUT)

        self.port     = self.get_parameter(PARAM_PORT).value
        self.baudrate = self.get_parameter(PARAM_BAUDRATE).value
        self.timeout  = self.get_parameter(PARAM_TIMEOUT).value

    def _init_state(self):
        self.left_encoder  = 0
        self.right_encoder = 0
        self.tx_packets    = 0
        self.rx_packets    = 0
        self.bad_packets   = 0

        self.ser          = None
        self.serial_lock  = threading.Lock()
        self.running      = True

    def _init_serial(self):
        self._connect()

    def _init_ros_interfaces(self):
        self.command_sub = self.create_subscription(
            String,
            '/safe_robot_command',
            self._on_command,
            10,
        )

        self.encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/wheel_encoder',
            10,
        )

        self.status_timer = self.create_timer(5.0, self._on_status_timer)

    def _start_reader_thread(self):
        self.serial_thread = threading.Thread(
            target=self._reader_loop,
            daemon=True,
        )
        self.serial_thread.start()

    # ------------------------------------------------------------------ #
    # Serial connection                                                    #
    # ------------------------------------------------------------------ #

    def _connect(self):
        """Block until a serial connection is established."""
        while rclpy.ok():
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                self.get_logger().info("Connected to PRIZM.")
                return
            except Exception as e:
                self.get_logger().error(f"Unable to connect : {e}")
                time.sleep(RECONNECT_DELAY)

    def _reconnect(self):
        """Close the current port and attempt to reconnect."""
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass

        self.get_logger().warning("Attempting reconnect...")
        self._connect()

    # ------------------------------------------------------------------ #
    # Command subscriber                                                   #
    # ------------------------------------------------------------------ #

    def _on_command(self, msg: String):
        command = msg.data.strip()
        if not command:
            return

        try:
            with self.serial_lock:
                self.ser.write((command + "\n").encode())
                self.ser.flush()

            self.tx_packets += 1
            self.get_logger().info(f"TX -> {command}")

        except Exception as e:
            self.get_logger().error(f"TX Error : {e}")
            self._reconnect()

    # ------------------------------------------------------------------ #
    # Background reader thread                                             #
    # ------------------------------------------------------------------ #

    def _reader_loop(self):
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting == 0:
                    continue

                with self.serial_lock:
                    raw = self.ser.readline()

                line = raw.decode(errors="ignore").strip()
                if line:
                    self._parse_packet(line)

            except Exception as e:
                self.get_logger().error(f"RX Error : {e}")
                self._reconnect()

    # ------------------------------------------------------------------ #
    # Packet parser                                                        #
    # ------------------------------------------------------------------ #

    def _parse_packet(self, packet: str):
        """
        Supported packet format:
            E,<left_ticks>,<right_ticks>   — wheel encoder counts
        """
        fields = packet.split(",")
        if not fields:
            return

        packet_type = fields[0]

        if packet_type == "E":
            self._handle_encoder_packet(fields, packet)
        else:
            self.bad_packets += 1
            self.get_logger().warning(f"Unknown Packet : {packet}")

    def _handle_encoder_packet(self, fields: list[str], raw: str):
        if len(fields) != 3:
            self.bad_packets += 1
            self.get_logger().warning(f"Malformed Encoder Packet : {raw}")
            return

        try:
            left  = int(fields[1])
            right = int(fields[2])
        except ValueError:
            self.bad_packets += 1
            self.get_logger().warning(f"Invalid Encoder Data : {raw}")
            return

        self.left_encoder  = left
        self.right_encoder = right

        msg = Int32MultiArray()
        msg.data = [left, right]
        self.encoder_pub.publish(msg)

        self.rx_packets += 1

    # ------------------------------------------------------------------ #
    # Status timer                                                         #
    # ------------------------------------------------------------------ #

    def _on_status_timer(self):
        self.get_logger().info("========== SERIAL STATUS ==========")
        self.get_logger().info(f"Left Encoder  : {self.left_encoder}")
        self.get_logger().info(f"Right Encoder : {self.right_encoder}")
        self.get_logger().info(f"TX Packets    : {self.tx_packets}")
        self.get_logger().info(f"RX Packets    : {self.rx_packets}")
        self.get_logger().info(f"Bad Packets   : {self.bad_packets}")
        self.get_logger().info("===================================")

    # ------------------------------------------------------------------ #
    # Shutdown                                                             #
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        self.running = False

        if self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1)

        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


# ---------------------------------------------------------------------- #
# Entry point                                                             #
# ---------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
