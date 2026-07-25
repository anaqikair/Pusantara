import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main():

    rclpy.init()

    node = Node("command_sender")

    pub = node.create_publisher(
        String,
        "/robot_command",
        10
    )

    while rclpy.ok():

        cmd = input("> ")

        msg = String()
        msg.data = cmd

        pub.publish(msg)

        print(f"Sent: {cmd}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
