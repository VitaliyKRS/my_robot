from geometry_msgs.msg import Twist
from rclpy.node import Node


class RemapNode(Node):
    def __init__(self):
        super().__init__("remap_node")
        self.vel_s = self.create_subscription(Twist, "/cmd_vel_nav", self.on_vel_received, 10)
        self.val_p = self.create_publisher(Twist, "/diffbot_base_controller/cmd_vel_unstamped", 10)

    def on_vel_received(self, msg):
        self.val_p.publish(msg)