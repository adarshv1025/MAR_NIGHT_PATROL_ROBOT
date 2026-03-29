import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.emergency = False
        self.current_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        # Subscriptions
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_input', self.cmd_callback, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def emergency_callback(self, msg):
        self.emergency = msg.data

    def cmd_callback(self, msg):
        if not self.emergency:
            self.current_cmd = msg
            self.last_cmd_time = self.get_clock().now()

    def publish_cmd(self):
        now = self.get_clock().now()
        diff = (now - self.last_cmd_time).nanoseconds / 1e9

        if self.emergency or diff > 1.0:
            self.cmd_pub.publish(Twist())
        else:
            self.cmd_pub.publish(self.current_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
