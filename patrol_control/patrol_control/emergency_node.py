import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class EmergencyNode(Node):
    def __init__(self):
        super().__init__('emergency_node')
        self.pub = self.create_publisher(Bool, '/emergency_stop', 10)

    def trigger_stop(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)
        self.get_logger().info("🚨 Emergency Stop Sent")

    def resume(self):
        msg = Bool()
        msg.data = False
        self.pub.publish(msg)
        self.get_logger().info("✅ Resumed")


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyNode()

    while True:
        cmd = input("Enter (e=STOP, r=RESUME): ").lower()

        if cmd == 'e':
            node.trigger_stop()
        elif cmd == 'r':
            node.resume()
        else:
            print("Invalid input")
