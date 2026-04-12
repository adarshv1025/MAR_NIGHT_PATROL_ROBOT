import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String


class ControlNode(Node):

    def __init__(self):
        super().__init__('cmd_mux_node')

        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('teleop_timeout_sec', 0.8)

        self.manual_mode = False
        self.estop_active = False
        self.auto_cmd = Twist()
        self.teleop_cmd = Twist()
        self._estop_logged = False
        self._teleop_alive = False
        self._last_teleop_heartbeat = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)

        self.create_subscription(Twist, '/auto_cmd_vel', self.auto_callback, 10)
        self.create_subscription(Twist, '/teleop_cmd_vel', self.teleop_callback, 10)
        self.create_subscription(Bool, '/manual_mode', self.manual_mode_callback, 10)
        self.create_subscription(Empty, '/teleop_heartbeat', self.teleop_heartbeat_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.estop_callback, 10)

        publish_hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / max(publish_hz, 5.0), self.publish_cmd)
        self.get_logger().info('Command mux node ready (auto/teleop + emergency stop).')

    def auto_callback(self, msg: Twist):
        self.auto_cmd = msg

    def teleop_callback(self, msg: Twist):
        self.teleop_cmd = msg

    def manual_mode_callback(self, msg: Bool):
        if self.manual_mode == msg.data:
            return
        self.manual_mode = msg.data
        self.get_logger().info(f'Manual mode set to {self.manual_mode}')

    def teleop_heartbeat_callback(self, _msg: Empty):
        self._teleop_alive = True
        self._last_teleop_heartbeat = self.get_clock().now()

    def _check_teleop_timeout(self):
        if not self.manual_mode:
            return

        if not self._teleop_alive:
            return

        timeout_sec = float(self.get_parameter('teleop_timeout_sec').value)
        elapsed = (self.get_clock().now() - self._last_teleop_heartbeat).nanoseconds / 1e9
        if elapsed <= max(0.2, timeout_sec):
            return

        self.manual_mode = False
        self._teleop_alive = False
        self.teleop_cmd = Twist()
        self.get_logger().warn('Teleop heartbeat lost. Switching back to AUTO patrol mode.')

    def estop_callback(self, msg: Bool):
        self.estop_active = msg.data
        if not self.estop_active:
            self._estop_logged = False

    def publish_cmd(self):
        self._check_teleop_timeout()

        mode = 'MANUAL' if self.manual_mode else 'AUTO'
        self.mode_pub.publish(String(data=mode))

        if self.estop_active:
            if not self._estop_logged:
                self.get_logger().warn('Emergency stop active. Publishing zero cmd_vel.')
                self._estop_logged = True
            self.cmd_pub.publish(Twist())
            return

        cmd = self.teleop_cmd if self.manual_mode else self.auto_cmd
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
