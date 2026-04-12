import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty


class WasdTeleopNode(Node):

    def __init__(self):
        super().__init__('wasd_teleop')

        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('max_linear', 0.8)
        self.declare_parameter('max_angular', 1.8)
        self.declare_parameter('linear_step', 0.06)
        self.declare_parameter('angular_step', 0.18)
        self.declare_parameter('linear_accel_limit', 1.25)
        self.declare_parameter('angular_accel_limit', 3.2)
        self.declare_parameter('key_timeout', 0.35)

        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)
        self.linear_accel_limit = float(self.get_parameter('linear_accel_limit').value)
        self.angular_accel_limit = float(self.get_parameter('angular_accel_limit').value)
        self.key_timeout = float(self.get_parameter('key_timeout').value)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

        self.last_key_time = time.monotonic()
        self.last_tick = time.monotonic()
        self._stop_event = threading.Event()
        self._key_lock = threading.Lock()
        self._pending_keys = []
        self._keyboard_thread = None

        self.cmd_pub = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        self.manual_pub = self.create_publisher(Bool, '/manual_mode', 10)
        self.heartbeat_pub = self.create_publisher(Empty, '/teleop_heartbeat', 10)

        self._tty_ready = sys.stdin.isatty()
        self._terminal_settings = None
        if self._tty_ready:
            self._terminal_settings = termios.tcgetattr(sys.stdin)
            self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
            self._keyboard_thread.start()
        else:
            self.get_logger().warn('stdin is not a TTY. WASD input will not be available in this process.')

        self.create_timer(1.0 / max(self.publish_hz, 5.0), self.control_loop)
        self.create_timer(0.5, self.publish_manual_mode)
        self.create_timer(0.2, self.publish_heartbeat)

        self.get_logger().info('WASD teleop ready: w/s forward-back, a/d turn, space stop, q quit.')

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _ramp(self, current: float, target: float, max_delta: float) -> float:
        if target > current:
            return min(target, current + max_delta)
        return max(target, current - max_delta)

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        try:
            tty.setraw(fd)
            while not self._stop_event.is_set() and rclpy.ok():
                readable, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not readable:
                    continue

                key = sys.stdin.read(1)
                with self._key_lock:
                    self._pending_keys.append(key)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, self._terminal_settings)

    def _drain_keys(self):
        with self._key_lock:
            if not self._pending_keys:
                return []
            keys = self._pending_keys[:]
            self._pending_keys.clear()
            return keys

    def _handle_key(self, key: str):
        if key == 'w':
            self.target_linear = self._clamp(self.target_linear + self.linear_step, -self.max_linear, self.max_linear)
        elif key == 's':
            self.target_linear = self._clamp(self.target_linear - self.linear_step, -self.max_linear, self.max_linear)
        elif key == 'a':
            self.target_angular = self._clamp(self.target_angular + self.angular_step, -self.max_angular, self.max_angular)
        elif key == 'd':
            self.target_angular = self._clamp(self.target_angular - self.angular_step, -self.max_angular, self.max_angular)
        elif key in (' ', 'x'):
            self.target_linear = 0.0
            self.target_angular = 0.0
        elif key in ('q', '\x03'):
            self.target_linear = 0.0
            self.target_angular = 0.0
            self._publish_zero_cmd()
            self.manual_pub.publish(Bool(data=False))
            self.get_logger().info('Exiting WASD teleop and returning to AUTO mode.')
            rclpy.shutdown()

    def publish_manual_mode(self):
        self.manual_pub.publish(Bool(data=True))

    def publish_heartbeat(self):
        self.heartbeat_pub.publish(Empty())

    def _publish_zero_cmd(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        now = time.monotonic()
        keys = self._drain_keys()
        if keys:
            self.last_key_time = now
            for key in keys:
                self._handle_key(key)
        elif now - self.last_key_time > self.key_timeout:
            self.target_linear = 0.0
            self.target_angular = 0.0

        dt = max(0.001, now - self.last_tick)
        self.last_tick = now

        self.current_linear = self._ramp(
            self.current_linear,
            self.target_linear,
            self.linear_accel_limit * dt,
        )
        self.current_angular = self._ramp(
            self.current_angular,
            self.target_angular,
            self.angular_accel_limit * dt,
        )

        cmd = Twist()
        cmd.linear.x = self.current_linear
        cmd.angular.z = self.current_angular
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        self._stop_event.set()
        if self._keyboard_thread is not None:
            self._keyboard_thread.join(timeout=0.3)
        self._publish_zero_cmd()
        self.manual_pub.publish(Bool(data=False))
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WasdTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
