import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String


class EmergencyManager(Node):

    def __init__(self):
        super().__init__('emergency_manager')

        self.declare_parameter('recovery_delay_sec', 2.0)
        self.declare_parameter('heartbeat_timeout_sec', 3.0)
        self.declare_parameter('startup_grace_sec', 5.0)

        self.recovery_delay_sec = float(self.get_parameter('recovery_delay_sec').value)
        self.heartbeat_timeout_sec = float(self.get_parameter('heartbeat_timeout_sec').value)
        self.startup_grace_sec = float(self.get_parameter('startup_grace_sec').value)

        self.kill_switch = False
        self.estop_active = False
        self.estop_reason = 'NONE'
        self.recovery_until = 0.0

        self.last_heartbeat = time.monotonic()
        self.start_time = time.monotonic()
        self.heartbeat_missing = False

        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.status_pub = self.create_publisher(String, '/system_status', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Bool, '/kill_switch', self.kill_switch_callback, 10)
        self.create_subscription(Empty, '/patrol_heartbeat', self.heartbeat_callback, 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Emergency manager node ready.')

    def heartbeat_callback(self, _msg: Empty):
        self.last_heartbeat = time.monotonic()
        if self.heartbeat_missing:
            self.heartbeat_missing = False
            self.get_logger().info('Patrol heartbeat recovered.')

    def kill_switch_callback(self, msg: Bool):
        previous = self.kill_switch
        self.kill_switch = msg.data

        if self.kill_switch and not previous:
            self.estop_active = True
            self.estop_reason = 'KILL_SWITCH'
            self.get_logger().warn('Kill switch engaged. Emergency stop ON.')
        elif (not self.kill_switch) and previous:
            self.recovery_until = time.monotonic() + self.recovery_delay_sec
            self.estop_reason = 'RECOVERING'
            self.get_logger().info('Kill switch released. Starting recovery countdown.')

    def _set_estop(self, active: bool, reason: str):
        self.estop_active = active
        self.estop_reason = reason

    def control_loop(self):
        now = time.monotonic()

        heartbeat_watchdog_active = (now - self.start_time) > self.startup_grace_sec
        if heartbeat_watchdog_active and now - self.last_heartbeat > self.heartbeat_timeout_sec:
            if not self.heartbeat_missing:
                self.heartbeat_missing = True
                self.get_logger().warn('Patrol heartbeat timeout. Entering safe stop.')
            self._set_estop(True, 'HEARTBEAT_LOST')

        if not self.kill_switch and self.estop_reason in ('RECOVERING', 'KILL_SWITCH', 'HEARTBEAT_LOST'):
            if self.estop_reason == 'RECOVERING' and now >= self.recovery_until:
                if now - self.last_heartbeat <= self.heartbeat_timeout_sec:
                    self._set_estop(False, 'NONE')
                    self.get_logger().info('Recovery complete. Emergency stop OFF.')
                else:
                    self._set_estop(True, 'HEARTBEAT_LOST')

            if self.estop_reason == 'HEARTBEAT_LOST' and now - self.last_heartbeat <= self.heartbeat_timeout_sec:
                self.recovery_until = now + self.recovery_delay_sec
                self._set_estop(True, 'RECOVERING')
                self.get_logger().info('Heartbeat restored. Running recovery delay before resume.')

        estop_msg = Bool(data=self.estop_active)
        self.estop_pub.publish(estop_msg)

        if self.estop_active:
            self.cmd_pub.publish(Twist())

        status = f'ESTOP={self.estop_active} reason={self.estop_reason}'
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
