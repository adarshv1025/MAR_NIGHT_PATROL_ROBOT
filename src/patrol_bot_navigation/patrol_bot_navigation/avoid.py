import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class AvoidNode(Node):

    def __init__(self):
        super().__init__('avoid_node')

        self.declare_parameter('obstacle_threshold', 1.10)
        self.declare_parameter('obstacle_clearance', 1.45)
        self.declare_parameter('critical_distance', 0.45)
        self.declare_parameter('slow_down_distance', 1.90)
        self.declare_parameter('min_slow_scale', 0.20)
        self.declare_parameter('turn_speed', 1.0)
        self.declare_parameter('recovery_turn_speed', 1.3)
        self.declare_parameter('reverse_speed', -0.12)
        self.declare_parameter('blocked_timeout', 2.5)
        self.declare_parameter('recovery_duration', 1.0)
        self.declare_parameter('obstacle_vote_in', 2)
        self.declare_parameter('obstacle_vote_out', 5)
        self.declare_parameter('avoid_only_in_auto', True)

        self.obstacle_threshold = float(self.get_parameter('obstacle_threshold').value)
        self.obstacle_clearance = float(self.get_parameter('obstacle_clearance').value)
        self.critical_distance = float(self.get_parameter('critical_distance').value)
        self.slow_down_distance = float(self.get_parameter('slow_down_distance').value)
        self.min_slow_scale = float(self.get_parameter('min_slow_scale').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.recovery_turn_speed = float(self.get_parameter('recovery_turn_speed').value)
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.blocked_timeout = float(self.get_parameter('blocked_timeout').value)
        self.recovery_duration = float(self.get_parameter('recovery_duration').value)
        self.obstacle_vote_in = int(self.get_parameter('obstacle_vote_in').value)
        self.obstacle_vote_out = int(self.get_parameter('obstacle_vote_out').value)
        self.avoid_only_in_auto = bool(self.get_parameter('avoid_only_in_auto').value)
        self.sensors_enabled = True
        self.control_mode = 'AUTO'

        self.latest_scan = None
        self.latest_patrol_cmd = Twist()
        self.estop_active = False
        self.obstacle_active = False
        self.blocked_since = None
        self.recovery_until = 0.0
        self.last_turn_direction = 1.0
        self.last_mode = 'CLEAR'
        self.obstacle_votes = 0
        self.clear_votes = 0

        self.pub = self.create_publisher(Twist, '/auto_cmd_vel', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Twist, '/patrol_cmd_vel', self.patrol_cmd_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.estop_callback, 10)
        self.create_subscription(Bool, '/sensor_toggle', self.sensor_toggle_callback, 10)
        self.create_subscription(String, '/control_mode', self.control_mode_callback, 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Obstacle avoidance node ready.')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def patrol_cmd_callback(self, msg: Twist):
        self.latest_patrol_cmd = msg

    def estop_callback(self, msg: Bool):
        self.estop_active = msg.data

    def sensor_toggle_callback(self, msg: Bool):
        self.sensors_enabled = msg.data
        self.get_logger().info(f'Sensor toggle update received. enabled={self.sensors_enabled}')

    def control_mode_callback(self, msg: String):
        mode = msg.data.strip().upper()
        if not mode or mode == self.control_mode:
            return
        self.control_mode = mode
        self.get_logger().info(f'Avoid node control mode update: {self.control_mode}')

    def _valid(self, values):
        return [v for v in values if not (math.isinf(v) or math.isnan(v)) and v > 0.05]

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _set_mode(self, mode: str, message: str):
        if mode == self.last_mode:
            return
        self.last_mode = mode
        self.get_logger().info(message)

    def _reset_avoid_state(self):
        self.obstacle_active = False
        self.blocked_since = None
        self.recovery_until = 0.0
        self.obstacle_votes = 0
        self.clear_votes = 0

    def _pick_turn_direction(self, left_sector, right_sector) -> float:
        left_mean = sum(left_sector) / len(left_sector) if left_sector else 0.0
        right_mean = sum(right_sector) / len(right_sector) if right_sector else 0.0

        if abs(left_mean - right_mean) < 0.08:
            self.last_turn_direction *= -1.0
        else:
            self.last_turn_direction = 1.0 if left_mean >= right_mean else -1.0

        return self.last_turn_direction

    def _front_distance(self, front_sector):
        ordered = sorted(front_sector)
        k = min(6, len(ordered))
        if k == 0:
            return None
        return sum(ordered[:k]) / float(k)

    def control_loop(self):
        cmd = Twist()

        if self.estop_active:
            self.pub.publish(cmd)
            self.obstacle_pub.publish(Bool(data=False))
            return

        if self.avoid_only_in_auto and self.control_mode != 'AUTO':
            self._reset_avoid_state()
            self.pub.publish(self.latest_patrol_cmd)
            self.obstacle_pub.publish(Bool(data=False))
            self._set_mode('BYPASS', f'Avoidance bypassed in {self.control_mode} mode.')
            return

        if not self.sensors_enabled or self.latest_scan is None:
            self._reset_avoid_state()
            self.pub.publish(self.latest_patrol_cmd)
            self.obstacle_pub.publish(Bool(data=False))
            return

        ranges = self.latest_scan.ranges
        if len(ranges) < 20:
            self.pub.publish(self.latest_patrol_cmd)
            self.obstacle_pub.publish(Bool(data=False))
            return

        width = max(10, len(ranges) // 24)
        front_sector = self._valid(list(ranges[:width]) + list(ranges[-width:]))
        left_sector = self._valid(list(ranges[len(ranges) // 6:len(ranges) // 3]))
        right_sector = self._valid(list(ranges[-len(ranges) // 3:-len(ranges) // 6]))

        if not front_sector:
            self.pub.publish(self.latest_patrol_cmd)
            self.obstacle_pub.publish(Bool(data=False))
            return

        now = self._now_sec()
        min_front = self._front_distance(front_sector)
        if min_front is None:
            self.pub.publish(self.latest_patrol_cmd)
            self.obstacle_pub.publish(Bool(data=False))
            return
        critical = min_front < self.critical_distance
        if critical:
            self.obstacle_active = True
            self.obstacle_votes = self.obstacle_vote_in
            self.clear_votes = 0
            if self.blocked_since is None:
                self.blocked_since = now
            self.recovery_until = max(self.recovery_until, now + self.recovery_duration)
        else:
            if self.obstacle_active:
                if min_front > self.obstacle_clearance:
                    self.clear_votes += 1
                else:
                    self.clear_votes = 0

                if self.clear_votes >= max(1, self.obstacle_vote_out) and now > self.recovery_until:
                    self.obstacle_active = False
                    self.blocked_since = None
                    self.clear_votes = 0
                    self.obstacle_votes = 0
            elif min_front < self.obstacle_threshold:
                self.obstacle_votes += 1
                if self.obstacle_votes >= max(1, self.obstacle_vote_in):
                    self.obstacle_active = True
                    self.blocked_since = now
                    self.obstacle_votes = 0
                    self.clear_votes = 0
            else:
                self.obstacle_votes = max(0, self.obstacle_votes - 1)

        obstacle = self.obstacle_active or (now < self.recovery_until)
        self.obstacle_pub.publish(Bool(data=obstacle))

        turn_direction = self._pick_turn_direction(left_sector, right_sector)

        if obstacle and self.blocked_since is not None and (now - self.blocked_since) > self.blocked_timeout:
            self.recovery_until = max(self.recovery_until, now + self.recovery_duration)
            self.blocked_since = now

        if now < self.recovery_until:
            cmd.linear.x = self.reverse_speed
            cmd.angular.z = self.recovery_turn_speed * turn_direction
            self._set_mode('RECOVERY', f'Recovery maneuver active. front={min_front:.2f} m')
        elif self.obstacle_active:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed * turn_direction
            self._set_mode('AVOID', f'Obstacle ahead at {min_front:.2f} m. Turning to clear path.')
        else:
            cmd = Twist()
            cmd.linear.x = self.latest_patrol_cmd.linear.x
            cmd.angular.z = self.latest_patrol_cmd.angular.z
            if cmd.linear.x > 0.0 and min_front < self.slow_down_distance:
                denom = max(0.05, self.slow_down_distance - self.critical_distance)
                scale = (min_front - self.critical_distance) / denom
                scale = max(self.min_slow_scale, min(1.0, scale))
                cmd.linear.x *= scale
            self._set_mode('CLEAR', 'Path clear. Following patrol command.')

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
