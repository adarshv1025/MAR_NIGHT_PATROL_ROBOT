import math
import random
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray, String


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        self.declare_parameter('waypoints', [-3.0, -3.0, 3.0, -3.0, 3.0, 3.0, -3.0, 3.0])
        self.declare_parameter('waypoint_tolerance', 0.35)
        self.declare_parameter('linear_speed', 0.50)
        self.declare_parameter('angular_speed', 1.2)
        self.declare_parameter('turn_in_place_angle', 0.42)
        self.declare_parameter('turn_exit_angle', 0.18)
        self.declare_parameter('turning_angular_gain', 1.8)
        self.declare_parameter('tracking_angular_gain', 1.2)
        self.declare_parameter('min_turn_speed', 0.35)
        self.declare_parameter('min_move_speed', 0.05)
        self.declare_parameter('random_deviation_max', 0.35)
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('stuck_timeout_sec', 4.5)
        self.declare_parameter('stuck_progress_epsilon', 0.10)

        raw_waypoints = self.get_parameter('waypoints').value
        self.waypoints = self._parse_waypoints(raw_waypoints)
        if not self.waypoints:
            self.waypoints = [(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)]

        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.turn_in_place_angle = float(self.get_parameter('turn_in_place_angle').value)
        self.turn_exit_angle = float(self.get_parameter('turn_exit_angle').value)
        self.turning_angular_gain = float(self.get_parameter('turning_angular_gain').value)
        self.tracking_angular_gain = float(self.get_parameter('tracking_angular_gain').value)
        self.min_turn_speed = float(self.get_parameter('min_turn_speed').value)
        self.min_move_speed = float(self.get_parameter('min_move_speed').value)
        self.random_deviation_max = float(self.get_parameter('random_deviation_max').value)
        self.stuck_timeout_sec = float(self.get_parameter('stuck_timeout_sec').value)
        self.stuck_progress_epsilon = float(self.get_parameter('stuck_progress_epsilon').value)
        self.speed_scale = 1.0

        self.pose_x = None
        self.pose_y = None
        self.yaw = 0.0
        self.estop_active = False
        self.current_index = 0
        self.current_target = self._deviated_target(self.waypoints[self.current_index])
        self.last_status = ''
        self.turning_in_place = False
        self.progress_ref_x = None
        self.progress_ref_y = None
        self.progress_ref_time = self.get_clock().now().nanoseconds / 1e9

        self.cmd_pub = self.create_publisher(Twist, '/patrol_cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/patrol_status', 10)
        self.heartbeat_pub = self.create_publisher(Empty, '/patrol_heartbeat', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.estop_callback, 10)
        self.create_subscription(Float32, '/patrol_speed_scale', self.speed_scale_callback, 10)
        self.create_subscription(Float32MultiArray, '/patrol_waypoints', self.path_update_callback, 10)

        control_hz = float(self.get_parameter('control_hz').value)
        self.create_timer(1.0 / max(control_hz, 1.0), self.control_loop)
        self.create_timer(1.0, self.publish_heartbeat)

        self.get_logger().info('Patrol node ready with waypoint loop and random deviation.')

    def _parse_waypoints(self, data: List[float]) -> List[Tuple[float, float]]:
        if len(data) < 4 or len(data) % 2 != 0:
            return []
        return [(float(data[i]), float(data[i + 1])) for i in range(0, len(data), 2)]

    def _deviated_target(self, point: Tuple[float, float]) -> Tuple[float, float]:
        deviation_x = random.uniform(-self.random_deviation_max, self.random_deviation_max)
        deviation_y = random.uniform(-self.random_deviation_max, self.random_deviation_max)
        return point[0] + deviation_x, point[1] + deviation_y

    def publish_heartbeat(self):
        self.heartbeat_pub.publish(Empty())

    def odom_callback(self, msg: Odometry):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def estop_callback(self, msg: Bool):
        self.estop_active = msg.data

    def speed_scale_callback(self, msg: Float32):
        self.speed_scale = max(0.1, min(2.0, float(msg.data)))
        self.get_logger().info(f'Updated patrol speed scale: {self.speed_scale:.2f}')

    def path_update_callback(self, msg: Float32MultiArray):
        new_points = self._parse_waypoints(list(msg.data))
        if not new_points:
            self.get_logger().warn('Ignored invalid patrol_waypoints update (need flattened x,y list).')
            return

        self.waypoints = new_points
        self.current_index = 0
        self.current_target = self._deviated_target(self.waypoints[self.current_index])
        self.turning_in_place = False
        self.get_logger().info(f'Patrol path updated live. Waypoint count: {len(self.waypoints)}')

    def _normalize(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _publish_status(self, text: str):
        if text == self.last_status:
            return
        self.last_status = text
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _reset_progress_reference(self):
        if self.pose_x is None or self.pose_y is None:
            return
        self.progress_ref_x = self.pose_x
        self.progress_ref_y = self.pose_y
        self.progress_ref_time = self.get_clock().now().nanoseconds / 1e9

    def _is_stuck(self) -> bool:
        if self.pose_x is None or self.pose_y is None:
            return False

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.progress_ref_x is None or self.progress_ref_y is None:
            self._reset_progress_reference()
            return False

        progress = math.hypot(self.pose_x - self.progress_ref_x, self.pose_y - self.progress_ref_y)
        if progress >= self.stuck_progress_epsilon:
            self._reset_progress_reference()
            return False

        return (now_sec - self.progress_ref_time) > self.stuck_timeout_sec

    def control_loop(self):
        cmd = Twist()

        if self.estop_active:
            self.cmd_pub.publish(cmd)
            self._reset_progress_reference()
            self._publish_status('PATROL_PAUSED: emergency stop active')
            return

        if self.pose_x is None or self.pose_y is None:
            self.cmd_pub.publish(cmd)
            self._publish_status('PATROL_WAITING: odometry not ready')
            return

        target_x, target_y = self.current_target
        dx = target_x - self.pose_x
        dy = target_y - self.pose_y
        distance = math.hypot(dx, dy)

        if distance < self.waypoint_tolerance:
            self.current_index = (self.current_index + 1) % len(self.waypoints)
            self.current_target = self._deviated_target(self.waypoints[self.current_index])
            self.turning_in_place = False
            self._reset_progress_reference()
            target_x, target_y = self.current_target
            dx = target_x - self.pose_x
            dy = target_y - self.pose_y
            distance = math.hypot(dx, dy)
            self._publish_status(f'PATROL_ADVANCE: waypoint={self.current_index}')

        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize(target_yaw - self.yaw)
        abs_yaw_error = abs(yaw_error)

        turn_limit = self.angular_speed * self.speed_scale
        if self.turning_in_place:
            if abs_yaw_error <= self.turn_exit_angle:
                self.turning_in_place = False
        elif abs_yaw_error >= self.turn_in_place_angle:
            self.turning_in_place = True

        if self.turning_in_place:
            self._reset_progress_reference()
            cmd.linear.x = 0.0
            turn_cmd = self.turning_angular_gain * yaw_error
            if abs(turn_cmd) < self.min_turn_speed and abs_yaw_error > 1e-3:
                turn_cmd = math.copysign(self.min_turn_speed, yaw_error)
            cmd.angular.z = max(-turn_limit, min(turn_limit, turn_cmd))
            state = 'PATROL_TURNING'
        else:
            if self._is_stuck():
                self.current_index = (self.current_index + 1) % len(self.waypoints)
                self.current_target = self._deviated_target(self.waypoints[self.current_index])
                self.turning_in_place = False
                self._reset_progress_reference()
                self.cmd_pub.publish(Twist())
                self._publish_status(f'PATROL_RECOVERY: stuck, skipping to waypoint={self.current_index}')
                return

            base_speed = self.linear_speed * self.speed_scale
            heading_scale = 1.0 - min(1.0, abs_yaw_error / max(self.turn_in_place_angle, 1e-3))
            cmd.linear.x = max(self.min_move_speed, base_speed * heading_scale)
            cmd.angular.z = max(-turn_limit, min(turn_limit, self.tracking_angular_gain * yaw_error))
            state = 'PATROL_MOVING'

        self.cmd_pub.publish(cmd)
        self._publish_status(
            f'{state}: wp={self.current_index} target=({target_x:.2f},{target_y:.2f}) dist={distance:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
