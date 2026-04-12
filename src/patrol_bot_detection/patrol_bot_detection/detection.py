import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class DetectionNode(Node):

    def __init__(self):
        super().__init__('intruder_detection_node')

        self.declare_parameter('intruder_distance_threshold', 3.0)
        self.declare_parameter('alert_repeat_sec', 1.0)
        self.declare_parameter('intruder_min_distance', 0.35)
        self.declare_parameter('min_cluster_points', 4)
        self.declare_parameter('min_intruder_width_deg', 1.5)
        self.declare_parameter('max_intruder_width_deg', 12.0)
        self.declare_parameter('detect_in_manual', True)
        self.threshold = float(self.get_parameter('intruder_distance_threshold').value)
        self.alert_repeat_sec = float(self.get_parameter('alert_repeat_sec').value)
        self.intruder_min_distance = float(self.get_parameter('intruder_min_distance').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.min_intruder_width_deg = float(self.get_parameter('min_intruder_width_deg').value)
        self.max_intruder_width_deg = float(self.get_parameter('max_intruder_width_deg').value)
        self.detect_in_manual = bool(self.get_parameter('detect_in_manual').value)
        self.sensors_enabled = True
        self.control_mode = 'AUTO'

        self.alert_pub = self.create_publisher(Bool, '/intruder_alert', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Bool, '/sensor_toggle', self.sensor_toggle_callback, 10)
        self.create_subscription(String, '/control_mode', self.control_mode_callback, 10)

        self.last_alert = False
        self.last_alert_log_time = 0.0
        self.get_logger().info('Intruder detection node ready.')

    def sensor_toggle_callback(self, msg: Bool):
        self.sensors_enabled = msg.data

    def control_mode_callback(self, msg: String):
        mode = msg.data.strip().upper()
        if not mode:
            return
        if mode != self.control_mode:
            self.control_mode = mode
            self.get_logger().info(f'Intruder detection control mode update: {self.control_mode}')

    def _set_alert(self, active: bool):
        if active == self.last_alert:
            return
        self.alert_pub.publish(Bool(data=active))
        self.last_alert = active

    def _valid_samples(self, msg: LaserScan):
        points = []
        for i, value in enumerate(msg.ranges):
            if math.isinf(value) or math.isnan(value) or value <= 0.05:
                continue
            points.append((i, value))
        return points

    def _cluster_candidates(self, candidates, sample_count: int):
        if not candidates:
            return []

        clusters = []
        current = [candidates[0]]
        for idx, value in candidates[1:]:
            if idx == current[-1][0] + 1:
                current.append((idx, value))
            else:
                clusters.append(current)
                current = [(idx, value)]
        clusters.append(current)

        # Merge wrap-around cluster crossing index boundary of a 360 scan.
        if (
            len(clusters) > 1
            and clusters[0][0][0] == 0
            and clusters[-1][-1][0] == sample_count - 1
        ):
            merged = clusters[-1] + clusters[0]
            clusters = [merged] + clusters[1:-1]

        return clusters

    def scan_callback(self, msg: LaserScan):
        if not self.sensors_enabled:
            if self.last_alert:
                self.last_alert = False
                self.alert_pub.publish(Bool(data=False))
                self.get_logger().info('Intruder monitoring disabled. Alert reset.')
            return

        if self.control_mode == 'MANUAL' and not self.detect_in_manual:
            if self.last_alert:
                self._set_alert(False)
                self.last_alert_log_time = 0.0
            return

        points = self._valid_samples(msg)
        if not points:
            return

        close_candidates = [
            (idx, value)
            for idx, value in points
            if self.intruder_min_distance <= value <= self.threshold
        ]
        clusters = self._cluster_candidates(close_candidates, len(msg.ranges))

        intruder_detected = False
        closest = self.threshold
        for cluster in clusters:
            if len(cluster) < max(1, self.min_cluster_points):
                continue
            width_deg = len(cluster) * abs(msg.angle_increment) * 180.0 / math.pi
            if width_deg < self.min_intruder_width_deg or width_deg > self.max_intruder_width_deg:
                continue
            cluster_min = min(value for _, value in cluster)
            if not intruder_detected or cluster_min < closest:
                intruder_detected = True
                closest = cluster_min

        now = time.monotonic()

        if intruder_detected != self.last_alert:
            self._set_alert(intruder_detected)

            if intruder_detected:
                self.last_alert_log_time = now
                self.get_logger().warn(f'\a!!! INTRUDER ALERT !!! distance={closest:.2f} m')
            else:
                self.get_logger().info('Intruder condition cleared')
                self.last_alert_log_time = 0.0
        elif intruder_detected and (now - self.last_alert_log_time) >= max(0.2, self.alert_repeat_sec):
            self.last_alert_log_time = now
            self.get_logger().warn(f'\a!!! INTRUDER ALERT (ACTIVE) !!! distance={closest:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
