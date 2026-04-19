import math
import time

import rclpy
from gazebo_msgs.msg import ModelStates
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
        self.declare_parameter('max_intruder_width_deg', 28.0)
        self.declare_parameter('min_intruder_width_m', 0.08)
        self.declare_parameter('max_intruder_width_m', 0.70)
        self.declare_parameter('max_index_gap', 1)
        self.declare_parameter('cluster_jump_threshold_m', 0.28)
        self.declare_parameter('max_cluster_range_spread_m', 0.85)
        self.declare_parameter('confirm_scans', 2)
        self.declare_parameter('clear_scans', 4)
        self.declare_parameter('hold_alert_sec', 0.9)
        self.declare_parameter('detect_in_manual', True)
        self.declare_parameter('use_model_state_detection', True)
        self.declare_parameter('fallback_to_lidar', True)
        self.declare_parameter('robot_model_name', 'patrol_bot')
        self.declare_parameter('intruder_model_prefix', 'intruder_')
        self.declare_parameter('model_state_timeout_sec', 1.5)

        self.threshold = float(self.get_parameter('intruder_distance_threshold').value)
        self.alert_repeat_sec = float(self.get_parameter('alert_repeat_sec').value)
        self.intruder_min_distance = float(self.get_parameter('intruder_min_distance').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.min_intruder_width_deg = float(self.get_parameter('min_intruder_width_deg').value)
        self.max_intruder_width_deg = float(self.get_parameter('max_intruder_width_deg').value)
        self.min_intruder_width_m = float(self.get_parameter('min_intruder_width_m').value)
        self.max_intruder_width_m = float(self.get_parameter('max_intruder_width_m').value)
        self.max_index_gap = int(self.get_parameter('max_index_gap').value)
        self.cluster_jump_threshold_m = float(self.get_parameter('cluster_jump_threshold_m').value)
        self.max_cluster_range_spread_m = float(self.get_parameter('max_cluster_range_spread_m').value)
        self.confirm_scans = int(self.get_parameter('confirm_scans').value)
        self.clear_scans = int(self.get_parameter('clear_scans').value)
        self.hold_alert_sec = float(self.get_parameter('hold_alert_sec').value)
        self.detect_in_manual = bool(self.get_parameter('detect_in_manual').value)
        self.use_model_state_detection = bool(self.get_parameter('use_model_state_detection').value)
        self.fallback_to_lidar = bool(self.get_parameter('fallback_to_lidar').value)
        self.robot_model_name = str(self.get_parameter('robot_model_name').value)
        self.intruder_model_prefix = str(self.get_parameter('intruder_model_prefix').value)
        self.model_state_timeout_sec = float(self.get_parameter('model_state_timeout_sec').value)
        self.sensors_enabled = True
        self.control_mode = 'AUTO'

        self.alert_pub = self.create_publisher(Bool, '/intruder_alert', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)
        self.create_subscription(Bool, '/sensor_toggle', self.sensor_toggle_callback, 10)
        self.create_subscription(String, '/control_mode', self.control_mode_callback, 10)

        self.latest_scan = None
        self.latest_model_states = None
        self.last_model_state_time = 0.0
        self.last_alert = False
        self.last_alert_log_time = 0.0
        self.detection_streak = 0
        self.clear_streak = 0
        self.last_confirmed_distance = self.threshold
        self.last_seen_time = 0.0
        self.last_source = 'none'
        self._model_state_warned = False
        self._fallback_warned = False
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

    def model_states_callback(self, msg: ModelStates):
        self.latest_model_states = msg
        self.last_model_state_time = time.monotonic()
        self._model_state_warned = False
        self._fallback_warned = False

    def _set_alert(self, active: bool):
        if active == self.last_alert:
            return
        self.alert_pub.publish(Bool(data=active))
        self.last_alert = active

    def _reset_state(self):
        self.detection_streak = 0
        self.clear_streak = 0

    def _valid_samples(self, msg: LaserScan):
        points = []
        range_min = max(0.05, float(msg.range_min))
        range_max = float(msg.range_max)
        for i, value in enumerate(msg.ranges):
            if not math.isfinite(value):
                continue
            if value < range_min:
                continue
            if math.isfinite(range_max) and value > range_max:
                continue
            points.append((i, value))
        return points

    def _find_model_index(self, names, target_name: str):
        for i, name in enumerate(names):
            if name == target_name:
                return i
        return -1

    def _find_intruder_from_model_states(self, now: float):
        if self.latest_model_states is None:
            return None

        if (now - self.last_model_state_time) > max(0.2, self.model_state_timeout_sec):
            return None

        names = self.latest_model_states.name
        poses = self.latest_model_states.pose
        robot_idx = self._find_model_index(names, self.robot_model_name)
        if robot_idx < 0 or robot_idx >= len(poses):
            return None

        robot_pos = poses[robot_idx].position
        closest = self.threshold
        target_name = ''
        detected = False

        prefix = self.intruder_model_prefix.strip()
        for i, name in enumerate(names):
            if not prefix or not name.startswith(prefix):
                continue
            if i >= len(poses):
                continue

            intruder_pos = poses[i].position
            distance = math.hypot(intruder_pos.x - robot_pos.x, intruder_pos.y - robot_pos.y)
            if self.intruder_min_distance <= distance <= self.threshold:
                if not detected or distance < closest:
                    closest = distance
                    target_name = name
                    detected = True

        return detected, closest, target_name

    def _cluster_candidates(self, candidates, sample_count: int):
        if not candidates:
            return []

        clusters = []
        current = [candidates[0]]
        for idx, value in candidates[1:]:
            prev_idx, prev_value = current[-1]
            contiguous_idx = (idx - prev_idx) <= max(1, self.max_index_gap + 1)
            close_enough = abs(value - prev_value) <= max(0.02, self.cluster_jump_threshold_m)
            if contiguous_idx and close_enough:
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
            first_value = clusters[0][0][1]
            last_value = clusters[-1][-1][1]
            first_idx = clusters[0][0][0]
            last_idx = clusters[-1][-1][0]
            index_gap = (sample_count - 1 - last_idx) + first_idx
            if (
                index_gap <= max(1, self.max_index_gap)
                and abs(first_value - last_value) <= max(0.02, self.cluster_jump_threshold_m)
            ):
                merged = clusters[-1] + clusters[0]
                clusters = [merged] + clusters[1:-1]

        return clusters

    def _find_intruder_from_lidar(self, msg: LaserScan, points):
        close_candidates = [
            (idx, value)
            for idx, value in points
            if self.intruder_min_distance <= value <= self.threshold
        ]
        clusters = self._cluster_candidates(close_candidates, len(msg.ranges))

        intruder_detected = False
        closest = self.threshold

        for cluster in clusters:
            cluster_size = len(cluster)
            if cluster_size < max(1, self.min_cluster_points):
                continue

            width_deg = cluster_size * abs(msg.angle_increment) * 180.0 / math.pi
            if width_deg < self.min_intruder_width_deg or width_deg > self.max_intruder_width_deg:
                continue

            mean_distance = sum(value for _, value in cluster) / float(cluster_size)
            width_m = mean_distance * (cluster_size * abs(msg.angle_increment))
            if width_m < self.min_intruder_width_m or width_m > self.max_intruder_width_m:
                continue

            distances = [value for _, value in cluster]
            cluster_min = min(distances)
            cluster_max = max(distances)
            if (cluster_max - cluster_min) > max(0.1, self.max_cluster_range_spread_m):
                continue

            intruder_detected = True
            if cluster_min < closest:
                closest = cluster_min

        return intruder_detected, closest

    def _update_alert_with_hysteresis(self, intruder_detected: bool, closest: float, now: float, source: str):
        confirm_scans = max(1, self.confirm_scans)
        clear_scans = max(1, self.clear_scans)
        hold_alert_sec = max(0.0, self.hold_alert_sec)

        if intruder_detected:
            self.detection_streak += 1
            self.clear_streak = 0
            self.last_confirmed_distance = closest
            self.last_seen_time = now
        else:
            self.clear_streak += 1
            self.detection_streak = 0

        if intruder_detected and not self.last_alert and self.detection_streak >= confirm_scans:
            self._set_alert(True)
            self.last_alert_log_time = now
            self.last_source = source
            self.get_logger().warn(
                f'\a!!! INTRUDER ALERT !!! source={source} distance={self.last_confirmed_distance:.2f} m'
            )
            return

        missed_long_enough = (now - self.last_seen_time) >= hold_alert_sec
        if (
            (not intruder_detected)
            and self.last_alert
            and self.clear_streak >= clear_scans
            and missed_long_enough
        ):
            self._set_alert(False)
            self.get_logger().info('Intruder condition cleared')
            self.last_alert_log_time = 0.0
            return

        if self.last_alert and intruder_detected and (now - self.last_alert_log_time) >= max(0.2, self.alert_repeat_sec):
            self.last_alert_log_time = now
            self.last_source = source
            self.get_logger().warn(
                f'\a!!! INTRUDER ALERT (ACTIVE) !!! source={source} distance={self.last_confirmed_distance:.2f} m'
            )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

        if not self.sensors_enabled:
            if self.last_alert:
                self._set_alert(False)
                self.get_logger().info('Intruder monitoring disabled. Alert reset.')
            self._reset_state()
            self.last_alert_log_time = 0.0
            return

        if self.control_mode == 'MANUAL' and not self.detect_in_manual:
            if self.last_alert:
                self._set_alert(False)
                self.last_alert_log_time = 0.0
            self._reset_state()
            return

        now = time.monotonic()
        intruder_detected = False
        closest = self.last_confirmed_distance
        source = 'none'

        if self.use_model_state_detection:
            model_detection = self._find_intruder_from_model_states(now)
            if model_detection is None:
                if self.fallback_to_lidar:
                    points = self._valid_samples(msg)
                    if points:
                        intruder_detected, closest = self._find_intruder_from_lidar(msg, points)
                    source = 'lidar-fallback'
                    if not self._fallback_warned:
                        self.get_logger().warn(
                            'No /gazebo/model_states publisher detected. Using LiDAR fallback intruder detection.'
                        )
                        self._fallback_warned = True
                else:
                    if not self._model_state_warned:
                        self.get_logger().warn(
                            'Model-state intruder detection enabled but no recent /gazebo/model_states data.'
                        )
                        self._model_state_warned = True
                    intruder_detected = False
                    closest = self.last_confirmed_distance
                    source = 'model-states-missing'
            else:
                intruder_detected, closest, target_name = model_detection
                source = f'model:{target_name}' if target_name else 'model:none'
        else:
            points = self._valid_samples(msg)
            if points:
                intruder_detected, closest = self._find_intruder_from_lidar(msg, points)
            source = 'lidar'

        self._update_alert_with_hysteresis(intruder_detected, closest, now, source)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
