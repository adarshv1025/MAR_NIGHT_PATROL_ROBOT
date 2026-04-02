#!/usr/bin/env python3
"""
ROS2 Night Patrol Robot Controller
===================================
A comprehensive navigation system for autonomous patrol with obstacle avoidance.

Features:
- Waypoint-based navigation with looping
- Odometry-based position tracking
- LiDAR-based obstacle avoidance
- Random deviation for realistic motion
- Configurable parameters
- Status publishing with timeout detection
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import random
from enum import Enum


def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    
    Args:
        quaternion: list [x, y, z, w]
    
    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    x, y, z, w = quaternion
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class PatrolState(Enum):
    """Robot operational states"""
    IDLE = "IDLE"
    MOVING = "MOVING"
    AVOIDING = "AVOIDING"


class PatrolController(Node):
    """
    Main patrol controller node for autonomous navigation.
    
    Implements waypoint following, obstacle avoidance, and status reporting.
    """
    
    def __init__(self):
        super().__init__('patrol_controller')
        
        # Declare and get parameters
        self._declare_parameters()
        self._load_parameters()
        
        # State variables
        self.current_waypoint_index = 0
        self.state = PatrolState.IDLE
        self.scan_data = None
        self.scan_received = False
        self.last_scan_time = self.get_clock().now()
        
        # Odometry variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        # Random deviation
        self.noise_timer_counter = 0
        self.current_noise_linear = 0.0
        self.current_noise_angular = 0.0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/patrol_status', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Main control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Status publishing timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Patrol Controller initialized')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')
        self.get_logger().info(f'Obstacle threshold: {self.obstacle_threshold} m')
        self.get_logger().info(f'Noise enabled: {self.noise_enabled}')
        self.get_logger().info('Odometry support: ENABLED')
        
    def _declare_parameters(self):
        """Declare all ROS2 parameters with default values"""
        # Waypoints as a flat list [x1, y1, x2, y2, ...]
        self.declare_parameter('waypoints', [0.0, 0.0, 5.0, 0.0, 5.0, 5.0, 0.0, 5.0])
        
        # Speed parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        
        # Navigation parameters
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('obstacle_threshold', 0.8)
        
        # Noise parameters
        self.declare_parameter('noise_enabled', False)
        self.declare_parameter('noise_magnitude', 0.05)
        
        # Safety parameters
        self.declare_parameter('scan_timeout', 2.0)  # seconds
        
    def _load_parameters(self):
        """Load parameters from parameter server"""
        waypoints_flat = self.get_parameter('waypoints').value
        
        # Convert flat list to list of tuples
        self.waypoints = []
        for i in range(0, len(waypoints_flat), 2):
            if i + 1 < len(waypoints_flat):
                self.waypoints.append((waypoints_flat[i], waypoints_flat[i + 1]))
        
        if not self.waypoints:
            self.get_logger().warn('No waypoints configured, using default square')
            self.waypoints = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.noise_enabled = self.get_parameter('noise_enabled').value
        self.noise_magnitude = self.get_parameter('noise_magnitude').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        
    def scan_callback(self, msg: LaserScan):
        """
        Callback for LiDAR scan data.
        
        Args:
            msg: LaserScan message from /scan topic
        """
        self.scan_data = msg
        self.scan_received = True
        self.last_scan_time = self.get_clock().now()
    
    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry data.
        
        Args:
            msg: Odometry message from /odom topic
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        
        self.odom_received = True
        
    def check_obstacle(self) -> bool:
        """
        Check if there's an obstacle in the front sector of the robot.
        
        Returns:
            bool: True if obstacle detected within threshold, False otherwise
        """
        if not self.scan_received or self.scan_data is None:
            # No scan data, assume clear (fail-safe)
            return False
        
        # Check for scan timeout
        time_since_scan = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if time_since_scan > self.scan_timeout:
            self.get_logger().warn(
                f'Scan data timeout ({time_since_scan:.2f}s). Proceeding cautiously.',
                throttle_duration_sec=5.0
            )
            return False
        
        # Check front sector (±30 degrees)
        # LaserScan typically has 0 degrees at front
        ranges = self.scan_data.ranges
        num_points = len(ranges)
        
        if num_points == 0:
            return False
        
        # Calculate indices for ±30 degrees (60 degree sector)
        angle_range = 30.0  # degrees on each side
        angle_increment = math.degrees(self.scan_data.angle_increment)
        indices_range = int(angle_range / angle_increment)
        
        # Front center indices
        front_center = num_points // 2
        start_idx = max(0, front_center - indices_range)
        end_idx = min(num_points, front_center + indices_range)
        
        # Also check wrapping for 360-degree scans (0 degrees at front)
        # Check both beginning and end of array
        check_indices = list(range(start_idx, end_idx))
        
        # Add wraparound indices (first and last 30 degrees)
        if num_points > indices_range * 2:
            check_indices.extend(range(0, min(indices_range, num_points)))
            check_indices.extend(range(max(0, num_points - indices_range), num_points))
        
        # Check for obstacles in front sector
        for idx in check_indices:
            if 0 <= idx < len(ranges):
                distance = ranges[idx]
                # Check if valid reading and within threshold
                if (self.scan_data.range_min < distance < self.scan_data.range_max and
                    distance < self.obstacle_threshold):
                    return True
        
        return False
    
    def get_current_waypoint(self) -> tuple:
        """
        Get the current target waypoint.
        
        Returns:
            tuple: (x, y) coordinates of current waypoint
        """
        return self.waypoints[self.current_waypoint_index]
    
    def distance_to_waypoint(self, x: float, y: float) -> float:
        """
        Calculate Euclidean distance to target waypoint.
        
        Args:
            x: Current x position
            y: Current y position
            
        Returns:
            float: Distance to current waypoint
        """
        wx, wy = self.get_current_waypoint()
        return math.sqrt((wx - x) ** 2 + (wy - y) ** 2)
    
    def angle_to_waypoint(self, current_x: float, current_y: float, 
                          current_yaw: float) -> float:
        """
        Calculate angle error to target waypoint.
        
        Args:
            current_x: Current x position
            current_y: Current y position
            current_yaw: Current orientation (radians)
            
        Returns:
            float: Angle error in radians [-pi, pi]
        """
        wx, wy = self.get_current_waypoint()
        
        # Calculate desired angle
        desired_angle = math.atan2(wy - current_y, wx - current_x)
        
        # Calculate angle error
        angle_error = desired_angle - current_yaw
        
        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        return angle_error
    
    def apply_noise(self):
        """
        Apply random noise to movement for realistic motion.
        Updates current noise values periodically.
        """
        if not self.noise_enabled:
            self.current_noise_linear = 0.0
            self.current_noise_angular = 0.0
            return
        
        # Update noise every 2 seconds (40 control cycles at 20Hz)
        self.noise_timer_counter += 1
        if self.noise_timer_counter >= 40:
            self.noise_timer_counter = 0
            self.current_noise_linear = random.uniform(
                -self.noise_magnitude, 
                self.noise_magnitude
            )
            self.current_noise_angular = random.uniform(
                -self.noise_magnitude * 2, 
                self.noise_magnitude * 2
            )
    
    def move_to_goal(self) -> Twist:
        """
        Calculate velocity command to move toward current waypoint.
        Uses proportional controller with odometry feedback.
        
        Returns:
            Twist: Velocity command
        """
        cmd = Twist()
        
        if not self.odom_received:
            # No odometry yet, just move forward slowly
            cmd.linear.x = self.linear_speed * 0.5
            cmd.angular.z = 0.0
            return cmd
        
        # Calculate angle error to waypoint
        angle_error = self.angle_to_waypoint(self.current_x, self.current_y, self.current_yaw)
        
        # Calculate distance to waypoint
        distance = self.distance_to_waypoint(self.current_x, self.current_y)
        
        # Proportional controller gains
        k_angular = 2.0  # Angular velocity gain
        k_linear = 0.5   # Linear velocity gain
        
        # If we need to turn significantly, slow down or stop
        if abs(angle_error) > 0.5:  # ~30 degrees
            # Turn in place
            cmd.linear.x = 0.0
            cmd.angular.z = k_angular * angle_error
        else:
            # Move forward while adjusting heading
            cmd.linear.x = min(self.linear_speed, k_linear * distance)
            cmd.angular.z = k_angular * angle_error
        
        # Apply noise if enabled
        self.apply_noise()
        cmd.linear.x += self.current_noise_linear
        cmd.angular.z += self.current_noise_angular
        
        # Ensure velocities are within limits
        cmd.linear.x = max(-self.linear_speed, min(self.linear_speed, cmd.linear.x))
        cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))
        
        return cmd
    
    def avoid_obstacle(self) -> Twist:
        """
        Calculate velocity command to avoid obstacles.
        Rotates in place until path is clear.
        
        Returns:
            Twist: Velocity command (rotation only)
        """
        cmd = Twist()
        cmd.linear.x = 0.0  # Stop forward motion
        cmd.angular.z = self.angular_speed * 0.7  # Rotate to find clear path
        
        return cmd
    
    def stop(self) -> Twist:
        """
        Create stop command.
        
        Returns:
            Twist: Zero velocity command
        """
        return Twist()
    
    def check_waypoint_reached(self) -> bool:
        """
        Check if current waypoint is reached using odometry.
        
        Returns:
            bool: True if waypoint reached
        """
        if not self.odom_received:
            # No odometry data yet
            return False
        
        # Calculate distance to current waypoint
        distance = self.distance_to_waypoint(self.current_x, self.current_y)
        
        # Check if within tolerance
        return distance < self.waypoint_tolerance
    
    def advance_waypoint(self):
        """Advance to next waypoint in the patrol route"""
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
        wx, wy = self.get_current_waypoint()
        self.get_logger().info(
            f'Waypoint {self.current_waypoint_index} reached. '
            f'Moving to waypoint {self.current_waypoint_index}: ({wx:.2f}, {wy:.2f})'
        )
    
    def control_loop(self):
        """
        Main control loop - executes at 20 Hz.
        Implements state machine for patrol behavior.
        """
        cmd = Twist()
        
        # Check for obstacles
        obstacle_detected = self.check_obstacle()
        
        if obstacle_detected and self.state != PatrolState.AVOIDING:
            # Transition to obstacle avoidance
            self.state = PatrolState.AVOIDING
            self.get_logger().warn(
                'Obstacle detected! Initiating avoidance maneuver.',
                throttle_duration_sec=2.0
            )
        
        # State machine
        if self.state == PatrolState.IDLE:
            # Start patrol
            self.state = PatrolState.MOVING
            self.get_logger().info('Starting patrol...')
            cmd = self.move_to_goal()
            
        elif self.state == PatrolState.MOVING:
            if obstacle_detected:
                # Handled above - will transition next cycle
                cmd = self.stop()
            else:
                # Continue moving to waypoint
                cmd = self.move_to_goal()
                
                # Check if waypoint reached with odometry
                if self.check_waypoint_reached():
                    self.advance_waypoint()
                
        elif self.state == PatrolState.AVOIDING:
            if not obstacle_detected:
                # Path clear, resume patrol
                self.state = PatrolState.MOVING
                self.get_logger().info('Path clear. Resuming patrol.')
                cmd = self.move_to_goal()
            else:
                # Continue avoiding
                cmd = self.avoid_obstacle()
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)
    
    def publish_status(self):
        """Publish current patrol status with scan timeout detection"""
        status_msg = String()
        wx, wy = self.get_current_waypoint()
        
        # Determine scan status
        if not self.scan_received:
            scan_status = "NO DATA"
        else:
            time_since_scan = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
            if time_since_scan > self.scan_timeout:
                scan_status = "TIMEOUT"
            else:
                scan_status = "OK"
        
        # Determine odometry status
        odom_status = "OK" if self.odom_received else "NO ODOM"
        
        # Add current position if odometry available
        if self.odom_received:
            position_str = f' | Pos: ({self.current_x:.2f}, {self.current_y:.2f})'
        else:
            position_str = ''
        
        status_msg.data = (
            f'State: {self.state.value} | '
            f'Waypoint: {self.current_waypoint_index}/{len(self.waypoints)-1} '
            f'({wx:.2f}, {wy:.2f}){position_str} | '
            f'Scan: {scan_status} | Odom: {odom_status}'
        )
        self.status_pub.publish(status_msg)
    
    def destroy_node(self):
        """Clean shutdown"""
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('Patrol Controller shutting down')
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = PatrolController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
