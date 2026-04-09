import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class IntruderDetector(Node):
    def __init__(self):
        super().__init__('intruder_detector')

        # Subscribe to LiDAR scan
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10
        )

        # Track previous state to avoid spamming
        self.intruder_present = False

        # Threshold distance (meters)
        self.threshold = 0.5

    def callback(self, msg):
        # Filter out invalid readings (0, inf, nan)
        valid_ranges = [
            r for r in msg.ranges
            if r > 0.0 and not math.isinf(r) and not math.isnan(r)
        ]

        if not valid_ranges:
            return

        min_dist = min(valid_ranges)

        # Intruder detected
        if min_dist < self.threshold:
            if not self.intruder_present:
                self.get_logger().warn("INTRUDER DETECTED!")
                self.intruder_present = True

        # Area safe
        else:
            if self.intruder_present:
                self.get_logger().info("Area is safe again!!")
                self.intruder_present = False
            else:
                self.get_logger().info(f"Safe. Closest object: {min_dist:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = IntruderDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
