import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random

class FakeScan(Node):
    def __init__(self):
        super().__init__('fake_scan')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(1.0, self.publish_scan)

    def publish_scan(self):
    	msg = LaserScan()

    	# 30% chance intruder
    	if random.random() < 0.3:
        	msg.ranges = [random.uniform(0.1, 0.4) for _ in range(360)]
    	else:
        	msg.ranges = [random.uniform(0.6, 2.0) for _ in range(360)]

    	self.pub.publish(msg)
    
def main():
    rclpy.init()
    node = FakeScan()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
