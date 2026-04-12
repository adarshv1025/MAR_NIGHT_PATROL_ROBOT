import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray


class DashboardNode(Node):

    def __init__(self):
        super().__init__('demo_dashboard')

        self.manual_pub = self.create_publisher(Bool, '/manual_mode', 10)
        self.kill_pub = self.create_publisher(Bool, '/kill_switch', 10)
        self.sensor_pub = self.create_publisher(Bool, '/sensor_toggle', 10)
        self.speed_pub = self.create_publisher(Float32, '/patrol_speed_scale', 10)
        self.path_pub = self.create_publisher(Float32MultiArray, '/patrol_waypoints', 10)

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._menu_loop, daemon=True)
        self._thread.start()
        self.get_logger().info('Terminal dashboard ready. Type "help" for commands.')

    def _publish_bool(self, publisher, value: bool):
        publisher.publish(Bool(data=value))

    def _menu_loop(self):
        print('\n=== NIGHT PATROL DASHBOARD ===')
        print('Commands: help, auto, manual, kill, unkill, sensors on, sensors off, speed <0.1-2.0>, path square|rectangle|diamond, quit')

        while not self._stop_event.is_set() and rclpy.ok():
            try:
                raw = input('dashboard> ').strip().lower()
            except EOFError:
                raw = 'quit'

            if not raw:
                continue

            if raw == 'help':
                print('auto/manual            -> control mode')
                print('kill/unkill            -> emergency kill switch')
                print('sensors on/off         -> toggle LiDAR-based logic')
                print('speed <value>          -> patrol speed scale 0.1 to 2.0')
                print('path square|rectangle|diamond -> live waypoint update')
                print('quit                   -> close dashboard node')
                continue

            if raw == 'auto':
                self._publish_bool(self.manual_pub, False)
                print('Mode set: AUTO')
                continue

            if raw == 'manual':
                self._publish_bool(self.manual_pub, True)
                print('Mode set: MANUAL')
                continue

            if raw == 'kill':
                self._publish_bool(self.kill_pub, True)
                print('Kill switch: ON')
                continue

            if raw == 'unkill':
                self._publish_bool(self.kill_pub, False)
                print('Kill switch: OFF (recovery delay applies)')
                continue

            if raw == 'sensors on':
                self._publish_bool(self.sensor_pub, True)
                print('Sensors: ENABLED')
                continue

            if raw == 'sensors off':
                self._publish_bool(self.sensor_pub, False)
                print('Sensors: DISABLED')
                continue

            if raw.startswith('speed '):
                try:
                    value = float(raw.split()[1])
                    value = max(0.1, min(2.0, value))
                    self.speed_pub.publish(Float32(data=value))
                    print(f'Speed scale set: {value:.2f}')
                except (ValueError, IndexError):
                    print('Invalid speed value. Example: speed 1.3')
                continue

            if raw.startswith('path '):
                key = raw.split()[1] if len(raw.split()) > 1 else ''
                if key == 'square':
                    points = [-3.0, -3.0, 3.0, -3.0, 3.0, 3.0, -3.0, 3.0]
                elif key == 'rectangle':
                    points = [-4.0, -2.0, 4.0, -2.0, 4.0, 2.0, -4.0, 2.0]
                elif key == 'diamond':
                    points = [0.0, -3.5, 3.5, 0.0, 0.0, 3.5, -3.5, 0.0]
                else:
                    print('Unknown path preset. Use square, rectangle, or diamond.')
                    continue

                self.path_pub.publish(Float32MultiArray(data=points))
                print(f'Path updated: {key}')
                continue

            if raw == 'quit':
                print('Shutting down dashboard node...')
                self._stop_event.set()
                rclpy.shutdown()
                break

            print('Unknown command. Type help.')


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
