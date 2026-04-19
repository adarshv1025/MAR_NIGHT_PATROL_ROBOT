import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray, String


class ControlGuiNode(Node):

    def __init__(self):
        super().__init__('control_gui')

        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('heartbeat_hz', 5.0)
        self.declare_parameter('max_linear', 0.8)
        self.declare_parameter('max_angular', 1.8)
        self.declare_parameter('linear_accel_limit', 1.25)
        self.declare_parameter('angular_accel_limit', 3.2)

        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.heartbeat_hz = float(self.get_parameter('heartbeat_hz').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.linear_accel_limit = float(self.get_parameter('linear_accel_limit').value)
        self.angular_accel_limit = float(self.get_parameter('angular_accel_limit').value)

        self.manual_pub = self.create_publisher(Bool, '/manual_mode', 10)
        self.kill_pub = self.create_publisher(Bool, '/kill_switch', 10)
        self.sensor_pub = self.create_publisher(Bool, '/sensor_toggle', 10)
        self.speed_pub = self.create_publisher(Float32, '/patrol_speed_scale', 10)
        self.path_pub = self.create_publisher(Float32MultiArray, '/patrol_waypoints', 10)
        self.cmd_pub = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        self.heartbeat_pub = self.create_publisher(Empty, '/teleop_heartbeat', 10)

        self.create_subscription(String, '/control_mode', self._control_mode_cb, 10)
        self.create_subscription(Bool, '/intruder_alert', self._intruder_cb, 10)
        self.create_subscription(Bool, '/emergency_stop', self._estop_cb, 10)
        self.create_subscription(String, '/system_status', self._system_status_cb, 10)

        self._lock = threading.Lock()
        self._pressed_keys = set()
        self._manual_enabled = False
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._last_tick = time.monotonic()

        self._status_control_mode = 'AUTO'
        self._status_intruder = False
        self._status_estop = False
        self._status_text = 'INIT'

        self.create_timer(1.0 / max(self.publish_hz, 5.0), self._control_tick)
        self.create_timer(1.0 / max(self.heartbeat_hz, 1.0), self._heartbeat_tick)
        self.create_timer(0.5, self._manual_keepalive_tick)

        self.get_logger().info('Control GUI node ready.')

    def _control_mode_cb(self, msg: String):
        with self._lock:
            self._status_control_mode = msg.data

    def _intruder_cb(self, msg: Bool):
        with self._lock:
            self._status_intruder = msg.data

    def _estop_cb(self, msg: Bool):
        with self._lock:
            self._status_estop = msg.data

    def _system_status_cb(self, msg: String):
        with self._lock:
            self._status_text = msg.data

    def _ramp(self, current: float, target: float, max_delta: float) -> float:
        if target > current:
            return min(target, current + max_delta)
        return max(target, current - max_delta)

    def _publish_zero_cmd(self):
        self.cmd_pub.publish(Twist())

    def set_manual_mode(self, enabled: bool):
        with self._lock:
            self._manual_enabled = enabled
            if not enabled:
                self._pressed_keys.clear()
        self.manual_pub.publish(Bool(data=enabled))
        if not enabled:
            self._publish_zero_cmd()

    def set_kill_switch(self, enabled: bool):
        self.kill_pub.publish(Bool(data=enabled))

    def set_sensors(self, enabled: bool):
        self.sensor_pub.publish(Bool(data=enabled))

    def set_speed_scale(self, value: float):
        value = max(0.1, min(2.0, float(value)))
        self.speed_pub.publish(Float32(data=value))

    def set_path_preset(self, preset: str):
        if preset == 'square':
            points = [-3.0, -3.0, 3.0, -3.0, 3.0, 3.0, -3.0, 3.0]
        elif preset == 'rectangle':
            points = [-4.0, -2.0, 4.0, -2.0, 4.0, 2.0, -4.0, 2.0]
        else:
            points = [0.0, -3.5, 3.5, 0.0, 0.0, 3.5, -3.5, 0.0]
        self.path_pub.publish(Float32MultiArray(data=points))

    def set_key_state(self, key: str, pressed: bool):
        if key not in {'w', 'a', 's', 'd'}:
            return
        with self._lock:
            if pressed:
                self._pressed_keys.add(key)
                self._manual_enabled = True
            else:
                self._pressed_keys.discard(key)

    def emergency_stop_motion(self):
        with self._lock:
            self._pressed_keys.clear()
            self._current_linear = 0.0
            self._current_angular = 0.0
        self._publish_zero_cmd()

    def get_status_snapshot(self):
        with self._lock:
            return {
                'control_mode': self._status_control_mode,
                'intruder': self._status_intruder,
                'estop': self._status_estop,
                'system_status': self._status_text,
                'manual_enabled': self._manual_enabled,
            }

    def _manual_keepalive_tick(self):
        with self._lock:
            manual = self._manual_enabled
        self.manual_pub.publish(Bool(data=manual))

    def _heartbeat_tick(self):
        with self._lock:
            manual = self._manual_enabled
        if manual:
            self.heartbeat_pub.publish(Empty())

    def _control_tick(self):
        now = time.monotonic()
        dt = max(0.001, now - self._last_tick)
        self._last_tick = now

        with self._lock:
            keys = set(self._pressed_keys)
            manual = self._manual_enabled

            target_linear = 0.0
            target_angular = 0.0

            if 'w' in keys and 's' not in keys:
                target_linear = self.max_linear
            elif 's' in keys and 'w' not in keys:
                target_linear = -self.max_linear

            if 'a' in keys and 'd' not in keys:
                target_angular = self.max_angular
            elif 'd' in keys and 'a' not in keys:
                target_angular = -self.max_angular

            self._current_linear = self._ramp(
                self._current_linear,
                target_linear,
                self.linear_accel_limit * dt,
            )
            self._current_angular = self._ramp(
                self._current_angular,
                target_angular,
                self.angular_accel_limit * dt,
            )

            current_linear = self._current_linear
            current_angular = self._current_angular

        if not manual:
            return

        cmd = Twist()
        cmd.linear.x = current_linear
        cmd.angular.z = current_angular
        self.cmd_pub.publish(cmd)

    def shutdown_cleanup(self):
        self.emergency_stop_motion()
        self.set_manual_mode(False)


class ControlGuiApp:

    def __init__(self, root: tk.Tk, node: ControlGuiNode):
        self.root = root
        self.node = node

        self.root.title('Night Patrol Control Center')
        self.root.geometry('980x620')
        self.root.minsize(900, 560)

        self._build_ui()
        self._bind_keyboard()
        self._schedule_status_refresh()

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)
        main.rowconfigure(1, weight=1)

        mode_box = ttk.LabelFrame(main, text='Mode and Safety', padding=10)
        mode_box.grid(row=0, column=0, sticky='nsew', padx=6, pady=6)

        ttk.Button(mode_box, text='AUTO', command=lambda: self.node.set_manual_mode(False)).grid(row=0, column=0, padx=5, pady=5, sticky='ew')
        ttk.Button(mode_box, text='MANUAL', command=lambda: self.node.set_manual_mode(True)).grid(row=0, column=1, padx=5, pady=5, sticky='ew')

        ttk.Button(mode_box, text='EMERGENCY STOP', command=self._emergency_on).grid(row=1, column=0, padx=5, pady=5, sticky='ew')
        ttk.Button(mode_box, text='RELEASE E-STOP', command=self._emergency_off).grid(row=1, column=1, padx=5, pady=5, sticky='ew')

        ttk.Button(mode_box, text='Sensors ON', command=lambda: self.node.set_sensors(True)).grid(row=2, column=0, padx=5, pady=5, sticky='ew')
        ttk.Button(mode_box, text='Sensors OFF', command=lambda: self.node.set_sensors(False)).grid(row=2, column=1, padx=5, pady=5, sticky='ew')

        for col in range(2):
            mode_box.columnconfigure(col, weight=1)

        tweak_box = ttk.LabelFrame(main, text='Patrol Tweaks', padding=10)
        tweak_box.grid(row=0, column=1, sticky='nsew', padx=6, pady=6)

        ttk.Label(tweak_box, text='Speed Scale').grid(row=0, column=0, sticky='w')
        self.speed_var = tk.DoubleVar(value=1.3)
        self.speed_slider = ttk.Scale(tweak_box, from_=0.1, to=2.0, orient=tk.HORIZONTAL, variable=self.speed_var)
        self.speed_slider.grid(row=1, column=0, sticky='ew', padx=5)
        ttk.Button(tweak_box, text='Apply Speed', command=self._apply_speed).grid(row=1, column=1, padx=5)

        ttk.Label(tweak_box, text='Path Presets').grid(row=2, column=0, sticky='w', pady=(10, 0))
        path_row = ttk.Frame(tweak_box)
        path_row.grid(row=3, column=0, columnspan=2, sticky='ew')
        ttk.Button(path_row, text='Square', command=lambda: self.node.set_path_preset('square')).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(path_row, text='Rectangle', command=lambda: self.node.set_path_preset('rectangle')).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(path_row, text='Diamond', command=lambda: self.node.set_path_preset('diamond')).pack(side=tk.LEFT, padx=4, pady=4)

        tweak_box.columnconfigure(0, weight=1)

        drive_box = ttk.LabelFrame(main, text='Manual Drive (WASD or Buttons)', padding=10)
        drive_box.grid(row=1, column=0, sticky='nsew', padx=6, pady=6)

        up = ttk.Button(drive_box, text='W', width=8)
        left = ttk.Button(drive_box, text='A', width=8)
        down = ttk.Button(drive_box, text='S', width=8)
        right = ttk.Button(drive_box, text='D', width=8)
        stop = ttk.Button(drive_box, text='STOP', command=self.node.emergency_stop_motion)

        up.grid(row=0, column=1, padx=6, pady=6)
        left.grid(row=1, column=0, padx=6, pady=6)
        down.grid(row=1, column=1, padx=6, pady=6)
        right.grid(row=1, column=2, padx=6, pady=6)
        stop.grid(row=2, column=1, padx=6, pady=6, sticky='ew')

        self._bind_hold_button(up, 'w')
        self._bind_hold_button(left, 'a')
        self._bind_hold_button(down, 's')
        self._bind_hold_button(right, 'd')

        status_box = ttk.LabelFrame(main, text='Live Status', padding=10)
        status_box.grid(row=1, column=1, sticky='nsew', padx=6, pady=6)

        self.mode_label = ttk.Label(status_box, text='Control Mode: AUTO')
        self.mode_label.pack(anchor='w', pady=3)

        self.manual_label = ttk.Label(status_box, text='GUI Manual: OFF')
        self.manual_label.pack(anchor='w', pady=3)

        self.estop_label = ttk.Label(status_box, text='Emergency Stop: INACTIVE')
        self.estop_label.pack(anchor='w', pady=3)

        self.intruder_label = ttk.Label(status_box, text='Intruder Alert: NO')
        self.intruder_label.pack(anchor='w', pady=3)

        self.status_label = ttk.Label(status_box, text='System Status: INIT', wraplength=420, justify=tk.LEFT)
        self.status_label.pack(anchor='w', pady=3)

    def _bind_hold_button(self, button: ttk.Button, key: str):
        button.bind('<ButtonPress-1>', lambda _e: self.node.set_key_state(key, True))
        button.bind('<ButtonRelease-1>', lambda _e: self.node.set_key_state(key, False))

    def _bind_keyboard(self):
        self.root.bind('<KeyPress>', self._on_key_press)
        self.root.bind('<KeyRelease>', self._on_key_release)
        self.root.focus_set()

    def _to_wasd(self, keysym: str):
        key = keysym.lower()
        mapping = {
            'up': 'w',
            'left': 'a',
            'down': 's',
            'right': 'd',
        }
        return mapping.get(key, key)

    def _on_key_press(self, event):
        key = self._to_wasd(event.keysym)
        if key in {'w', 'a', 's', 'd'}:
            self.node.set_key_state(key, True)
            return
        if key in {'space', 'x'}:
            self.node.emergency_stop_motion()
            return
        if key == 'q':
            self.node.set_manual_mode(False)

    def _on_key_release(self, event):
        key = self._to_wasd(event.keysym)
        if key in {'w', 'a', 's', 'd'}:
            self.node.set_key_state(key, False)

    def _apply_speed(self):
        self.node.set_speed_scale(self.speed_var.get())

    def _emergency_on(self):
        self.node.emergency_stop_motion()
        self.node.set_kill_switch(True)

    def _emergency_off(self):
        self.node.set_kill_switch(False)

    def _schedule_status_refresh(self):
        snap = self.node.get_status_snapshot()
        self.mode_label.configure(text=f"Control Mode: {snap['control_mode']}")
        self.manual_label.configure(text=f"GUI Manual: {'ON' if snap['manual_enabled'] else 'OFF'}")
        self.estop_label.configure(text=f"Emergency Stop: {'ACTIVE' if snap['estop'] else 'INACTIVE'}")
        self.intruder_label.configure(text=f"Intruder Alert: {'YES' if snap['intruder'] else 'NO'}")
        self.status_label.configure(text=f"System Status: {snap['system_status']}")

        self.root.after(120, self._schedule_status_refresh)


def main(args=None):
    rclpy.init(args=args)
    node = ControlGuiNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spinner_stop = threading.Event()

    def spin_worker():
        while rclpy.ok() and not spinner_stop.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin_worker, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = ControlGuiApp(root, node)

    def on_close():
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)

    try:
        root.mainloop()
    finally:
        spinner_stop.set()
        node.shutdown_cleanup()
        executor.shutdown()
        spin_thread.join(timeout=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()