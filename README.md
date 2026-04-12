# ROS2 Night Patrol Robot (Gazebo)

This workspace now contains a full simulation pipeline for a night patrol robot with:

- Low-light Gazebo world
- Differential-drive robot model (Xacro)
- LiDAR scan simulation and optional camera stream
- Waypoint patrol with looped path and random deviation
- Basic LiDAR obstacle avoidance
- Fake intruder detection by distance threshold
- Emergency stop + kill switch + recovery logic
- Teleop keyboard support
- Optional terminal dashboard for live demo controls

## Workspace Layout

- `src/patrol_bot_description`: robot model (URDF/Xacro)
- `src/patrol_bot_gazebo`: world + full launch files
- `src/patrol_bot_navigation`: patrol and obstacle avoidance nodes
- `src/patrol_bot_detection`: intruder detection node
- `src/patrol_bot_control`: cmd mux, emergency manager, optional dashboard

## Build

```bash
cd ~/night_patrol_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Run Full Simulation

In every new terminal, source the workspace before launching:

```bash
cd ~/night_patrol_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Shortcut (recommended):

```bash
cd ~/night_patrol_ws
source setup.bash
```

Or use the wrapper script (auto-sources both underlay and overlay):

```bash
./run_night_patrol_demo.sh gui:=true start_teleop:=true use_dashboard:=true
```

Headless mode:

```bash
ros2 launch patrol_bot_gazebo night_patrol_demo.launch.py gui:=false
```

With Gazebo GUI:

```bash
ros2 launch patrol_bot_gazebo night_patrol_demo.launch.py gui:=true
```

With teleop node and dashboard enabled:

```bash
ros2 launch patrol_bot_gazebo night_patrol_demo.launch.py gui:=true start_teleop:=true use_dashboard:=true
```

Launch Gazebo + RViz together (LiDAR, TF, odometry preloaded in RViz):

```bash
ros2 launch patrol_bot_gazebo night_patrol_demo.launch.py gui:=true use_rviz:=true
```

## Teleop (Keyboard)

Smooth WASD teleop in a separate terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/night_patrol_ws/install/setup.bash
ros2 run patrol_bot_control wasd_teleop
```

WASD controls:

- `w` forward
- `s` reverse
- `a` turn left
- `d` turn right
- `space` or `x` smooth stop
- `q` quit teleop and return to auto mode

The `wasd_teleop` node automatically enables manual mode while running.
If the teleop process is killed or terminal is closed, command mux automatically falls back to AUTO patrol.

Switch mux to manual mode:

```bash
ros2 topic pub /manual_mode std_msgs/msg/Bool "{data: true}" -1
```

Switch back to auto patrol:

```bash
ros2 topic pub /manual_mode std_msgs/msg/Bool "{data: false}" -1
```

## Emergency Stop and Recovery

Engage kill switch:

```bash
ros2 topic pub /kill_switch std_msgs/msg/Bool "{data: true}" -1
```

Release kill switch (recovery delay applies):

```bash
ros2 topic pub /kill_switch std_msgs/msg/Bool "{data: false}" -1
```

## Live Demo Tweaks

Change patrol speed:

```bash
ros2 topic pub /patrol_speed_scale std_msgs/msg/Float32 "{data: 1.3}" -1
```

Change patrol waypoints live (flattened x,y list):

```bash
ros2 topic pub /patrol_waypoints std_msgs/msg/Float32MultiArray "{data: [0.0, -3.5, 3.5, 0.0, 0.0, 3.5, -3.5, 0.0]}" -1
```

Toggle sensor-based behaviors:

```bash
ros2 topic pub /sensor_toggle std_msgs/msg/Bool "{data: false}" -1
ros2 topic pub /sensor_toggle std_msgs/msg/Bool "{data: true}" -1
```

## Key Topics

- `/scan` (LaserScan)
- `/front_camera/image_raw` (Image, optional camera)
- `/patrol_cmd_vel` (raw patrol command)
- `/auto_cmd_vel` (after obstacle logic)
- `/teleop_cmd_vel` (manual keyboard command)
- `/cmd_vel` (final command to robot)
- `/intruder_alert` (Bool)
- `/kill_switch` (Bool)
- `/emergency_stop` (Bool)
- `/system_status` (String)

RViz profile intentionally excludes camera/image display.

## Recovery Logic Notes

- Critical nodes launch with `respawn=True`.
- Emergency manager watches patrol heartbeat.
- On heartbeat loss or kill switch: robot is forced to stop.
- On recovery: delay window is applied before movement resumes.
- Auto patrol now includes anti-stuck recovery: if forward progress stalls, the node skips to the next waypoint.
- Obstacle avoidance now uses slowdown + reverse-turn recovery to reduce collisions and escape tight spots.
- Obstacle avoidance keeps a larger stand-off distance and uses hysteresis to avoid rapid clear/avoid toggling.
- Intruder detection now repeats high-visibility terminal alerts while the intruder condition remains active.
- Intruder alerting now evaluates full 360 LiDAR and only triggers on intruder-like point clusters.
- Default intruder detection range is increased and remains active in MANUAL teleop mode as well.
