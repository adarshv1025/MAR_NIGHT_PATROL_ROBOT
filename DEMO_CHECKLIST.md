# Demo Readiness Checklist

## Week 1 (Basic Skeleton)

- [x] Robot model ready (base + wheels + LiDAR + optional camera)
- [x] Gazebo night world loads
- [x] Robot spawns in world
- [x] Manual control works through teleop + mux

## Week 2 (Intelligence Layer)

- [x] Waypoint patrol path working (loop path)
- [x] Random deviation added for realism
- [x] Obstacle avoidance from LiDAR (front-sector logic)
- [x] Sensor integration (`/scan`, optional camera)
- [x] Basic intruder detection (distance threshold on LiDAR)

## Week 3 (Polish + Rubric Optimization)

- [x] Emergency stop via kill switch topic
- [x] Restart/recovery logic (node respawn + heartbeat-aware recovery)
- [x] Optional UI (terminal dashboard)
- [x] Flexibility tweaks for demo
- [x] Change patrol path live
- [x] Adjust speed live
- [x] Toggle sensors live

## Final Demo Sequence (Suggested)

1. Launch full stack (GUI mode for audience).
2. Let autonomous patrol run for one loop.
3. Trigger obstacle interaction and show avoidance behavior.
4. Show fake intruder alert from `/intruder_alert`.
5. Toggle manual mode and drive with keyboard.
6. Engage kill switch and verify hard stop.
7. Release kill switch and show recovery resume.
8. Use dashboard commands to change path and speed in real time.
