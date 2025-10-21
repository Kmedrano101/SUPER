# Quick Start: SUPER + FAST-LIO + PX4 Integration

**Get up and running in 5 minutes**

---

## Complete Data Flow

```
GAZEBO SENSORS → FAST-LIO → ROG-MAP → SUPER → (px4_super_bridge) → PX4
```

---

## Topic Connections Summary

| Source | Topic | → | Destination | Message Type |
|--------|-------|---|-------------|--------------|
| **Gazebo** | `/sim_lidar/lidar` | → | **FAST-LIO** | PointCloud2 |
| **Gazebo** | `/sim_imu/imu` | → | **FAST-LIO** | Imu |
| **FAST-LIO** | `/Odometry` | → | **ROG-MAP** | Odometry @ 40 Hz |
| **FAST-LIO** | `/cloud_registered` | → | **ROG-MAP** | PointCloud2 @ 10 Hz |
| **ROG-MAP** | *(API calls)* | → | **SUPER** | *(no topics)* |
| **Mission** | `/goal_pose` | → | **SUPER** | PoseStamped |
| **SUPER** | `/planning/pos_cmd` | → | **Bridge** | PositionCommand @ 100 Hz |

---

## Launch Sequence (4 Terminals)

### Terminal 1: Gazebo + PX4
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch px4_offboard_sim simulation.launch.py
```
**Wait for**: Gazebo GUI, vehicle spawned

---

### Terminal 2: FAST-LIO
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch fast_lio_ros2 simulation_mapping.launch.py config_file:=gazebosim.yaml
```
**Wait for**: "Publishing to /Odometry" message

---

### Terminal 3: SUPER Planner
```bash
cd ~/super_ws
source install/setup.bash
ros2 run super_planner fsm_node \
  --ros-args \
  --params-file ~/super_ws/src/SUPER/super_planner/config/px4_integration.yaml
```
**Wait for**: "Odometry received! Building map..." message

---

### Terminal 4: Send Test Goal
```bash
source ~/super_ws/install/setup.bash

# Send a goal 5 meters ahead
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "camera_init"},
  pose: {position: {x: 5.0, y: 0.0, z: 1.5},
         orientation: {w: 1.0}}}'
```
**Expected**: Commands start publishing at `/planning/pos_cmd`

---

## Quick Verification

### Check all topics are connected:
```bash
# In a new terminal
ros2 topic hz /sim_lidar/lidar       # Should show ~10 Hz
ros2 topic hz /Odometry              # Should show ~40 Hz
ros2 topic hz /cloud_registered      # Should show ~10 Hz
ros2 topic hz /planning/pos_cmd      # Should show ~100 Hz (after goal sent)
```

### Visual check in RViz:
```bash
rviz2
```
- Set Fixed Frame: `camera_init`
- Add: Odometry (`/Odometry`), PointCloud2 (`/cloud_registered`), Path (`/fsm/path`)

---

## Configuration Files

All settings in one place: `super_planner/config/px4_integration.yaml`

**Key settings already configured**:
- ✅ ROG-MAP subscribes to `/Odometry` (from FAST-LIO)
- ✅ ROG-MAP subscribes to `/cloud_registered` (from FAST-LIO)
- ✅ Safety limits set for PX4 (max_vel: 5 m/s, max_acc: 5 m/s²)
- ✅ Goal input from `/goal_pose` (RViz 2D Goal Pose)
- ✅ Commands published to `/planning/pos_cmd` (for bridge)

---

## Troubleshooting

### Problem: FAST-LIO not publishing odometry
**Check**: `ros2 topic list | grep -E "(sim_lidar|sim_imu)"`
**Solution**: Restart Gazebo, verify sensors in simulation

### Problem: SUPER waiting for odometry
**Check**: `ros2 topic echo /Odometry --once`
**Solution**: FAST-LIO must be running first

### Problem: No planning after goal
**Check**: SUPER logs for error messages
**Solution**: Move robot to build initial map, then send goal

---

## What's Working Now

✅ Sensor simulation (Gazebo LiDAR + IMU)
✅ Localization (FAST-LIO SLAM)
✅ Mapping (ROG-MAP occupancy grid)
✅ Planning (SUPER trajectory generation)
✅ Custom messages (super_planner/PositionCommand)

## What's Next

⏳ **px4_super_bridge**: Convert SUPER commands → PX4 setpoints
⏳ **Mission planner**: Waypoint sequencing
⏳ **Safety monitoring**: Geofencing, battery, failsafes

---

## Full Documentation

- **Complete topic map**: `INTEGRATION_TOPIC_MAP.md`
- **Detailed test procedure**: `ROG_MAP_INTEGRATION_TEST.md`
- **Architecture overview**: `ARCHITECTURE_BREAKDOWN.md`
- **Quick topic reference**: `TOPIC_MAP_QUICK_REFERENCE.md`

---

**Ready to test ROG-MAP integration!** 🚀

Start with Terminal 1, then 2, then 3, then send a goal from Terminal 4.

