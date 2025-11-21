# Quick Start: SUPER + FAST-LIO + PX4 Integration

**Get up and running in 5 minutes**

---

## 🎯 Full Navigation Mode (ENABLED)

**Current Configuration: Continuous Replanning Active**

The system is configured for **full autonomous navigation** with dynamic replanning:

✅ **What's ENABLED:**
- Goal point input (via `/goal_pose` topic or RViz)
- **Continuous replanning** at 10 Hz during trajectory following
- **Real-time path updates** as drone moves
- Dynamic trajectory optimization based on current position
- Automatic trajectory regeneration when not at goal
- Automatic retry on planning failure
- Command publishing to `/planning/pos_cmd` (at 100 Hz)

**How it works:**
1. Send a goal point to SUPER
2. SUPER generates initial trajectory to goal
3. **As you move the drone** (manually or autonomously):
   - Planner continuously replans trajectory from current position
   - Path adapts to drone's actual movement
   - Trajectory updates in real-time (10 Hz)
4. When trajectory completes:
   - If close to goal (< 0.1m): Waits for new goal
   - If far from goal: Automatically regenerates trajectory
5. If planning fails: Automatically retries until successful

**Perfect for:**
- Manual keyboard control + autonomous path planning
- Testing dynamic replanning with obstacle avoidance
- Preparing for full mission planner integration

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
ros2 launch px4_offboard_sim slam_simulation.launch.py
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
ros2 launch super_planner test_super_planner.launch.py
```
**Wait for**: "Odometry received! Building map..." message

**Note**: This launches SUPER with the `px4_integration.yaml` config by default. To use a different config:
```bash
ros2 launch super_planner test_super_planner.launch.py \
  config_file:=/path/to/your/config.yaml
```

---

### Terminal 4: Send Test Goal
```bash
source ~/super_ws/install/setup.bash

# Send a goal 5 meters ahead
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "world"},
  pose: {position: {x: 5.0, y: 0.0, z: 1.5},
         orientation: {w: 1.0}}}'
```
**Expected**: Commands start publishing at `/planning/pos_cmd`

**What you'll see in Quick Start Test Mode:**
1. SUPER generates the **initial path** from current position to goal
2. The trajectory is published to `/planning/pos_cmd` at 100 Hz
3. **The path will NOT update** as the robot moves (replanning disabled)
4. You can visualize the full trajectory in RViz
5. Send a new goal to generate a new path from the current position

**To test:**
```bash
# Monitor the generated commands
ros2 topic echo /planning/pos_cmd --once

# Check trajectory visualization
ros2 topic list | grep planning
```

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

**Option 1: Launch with SUPER (automatic)**
```bash
# RViz is launched automatically with Terminal 3 command above
# Uses super_test.rviz configuration
```

**Option 2: Launch RViz separately**
```bash
# In a new terminal
cd ~/super_ws
source install/setup.bash
rviz2 -d ~/super_ws/src/SUPER/super_planner/rviz/super_integration.rviz
```

**Option 3: Launch SUPER without RViz**
```bash
ros2 launch super_planner test_super_planner.launch.py rviz:=false
```

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

## Quick Launch Summary

**Sequential startup (recommended):**
1. Terminal 1: `ros2 launch px4_offboard_sim slam_simulation.launch.py`
2. Terminal 2: `ros2 launch fast_lio_ros2 simulation_mapping.launch.py config_file:=gazebosim.yaml`
3. Terminal 3: `ros2 launch super_planner test_super_planner.launch.py`
4. Terminal 4: Send goal via command line or use RViz "2D Goal Pose" tool

**Launch options for Terminal 3:**
```bash
# Default (with RViz using super_test.rviz)
ros2 launch super_planner test_super_planner.launch.py

# Without RViz
ros2 launch super_planner test_super_planner.launch.py rviz:=false

# Custom config file
ros2 launch super_planner test_super_planner.launch.py \
  config_file:=/path/to/custom.yaml

# All options combined
ros2 launch super_planner test_super_planner.launch.py \
  config_file:=/path/to/custom.yaml \
  rviz:=false \
  use_sim_time:=true
```

