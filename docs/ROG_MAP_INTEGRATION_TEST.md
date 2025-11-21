# ROG-MAP Integration Test Procedure

**Testing FAST-LIO → ROG-MAP → SUPER Planner pipeline**

Date: 2025-10-21
System: Gazebo + PX4 + FAST-LIO + SUPER

---

## Prerequisites

- [x] Gazebo simulation with LiDAR and IMU sensors
- [x] FAST-LIO ROS2 installed and configured (`gazebosim.yaml`)
- [x] SUPER planner built successfully
- [x] ROG-MAP config updated (`px4_integration.yaml`)

---

## Test Sequence

### Phase 1: Sensor Data Flow (Gazebo → FAST-LIO)

#### Step 1.1: Launch Gazebo + PX4

```bash
# Terminal 1
cd ~/ros2_ws
source install/setup.bash
ros2 launch px4_offboard_sim simulation.launch.py
```

**Expected Output**:
- Gazebo GUI opens
- PX4 SITL starts
- Vehicle spawns in simulation

#### Step 1.2: Verify Sensor Topics

```bash
# Terminal 2 (new)
source ~/ros2_ws/install/setup.bash

# Check LiDAR
ros2 topic hz /sim_lidar/lidar
# Expected: ~10 Hz

# Check IMU
ros2 topic hz /sim_imu/imu
# Expected: ~250 Hz

# Inspect one LiDAR message
ros2 topic echo /sim_lidar/lidar --once | head -30
```

**✅ Pass Criteria**:
- `/sim_lidar/lidar` publishes at ~10 Hz
- `/sim_imu/imu` publishes at ~250 Hz
- Point cloud has > 1000 points
- Frame IDs are correct

---

### Phase 2: Localization (FAST-LIO)

#### Step 2.1: Launch FAST-LIO

```bash
# Terminal 3 (new)
cd ~/ros2_ws
source install/setup.bash
ros2 launch fast_lio_ros2 mapping.launch.py \
  config_file:=gazebosim.yaml
```

**Expected Output**:
```
[INFO] [fast_lio]: Initializing Fast-LIO...
[INFO] [fast_lio]: Subscribing to /sim_lidar/lidar
[INFO] [fast_lio]: Subscribing to /sim_imu/imu
[INFO] [fast_lio]: Publishing to /Odometry
[INFO] [fast_lio]: Publishing to /cloud_registered
```

#### Step 2.2: Verify FAST-LIO Outputs

```bash
# Check odometry
ros2 topic hz /Odometry
# Expected: 20-40 Hz

# Check registered cloud
ros2 topic hz /cloud_registered
# Expected: ~10 Hz

# Check path (optional)
ros2 topic hz /path
# Expected: ~10 Hz

# Inspect odometry
ros2 topic echo /Odometry --once
```

**✅ Pass Criteria**:
- `/Odometry` publishes at 20-40 Hz
- `/cloud_registered` publishes at ~10 Hz
- Odometry pose is reasonable (not NaN, not stuck at zero)
- TF `camera_init` → `body` is being broadcast

---

### Phase 3: Mapping (ROG-MAP)

#### Step 3.1: Launch SUPER with ROG-MAP

```bash
# Terminal 4 (new)
cd ~/super_ws
source install/setup.bash

ros2 launch super_planner test_super_planner.launch.py
```

**Alternative** (manual node execution):
```bash
ros2 run super_planner fsm_node \
  --ros-args \
  --params-file ~/super_ws/src/SUPER/super_planner/config/px4_integration.yaml \
  -r __node:=super_fsm
```

**Expected Output**:
```
[INFO] [super_fsm]: Loading config from px4_integration.yaml
[INFO] [super_fsm]: ROG-Map callback enabled
[INFO] [super_fsm]: Subscribing to /cloud_registered
[INFO] [super_fsm]: Subscribing to /Odometry
[INFO] [super_fsm]: Waiting for odometry...
[INFO] [super_fsm]: Odometry received! Building map...
```

#### Step 3.2: Verify ROG-MAP Subscriptions

```bash
# Terminal 5 (new)
source ~/super_ws/install/setup.bash

# Check node subscriptions
ros2 node info /super_fsm | grep -A20 "Subscriptions"

# Should show:
#   /Odometry
#   /cloud_registered
#   /planning/click_goal (or /goal_pose)
```

#### Step 3.3: Monitor ROG-MAP Operation

```bash
# Watch ROG-MAP visualization (if enabled)
ros2 topic hz /rog_map/occupancy
# Expected: 0.5-2 Hz (depends on config)

# Check if map is being updated
ros2 topic echo /rog_map/occupancy --once | grep "data" | head -5
```

**✅ Pass Criteria**:
- SUPER node subscribes to both `/Odometry` and `/cloud_registered`
- No errors about missing topics
- ROG-MAP visualization topics publish (if enabled)
- Logs show "Map updated" messages

---

### Phase 4: Planning Test

#### Step 4.1: Send a Test Goal

**Option A: Using command line**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{
  header: {
    frame_id: "camera_init"
  },
  pose: {
    position: {x: 5.0, y: 0.0, z: 1.5},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

**Option B: Using RViz (if running)**
1. Open RViz
2. Set Fixed Frame to `camera_init`
3. Add "2D Goal Pose" tool
4. Click on map to set goal

#### Step 4.2: Verify Planning Response

```bash
# Monitor FSM logs in Terminal 4
# Should see:
# [INFO] [super_fsm]: Goal received: (5.0, 0.0, 1.5)
# [INFO] [super_fsm]: Running A* search...
# [INFO] [super_fsm]: Path found! Generating corridors...
# [INFO] [super_fsm]: Optimizing trajectory...
# [INFO] [super_fsm]: Publishing commands...

# Check command output
ros2 topic hz /planning/pos_cmd
# Expected: ~100 Hz

# Inspect command
ros2 topic echo /planning/pos_cmd | head -50
```

**✅ Pass Criteria**:
- Goal is received and acknowledged in logs
- A* search completes successfully
- Trajectory optimization succeeds
- `/planning/pos_cmd` publishes at 100 Hz
- Command values are reasonable (not NaN, velocities < max_vel)

---

## Visualization in RViz

### Minimal RViz Setup

```bash
# Terminal 6
rviz2
```

**Add these displays**:
1. **TF** - Shows coordinate frames
2. **Odometry** → `/Odometry` (Green)
3. **PointCloud2** → `/cloud_registered` (White/Intensity)
4. **Path** → `/fsm/path` (Blue)
5. **MarkerArray** → `/trajectory` (Green)
6. **MarkerArray** → `/corridor` (Green transparent)

**Fixed Frame**: `camera_init`

---

## Debugging Common Issues

### Issue 1: FAST-LIO not publishing odometry

**Symptoms**:
- No `/Odometry` topic
- `/cloud_registered` not appearing

**Debug Steps**:
```bash
# Check if FAST-LIO is receiving sensor data
ros2 topic hz /sim_lidar/lidar
ros2 topic hz /sim_imu/imu

# Check FAST-LIO logs
# Look for "Initialization" or error messages
```

**Solutions**:
- Verify Gazebo sensors are publishing
- Check FAST-LIO config file path
- Ensure sensor topics match config

### Issue 2: ROG-MAP not receiving data

**Symptoms**:
- SUPER logs show "Waiting for odometry..."
- No map building

**Debug Steps**:
```bash
# Verify topic names
ros2 topic list | grep -E "(Odometry|cloud_registered)"

# Check SUPER subscription
ros2 node info /super_fsm | grep -A20 Subscriptions
```

**Solutions**:
- Verify `ros_callback_en: true` in config
- Check `odom_topic` and `cloud_topic` match FAST-LIO outputs
- Restart SUPER node after config changes

### Issue 3: Planning fails

**Symptoms**:
- No commands published after goal
- Errors in SUPER logs

**Debug Steps**:
```bash
# Check if map has obstacles
ros2 topic echo /rog_map/occupancy --once | grep "data" | wc -l

# Verify goal is in free space
# Goal should not be inside obstacles
```

**Solutions**:
- Move robot in Gazebo to build map first
- Send goal to known free space
- Check `planning_horizon` is reasonable
- Verify `robot_r` matches your drone size

### Issue 4: Commands are erratic

**Symptoms**:
- `/planning/pos_cmd` has unreasonable values
- Velocities exceed limits

**Debug Steps**:
```bash
# Monitor command values
ros2 topic echo /planning/pos_cmd | grep -E "(velocity|acceleration)"
```

**Solutions**:
- Reduce `max_vel`, `max_acc` in config
- Increase `corridor_bound_dis` for more conservative planning
- Check flatness parameters (mass, drag coefficients)

---

## Performance Benchmarks

### Expected Rates

| Topic | Expected Rate | Actual | Status |
|-------|--------------|--------|--------|
| `/sim_lidar/lidar` | ~10 Hz | _____ Hz | [ ] |
| `/sim_imu/imu` | ~250 Hz | _____ Hz | [ ] |
| `/Odometry` | 20-40 Hz | _____ Hz | [ ] |
| `/cloud_registered` | ~10 Hz | _____ Hz | [ ] |
| `/planning/pos_cmd` | ~100 Hz | _____ Hz | [ ] |

### CPU Usage

```bash
# Monitor node CPU usage
top -p $(pgrep -f fast_lio)
top -p $(pgrep -f fsm_node)
```

**Expected**:
- FAST-LIO: 30-60% CPU (single core)
- SUPER FSM: 10-30% CPU when planning, <5% when idle

---

## Next Steps After Successful Test

1. ✅ **Verified sensor data flow** (Gazebo → FAST-LIO)
2. ✅ **Verified localization** (FAST-LIO odometry)
3. ✅ **Verified mapping** (ROG-MAP building occupancy grid)
4. ✅ **Verified planning** (SUPER generating trajectories)

### What's Next:

1. **Create px4_super_bridge**
   - Convert `super_planner/PositionCommand` → `px4_msgs/TrajectorySetpoint`
   - Handle frame transformations if needed
   - Add safety limits and watchdog

2. **Tune planner parameters**
   - Adjust `max_vel`, `max_acc` for your drone
   - Fine-tune corridor generation
   - Test in complex environments

3. **Add mission planner**
   - Waypoint sequencing
   - RC trigger integration
   - Failsafe behaviors

4. **Safety features**
   - Geofencing
   - Battery monitoring
   - Emergency stop

---

## Test Log Template

**Test Date**: _____________
**Tester**: _____________
**Environment**: _____________

### Checklist

- [ ] Gazebo launched successfully
- [ ] Sensors publishing (`/sim_lidar/lidar`, `/sim_imu/imu`)
- [ ] FAST-LIO localization working (`/Odometry` @ 20+ Hz)
- [ ] FAST-LIO mapping working (`/cloud_registered` @ 10 Hz)
- [ ] ROG-MAP subscribed to topics
- [ ] ROG-MAP building map (visualization shows obstacles)
- [ ] SUPER received test goal
- [ ] A* path found
- [ ] Trajectory optimized
- [ ] Commands publishing (`/planning/pos_cmd` @ 100 Hz)
- [ ] Command values reasonable (velocity < max_vel)
- [ ] No NaN or infinite values in commands
- [ ] RViz visualization working

### Notes

```
_______________________________________________________________________
_______________________________________________________________________
_______________________________________________________________________
```

### Issues Encountered

```
_______________________________________________________________________
_______________________________________________________________________
_______________________________________________________________________
```

---

**Integration Status**: Ready for ROG-MAP testing ✅
**Next Milestone**: Create px4_super_bridge package

