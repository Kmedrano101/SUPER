# SUPER Planner Testing Guide

**Quick guide to test SUPER with your PX4 + FAST-LIO system**

---

## Prerequisites

### 1. Build Everything

```bash
# Build SUPER
cd ~/super_ws
colcon build --packages-select mars_quadrotor_msgs rog_map super_planner
source install/setup.bash

# Build px4_super_bridge (if not done)
cd ~/ros2_ws
colcon build --packages-select px4_super_bridge
source install/setup.bash
```

### 2. Verify ROS2 Configuration

```bash
# Check SUPER is ROS2
cd ~/super_ws/src/SUPER
grep "USE_ROS2" super_planner/CMakeLists.txt
# Should output: add_definitions(-DUSE_ROS2)
```

---

## Test Method 1: Standalone SUPER Test

**Test SUPER planner alone (without bridge)**

### Terminal 1: Gazebo + PX4
```bash
cd ~/px4-autopilot
make px4_sitl gz_x500
```

### Terminal 2: FAST-LIO + Frame Manager
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch px4_offboard_sim slam_simulation.launch.py
```

Wait for output:
```
[frame_manager]: Republished Lidar Points with corrected frame_id
[fastlio_mapping]: Received first LiDAR frame
```

### Terminal 3: Check Topics
```bash
# Verify FAST-LIO is publishing
ros2 topic hz /Odometry
# Expected: 20-40 Hz

ros2 topic hz /cloud_registered
# Expected: ~10 Hz

# Check data
ros2 topic echo /cloud_registered --field width
# Should be > 0
```

### Terminal 4: Launch SUPER with Test Launch File
```bash
cd ~/super_ws
source install/setup.bash

# Method 1: Simple launch (just SUPER + RViz)
ros2 launch super_planner test_super_planner.launch.py

# Method 2: Specify custom config
ros2 launch super_planner test_super_planner.launch.py \
  config_file:=~/super_ws/src/SUPER/super_planner/config/px4_integration.yaml
```

**Expected Output**:
```
 -- [Fsm-Test] Begin.
 -- [Fsm] Current state: INIT
 -- [Fsm] Current state: WAIT_GOAL
Waiting for goal...
```

### Terminal 5: Send Test Goal

**Option A: Use RViz** (opens automatically with launch file)
1. Click **"2D Goal Pose"** button in toolbar
2. Click on the map to set a goal
3. Watch SUPER plan and publish trajectory

**Option B: Command Line**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'camera_init'}, \
    pose: {position: {x: 5.0, y: 0.0, z: 1.5}}}"
```

**Expected Output in Terminal 4**:
```
 -- [Fsm] Get goal at [5.0 0.0 1.5]
 -- [Fsm] Current state: GENERATE_TRAJ
 -- [Fsm] ReplanOnce succeed.
 -- [Fsm] Current state: FOLLOW_TRAJ
```

### Verify Planning Works

```bash
# Check SUPER is publishing commands
ros2 topic hz /planning/pos_cmd
# Expected: ~100 Hz

# View command data
ros2 topic echo /planning/pos_cmd --field position
```

**Success!** SUPER is planning trajectories.

---

## Test Method 2: Full Integration Test

**Test complete system: SUPER + Bridge + PX4 Offboard**

### Terminal 1: Gazebo + PX4 (same as Method 1)
```bash
cd ~/px4-autopilot
make px4_sitl gz_x500
```

### Terminal 2: Full Integration Launch
```bash
cd ~/super_ws
source install/setup.bash
source ~/ros2_ws/install/setup.bash  # For px4_super_bridge

ros2 launch super_planner full_integration_test.launch.py
```

This launches:
- ✅ FAST-LIO + frame_manager
- ✅ px4_offboard_sim (offboard control)
- ✅ SUPER FSM (after 3s delay)
- ✅ px4_super_bridge (after 5s delay)
- ✅ RViz

### Terminal 3: Arm Drone
```bash
cd ~/ros2_ws
source install/setup.bash

# Start keyboard control
ros2 run px4_offboard_sim joy_control
```

**Press SPACE** to arm drone

**Wait for**:
```
ARMING
TAKEOFF
LOITER
OFFBOARD  ← Ready!
```

### Terminal 4: Send Goal
```bash
# Use RViz 2D Goal Pose or:
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'camera_init'}, \
    pose: {position: {x: 5.0, y: 0.0, z: 1.5}}}"
```

### Expected Behavior

1. **SUPER plans**: Corridor + trajectory generation
2. **Bridge converts**: PositionCommand → Twist
3. **px4_offboard_sim sends**: Velocity to PX4
4. **Drone flies**: Autonomously to goal!

---

## Test Method 3: Manual Component Launch

**Launch each component separately for debugging**

### Terminal 1: Gazebo + PX4
```bash
cd ~/px4-autopilot
make px4_sitl gz_x500
```

### Terminal 2: FAST-LIO
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch px4_offboard_sim slam_simulation.launch.py
```

### Terminal 3: SUPER FSM
```bash
cd ~/super_ws
source install/setup.bash

ros2 run super_planner fsm_node \
  --ros-args \
  --params-file ~/super_ws/src/SUPER/super_planner/config/px4_integration.yaml
```

### Terminal 4: px4_super_bridge
```bash
cd ~/ros2_ws
source install/setup.bash

ros2 launch px4_super_bridge bridge.launch.py
```

### Terminal 5: Arm Drone
```bash
ros2 run px4_offboard_sim joy_control
# Press SPACE
```

### Terminal 6: RViz
```bash
ros2 run rviz2 rviz2 \
  -d ~/super_ws/src/SUPER/super_planner/rviz/super_test.rviz
```

### Terminal 7: Send Goal
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'camera_init'}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}}}"
```

---

## Verification Checklist

### Stage 1: FAST-LIO Running
- [ ] `/Odometry` publishing @ 20-40 Hz
- [ ] `/cloud_registered` publishing @ ~10 Hz
- [ ] Point cloud has data (width > 0)

### Stage 2: SUPER Planning
- [ ] SUPER state = WAIT_GOAL
- [ ] Goal received: " -- [Fsm] Get goal at [...]"
- [ ] State → GENERATE_TRAJ
- [ ] Planning succeeds: "ReplanOnce succeed"
- [ ] State → FOLLOW_TRAJ
- [ ] `/planning/pos_cmd` publishing @ 100 Hz

### Stage 3: Bridge Converting (if testing full integration)
- [ ] Bridge receives commands: "SUPER command received!"
- [ ] Bridge outputs: "Vel Cmd (NED): [...]"
- [ ] `/px4_offboard_sim/offboard_velocity_cmd` publishing

### Stage 4: PX4 Execution (if testing full integration)
- [ ] Drone armed and in OFFBOARD mode
- [ ] PX4 receiving setpoints: `/fmu/in/trajectory_setpoint` @ 50 Hz
- [ ] Drone moving towards goal

---

## Common Issues & Solutions

### Issue 1: SUPER says "No odom"

**Check**:
```bash
ros2 topic list | grep Odometry
ros2 topic hz /Odometry
```

**Solution**: Verify FAST-LIO is running and publishing.

---

### Issue 2: SUPER doesn't receive goal

**Check topic name**:
```bash
# What is SUPER listening to?
ros2 node info /super_fsm | grep -A10 Subscriptions

# Is /goal_pose in the list?
```

**Solution**: Config should have `click_goal_topic: "/goal_pose"`

**Alternative**: Publish to the topic SUPER is listening to:
```bash
ros2 topic pub --once /planning/click_goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'camera_init'}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}}}"
```

---

### Issue 3: Planning fails

**Check map is being built**:
```bash
ros2 topic echo /cloud_registered --field width
# Should be > 0
```

**Check goal is in free space**:
- Open RViz
- Visualize `/cloud_registered`
- Ensure goal is away from obstacles

**Try closer goal**:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'camera_init'}, pose: {position: {x: 3.0, y: 0.0, z: 1.5}}}"
```

---

### Issue 4: Launch file fails

**Error: Package not found**

**Fix**:
```bash
# Rebuild and source
cd ~/super_ws
colcon build --packages-select super_planner
source install/setup.bash

# Verify launch file installed
ls ~/super_ws/install/super_planner/share/super_planner/launch/
```

**Error: Config file not found**

**Fix**: Use absolute path:
```bash
ros2 launch super_planner test_super_planner.launch.py \
  config_file:=$(pwd)/src/SUPER/super_planner/config/px4_integration.yaml
```

---

## RViz Visualization

### Topics to Visualize

**Essential**:
- `/cloud_registered` (PointCloud2) - Map
- `/path` (Path) - FAST-LIO trajectory
- `/goal_pose` (PoseStamped) - Goal position

**SUPER Planning**:
- `/fsm/path` (Path) - Executed trajectory
- `/trajectory` (MarkerArray) - Planned trajectory
- `/corridor` (MarkerArray) - Safe corridors
- `/searched_path` (Marker) - A* search result

**Mapping**:
- `/rog_map/occupancy` (PointCloud2) - Occupied voxels

### RViz Setup

1. **Set Fixed Frame**: `camera_init`

2. **Add Point Cloud**:
   - Add → By topic → `/cloud_registered` → PointCloud2
   - Size: 0.05
   - Color: Intensity

3. **Add Goal Tool**:
   - Toolbar → Add → 2D Goal Pose
   - Topic: `/goal_pose`

4. **Load Config** (or create manually):
   ```bash
   ros2 run rviz2 rviz2 \
     -d ~/super_ws/src/SUPER/super_planner/rviz/super_test.rviz
   ```

---

## Performance Monitoring

### Check Update Rates

```bash
# All important topics
ros2 topic hz /Odometry &
ros2 topic hz /cloud_registered &
ros2 topic hz /planning/pos_cmd &
ros2 topic hz /px4_offboard_sim/offboard_velocity_cmd &

# Wait a few seconds, then:
pkill -f "ros2 topic hz"
```

### Monitor CPU Usage

```bash
# Check node CPU usage
top -p $(pgrep -f fsm_node)
top -p $(pgrep -f fastlio_mapping)
```

### Check Message Flow

```bash
# Pipeline test
ros2 topic echo /Odometry --field pose.pose.position | head -5
ros2 topic echo /planning/pos_cmd --field position | head -5
ros2 topic echo /px4_offboard_sim/offboard_velocity_cmd | head -5
```

---

## Quick Test Commands

### Send Multiple Goals

```bash
# Goal sequence
for i in {1..3}; do
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'camera_init'}, pose: {position: {x: $((i*3)).0, y: 0.0, z: 1.5}}}"
  sleep 10
done
```

### Emergency Stop

```bash
# Stop SUPER planning (Ctrl+C in SUPER terminal)

# Or publish emergency stop (if implemented)
# ros2 topic pub /emergency_stop std_msgs/Bool "data: true"
```

---

## Success Criteria

Your test is successful if:

- ✅ FAST-LIO builds map and publishes odometry
- ✅ SUPER receives goal and generates trajectory
- ✅ SUPER publishes commands at ~100 Hz
- ✅ RViz shows corridors and planned trajectory
- ✅ (Full integration) Drone flies to goal autonomously
- ✅ No error messages in any terminal

---

## Next Steps After Successful Test

1. **Tune parameters**: Adjust velocities in `px4_integration.yaml`
2. **Test with obstacles**: Add objects in Gazebo
3. **Test waypoint mission**: Use mission_planner
4. **Real flight preparation**: See SUPER hardware guide

---

## Test Report Template

After testing, document:

```
Date: _______________
Test Method: [ ] Standalone [ ] Full Integration [ ] Manual

Results:
[ ] FAST-LIO running
[ ] SUPER planning working
[ ] Bridge converting (if applicable)
[ ] Drone flying (if applicable)

Issues Encountered:
-

Solutions Applied:
-

Performance:
- Planning time: ___ ms
- Command rate: ___ Hz
- Drone max velocity: ___ m/s

Notes:
```

---

**Happy Testing!** 🚁

For issues, check:
1. This guide's troubleshooting section
2. `ARCHITECTURE_BREAKDOWN.md` for topic details
3. `INTEGRATION_STEPS.md` for full setup

**Author**: Kevin Medrano Ayala
**Created**: 2025
