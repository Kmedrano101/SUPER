# SUPER ROS2 Topic Map - Quick Reference

**Fast lookup guide for all ROS2 topics in SUPER system**

---

## Visual Topic Map

```
                    INPUTS                           SUPER SYSTEM                        OUTPUTS

┌──────────────────────────┐                    ┌─────────────────────────┐     ┌──────────────────────────┐
│   FAST-LIO / SLAM        │                    │   ROG-MAP               │     │   VISUALIZATION          │
├──────────────────────────┤                    ├─────────────────────────┤     ├──────────────────────────┤
│ /Odometry (nav_msgs)     │─────remap─────────>│ /lidar_slam/odom (sub)  │     │ /rog_map/occupancy       │
│ /cloud_registered (PC2)  │───────────────────>│ /cloud_registered (sub) │────>│ /rog_map/inflated        │
└──────────────────────────┘                    │                         │     │ /rog_map/unknown         │
                                                └─────────────────────────┘     └──────────────────────────┘
                                                          │ API
                                                          │ (no topics)
┌──────────────────────────┐                             ▼                      ┌──────────────────────────┐
│   MISSION PLANNER        │                    ┌─────────────────────────┐     │   CONTROLLER / BRIDGE    │
├──────────────────────────┤                    │   SUPER FSM             │     ├──────────────────────────┤
│ /Odometry (sub)          │                    ├─────────────────────────┤     │                          │
│ /goal (RViz) (sub)       │                    │ INPUTS:                 │     │ /planning/pos_cmd (sub)  │<──┐
│ /mavros/rc/in (sub)      │                    │ /planning/click_goal    │<────│   PositionCommand        │   │
├──────────────────────────┤                    │   (PoseStamped)         │     │                          │   │
│ /planning/click_goal     │───────────────────>│                         │     │ /planning_cmd/poly_traj  │<──┤
│   (PoseStamped) (pub)    │                    │ OUTPUTS:                │     │   (sub) PolynomialTraj   │   │
│ /mission/mkr (pub)       │                    │ /planning/pos_cmd       │────>│                          │   │
└──────────────────────────┘                    │   (PositionCommand)     │     │ YOUR BRIDGE CONVERTS:    │   │
                                                │ /planning_cmd/poly_traj │     │ PositionCommand → Twist  │   │
                                                │   (PolynomialTrajectory)│     │ → px4_offboard_sim       │   │
                                                │ /fsm/path (Path)        │───┐ └──────────────────────────┘   │
                                                └─────────────────────────┘   │                                │
                                                                               │ ┌──────────────────────────┐  │
                                                                               └>│   VISUALIZATION          │  │
                                                                                 ├──────────────────────────┤  │
                                                                                 │ /fsm/path                │  │
                                                                                 │ /trajectory              │<─┘
                                                                                 │ /corridor                │<─┘
                                                                                 │ /searched_path           │<─┘
                                                                                 │ /backup_traj             │<─┘
                                                                                 └──────────────────────────┘
```

---

## Topic Reference Table

### Input Topics (SUPER Subscribes)

| Topic | Message Type | Source | Config Location | Purpose |
|-------|-------------|---------|-----------------|---------|
| `/planning/click_goal` | `geometry_msgs/PoseStamped` | Mission Planner / RViz | `fsm.click_goal_topic` | Goal position |
| `/lidar_slam/odom` | `nav_msgs/Odometry` | FAST-LIO (remapped) | `rog_map.ros_callback.odom_topic` | Robot odometry |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO | `rog_map.ros_callback.cloud_topic` | Point cloud map |

### Output Topics (SUPER Publishes)

| Topic | Message Type | Subscriber | Config Location | Update Rate | Purpose |
|-------|-------------|------------|-----------------|-------------|---------|
| `/planning/pos_cmd` | `mars_quadrotor_msgs/PositionCommand` | px4_super_bridge | `fsm.cmd_topic` | 100 Hz | PVAJ command |
| `/planning_cmd/poly_traj` | `mars_quadrotor_msgs/PolynomialTrajectory` | MPC Controller | `fsm.mpc_cmd_topic` | 100 Hz | Full trajectory |
| `/fsm/path` | `nav_msgs/Path` | RViz | Hardcoded | As executed | Flight path history |

### Mission Planner Topics

| Topic | Message Type | Direction | Purpose |
|-------|-------------|-----------|---------|
| `/Odometry` | `nav_msgs/Odometry` | Subscribe | Current position for waypoint switching |
| `/goal` | `geometry_msgs/PoseStamped` | Subscribe | Manual goal from RViz 2D Goal Pose |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | Subscribe | RC trigger |
| `/planning/click_goal` | `geometry_msgs/PoseStamped` | Publish | Send waypoint to SUPER |
| `/mission/mkr` | `visualization_msgs/MarkerArray` | Publish | Waypoint visualization |

### Visualization Topics (Optional)

| Topic | Message Type | Publisher | Purpose |
|-------|-------------|-----------|---------|
| `/rog_map/occupancy` | `sensor_msgs/PointCloud2` | ROG-Map | Occupied voxels |
| `/rog_map/inflated` | `sensor_msgs/PointCloud2` | ROG-Map | Inflated obstacles |
| `/trajectory` | `visualization_msgs/MarkerArray` | SUPER | Planned trajectory |
| `/corridor` | `visualization_msgs/MarkerArray` | SUPER | Safe flight corridors |
| `/searched_path` | `visualization_msgs/Marker` | SUPER | A* search result |
| `/backup_traj` | `visualization_msgs/MarkerArray` | SUPER | Emergency trajectory |

---

## Required Remappings for Your Setup

### Launch Command with Remappings

```bash
# Terminal 1: FAST-LIO
ros2 launch fast_lio_ros2 simulation_mapping.launch.py
# Publishes: /Odometry, /cloud_registered

# Terminal 2: SUPER FSM with remappings
ros2 run super_planner fsm_node \
  --ros-args \
  --params-file ~/super_ws/src/SUPER/super_planner/config/static_high_speed.yaml \
  -r /Odometry:=/lidar_slam/odom
# Note: /cloud_registered already matches, no remap needed

# Terminal 3: px4_super_bridge
ros2 launch px4_super_bridge bridge.launch.py
```

### Alternative: Config File Changes

**File**: `super_planner/config/static_high_speed.yaml`

```yaml
# Change ROG-Map input topics (lines 138-139)
rog_map:
  ros_callback:
    cloud_topic: "/cloud_registered"        # ✅ Already matches FAST-LIO
    odom_topic: "/lidar_slam/odom"          # 🔧 Change if needed

# Change FSM topics (lines 3-8)
fsm:
  click_goal_topic: "/planning/click_goal"  # Goal input
  cmd_topic: "/planning/pos_cmd"            # Command output
  mpc_cmd_topic: "/planning_cmd/poly_traj"  # MPC output
```

---

## Topic Data Flow Sequence

### Normal Operation Sequence

```
1. FAST-LIO produces odometry + point cloud
   └─> /Odometry @ 20-40 Hz
   └─> /cloud_registered @ 10 Hz

2. ROG-Map receives and builds occupancy grid
   (Internal API, no topics)

3. Mission Planner publishes waypoint
   └─> /planning/click_goal @ varies

4. SUPER FSM receives goal
   ├─ Runs A* search
   ├─ Generates corridors
   ├─ Optimizes trajectory
   └─> Publishes commands

5. Commands published continuously
   └─> /planning/pos_cmd @ 100 Hz
   └─> /planning_cmd/poly_traj @ 100 Hz

6. px4_super_bridge converts
   └─> /px4_offboard_sim/offboard_velocity_cmd @ matches SUPER rate

7. px4_offboard_sim sends to PX4
   └─> /fmu/in/trajectory_setpoint @ 50 Hz
```

---

## Debugging: Monitor Topic Activity

```bash
# Check all SUPER-related topics
ros2 topic list | grep -E "(planning|fsm|rog_map|mission)"

# Monitor goal reception
ros2 topic echo /planning/click_goal

# Check command output rate
ros2 topic hz /planning/pos_cmd
# Expected: ~100 Hz

# Monitor odometry input
ros2 topic hz /lidar_slam/odom
# Expected: 20-40 Hz

# Check point cloud
ros2 topic hz /cloud_registered
# Expected: ~10 Hz

# Monitor mission planner output
ros2 topic echo /mission/mkr
```

---

## Common Issues & Solutions

### Issue: SUPER not receiving goals

**Check 1**: Is mission planner publishing?
```bash
ros2 topic hz /planning/click_goal
# Should show activity when waypoints switch
```

**Check 2**: Topic name mismatch?
```bash
ros2 param get /fsm_node click_goal_topic
# Should match mission planner's goal_pub_topic
```

**Solution**: Remap or change config to match

### Issue: SUPER not receiving odometry

**Check 1**: Is FAST-LIO publishing?
```bash
ros2 topic hz /Odometry
```

**Check 2**: Remapping applied?
```bash
ros2 node info /fsm_node | grep -A20 Subscriptions
# Should show /lidar_slam/odom in subscription list
```

**Solution**: Add `-r /Odometry:=/lidar_slam/odom` to launch command

### Issue: Bridge not receiving commands

**Check 1**: Is SUPER publishing?
```bash
ros2 topic hz /planning/pos_cmd
```

**Check 2**: Is bridge subscribed?
```bash
ros2 node info /px4_super_bridge | grep -A10 Subscriptions
```

**Solution**: Check bridge node is running and topic names match

---

## Integration Checklist

- [ ] FAST-LIO publishing `/Odometry` → Remap to `/lidar_slam/odom`
- [ ] FAST-LIO publishing `/cloud_registered` → Already matches
- [ ] Mission planner publishing `/planning/click_goal` → Matches SUPER input
- [ ] SUPER publishing `/planning/pos_cmd` → Bridge subscribes here
- [ ] Bridge publishing `/px4_offboard_sim/offboard_velocity_cmd` → px4_offboard_sim subscribes
- [ ] All topic rates verified (odom 20+ Hz, commands 100 Hz)

---

## RViz Configuration

Add these topics for full visualization:

```yaml
# Navigation
- /fsm/path (nav_msgs/Path) - Blue
- /planning/click_goal (PoseStamped) - Red arrow

# Map
- /cloud_registered (PointCloud2) - White/Gray
- /rog_map/occupancy (PointCloud2) - Red

# Planning
- /trajectory (MarkerArray) - Green
- /corridor (MarkerArray) - Green transparent
- /searched_path (Marker) - Yellow
- /backup_traj (MarkerArray) - Orange

# Mission
- /mission/mkr (MarkerArray) - Blue spheres
```

---

**Quick tip**: Keep this file open while testing your integration to quickly verify topic connections!

**Author**: Claude Code Integration Guide
**Last Updated**: Based on SUPER ROS2 version (2025)
