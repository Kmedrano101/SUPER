# SUPER + FAST-LIO + PX4 Integration Topic Map

**Complete topic mapping for Gazebo simulation integration**

Last Updated: 2025-10-21

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            GAZEBO SIMULATION                                │
│  ┌──────────────────┐           ┌──────────────────┐                        │
│  │ LiDAR Sensor     │           │ IMU Sensor       │                        │
│  │ /sim_lidar/lidar │           │ /sim_imu/imu     │                        │
│  └──────────────────┘           └──────────────────┘                        │
└─────────────────────────────────────────────────────────────────────────────┘
            │                                 │
            │ sensor_msgs/PointCloud2         │ sensor_msgs/Imu
            │                                 │
            ▼                                 ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         FAST-LIO ROS2 Node                                  │
│  Config: gazebosim.yaml                                                     │
│  Subscriptions:                                                             │
│    - /sim_lidar/lidar (PointCloud2)                                         │
│    - /sim_imu/imu (Imu)                                                     │
└─────────────────────────────────────────────────────────────────────────────┘
            │                                 │
            │ /cloud_registered               │ /Odometry
            │ sensor_msgs/PointCloud2         │ nav_msgs/Odometry
            │ @ ~10 Hz                        │ @ ~40 Hz
            ▼                                 ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ROG-MAP Node                                       │
│  Config: rog_map section in super_planner config                           │
│  Default Subscriptions:                                                     │
│    - /cloud_registered (PointCloud2) ✅ MATCHES FAST-LIO                    │
│    - /lidar_slam/odom (Odometry) ⚠️ NEEDS REMAP from /Odometry             │
│                                                                             │
│  Publishes (Optional visualization):                                       │
│    - /rog_map/occupancy (PointCloud2)                                      │
│    - /rog_map/inflated (PointCloud2)                                       │
└─────────────────────────────────────────────────────────────────────────────┘
            │
            │ API calls (no topics, direct function calls)
            │
            ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                       SUPER Planner FSM Node                                │
│  Subscribes:                                                                │
│    - /planning/click_goal (PoseStamped) - from mission planner             │
│                                                                             │
│  Publishes:                                                                 │
│    - /planning/pos_cmd (super_planner/PositionCommand) @ 100 Hz            │
│    - /planning_cmd/poly_traj (super_planner/PolynomialTrajectory) @ 100 Hz │
│    - /fsm/path (nav_msgs/Path) - execution path                            │
└─────────────────────────────────────────────────────────────────────────────┘
            │
            │ /planning/pos_cmd
            │
            ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                    px4_super_bridge (TO BE CREATED)                         │
│  Converts super_planner/PositionCommand → px4_msgs/TrajectorySetpoint      │
│  OR                                                                         │
│  Converts to geometry_msgs/Twist for velocity control                      │
└─────────────────────────────────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PX4 Autopilot                                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Topic Reference Table

### Gazebo → FAST-LIO

| Topic | Message Type | Rate | Frame | Notes |
|-------|-------------|------|-------|-------|
| `/sim_lidar/lidar` | `sensor_msgs/PointCloud2` | 10 Hz | `X1/OakD-Lite/OakD-Lite` | Published by Gazebo |
| `/sim_imu/imu` | `sensor_msgs/Imu` | 250 Hz | `X1/imu_link/imu` | Published by Gazebo |

**FAST-LIO Config**: `/home/kmedrano/ros2_ws/src/fast_lio_ros2/config/gazebosim.yaml`
```yaml
common:
    lid_topic:  "/sim_lidar/lidar"
    imu_topic:  "/sim_imu/imu"
```

### FAST-LIO → ROG-MAP

| Publisher | Topic | Message Type | Rate | Frame | ROG-MAP Default |
|-----------|-------|-------------|------|-------|-----------------|
| FAST-LIO | `/Odometry` | `nav_msgs/Odometry` | ~40 Hz | `camera_init` → `body` | `/lidar_slam/odom` ⚠️ |
| FAST-LIO | `/cloud_registered` | `sensor_msgs/PointCloud2` | ~10 Hz | `camera_init` | `/cloud_registered` ✅ |

**⚠️ ACTION REQUIRED**: Need to either:
- **Option A**: Remap `/Odometry` → `/lidar_slam/odom` when launching SUPER
- **Option B**: Change ROG-MAP config `odom_topic` from `/lidar_slam/odom` to `/Odometry`

### ROG-MAP Configuration

**File**: `super_planner/config/*.yaml` (any config file)

```yaml
rog_map:
  ros_callback:
    ros_callback_en: true                    # Enable ROS topic subscriptions
    cloud_topic: "/cloud_registered"         # ✅ Matches FAST-LIO output
    odom_topic: "/Odometry"                  # 🔧 CHANGE THIS from /lidar_slam/odom
```

### SUPER Planner Topics

| Direction | Topic | Message Type | Rate | Purpose |
|-----------|-------|-------------|------|---------|
| Subscribe | `/planning/click_goal` | `geometry_msgs/PoseStamped` | On demand | Goal from mission planner |
| Publish | `/planning/pos_cmd` | `super_planner/PositionCommand` | 100 Hz | PVAJ commands |
| Publish | `/planning_cmd/poly_traj` | `super_planner/PolynomialTrajectory` | 100 Hz | Full polynomial trajectory |
| Publish | `/fsm/path` | `nav_msgs/Path` | As executed | Flight path history |

**Custom Message Fields**:

`super_planner/PositionCommand`:
- `position` (Point): xyz position setpoint
- `velocity` (Vector3): xyz velocity setpoint
- `acceleration` (Vector3): xyz acceleration setpoint
- `jerk` (Vector3): xyz jerk (snap)
- `yaw` (float64): yaw angle
- `yaw_dot` (float64): yaw rate

`super_planner/PolynomialTrajectory`:
- `piece_num_pos`, `piece_num_yaw`: number of trajectory pieces
- `coef_pos_x/y/z`: polynomial coefficients
- `time_pos`, `time_yaw`: time arrays
- See full definition in `super_planner/msg/`

---

## Integration Configurations

### Step 1: Update ROG-MAP Config

**File**: `super_planner/config/px4_integration.yaml` (or your config)

```yaml
rog_map:
  ros_callback:
    ros_callback_en: true
    cloud_topic: "/cloud_registered"      # Matches FAST-LIO
    odom_topic: "/Odometry"                # Changed from /lidar_slam/odom
```

### Step 2: Launch Command Sequence

```bash
# Terminal 1: Launch Gazebo + PX4
cd ~/ros2_ws
ros2 launch px4_offboard_sim simulation.launch.py
# Publishes: /sim_lidar/lidar, /sim_imu/imu

# Terminal 2: Launch FAST-LIO
ros2 launch fast_lio_ros2 mapping.launch.py \
  config_file:=gazebosim.yaml
# Publishes: /Odometry, /cloud_registered, /path

# Terminal 3: Launch SUPER Planner
cd ~/super_ws
source install/setup.bash
ros2 launch super_planner test_super_planner.launch.py
# Publishes: /planning/pos_cmd, /planning_cmd/poly_traj

# Terminal 4: Monitor topics
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /planning/pos_cmd
```

---

## Verification Checklist

### Before Launch
- [ ] Gazebo simulation ready with LiDAR and IMU sensors
- [ ] FAST-LIO config points to `/sim_lidar/lidar` and `/sim_imu/imu`
- [ ] ROG-MAP config updated to subscribe to `/Odometry` (not `/lidar_slam/odom`)
- [ ] SUPER config has correct topic names

### After Launch - Check Publications
```bash
# Check Gazebo sensors
ros2 topic hz /sim_lidar/lidar    # Should show ~10 Hz
ros2 topic hz /sim_imu/imu        # Should show ~250 Hz

# Check FAST-LIO outputs
ros2 topic hz /Odometry           # Should show ~40 Hz
ros2 topic hz /cloud_registered   # Should show ~10 Hz
ros2 topic list | grep fast_lio   # Should show FAST-LIO topics

# Check ROG-MAP subscriptions
ros2 node info /rog_map_node | grep -A20 "Subscriptions"
# Should show:
#   /Odometry
#   /cloud_registered

# Check SUPER planner
ros2 topic hz /planning/pos_cmd   # Should show ~100 Hz after goal received
ros2 topic list | grep -E "(planning|fsm)"
```

### After Sending Goal
- [ ] `/planning/pos_cmd` starts publishing at 100 Hz
- [ ] `/fsm/path` shows growing trajectory
- [ ] RViz shows trajectory visualization (if configured)

---

## Common Integration Issues

### Issue 1: ROG-MAP not receiving odometry

**Symptom**: ROG-MAP shows no odometry updates, can't build map

**Debug**:
```bash
ros2 topic hz /Odometry  # Check if FAST-LIO is publishing
ros2 node info /rog_map_node | grep Subscriptions  # Check subscribed topics
```

**Solution**: Verify ROG-MAP config has `odom_topic: "/Odometry"`

### Issue 2: FAST-LIO not starting

**Symptom**: No `/Odometry` or `/cloud_registered` topics

**Debug**:
```bash
ros2 topic list | grep sim  # Check if Gazebo sensors publishing
ros2 topic echo /sim_lidar/lidar --once  # Test LiDAR data
```

**Solution**:
- Check Gazebo is running
- Verify sensor plugins loaded in URDF/SDF
- Check FAST-LIO config file path

### Issue 3: SUPER not planning

**Symptom**: No `/planning/pos_cmd` output

**Debug**:
```bash
ros2 topic echo /planning/click_goal  # Check if goal received
ros2 node info /fsm_node | grep Subscriptions
```

**Solution**:
- Send a goal: `ros2 topic pub /planning/click_goal geometry_msgs/PoseStamped ...`
- Check ROG-MAP has valid map data
- Verify FSM state (check logs)

---

## RViz Visualization Setup

Add these displays in RViz:

```yaml
Displays:
  - Class: rviz/TF
    Name: TF

  - Class: rviz/Odometry
    Topic: /Odometry
    Color: 0; 255; 0

  - Class: rviz/PointCloud2
    Topic: /cloud_registered
    Color Transform: Intensity

  - Class: rviz/Path
    Topic: /fsm/path
    Color: 0; 0; 255

  - Class: rviz/MarkerArray
    Topic: /trajectory

  - Class: rviz/MarkerArray
    Topic: /corridor
```

**Fixed Frame**: `camera_init` (FAST-LIO world frame)

---

## Next Steps

1. **Create px4_super_bridge package** to convert `super_planner/PositionCommand` → PX4 commands
2. **Test in simulation** with simple waypoints
3. **Tune planner parameters** for your vehicle dynamics
4. **Add safety checks** (geofencing, velocity limits)
5. **Create mission planner** for autonomous waypoint missions

---

**Integration Status**:
- ✅ FAST-LIO → ROG-MAP: Topic mapping identified
- ✅ ROG-MAP → SUPER: API integration (no topics)
- ✅ SUPER messages: Custom messages created
- ⏳ SUPER → PX4: Bridge needed

