# Quick Start: SUPER + FAST-LIO + PX4 Integration

Go from zero to autonomous point-to-point navigation.

---

## System Overview

```
GAZEBO SENSORS --> FAST-LIO --> ROG-MAP --> SUPER --> Controller --> PX4
                   (SLAM)      (mapping)   (plan)   (bridge/MPC)
```

The system performs **full autonomous navigation** with continuous replanning:
- Goal input via `/goal_pose` topic or RViz
- Continuous replanning at 10 Hz during trajectory following
- Position commands published at 100 Hz
- Automatic trajectory regeneration when not at goal
- Automatic retry on planning failure

### Topic Connections

| Source | Topic | Destination | Rate |
|--------|-------|-------------|------|
| Gazebo | `/sim_lidar/lidar` | FAST-LIO | ~10 Hz |
| Gazebo | `/sim_imu/imu` | FAST-LIO | ~200 Hz |
| FAST-LIO | `/fast_lio_ros2/Odometry` | ROG-MAP | ~40 Hz |
| FAST-LIO | `/fast_lio_ros2/cloud_registered` | ROG-MAP | ~10 Hz |
| User/Mission | `/goal_pose` | SUPER FSM | on demand |
| SUPER | `/planning/pos_cmd` | Bridge | 100 Hz |
| SUPER | `/planning_cmd/poly_traj` | MPC Controller | on replan |

---

## Launch Sequence (4 Terminals)

### Terminal 1: Gazebo + PX4
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch px4_offboard_sim sim.launch.py
```
**Wait for**: Gazebo GUI, vehicle spawned

### Terminal 2: FAST-LIO
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch fast_lio_ros2 sim_slam.launch.py config_file:=gazebosim.yaml
```
**Wait for**: "Publishing to /Odometry" message

### Terminal 3: SUPER Planner + Controller
```bash
cd ~/super_ws
source install/setup.bash
ros2 launch super_planner full_integration_test.launch.py launch_bridge:=true
```
**Wait for**: "Odometry received! Building map..."

This launches:
- **SUPER FSM** -- trajectory planning + state machine
- **px4_super_bridge** -- converts SUPER commands to PX4 position setpoints

Launch options:
```bash
# With MPC controller instead of bridge
ros2 launch super_planner full_integration_test.launch.py use_mpc:=true

# With mission planner for waypoint sequences
ros2 launch super_planner full_integration_test.launch.py launch_bridge:=true launch_mission:=true

# Custom config file
ros2 launch super_planner full_integration_test.launch.py \
  launch_bridge:=true config_file:=/path/to/custom.yaml

# Without bridge (planner only, for testing)
ros2 launch super_planner test_super_planner.launch.py

# All options
ros2 launch super_planner full_integration_test.launch.py \
  launch_bridge:=true use_sim_time:=true config_file:=/path/to/custom.yaml
```

### Terminal 4: Send a Goal

**Option A: Command line**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'},
    pose: {position: {x: 5.0, y: 3.0, z: 1.5},
           orientation: {w: 1.0}}}"
```
Change `x`, `y`, `z` to your desired goal coordinates (meters, world frame).

**Option B: RViz**
1. Click **"2D Goal Pose"** in the toolbar.
2. Click on the map where you want the drone to go.
3. Z height defaults to the `click_height` config value (default: `1.5 m`).

**Option C: Mission Planner (waypoint sequence)**

Launch with `launch_mission:=true` and configure waypoints in `mission_planner/config/waypoint.yaml`.

---

## What Happens After Sending a Goal

1. SUPER generates a collision-free trajectory from current position to goal.
2. The trajectory is continuously replanned at **10 Hz** as the drone moves.
3. Position commands are published at **100 Hz** to the controller.
4. The controller forwards setpoints to PX4 for execution.
5. When the drone reaches the goal (< 0.1 m), it hovers and waits for a new goal.
6. If planning fails, it automatically retries until successful.

**Changing goal mid-flight**: Send a new goal at any time. SUPER automatically replans from the drone's current position.

---

## Controller Modes

The `use_mpc` launch argument selects between two controller modes:

### px4_super_bridge (default, `use_mpc:=false`)
- Extracts **position only** from SUPER's trajectory and forwards `PoseStamped` to PX4.
- PX4's internal PID loops handle velocity/acceleration tracking.
- Simple and reliable. Best for initial testing.

### px4_mpc_controller (`use_mpc:=true`)
- Receives the full **polynomial trajectory** and runs a Model Predictive Controller (acados solver).
- Sends **position + velocity + acceleration + yaw** commands to PX4.
- Better trajectory tracking at high speeds, smoother motion.
- Requires tuning via `px4_mpc_controller/config/mpc_params.yaml`.

---

## Configuration

All planner settings are in one file: **`super_planner/config/px4_integration.yaml`**

### Goal Input Settings (lines 9-14)

```yaml
fsm:
  click_goal_en: true                  # Enable/disable goal subscription
  click_goal_topic: "/goal_pose"       # Topic to receive goal poses
  click_height: 1.5                    # Default Z height for 2D goals (meters)
  click_yaw_en: true                   # Use yaw from goal orientation
```

- **`click_goal_topic`** -- Change if your goal publisher uses a different topic.
- **`click_height`** -- Z value for RViz 2D goals. Set to `-10.0` to use the goal's actual Z.

### Velocity and Acceleration Limits (lines 70-77)

```yaml
traj_opt:
  boundary:
    max_vel: 5.0       # Maximum velocity (m/s)
    max_acc: 5.0       # Maximum acceleration (m/s^2)
    max_jerk: 30.0     # Maximum jerk (m/s^3) -- trajectory smoothness
    max_omg: 3.0       # Maximum angular velocity (rad/s)
    max_acc_thr: 15.0  # Maximum thrust acceleration (m/s^2)
    min_acc_thr: 6.0   # Minimum thrust acceleration (m/s^2)
```

SUPER uses a **single global velocity constraint** applied uniformly across the entire trajectory. The optimizer naturally produces acceleration/deceleration near start and end points due to smoothness constraints -- there are no separate velocity profiles per flight phase.

There are **two layers** where velocity is constrained, and both must be consistent:

**Layer 1 -- Planner** (`px4_integration.yaml`): Determines the planned trajectory shape. The optimizer finds a time-optimal, collision-free trajectory that never exceeds `max_vel`/`max_acc`/`max_jerk`.

**Layer 2 -- Controller**: Caps execution velocity.
- **Bridge mode**: PX4's own parameters (`MPC_XY_VEL_MAX`, `MPC_Z_VEL_MAX_UP`, `MPC_Z_VEL_MAX_DN` in QGroundControl) act as the execution-level cap.
- **MPC mode** (`px4_mpc_controller/config/mpc_params.yaml`):
  ```yaml
  px4_mpc_controller:
    ros__parameters:
      max_vel: 1.0       # MPC velocity constraint per prediction stage
      max_acc: 2.0       # MPC acceleration constraint
      Q_pos: 50.0        # Position tracking weight (higher = tighter)
      Q_vel: 10.0        # Velocity tracking weight
      R_acc: 1.0         # Acceleration penalty (higher = smoother but slower)
  ```

**Key rule**: The controller velocity limit must match or exceed the planner's `max_vel`. If the controller limit is lower, the drone falls behind the planned trajectory.

### Recommended Settings by Use Case

| Use Case | `max_vel` | `max_acc` | `max_jerk` | MPC `max_vel` |
|----------|-----------|-----------|------------|---------------|
| First test / conservative | 1.0 | 2.0 | 10.0 | 1.0 |
| Indoor / slow flight | 2.0 | 3.0 | 20.0 | 2.0 |
| Outdoor / normal flight | 5.0 | 5.0 | 30.0 | 5.0 |
| High-speed (open area) | 10.0 | 8.0 | 50.0 | 10.0 |
| Aggressive (expert only) | 25.0 | 20.0 | 120.0 | 25.0 |

### Yaw Control

```yaml
super_planner:
  yaw_dot_max: 1.5     # Max yaw rate (rad/s)
  yaw_mode: 1          # 1 = face velocity direction, 2 = face goal
```

### Drone Physical Parameters

```yaml
super_planner:
  robot_r: 0.3               # Collision radius (meters) -- match your drone + safety margin

traj_opt:
  flatness:
    mass: 1.5           # Drone mass (kg) -- affects thrust calculations
    dh: 0.35            # Horizontal drag coefficient
    dv: 0.35            # Vertical drag coefficient
```

### Planning Horizon (lines 50-53)

```yaml
super_planner:
  planning_horizon: 20.0    # How far ahead the planner looks (meters)
  receding_dis: 10.0         # Receding horizon distance (meters)
```

For short-range goals (< 10 m), defaults work well. For longer distances, increase `planning_horizon`.

---

## Verification

```bash
# Sensor data flowing
ros2 topic hz /sim_lidar/lidar                  # ~10 Hz

# SLAM running
ros2 topic hz /fast_lio_ros2/Odometry           # ~40 Hz
ros2 topic hz /fast_lio_ros2/cloud_registered   # ~10 Hz

# Planner publishing commands (after goal sent)
ros2 topic hz /planning/pos_cmd                 # ~100 Hz

# Bridge forwarding to PX4
ros2 topic hz /px4_offboard_sim/offboard_control/target_pose

# Inspect a single command
ros2 topic echo /planning/pos_cmd --once
```

### RViz Visualization

```bash
# Launched automatically with test_super_planner.launch.py
# Or launch separately:
rviz2 -d ~/super_ws/src/SUPER/super_planner/rviz/super_integration.rviz

# Launch planner without RViz:
ros2 launch super_planner test_super_planner.launch.py rviz:=false
```

---

## Quick Example: Fly to (10, 5, 2)

```bash
# Terminal 3: Launch planner + bridge
cd ~/super_ws && source install/setup.bash
ros2 launch super_planner full_integration_test.launch.py launch_bridge:=true

# Terminal 4: Send goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'},
    pose: {position: {x: 10.0, y: 5.0, z: 2.0},
           orientation: {w: 1.0}}}"

# Terminal 4: Redirect mid-flight to a new point
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'},
    pose: {position: {x: -3.0, y: 2.0, z: 1.5},
           orientation: {w: 1.0}}}"
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| FAST-LIO not publishing odometry | Gazebo sensors not running | `ros2 topic list \| grep sim_lidar` -- restart Gazebo if missing |
| SUPER waiting for odometry | FAST-LIO not started | Start FAST-LIO before SUPER |
| No trajectory after goal | Map not built yet | Fly around briefly to build initial map |
| Drone doesn't move | Bridge not running | Check `launch_bridge:=true` in launch |
| Trajectory too slow | Conservative velocity limits | Increase `max_vel` in config |
| Drone avoids open space | Unmapped area treated as obstacle | Increase `map_size` or fly to map the area |
| Goal ignored | Goal subscription disabled | Set `click_goal_en: true` in config |

---

## Config File Quick Reference

### `super_planner/config/px4_integration.yaml`

| What to change | Parameter | Line |
|----------------|-----------|------|
| Goal topic | `fsm.click_goal_topic` | 12 |
| Default flight height | `fsm.click_height` | 13 |
| Max speed (planner) | `traj_opt.boundary.max_vel` | 72 |
| Max acceleration (planner) | `traj_opt.boundary.max_acc` | 73 |
| Max jerk (smoothness) | `traj_opt.boundary.max_jerk` | 74 |
| Max angular velocity | `traj_opt.boundary.max_omg` | 75 |
| Yaw rate limit | `super_planner.yaw_dot_max` | 57 |
| Yaw behavior | `super_planner.yaw_mode` | 60 |
| Drone mass | `traj_opt.flatness.mass` | 121 |
| Drone collision radius | `super_planner.robot_r` | 56 |
| Planning range | `super_planner.planning_horizon` | 51 |
| Map size | `rog_map.map_size` | 146 |
| Flight ceiling | `rog_map.virtual_ceil_height` | 152 |
| Flight floor | `rog_map.virtual_ground_height` | 153 |

### `px4_mpc_controller/config/mpc_params.yaml` (MPC mode only)

| What to change | Parameter | Line |
|----------------|-----------|------|
| Max speed (controller) | `max_vel` | 7 |
| Max acceleration (controller) | `max_acc` | 8 |
| Position tracking tightness | `Q_pos` | 14 |
| Velocity tracking tightness | `Q_vel` | 15 |
| Control smoothness | `R_acc` | 16 |

---

## Related Documentation

- **Architecture overview**: `ARCHITECTURE_BREAKDOWN.md`
- **Complete topic map**: `INTEGRATION_TOPIC_MAP.md`
- **Quick topic reference**: `TOPIC_MAP_QUICK_REFERENCE.md`
- **ROG-MAP integration details**: `ROG_MAP_INTEGRATION_TEST.md`
- **RViz setup**: `RVIZ_VISUALIZATION_GUIDE.md`
