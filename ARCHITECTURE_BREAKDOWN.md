# SUPER Architecture Breakdown - Complete ROS2 Topic & Module Analysis

**Complete technical breakdown of SUPER's internal architecture for integration and modification.**

---

## Table of Contents

1. [Package Structure](#1-package-structure)
2. [ROS2 Topics Communication Map](#2-ros2-topics-communication-map)
3. [FSM State Machine](#3-fsm-state-machine)
4. [Module-by-Module Breakdown](#4-module-by-module-breakdown)
5. [Data Flow Architecture](#5-data-flow-architecture)
6. [Key Modification Points](#6-key-modification-points)
7. [Message Type Reference](#7-message-type-reference)

---

## 1. Package Structure

```
SUPER/
├── super_planner/          # Core trajectory planning
│   ├── Apps/
│   │   ├── fsm_node_ros2.cpp          # ROS2 FSM node entry point
│   │   └── fsm_node_ros1.cpp          # ROS1 FSM node entry point
│   ├── include/
│   │   ├── fsm/
│   │   │   ├── fsm.h                  # FSM base class (state machine logic)
│   │   │   └── config.hpp              # FSM configuration
│   │   ├── super_core/
│   │   │   ├── super_planner.h         # Main planner class
│   │   │   ├── astar.h                 # A* search
│   │   │   ├── ciri.h                  # Corridor generator
│   │   │   └── corridor_generator.h    # Corridor wrapper
│   │   ├── traj_opt/
│   │   │   ├── exp_traj_optimizer_s4.h # Exploratory trajectory optimizer
│   │   │   ├── backup_traj_optimizer_s4.h # Backup trajectory optimizer
│   │   │   └── yaw_traj_opt.h         # Yaw trajectory optimizer
│   │   └── ros_interface/
│   │       ├── ros_interface.hpp       # Base ROS interface
│   │       └── ros2/
│   │           ├── ros2_interface.hpp  # ROS2 adapter
│   │           └── fsm_ros2.hpp        # ROS2 FSM implementation
│   └── src/                            # Implementation files
│
├── mission_planner/        # High-level mission management
│   ├── Apps/
│   │   └── ros2_waypoint_mission.cpp   # ROS2 waypoint mission node
│   └── include/
│       └── waypoint_mission/
│           └── ros2_waypoint_planner.hpp # Waypoint planner implementation
│
├── rog_map/               # Robocentric Occupancy Grid Map
│   └── include/
│       └── rog_map_ros/
│           └── rog_map_ros2.hpp        # ROS2 ROG-Map interface
│
└── mars_uav_sim/          # Simulation & message definitions
    ├── mars_quadrotor_msgs/            # Custom message package
    └── perfect_drone_sim/              # Quadrotor simulator
```

---

## 2. ROS2 Topics Communication Map

### 2.1 SUPER Planner (fsm_node) Topics

**File**: `super_planner/include/ros_interface/ros2/fsm_ros2.hpp`

#### **Subscribed Topics**

| Topic | Message Type | Purpose | Source | Callback | Line |
|-------|-------------|---------|--------|----------|------|
| `/planning/click_goal` | `geometry_msgs/PoseStamped` | Goal position from RViz/Mission | User/Mission Planner | `goalCallback()` | 277-282 |
| *(via ROG-Map)* `/cloud_registered` | `sensor_msgs/PointCloud2` | Point cloud for mapping | FAST-LIO / LiDAR | ROG-Map callback | rog_map_ros2.hpp |
| *(via ROG-Map)* `/lidar_slam/odom` | `nav_msgs/Odometry` | Robot odometry | FAST-LIO | ROG-Map callback | rog_map_ros2.hpp |

#### **Published Topics**

| Topic | Message Type | Purpose | Subscriber | Publish Function | Line |
|-------|-------------|---------|------------|------------------|------|
| `/planning/pos_cmd` | `mars_quadrotor_msgs/PositionCommand` | Position+Velocity+Acceleration command | Controller/Bridge | `cmd_pub_->publish()` | 380 |
| `/planning_cmd/poly_traj` | `mars_quadrotor_msgs/PolynomialTrajectory` | Full polynomial trajectory for MPC | MPC Controller | `mpc_cmd_pub_->publish()` | 379, 84 |
| `/fsm/path` | `nav_msgs/Path` | Executed path history | RViz visualization | `path_pub_->publish()` | 78 |

**Configuration Topic Names** (from `fsm/config.hpp` + `static_high_speed.yaml`):
- `cmd_topic`: Default `/planning/pos_cmd`
- `mpc_cmd_topic`: Default `/planning_cmd/poly_traj`
- `click_goal_topic`: Default `/planning/click_goal`

#### **Timers**

| Timer | Rate | Callback | Purpose | Line |
|-------|------|----------|---------|------|
| `execution_timer_` | 100 Hz (10ms) | `mainFsmTimerCallback()` | FSM state machine update | 325-329 |
| `replan_timer_` | Config: `replan_rate` (default 15 Hz) | `replanTimerCallback()` | Trigger trajectory replanning | 337-343 |
| `cmd_timer_` | 100 Hz (10ms) | `pubCmdTimerCallback()` | Publish commands continuously | 331-336 |

---

### 2.2 Mission Planner (waypoint_mission) Topics

**File**: `mission_planner/include/waypoint_mission/ros2_waypoint_planner.hpp`

#### **Subscribed Topics**

| Topic | Message Type | Purpose | Callback | Line |
|-------|-------------|---------|----------|------|
| `/Odometry` *(or configured)* | `nav_msgs/Odometry` | Current position for waypoint switching | `OdomCallback()` | 52-58, 199-201 |
| `/goal` | `geometry_msgs/PoseStamped` | Manual goal from RViz | `RvizClickCallback()` | 68-77, 214 |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | RC trigger for mission | `MavrosRcCallback()` | 79-93, 218 |

#### **Published Topics**

| Topic | Message Type | Purpose | Publish Function | Line |
|-------|-------------|---------|------------------|------|
| `/planning/click_goal` *(or configured)* | `geometry_msgs/PoseStamped` | Current waypoint goal → SUPER FSM | `goal_pub_->publish()` | 152, 210 |
| `/mission/mkr` | `visualization_msgs/MarkerArray` | Waypoint visualization | `mkr_pub_->publish()` | 237, 221 |

**Configuration Topic Names** (from `mission_planner/config/waypoint.yaml`):
- `odom_topic`: `/Odometry` (needs remapping to `/lidar_slam/odom`)
- `goal_pub_topic`: `/planning/click_goal`

#### **Timers**

| Timer | Rate | Callback | Purpose | Line |
|-------|------|----------|---------|------|
| `goal_pub_timer_` | 100 Hz (10ms) | `GoalPubTimerCallback()` | Check waypoint switching & publish goals | 204-208 |

---

### 2.3 ROG-Map (rog_map) Topics

**File**: `rog_map/include/rog_map_ros/rog_map_ros2.hpp`

#### **Subscribed Topics** (from config `static_high_speed.yaml`)

| Topic | Message Type | Purpose | Line in Config |
|-------|-------------|---------|----------------|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Accumulated point cloud from SLAM | 138 |
| `/lidar_slam/odom` | `nav_msgs/Odometry` | Odometry for map centering | 139 |

#### **Published Topics** (Visualization)

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/rog_map/occupancy` | `sensor_msgs/PointCloud2` | Occupied voxels visualization |
| `/rog_map/inflated` | `sensor_msgs/PointCloud2` | Inflated map visualization |
| `/rog_map/unknown` | `sensor_msgs/PointCloud2` | Unknown cells visualization |

---

## 3. FSM State Machine

**File**: `super_planner/include/fsm/fsm.h` (lines 75-90)

### State Definitions

```cpp
enum MACHINE_STATE {
    INIT = 0,        // Initialization state
    WAIT_GOAL,       // Waiting for goal command
    YAWING,          // (Unused in current version) Yaw alignment
    GENERATE_TRAJ,   // Planning trajectory from rest
    FOLLOW_TRAJ,     // Executing trajectory
    EMER_STOP        // Emergency stop (backup trajectory)
};
```

### State Transition Diagram

```
          START
            │
            ▼
        ┌───────┐
        │ INIT  │◄─────────────────┐
        └───────┘                  │
            │                      │
            │ started_ = true      │
            ▼                      │
      ┌────────────┐               │
      │ WAIT_GOAL  │◄──────┐       │
      └────────────┘       │       │
            │              │       │
            │ new_goal     │       │
            ▼              │       │
   ┌───────────────────┐   │       │
   │  GENERATE_TRAJ    │   │       │
   │  (A* + Corridor + │   │       │
   │   Optimization)   │   │       │
   └───────────────────┘   │       │
            │              │       │
            │ SUCCESS      │       │
            ▼              │       │
      ┌────────────┐       │       │
      │FOLLOW_TRAJ │───────┘       │
      │(Replanning)│               │
      └────────────┘               │
            │                      │
            │ closeToGoal()        │
            └──────────────────────┘

      ┌───────────┐
      │EMER_STOP  │ (Backup trajectory)
      └───────────┘
            │
            └──────────► WAIT_GOAL
```

### State Transition Triggers

**File**: `super_planner/src/super_core/fsm.cpp`

| From State | To State | Trigger Condition | Function | Line |
|------------|----------|-------------------|----------|------|
| INIT | WAIT_GOAL | `started_ = true` | `callMainFsmOnce()` | 120 |
| WAIT_GOAL | GENERATE_TRAJ | `gi_.new_goal = true` | `callMainFsmOnce()` | 127 |
| GENERATE_TRAJ | FOLLOW_TRAJ | `PlanFromRest()` returns SUCCESS | `callMainFsmOnce()` | 155 |
| GENERATE_TRAJ | WAIT_GOAL | `closeToGoal(0.1)` | `callMainFsmOnce()` | 134 |
| FOLLOW_TRAJ | WAIT_GOAL | `traj_finish_ && closeToGoal(0.1)` | `pubCmdTimerCallback()` | 383-384 |
| FOLLOW_TRAJ | EMER_STOP | `ReplanOnce()` returns EMER | `callReplanOnce()` | 73 |
| EMER_STOP | WAIT_GOAL | Immediate transition | `callMainFsmOnce()` | 168 |

### Goal Input Handling

**File**: `super_planner/include/ros_interface/ros2/fsm_ros2.hpp` (line 277-282)

```cpp
void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    Vec3f goal_p = Vec3f{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    Quatf goal_q = Quatf{msg->pose.orientation.w, ...};
    setGoalPosiAndYaw(goal_p, goal_q);  // Sets gi_.new_goal = true
}
```

**`setGoalPosiAndYaw()` (fsm.cpp line 184-200)**:
- Validates goal is not in occupied space
- Snaps goal to nearest free cell (within 3.0m)
- Sets `gi_.new_goal = true`
- Sets `gi_.goal_p` and `gi_.goal_yaw`

---

## 4. Module-by-Module Breakdown

### 4.1 SuperPlanner Core Planning Pipeline

**File**: `super_planner/include/super_core/super_planner.h`

#### Main Planning Functions

| Function | Purpose | Called From | Returns |
|----------|---------|-------------|---------|
| `PlanFromRest(goal_p, goal_yaw, new_goal)` | Plan trajectory from current robot state | FSM::GENERATE_TRAJ | RET_CODE |
| `ReplanOnce(goal_p, goal_yaw, new_goal)` | Replan during trajectory execution | FSM::FOLLOW_TRAJ | RET_CODE |
| `getOneCommandFromTraj(pvaj, yaw, yaw_dot, ...)` | Sample current command from trajectory | pubCmdTimerCallback | void |

#### Planning Pipeline Steps (PlanFromRest)

1. **A* Global Search** (`path_search/astar.h`)
   - Input: Current position → Goal position
   - Output: Waypoint sequence `vector<Vec3f>`
   - Uses: ROG-Map for collision checking

2. **Corridor Generation** (`super_core/corridor_generator.h` + `ciri.h`)
   - Input: A* waypoints + Point cloud
   - Output: Sequence of polytopes (safe flight corridors)
   - Algorithm: CIRI (Closed-loop Incremental Receding horizon Irregular)

3. **Exploratory Trajectory Optimization** (`traj_opt/exp_traj_optimizer_s4.h`)
   - Input: Polytopes + Start/Goal states
   - Output: Smooth 7th-order polynomial trajectory
   - Constraints: Corridors, velocity, acceleration, jerk limits

4. **Backup Trajectory Generation** (`traj_opt/backup_traj_optimizer_s4.h`)
   - Input: Current state
   - Output: Emergency stop trajectory
   - Purpose: Safety fallback if replanning fails

5. **Yaw Trajectory Planning** (`traj_opt/yaw_traj_opt.h`)
   - Input: Position trajectory
   - Output: Yaw angle trajectory
   - Modes: Heading to velocity (mode 1) or heading to goal (mode 2)

---

### 4.2 ROG-Map (Robocentric Occupancy Grid Map)

**File**: `rog_map/include/rog_map_ros/rog_map_ros2.hpp`

#### Key Components

| Component | File | Purpose |
|-----------|------|---------|
| `CounterMap` | `rog_map_core/counter_map.h` | Base occupancy grid with counter-based updates |
| `ProbMap` | `rog_map/prob_map.h` | Probabilistic occupancy map |
| `InfMap` | `rog_map/inf_map.h` | Inflated map for robot radius collision checking |
| `SlidingMap` | `rog_map_core/sliding_map.h` | Sliding window for local map updates |

#### Main Functions

| Function | Purpose | Used By |
|----------|---------|---------|
| `updateMap(cloud, pose)` | Insert point cloud into map | SUPER Planner (indirectly via callback) |
| `isOccupied(position)` | Check if voxel is occupied | A* Search |
| `isLineFree(p1, p2)` | Check line segment collision | Corridor generation |
| `getNearestInfCellNot(type, query, result, max_dis)` | Find nearest cell of given type | Goal validation, corridor |

---

### 4.3 Mission Planner Waypoint Management

**File**: `mission_planner/include/waypoint_mission/ros2_waypoint_planner.hpp`

#### Waypoint Switching Logic (GoalPubTimerCallback, line 95-160)

```
┌─────────────────────────────────────┐
│ Check if close to current waypoint │
│   distance < switch_dis             │
└─────────────────────────────────────┘
            │ YES
            ▼
┌─────────────────────────────────────┐
│ waypoint_counter++                  │
│ new_goal = true                     │
└─────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────┐
│ Publish new waypoint as goal        │
│ → /planning/click_goal              │
└─────────────────────────────────────┘
```

**Trigger Modes** (`start_trigger_type` from config):
- `0`: RVIZ trigger (click `/goal` in RViz)
- `1`: MAVROS trigger (RC channel 10 toggle)
- `2`: Timer trigger (auto-start after delay)

---

## 5. Data Flow Architecture

### 5.1 Complete Data Flow

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         SENSOR INPUTS                                    │
│  ┌──────────────────┐           ┌──────────────────┐                     │
│  │ LiDAR            │           │ IMU              │                     │
│  │ (Gazebo/Real)    │           │ (Gazebo/Real)    │                     │
│  └──────────────────┘           └──────────────────┘                     │
└──────────────────────────────────────────────────────────────────────────┘
            │                                 │
            │ /sim_lidar/lidar                │ /sim_imu/imu
            │                                 │
            ▼                                 ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                         FAST-LIO ROS2                                    │
│                    (Localization & Mapping)                              │
└──────────────────────────────────────────────────────────────────────────┘
            │                                 │
            │ /cloud_registered               │ /Odometry
            │ (world frame)                   │ (camera_init → body)
            ▼                                 ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                          ROG-MAP                                         │
│              (Robocentric Occupancy Grid Map)                            │
│  ┌──────────────────────────────────────────────────────────────────┐    │
│  │ • Receives point cloud + odometry                                │    │
│  │ • Builds local occupancy grid (sliding window)                   │    │
│  │ • Inflates obstacles by robot radius                             │    │
│  │ • Provides collision checking API                                │    │
│  └──────────────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────────────┘
            │
            │ (Direct API calls, no topics)
            │
            ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                       SUPER PLANNER FSM                                 │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ State: WAIT_GOAL                                                 │   │
│  │  ┌─ Subscribes: /planning/click_goal                             │   │
│  │  │             (from Mission Planner or RViz)                    │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│            │ new_goal = true                                            │
│            ▼                                                            │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ State: GENERATE_TRAJ                                             │   │
│  │  ├─ Step 1: A* Search (ROG-Map)                                  │   │
│  │  │   Output: Waypoint path                                       │   │
│  │  ├─ Step 2: CIRI Corridor Generation                             │   │
│  │  │   Output: Safe polytope sequence                              │   │
│  │  ├─ Step 3: MINCO Trajectory Optimization                        │   │
│  │  │   Output: Smooth polynomial trajectory                        │   │
│  │  ├─ Step 4: Backup Trajectory (emergency)                        │   │
│  │  └─ Step 5: Yaw Trajectory                                       │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│            │ SUCCESS                                                    │
│            ▼                                                            │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ State: FOLLOW_TRAJ                                               │   │
│  │  ├─ Continuous replanning (replan_timer @ 15 Hz)                 │   │
│  │  └─ Command publishing (cmd_timer @ 100 Hz)                      │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
            │                                 │
            │ /planning/pos_cmd               │ /planning_cmd/poly_traj
            │ (PositionCommand)               │ (PolynomialTrajectory)
            │                                 │
            ▼                                 ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                    CONTROLLER / BRIDGE                                   │
│  • px4_super_bridge (YOUR INTEGRATION)                                   │
│  • OR Perfect Drone Sim (SUPER's simulator)                              │
│  • OR Custom MPC controller                                              │
└──────────────────────────────────────────────────────────────────────────┘
            │
            │ Control commands (Twist, Attitude, etc.)
            ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                         VEHICLE                                          │
│  • PX4 Autopilot (Real/SITL)                                             │
│  • OR SUPER's Perfect Drone Sim                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

### 5.2 Mission Planner Data Flow

```
┌─────────────────────────────────────┐
│   Waypoint File (benchmark.txt)     │
│   or RViz Click (/goal)             │
└─────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────┐
│    Mission Planner (waypoint_       │
│     mission node)                   │
│  ┌──────────────────────────────┐   │
│  │ • Load waypoint list         │   │
│  │ • Subscribe /Odometry        │   │
│  │ • Check distance to waypoint │   │
│  │ • Switch when close          │   │
│  └──────────────────────────────┘   │
└─────────────────────────────────────┘
            │
            │ /planning/click_goal
            │ (Current waypoint)
            ▼
┌─────────────────────────────────────┐
│       SUPER FSM                     │
│   (Receives as new goal)            │
└─────────────────────────────────────┘
```

---

## 6. Key Modification Points

### 6.1 Change Command Output Topic

**File**: `super_planner/config/static_high_speed.yaml`

```yaml
fsm:
  cmd_topic: "/your_custom_topic"        # Default: "/planning/pos_cmd"
  mpc_cmd_topic: "/your_mpc_topic"       # Default: "/planning_cmd/poly_traj"
```

**Implementation**: Line 298 in `fsm_ros2.hpp`

### 6.2 Change Goal Input Topic

**File**: `super_planner/config/static_high_speed.yaml`

```yaml
fsm:
  click_goal_topic: "/your_goal_topic"   # Default: "/planning/click_goal"
```

**Implementation**: Line 309-313 in `fsm_ros2.hpp`

### 6.3 Change Odometry Input Topic

**Method 1: Launch file remapping** (Recommended)
```bash
ros2 run super_planner fsm_node \
  --ros-args -r /Odometry:=/your_odom_topic
```

**Method 2: Config file** (`static_high_speed.yaml`)
```yaml
rog_map:
  ros_callback:
    odom_topic: "/your_odom_topic"       # Default: "/lidar_slam/odom"
```

### 6.4 Change Point Cloud Input Topic

**File**: `super_planner/config/static_high_speed.yaml`

```yaml
rog_map:
  ros_callback:
    cloud_topic: "/your_cloud_topic"     # Default: "/cloud_registered"
```

### 6.5 Disable Mission Planner (Click-Only Mode)

**File**: Launch file or config

**Option 1**: Don't launch mission_planner node
**Option 2**: Use RViz 2D Goal Pose tool directly

### 6.6 Add Custom Planner Module

**Modification Points**:

1. **Add to planning pipeline** (`super_planner.cpp`)
   - Modify `PlanFromRest()` or `ReplanOnce()`
   - Insert custom logic between A* and corridor generation
   - Or replace corridor generation entirely

2. **Add new ROS topic** (`fsm_ros2.hpp`)
   ```cpp
   // In init() function (line 284+)
   custom_pub_ = nh_->create_publisher<YourMsgType>("/custom_topic", qos);

   // In appropriate callback
   custom_pub_->publish(your_message);
   ```

3. **Add configuration parameters** (`config.hpp` + `*.yaml`)

### 6.7 Change Control Rate

**File**: `super_planner/config/static_high_speed.yaml`

```yaml
fsm:
  replan_rate: 10.0    # Replanning frequency (Hz), default: 15.0
```

**Command publish rate**: Fixed at 100 Hz (line 332 in `fsm_ros2.hpp`)
- Modify timer period to change: `std::chrono::milliseconds(10)` → `(20)` for 50 Hz

---

## 7. Message Type Reference

### 7.1 PositionCommand.msg

**File**: `mars_quadrotor_msgs/msg/PositionCommand.msg`

```
Header header
geometry_msgs/Point position       # Target position (m)
geometry_msgs/Vector3 velocity     # Target velocity (m/s)
geometry_msgs/Vector3 acceleration # Target acceleration (m/s²)
geometry_msgs/Vector3 jerk         # Target jerk (m/s³)
geometry_msgs/Vector3 angular_velocity  # Angular velocity (rad/s)
geometry_msgs/Vector3 attitude     # Roll, pitch, yaw (rad)
geometry_msgs/Vector3 thrust       # Thrust vector
float64 yaw                        # Yaw angle (rad)
float64 yaw_dot                    # Yaw rate (rad/s)
float64 vel_norm                   # Velocity magnitude
float64 acc_norm                   # Acceleration magnitude
float64[3] kx                      # Position gains (for PID)
float64[3] kv                      # Velocity gains (for PID)
uint32 trajectory_id               # Trajectory ID
uint8 trajectory_flag              # Status: READY=1, EMER=2, ...
```

**Populated in**: `fsm_ros2.hpp`, line 141-178

### 7.2 PolynomialTrajectory.msg

**File**: `mars_quadrotor_msgs/msg/PolynomialTrajectory.msg`

```
Header header
uint32 trajectory_id
uint32 type                    # Bit flags: POSITION_TRAJ | YAW_TRAJ | HEART_BEAT
uint32 piece_num_pos           # Number of polynomial pieces (position)
uint32 piece_num_yaw           # Number of polynomial pieces (yaw)
uint32 order_pos               # Polynomial order (default: 7)
uint32 order_yaw               # Polynomial order (default: 7)
time start_WT_pos              # Start wall time (position)
time start_WT_yaw              # Start wall time (yaw)
float64[] coef_pos_x           # Coefficients for X trajectory
float64[] coef_pos_y           # Coefficients for Y trajectory
float64[] coef_pos_z           # Coefficients for Z trajectory
float64[] time_pos             # Duration of each piece
float64[] coef_yaw             # Coefficients for yaw trajectory
float64[] time_yaw             # Duration of yaw pieces
```

**Populated in**: `fsm_ros2.hpp`, line 96-139

---

## 8. Integration Points Summary

### For px4_super_bridge Integration

| What to Connect | SUPER Provides | Your Bridge Must | Destination |
|-----------------|----------------|------------------|-------------|
| **Goal Input** | Subscribes `/planning/click_goal` | Publish goals here | Mission planner or manual |
| **Odometry Input** | Subscribes `/lidar_slam/odom` (via ROG-Map) | Remap from FAST-LIO `/Odometry` | - |
| **Point Cloud Input** | Subscribes `/cloud_registered` (via ROG-Map) | Provide from FAST-LIO | - |
| **Command Output** | Publishes `/planning/pos_cmd` | Subscribe & convert to Twist | px4_offboard_sim |
| **Trajectory Output** | Publishes `/planning_cmd/poly_traj` | (Optional) For MPC controller | - |

### Topic Remapping for Your Setup

```bash
# Launch SUPER with remapped topics
ros2 run super_planner fsm_node \
  --ros-args \
  --params-file static_high_speed.yaml \
  -r /Odometry:=/lidar_slam/odom \
  -r /cloud_registered:=/cloud_registered
```

---

## 9. Debugging & Monitoring

### Key Logging Points

**FSM State**: Console output every 1 second (fsm.cpp line 98-110)
```
[Fsm X.XXX] Current state: WAIT_GOAL | GENERATE_TRAJ | FOLLOW_TRAJ
```

**Planning Success**: Green messages (fsm.cpp line 70)
```
 -- [Fsm] ReplanOnce succeed.
```

**Goal Reception**: Yellow message (fsm.cpp line 192)
```
 -- [Fsm] Get goal at [x y z]
```

### Useful Commands

```bash
# Monitor FSM state transitions
ros2 topic echo /fsm/path

# Check command publish rate
ros2 topic hz /planning/pos_cmd

# Check if goal received
ros2 topic echo /planning/click_goal

# Visualize in RViz
ros2 run rviz2 rviz2
# Add: /fsm/path (Path), /planning/click_goal (PoseStamped)
```

---

## 10. Configuration File Structure

### static_high_speed.yaml Structure

```yaml
fsm:                    # FSM node configuration
  click_goal_en: true
  click_goal_topic: "/planning/click_goal"
  replan_rate: 15.0
  cmd_topic: "/planning/pos_cmd"
  mpc_cmd_topic: "/planning_cmd/poly_traj"

super_planner:          # SuperPlanner core configuration
  backup_traj_en: true
  visualization_en: true
  planning_horizon: 30.0
  robot_r: 0.2

traj_opt:               # Trajectory optimization parameters
  boundary:
    max_vel: 25.0
    max_acc: 20.0
    max_jerk: 120.0
  exp_traj:             # Exploratory trajectory
    penna_vel: 5.0e+8
  backup_traj:          # Backup trajectory
    piece_num: 2

astar:                  # A* search parameters
  map_voxel_num: [500, 500, 100]
  heu_type: 2           # 0=DIAG, 1=MANHATTAN, 2=EUCLIDEAN

rog_map:                # ROG-Map configuration
  resolution: 0.3
  map_size: [15, 110, 6]
  ros_callback:
    cloud_topic: "/cloud_registered"
    odom_topic: "/lidar_slam/odom"
```

---

## Conclusion

This architecture breakdown provides all necessary information to:
1. ✅ Understand SUPER's internal communication
2. ✅ Modify topics for integration
3. ✅ Replace components (e.g., mission planner)
4. ✅ Debug data flow issues
5. ✅ Extend functionality

**Key Files for Modification**:
- **Topics**: `fsm_ros2.hpp` (publishers/subscribers)
- **Config**: `static_high_speed.yaml` (topic names, rates, parameters)
- **State Machine**: `fsm.cpp` + `fsm.h` (behavior logic)
- **Planning**: `super_planner.cpp` (algorithm pipeline)

**Author**: Analysis by Claude Code for SUPER integration
**SUPER Repository**: https://github.com/hku-mars/SUPER
