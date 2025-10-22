# RViz Visualization Guide for SUPER Integration

**Complete guide for visualizing FAST-LIO + ROG-MAP + SUPER pipeline**

Date: 2025-10-21

---

## Quick Start

### Launch RViz with SUPER Integration Config

```bash
cd ~/super_ws
source install/setup.bash
rviz2 -d ~/super_ws/src/SUPER/super_planner/rviz/super_integration.rviz
```

This will load a pre-configured RViz setup with all necessary displays for the complete pipeline.

---

## RViz Configuration Overview

**File**: `super_planner/rviz/super_integration.rviz`

**Fixed Frame**: `camera_init` (FAST-LIO world frame)

**Update Rate**: 30 Hz

### Display Organization

The RViz config is organized into three main sections:

1. **Localization (FAST-LIO)** - Green elements
2. **Mapping (FAST-LIO + ROG-MAP)** - White/Red elements
3. **Planning (SUPER)** - Blue/Yellow/Orange elements

---

## Display Reference

### 1. LOCALIZATION (FAST-LIO)

#### TF Frames
- **Purpose**: Shows coordinate frame transformations
- **Frames**: `camera_init` → `body`
- **Update**: Real-time
- **Visibility**: Always enabled

#### Odometry
- **Topic**: `/Odometry`
- **Type**: `nav_msgs/Odometry`
- **Color**: Red arrow (255, 25, 0)
- **Purpose**: Current vehicle pose from SLAM
- **Keep**: Last 100 poses
- **Shape**: Arrow (1m axes length)

#### SLAM Path
- **Topic**: `/path`
- **Type**: `nav_msgs/Path`
- **Color**: Green (25, 255, 0)
- **Line Width**: 0.03m
- **Purpose**: Historical trajectory from FAST-LIO SLAM
- **Style**: Solid lines

---

### 2. MAPPING (FAST-LIO + ROG-MAP)

#### CloudRegistered
- **Topic**: `/cloud_registered`
- **Type**: `sensor_msgs/PointCloud2`
- **Color**: Rainbow (axis-colored by Z)
- **Point Size**: 0.05m
- **Decay Time**: 30 seconds
- **Purpose**: SLAM-registered point cloud map
- **Style**: Flat Squares
- **Enabled**: Yes

#### CloudEffected (Optional)
- **Topic**: `/cloud_effected`
- **Type**: `sensor_msgs/PointCloud2`
- **Color**: White/Intensity
- **Point Size**: 0.1m
- **Purpose**: Undistorted raw point cloud
- **Enabled**: No (enable for debugging)

#### ROG-MAP Occupancy
- **Topic**: `/rog_map/occupancy`
- **Type**: `sensor_msgs/PointCloud2`
- **Color**: Red (255, 0, 0)
- **Point Size**: 0.15m
- **Style**: Boxes
- **Purpose**: Occupied voxels detected by ROG-MAP
- **Enabled**: Yes

#### ROG-MAP Inflated
- **Topic**: `/rog_map/inflated`
- **Type**: `sensor_msgs/PointCloud2`
- **Color**: Orange (255, 170, 0)
- **Point Size**: 0.12m
- **Style**: Boxes
- **Purpose**: Safety-inflated obstacles
- **Enabled**: No (enable to see safety margins)

---

### 3. SUPER PLANNING

#### Goal Pose
- **Topic**: `/goal_pose`
- **Type**: `geometry_msgs/PoseStamped`
- **Color**: Red (255, 25, 0)
- **Shape**: Arrow (1m shaft, 0.3m head)
- **Purpose**: Target goal position from mission planner or RViz click
- **Enabled**: Yes

#### Flight Path (Executed)
- **Topic**: `/fsm/path`
- **Type**: `nav_msgs/Path`
- **Color**: Blue (0, 0, 255)
- **Line Width**: 0.05m
- **Purpose**: Actual executed trajectory from SUPER FSM
- **Style**: Solid lines
- **Enabled**: Yes

#### Planned Trajectory
- **Topic**: `/trajectory`
- **Type**: `visualization_msgs/MarkerArray`
- **Purpose**: Optimized MINCO trajectory from planner
- **Markers**: LINE_STRIP showing smooth polynomial trajectory
- **Enabled**: Yes

#### Safe Corridors
- **Topic**: `/corridor`
- **Type**: `visualization_msgs/MarkerArray`
- **Purpose**: Collision-free corridors (CIRI algorithm)
- **Markers**: Semi-transparent green boxes
- **Enabled**: Yes

#### A* Search Path
- **Topic**: `/searched_path`
- **Type**: `visualization_msgs/Marker`
- **Purpose**: Global A* path through occupancy grid
- **Color**: Yellow/Cyan
- **Style**: Dashed line or waypoint spheres
- **Enabled**: Yes

#### Backup Trajectory
- **Topic**: `/backup_traj`
- **Type**: `visualization_msgs/MarkerArray`
- **Purpose**: Emergency stop trajectory
- **Color**: Orange/Red
- **Enabled**: Yes
- **Note**: Only visible when backup trajectory is active

#### Yaw Trajectory (Optional)
- **Topic**: `/yaw_traj`
- **Type**: `visualization_msgs/MarkerArray`
- **Purpose**: Heading direction over time
- **Enabled**: No (enable for yaw debugging)

#### Planning Markers
- **Topic**: `/visualization_marker_array`
- **Type**: `visualization_msgs/MarkerArray`
- **Purpose**: Additional debug markers from planner
- **Enabled**: Yes

---

## Full Integration Workflow

### Terminal Setup (4 terminals)

#### Terminal 1: Gazebo + PX4
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch px4_offboard_sim simulation.launch.py
```
**Wait for**: Gazebo GUI opens, vehicle spawns

#### Terminal 2: FAST-LIO
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch fast_lio_ros2 simulation_mapping.launch.py config_file:=gazebosim.yaml
```
**Wait for**: "Publishing to /Odometry" message

#### Terminal 3: SUPER Planner (with RViz)
```bash
cd ~/super_ws
source install/setup.bash
ros2 launch super_planner test_super_planner.launch.py
```
**Wait for**: "Odometry received! Building map..." message

**Note**: This automatically launches RViz with `super_test.rviz`. To disable RViz:
```bash
ros2 launch super_planner test_super_planner.launch.py rviz:=false
```

#### Terminal 4 (Optional): RViz with Full Integration Config
```bash
# Use this if you want the comprehensive visualization config instead
cd ~/super_ws
source install/setup.bash
rviz2 -d ~/super_ws/src/SUPER/super_planner/rviz/super_integration.rviz
```

---

## Using RViz Tools

### Setting a Goal

**Method 1: Using "2D Goal Pose" Tool**
1. Click the "2D Goal Pose" button in the toolbar
2. Click on the map to set goal position
3. Drag to set goal orientation
4. Goal will be published to `/goal_pose`

**Method 2: Using Command Line**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "camera_init"},
  pose: {position: {x: 5.0, y: 0.0, z: 1.5},
         orientation: {w: 1.0}}}'
```

### Camera Controls

**Orbit View (Default)**:
- **Left Click + Drag**: Rotate around focal point
- **Middle Click + Drag**: Pan camera
- **Scroll Wheel**: Zoom in/out
- **Shift + Left Click**: Move focal point

**Recommended Settings**:
- **Distance**: 25m (good overview for local planning)
- **Focal Point**: (0, 0, 1.5) - typical flight altitude
- **Pitch**: 45° (0.785 rad)
- **Yaw**: 45° (0.785 rad)

### Useful View Presets

**Top-Down View** (for 2D planning visualization):
- Pitch: 0° (looking straight down)
- Yaw: 0°
- Distance: 30m

**Side View** (for vertical trajectory):
- Pitch: 90° (1.57 rad)
- Yaw: 0° or 90°
- Distance: 20m

**Follow Vehicle**:
- Set Target Frame: `body` (instead of `<Fixed Frame>`)
- Distance: 10m
- Pitch: 60°

---

## Visualization Checklist

Use this checklist when starting a new test:

### Initial State (Before Goal)
- [ ] TF shows `camera_init` → `body` transform
- [ ] Odometry arrow is visible and updating
- [ ] SLAM Path (green) is growing as vehicle moves
- [ ] CloudRegistered shows environment point cloud
- [ ] ROG-MAP Occupancy (red boxes) shows obstacles
- [ ] No trajectory displays active yet

### After Sending Goal
- [ ] Goal Pose (red arrow) appears at target location
- [ ] A* Search Path appears (yellow/cyan line)
- [ ] Safe Corridors appear (green semi-transparent boxes)
- [ ] Planned Trajectory appears (smooth curve through corridors)
- [ ] Flight Path (blue) starts growing as vehicle moves
- [ ] `/planning/pos_cmd` is publishing at 100 Hz (check with `ros2 topic hz`)

### During Flight
- [ ] Planned Trajectory updates with replanning
- [ ] Flight Path (blue) follows Planned Trajectory closely
- [ ] Safe Corridors update as map changes
- [ ] Backup Trajectory visible if emergency stop triggered
- [ ] Odometry and SLAM Path continue updating

---

## Troubleshooting Display Issues

### Issue: No point cloud visible

**Symptoms**: CloudRegistered display is empty

**Debug**:
```bash
ros2 topic hz /cloud_registered
ros2 topic echo /cloud_registered --once | head -50
```

**Solutions**:
- Check FAST-LIO is running and publishing
- Verify Fixed Frame is set to `camera_init`
- Check CloudRegistered display is enabled
- Try increasing Decay Time to 60 seconds
- Verify point cloud has data (should show > 1000 points)

### Issue: No ROG-MAP occupancy

**Symptoms**: No red obstacle boxes visible

**Debug**:
```bash
ros2 topic hz /rog_map/occupancy
ros2 node info /fsm_node | grep -A20 "Subscriptions"
```

**Solutions**:
- Verify ROG-MAP visualization is enabled in config:
  ```yaml
  rog_map:
    visualization:
      enable: true
  ```
- Check that robot has moved to build map
- Ensure obstacles are in sensor range
- Verify `/cloud_registered` is being received by ROG-MAP

### Issue: No trajectory visible

**Symptoms**: No planned trajectory after sending goal

**Debug**:
```bash
ros2 topic hz /trajectory
ros2 topic list | grep -E "(trajectory|corridor|searched_path)"
```

**Solutions**:
- Verify goal was received: `ros2 topic echo /goal_pose --once`
- Check SUPER FSM logs for planning errors
- Ensure goal is in free space (not inside obstacle)
- Verify visualization is enabled in config:
  ```yaml
  super_planner:
    visualization_en: true
  ```

### Issue: TF errors

**Symptoms**: "No transform from [frame] to [frame]" errors

**Debug**:
```bash
ros2 run tf2_ros tf2_echo camera_init body
ros2 run tf2_tools view_frames
```

**Solutions**:
- Verify FAST-LIO is publishing TF
- Check Fixed Frame is set to `camera_init`
- Ensure all displays use correct frame IDs
- Restart FAST-LIO if TF tree is broken

### Issue: RViz is slow/laggy

**Symptoms**: Low frame rate, freezing

**Solutions**:
- Reduce point cloud size:
  - CloudRegistered: Increase "Size (m)" to 0.1 or disable
  - ROG-MAP: Disable inflated display if not needed
- Reduce Decay Time for point clouds
- Disable unused displays
- Reduce visualization publish rate in config:
  ```yaml
  rog_map:
    visualization:
      time_rate: 1  # Reduce from 2 Hz to 1 Hz
  ```

---

## Display Color Reference

Quick reference for identifying elements:

| Color | Display | Topic |
|-------|---------|-------|
| Green | SLAM Path | `/path` |
| Red | Odometry Arrow | `/Odometry` |
| Red | Goal Pose | `/goal_pose` |
| Red Boxes | ROG-MAP Occupancy | `/rog_map/occupancy` |
| Orange Boxes | ROG-MAP Inflated | `/rog_map/inflated` |
| Blue | Flight Path (Executed) | `/fsm/path` |
| Green | Planned Trajectory | `/trajectory` |
| Green Transparent | Safe Corridors | `/corridor` |
| Yellow/Cyan | A* Search Path | `/searched_path` |
| Orange/Red | Backup Trajectory | `/backup_traj` |
| Rainbow (Z-axis) | Point Cloud | `/cloud_registered` |

---

## Advanced Customization

### Changing Display Colors

To customize colors, edit `super_integration.rviz` and find the display section:

```yaml
- Alpha: 1
  Buffer Length: 1
  Class: rviz_default_plugins/Path
  Color: 0; 0; 255  # RGB values (0-255)
  Name: Flight Path (Executed)
  Topic:
    Value: /fsm/path
```

### Adding Custom Displays

To add a new display manually in RViz:
1. Click "Add" button in Displays panel
2. Select display type (Path, PointCloud2, MarkerArray, etc.)
3. Configure topic and visualization properties
4. Save config: File → Save Config As...

### Saving Custom Views

To save current camera view:
1. Position camera as desired
2. In Views panel, click "Save current as..." under Saved views
3. Give it a descriptive name (e.g., "Top-Down", "Side View")
4. Save RViz config to persist

---

## Performance Tips

**For High-End Systems**:
- Enable all displays
- Increase point cloud quality (smaller point size)
- Enable ROG-MAP inflated display
- Increase visualization rates

**For Low-End Systems**:
- Disable CloudEffected display
- Disable ROG-MAP inflated display
- Increase point size to 0.1-0.15m
- Reduce visualization rate to 1 Hz
- Disable Yaw Trajectory display

**Recommended Settings** (balanced):
```yaml
rog_map:
  visualization:
    enable: true
    time_rate: 2          # 2 Hz update
    range: [20, 20, 5]    # Moderate range

CloudRegistered:
  Size (m): 0.05          # Medium quality
  Decay Time: 30          # Keep 30 seconds of history
```

---

## Integration with Mission Planning

### Waypoint Mission Visualization

When integrating with a waypoint mission planner, you can add:

1. **Mission Waypoints Display**:
   - Topic: `/mission/waypoints`
   - Type: `visualization_msgs/MarkerArray`
   - Color: Purple spheres

2. **Current Waypoint Display**:
   - Topic: `/mission/current_waypoint`
   - Type: `geometry_msgs/PoseStamped`
   - Color: Bright green arrow

3. **Geofence Display**:
   - Topic: `/mission/geofence`
   - Type: `visualization_msgs/Marker`
   - Color: Red wireframe box

### RC-Triggered Goal

If using RC switch to trigger goals, the `/goal_pose` will automatically update and the red goal arrow will move to the new position.

---

## Comparison with FAST-LIO RViz Config

**Original**: `fast_lio_ros2/rviz/fastlio.rviz`

**SUPER Integration**: `super_planner/rviz/super_integration.rviz`

**Changes Made**:
- ✅ Added 8 new SUPER planning displays
- ✅ Added 2 ROG-MAP displays
- ✅ Organized displays into logical sections
- ✅ Adjusted default camera view for local planning
- ✅ Added comments for clarity
- ✅ Kept all original FAST-LIO displays

**What's Preserved**:
- All FAST-LIO displays (TF, Odometry, Path, CloudRegistered)
- Same Fixed Frame (`camera_init`)
- Same tools (2D Goal Pose, Publish Point, etc.)
- Same window layout

---

## Quick Reference Commands

### Launch Everything
```bash
# Terminal 1
ros2 launch px4_offboard_sim simulation.launch.py

# Terminal 2
ros2 launch fast_lio_ros2 simulation_mapping.launch.py config_file:=gazebosim.yaml

# Terminal 3 (with automatic RViz)
ros2 launch super_planner test_super_planner.launch.py

# OR Terminal 3 (without RViz) + Terminal 4 (custom RViz)
ros2 launch super_planner test_super_planner.launch.py rviz:=false
rviz2 -d ~/super_ws/src/SUPER/super_planner/rviz/super_integration.rviz
```

### Monitor Topics
```bash
# Check all planning topics
ros2 topic list | grep -E "(planning|trajectory|corridor|fsm)"

# Check topic rates
ros2 topic hz /Odometry              # ~40 Hz
ros2 topic hz /cloud_registered      # ~10 Hz
ros2 topic hz /planning/pos_cmd      # ~100 Hz (after goal)
ros2 topic hz /trajectory            # ~10 Hz (during planning)
```

### Send Test Goal
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "camera_init"},
  pose: {position: {x: 5.0, y: 0.0, z: 1.5},
         orientation: {w: 1.0}}}'
```

---

## Related Documentation

- **Complete Integration Guide**: `QUICK_START_INTEGRATION.md`
- **Topic Mapping Reference**: `INTEGRATION_TOPIC_MAP.md`
- **Detailed Test Procedure**: `ROG_MAP_INTEGRATION_TEST.md`
- **Architecture Overview**: `ARCHITECTURE_BREAKDOWN.md`
- **Topic Quick Reference**: `TOPIC_MAP_QUICK_REFERENCE.md`

---

**Status**: RViz integration complete ✅

**Next Steps**: Follow `QUICK_START_INTEGRATION.md` to test the full pipeline!
