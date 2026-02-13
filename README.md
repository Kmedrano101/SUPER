<div align="center">
    <h2>SUPER: Safety-assured High-speed Navigation for MAVs</h2>
    <strong>Science Robotics' 25</strong>
    <br>
    <br>
    <strong>Repository Maintainer:</strong> <a href="https://github.com/Kmedrano101" target="_blank">Kevin Medrano</a>
    <br>
    <em>This is an adapted version of SUPER to work with Gazebo Sim and ROS2 framework</em>
    <br>
    <br>
    <strong>Original Authors:</strong>
    <br>
        <a href="https://github.com/RENyunfan" target="_blank">Yunfan REN</a>,
<a href="https://github.com/zfc-zfc" target="_blank">Fangcheng Zhu</a>,
    <a href="https://github.com/genegzl" target="_blank">Guozheng Lu</a>,
    <a href="https://github.com/Ecstasy-EC" target="_blank">Yixi Cai</a>,
    <a href="https://github.com/YLJ6038" target="_blank">Longji Yin</a>,
    <a href="https://github.com/jackykongfz" target="_blank">Fanze Kong</a>,
    <a href="https://github.com/ziv-lin" target="_blank">Jiarong Lin</a>,
    <a href="https://github.com/lawrence-cn" target="_blank">Nan Chen</a>, and
        <a href="https://mars.hku.hk/people.html" target="_blank">Fu Zhang</a>
    <p>
        <h45>
            <br>
           <img src='./misc/mars_logo.svg' alt='HKU MaRS Lab'>
            <br>
        </h5>
    </p>
    <a href='https://www.science.org/doi/10.1126/scirobotics.ado6187'><img src='./misc/arXiv-super.svg' alt='arxiv'></a>
    <a href="https://www.bilibili.com/video/BV1BSFgeJEJn/"><img alt="Bilibili" src="./misc/Video-Bilibili-blue.svg"/></a>
    <a href="https://youtu.be/GPHuzG0ANmI?si=npW-FNp1rkQQ5YaF"><img alt="Youtube" src="./misc/Video-Youtube-red.svg"/></a>
</div>



# Updates
* **Feb. 13, 2026** - Adapted SUPER for **Gazebo Sim + PX4 SITL** integration with FAST-LIO SLAM. Removed MARSIM dependency. Added `px4_super_bridge` and goal-reached position hold.
* **Mar. 09, 2025** - The hardware components of SUPER have been released at [SUPER-Hardware](https://github.com/hku-mars/SUPER-Hardware) 🦾
* **Jan. 29, 2025** - The preview version of SUPER's planning module, supporting both ROS1 and ROS2, is now available! Try it out, and we welcome any issues or contributions.
* **Jan. 29, 2025** - The paper of SUPER is now featured on the official website of [*Science Robotics*](https://www.science.org/doi/10.1126/scirobotics.ado6187).
* **Dec. 12, 2024** - 🎉 Our paper has been accepted by *Science Robotics*!

Our paper is also aviliable at [here](misc/scirobotics.ado6187.pdf). If our repository supports your academic projects, please cite our work. Thank you!

```tex
@article{ren2025safety,
  title={Safety-assured high-speed navigation for MAVs},
  author={Ren, Yunfan and Zhu, Fangcheng and Lu, Guozheng and Cai, Yixi and Yin, Longji and Kong, Fanze and Lin, Jiarong and Chen, Nan and Zhang, Fu},
  journal={Science Robotics},
  volume={10},
  number={98},
  pages={eado6187},
  year={2025},
  publisher={American Association for the Advancement of Science}
}

@article{lu2025autonomous,
  title={Autonomous Tail-Sitter Flights in Unknown Environments},
  author={Lu, Guozheng and Ren, Yunfan and Zhu, Fangcheng and Li, Haotian and Xue, Ruize and Cai, Yixi and Lyu, Ximin and Zhang, Fu},
  journal={IEEE Transactions on Robotics},
  year={2025},
  publisher={IEEE}
}

@inproceedings{ren2024rog,
  title={Rog-map: An efficient robocentric occupancy grid map for large-scene and high-resolution lidar-based motion planning},
  author={Ren, Yunfan and Cai, Yixi and Zhu, Fangcheng and Liang, Siqi and Zhang, Fu},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={8119--8125},
  year={2024},
  organization={IEEE}
}
```

# 1 Highlights

## 1.1 Autonomous Navigation in Challenging Environments
(Click for video demo)
[![Video Demo](./misc/fig1.gif)](https://youtu.be/GPHuzG0ANmI?si=W83mDMxqfgWReWPF)

## 1.2 Applications: Object Tracking & Autonomous Exploration

SUPER has been successfully deployed in various applications, including large-scale autonomous exploration in an ongoing project by [@jackykongfz](https://github.com/jackykongfz) and [@ZbyLGsc](https://github.com/ZbyLGsc) from [STAR Lab](sysu-star.com), among others, as well as object tracking under both day and night conditions.

![exp](./misc/exp.gif)
> ⬆️ This segment is from an unpublished work by Kong [[@jackykongfz](https://github.com/jackykongfz) ] et al., conducted in collaboration with [STAR Lab](sysu-star.com), using SUPER.

![tracking](./misc/tracking.gif)



## 1.3 Supported Projects

### 1.3.1 Autonomous Tail-Sitter (TRO '25)

Building on SUPER, a similar planning system has been successfully validated in [Autonomous Navigation for Tail-Sitter UAVs](https://github.com/hku-mars/EFOPT)  by [@genegzl](https://github.com/genegzl)  et al.

![tailsitter](./misc/tailsitter.gif)

### 1.3.2 FAST-LIVO2 (TRO '24)
SUPER serves as the flight platform and navigation system in the video demonstration of [FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://github.com/hku-mars/FAST-LIVO2) by [@xuankuzcr](https://github.com/xuankuzcr) et al.

<img src="./misc/image-20250130031404057.png" alt="image-20250130031404057" style="zoom:50%;" />

# 2 Gazebo Sim + PX4 Integration

> **Adapted by [@Kmedrano101](https://github.com/Kmedrano101)**
>
> Key modifications from the original SUPER repository:
> - Replaced MARSIM simulator with **Gazebo Sim + PX4 SITL** for realistic physics and sensor simulation
> - Integrated **FAST-LIO** for LiDAR-inertial odometry (namespaced topics: `/fast_lio_ros2/*`)
> - Created **[px4_super_bridge](https://github.com/Kmedrano101)** package to convert SUPER PositionCommand (ENU) to PoseStamped for PX4
> - Updated `click.yaml` config for Gazebo integration (topics, frame_id, velocity limits, virtual ground)
> - Added `full_integration_test.launch.py` for launching the complete planning stack
> - Removed `mars_uav_sim` dependency (MARSIM no longer required)
> - Configured for **ROS2 Jazzy** + **Ubuntu 24.04**

This adapted version replaces MARSIM with **Gazebo Sim** and **PX4 SITL** for simulation, using **FAST-LIO** for LiDAR-inertial odometry. The pipeline is:

```
PX4 SITL + Gazebo Sim  -->  FAST-LIO SLAM  -->  ROG-Map  -->  SUPER Planner  -->  px4_super_bridge  -->  px4_offboard_sim  -->  PX4
       (sensors)            (odometry + map)    (occupancy)    (trajectory)       (ENU->PoseStamped)     (NED setpoints)
```

## 2.1 Prerequisites

**Required packages** (built in `~/ros2_ws`):
- [px4_offboard_sim](https://github.com/Kmedrano101/px4_offboard_sim) - Offboard flight control with TRAJECTORY mode
- [fast_lio_ros2](https://github.com/Kmedrano101/fast_lio_ros2) - FAST-LIO SLAM adapted for Gazebo simulation
- [px4_msgs](https://github.com/PX4/px4_msgs) - PX4 message definitions
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) - PX4 SITL firmware

**System dependencies:**
```bash
# Eigen and soft link
sudo apt-get install libeigen3-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
# dw for backward cpp
sudo apt-get install libdw-dev
# ROS2 dependencies
sudo apt-get install ros-jazzy-pcl-ros ros-jazzy-tf2-ros
```

**Tested environment:** Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic + PX4 v1.15

## 2.2 Installation

```bash
# Build SUPER + px4_super_bridge
mkdir -p ~/super_ws/src && cd ~/super_ws/src
git clone <this-repo> SUPER
git clone <bridge-repo> px4_super_bridge

cd ~/super_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash  # for px4_msgs
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 2.3 Running the Full Stack

Launch each component in separate terminals:

```bash
# Terminal 1: PX4 SITL + Gazebo
cd ~/ros2_ws && source install/setup.bash
ros2 launch px4_offboard_sim sim.launch.py world:=obstacle_course joy:=false

# Terminal 2: FAST-LIO SLAM
cd ~/ros2_ws && source install/setup.bash
ros2 launch fast_lio_ros2 slam_simulation.launch.py rviz:=true

# Terminal 3: Arm and take off
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /px4_offboard_sim/offboard_control/arm std_msgs/msg/Bool "{data: true}"

# Terminal 4: SUPER planner + bridge
cd ~/super_ws
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && source install/setup.bash
ros2 launch super_planner full_integration_test.launch.py launch_bridge:=true

# Terminal 5: Send goal (in camera_init / world frame)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 5.0, y: 0.0, z: 3.0}, orientation: {w: 1.0}}}"
```

The drone will autonomously plan and fly to the goal, then hold position once reached.

## 2.4 Configuration

SUPER loads its configuration from **`super_planner/config/click.yaml`** using a custom YAML loader (NOT ROS2 parameters). Key settings:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `cloud_topic` | `/fast_lio_ros2/cloud_registered` | FAST-LIO point cloud topic |
| `odom_topic` | `/fast_lio_ros2/Odometry` | FAST-LIO odometry topic |
| `frame_id` | `world` | Map frame (matches FAST-LIO output) |
| `click_height` | `-10.0` | Disabled (uses goal z directly) |
| `max_vel` | `1.0` | Max velocity (m/s) |
| `max_acc` | `2.0` | Max acceleration (m/s^2) |
| `virtual_ground_height` | `-5.0` | Virtual ground (accommodates camera_init frame) |

## 2.5 Architecture Notes

- **Coordinate frames**: SUPER operates in ENU (camera_init frame from FAST-LIO). The `px4_super_bridge` converts PositionCommand (ENU) to PoseStamped, and `offboard_control_node` converts ENU to NED for PX4.
- **Frame calibration**: On the first planner command, `offboard_control_node` calibrates the offset between FAST-LIO's camera_init frame and PX4's NED frame.
- **Goal reached detection**: When the drone is within 0.5m of the planner setpoint for 3 seconds, it switches to position hold. A new goal >1m away re-activates trajectory tracking.
- **SUPER config loading**: Uses a custom `YamlLoader` that reads YAML files directly. Edit `click.yaml` for configuration changes -- ROS2 parameter files are NOT used by SUPER's internal loader.

## 2.6 Logging System

SUPER includes a built-in logging system that records each run automatically. Logs are saved in:

- **[./super_planner/log/cmd_logs](./super_planner/log/cmd_logs)**
- **[./super_planner/log/replan_logs](./super_planner/log/replan_logs)**

After stopping the program with `Ctrl + C`, the latest log will be saved. Users can evaluate **trajectory quality** by running:

```bash
# Install dependencies
pip3 install numpy pandas matplotlib

# Plot the command log
python3 plotCmdLog.py
```

For advanced usage, refer to:

- **[read_replan_log.cpp](super_planner/Apps/read_replan_log.cpp)**
- **[traj_opt_tuning.cpp](super_planner/Apps/traj_opt_tuning.cpp)**

We are actively working on improving the logging system, and updates will be available soon! 

## 2.7 Tuning

To maximize performance, parameter tuning is crucial. The current version of SUPER has a large number of parameters (maybe TOOOO MUCH), requiring careful adjustment. Users can refer to the provided examples for guidance. We plan to provide detailed tuning instructions soon. In the meantime, feedback and issue reports are welcome.

## 2.8 Notable Known Issues
* [#10]: When using SUPER with your own simulator (e.g., Gazebo) or a LiDAR odometry system other than FAST-LIO2, ensure that the input point cloud is provided in the world frame. ROG-Map does not utilize `frame_id` or `/tf` information and assumes by default that all input point clouds are in the world frame rather than the body frame.

# 3 TODO

* Add a demo for autonomous exploration (SUPER is well-suited as a local planner for point-to-point navigation).
* Provide examples for using standalone tools in SUPER, such as:

  - **CIRI** - Generates safe flight corridors in C-space.

  - **ROG-Map** - An efficient occupancy grid map supporting both ROS1 and ROS2.
* Introduce the hardware components of SUPER.
* Detail the control module of SUPER.
* Develop a tutorial.



# 4. Acknowledgments

SUPER is built upon several outstanding open-source projects. We extend our gratitude to the developers of the following repositories:

* **[FAST_LIO](https://github.com/hku-mars/FAST_LIO)**, **[Swarm-LIO2](https://github.com/hku-mars/Swarm-LIO2)** and  **[LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init)**  for their excellent localization solutions.
* **[ROG-Map](https://github.com/hku-mars/ROG-Map)** - A high-performance mapping framework that influenced our approach to map representation and optimization.
* **[MARSIM](https://github.com/hku-mars/MARSIM)** - The original simulation environment used for testing SUPER (replaced by Gazebo Sim in this adaptation).
* **[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)** - Open-source flight control software used for SITL simulation.
* **[GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)** – A valuable resource that efficiently performs differentiable trajectory optimization and serves as the foundation of our trajectory optimization method.

  **[FIRI](https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/gcopter/include/gcopter/firi.hpp)** – An extremely efficient safe flight corridor generation method upon which our CIRI is built.
* [**FASTER**](https://github.com/mit-acl/faster) - Introduces the initial concept of a two-trajectory optimization framework.
* **[DecompUtil](https://github.com/sikang/DecompUtil)** - A convex decomposition tool that was instrumental in implementing our algorithms.
* **[Mockamap](https://github.com/HKUST-Aerial-Robotics/mockamap)** - A simple ROS-based map generator that assisted in our development and testing.
* [**Nxt-FC**](https://github.com/HKUST-Aerial-Robotics/Nxt-FC) – A compact yet powerful hardware platform for the PX4 flight controller.

We sincerely appreciate the efforts of these communities in advancing robotics research.
