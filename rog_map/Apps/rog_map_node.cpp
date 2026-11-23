// Standalone ROG-Map server: runs the ROG-Map ROS 2 wrapper on its own, without
// the SUPER planner. It subscribes to the odometry and cloud topics of the
// config (ros_callback) and publishes the occupied cells of the visualisation
// box on rog_map/occ (and rog_map/inf_occ, rog_map/unk when enabled) at
// visualization/time_rate. Other planners (e.g. MIGHTY) read rog_map/occ as
// their local occupancy map.
//
//   ros2 run rog_map rog_map_node --ros-args -p config_name:=/abs/path/cfg.yaml
#include <rclcpp/rclcpp.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rog_map_node");
  const std::string cfg = node->declare_parameter<std::string>("config_name", "");
  if (cfg.empty() || cfg[0] != '/') {
    RCLCPP_FATAL(node->get_logger(), "config_name must be an absolute path to a ROG-Map YAML");
    return 1;
  }
  auto map = std::make_shared<rog_map::ROGMapROS>(node, cfg);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
