/**
 * PX4 SUPER Bridge
 *
 * Converts SUPER planner's PositionCommand (ENU frame) to a
 * geometry_msgs/PoseStamped that offboard_control_node subscribes to.
 *
 * The offboard_control_node handles all PX4 communication (arming,
 * offboard mode, trajectory setpoints). This bridge only translates
 * SUPER commands into a format the offboard controller understands.
 *
 * Coordinate conventions:
 *   SUPER/ROS2 (ENU): X-East, Y-North, Z-Up, yaw from East CCW
 *   Output PoseStamped: ENU frame (offboard_control handles ENU→NED)
 *
 * Author: Kevin Medrano Ayala
 * Date: 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <super_planner/msg/position_command.hpp>
#include <cmath>

namespace px4_super_bridge {

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode() : Node("px4_super_bridge") {
    // Declare parameters
    this->declare_parameter("pos_cmd_topic", "/planning/pos_cmd");
    this->declare_parameter("target_pose_topic", "/px4_offboard_sim/offboard_control/target_pose");

    auto pos_cmd_topic = this->get_parameter("pos_cmd_topic").as_string();
    auto target_pose_topic = this->get_parameter("target_pose_topic").as_string();

    // QoS for SUPER planner (matches super_planner's QoS)
    auto super_qos = rclcpp::QoS(1)
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile)
      .history(rclcpp::HistoryPolicy::KeepLast);

    // Subscriber: SUPER PositionCommand
    pos_cmd_sub_ = this->create_subscription<super_planner::msg::PositionCommand>(
      pos_cmd_topic, super_qos,
      std::bind(&BridgeNode::pos_cmd_callback, this, std::placeholders::_1));

    // Publisher: PoseStamped for offboard_control_node
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      target_pose_topic, rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "PX4-SUPER Bridge initialized");
    RCLCPP_INFO(get_logger(), "  Subscribing: %s (PositionCommand)", pos_cmd_topic.c_str());
    RCLCPP_INFO(get_logger(), "  Publishing:  %s (PoseStamped)", target_pose_topic.c_str());
  }

private:
  void pos_cmd_callback(const super_planner::msg::PositionCommand::SharedPtr msg) {
    // Only forward valid trajectory commands (flag > 0 means active trajectory)
    if (msg->trajectory_flag == 0) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = "world";

    // Position in ENU frame (same as SUPER output)
    pose.pose.position.x = msg->position.x;
    pose.pose.position.y = msg->position.y;
    pose.pose.position.z = msg->position.z;

    // Yaw to quaternion (ENU frame: yaw=0 is East, CCW positive)
    double yaw = msg->yaw;
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);

    target_pose_pub_->publish(pose);

    cmd_count_++;
    if (cmd_count_ % 100 == 0) {
      RCLCPP_INFO(get_logger(), "Forwarded %d commands (pos: %.1f, %.1f, %.1f yaw: %.1f deg)",
        cmd_count_, msg->position.x, msg->position.y, msg->position.z,
        yaw * 180.0 / M_PI);
    }
  }

  rclcpp::Subscription<super_planner::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  int cmd_count_{0};
};

}  // namespace px4_super_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_super_bridge::BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
