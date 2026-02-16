#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <super_planner/msg/polynomial_trajectory.hpp>
#include <px4_mpc_controller/polynomial_evaluator.hpp>
#include <px4_mpc_controller/mpc_wrapper.hpp>

#include <cmath>
#include <chrono>
#include <limits>

namespace px4_mpc_controller {

class MpcTrajectoryNode : public rclcpp::Node {
public:
  explicit MpcTrajectoryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("px4_mpc_controller", options)
  {
    // Declare parameters
    this->declare_parameter("mpc_rate", 50.0);
    this->declare_parameter("max_vel", 1.0);
    this->declare_parameter("max_acc", 2.0);
    this->declare_parameter("yaw_kp", 2.0);
    this->declare_parameter("Q_pos", 50.0);
    this->declare_parameter("Q_vel", 10.0);
    this->declare_parameter("R_acc", 1.0);
    this->declare_parameter("planner_timeout", 3.0);
    this->declare_parameter("goal_threshold", 0.5);
    this->declare_parameter("goal_hold_time", 3.0);

    this->declare_parameter("topics.poly_traj", "/planning_cmd/poly_traj");
    this->declare_parameter("topics.vehicle_local_position", "/fmu/out/vehicle_local_position_v1");
    this->declare_parameter("topics.mpc_setpoint", "/px4_offboard_sim/offboard_control/mpc_setpoint");

    // Read parameters
    double mpc_rate = this->get_parameter("mpc_rate").as_double();
    max_vel_ = this->get_parameter("max_vel").as_double();
    max_acc_ = this->get_parameter("max_acc").as_double();
    yaw_kp_ = this->get_parameter("yaw_kp").as_double();
    planner_timeout_ = this->get_parameter("planner_timeout").as_double();
    goal_threshold_ = this->get_parameter("goal_threshold").as_double();
    goal_hold_time_ = this->get_parameter("goal_hold_time").as_double();

    auto t_poly = this->get_parameter("topics.poly_traj").as_string();
    auto t_local_pos = this->get_parameter("topics.vehicle_local_position").as_string();
    auto t_mpc_setpoint = this->get_parameter("topics.mpc_setpoint").as_string();

    // QoS for PX4 topics
    auto px4_qos = rclcpp::QoS(1)
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile)
      .history(rclcpp::HistoryPolicy::KeepLast);

    // QoS for SUPER topics (BEST_EFFORT to match SUPER's publisher)
    auto super_qos = rclcpp::QoS(10)
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile);

    // Subscriptions
    poly_traj_sub_ = this->create_subscription<super_planner::msg::PolynomialTrajectory>(
      t_poly, super_qos,
      std::bind(&MpcTrajectoryNode::poly_traj_cb, this, std::placeholders::_1));

    local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      t_local_pos, px4_qos,
      std::bind(&MpcTrajectoryNode::local_position_cb, this, std::placeholders::_1));

    // Publisher
    mpc_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      t_mpc_setpoint, px4_qos);

    // Initialize MPC solver
    try {
      mpc_ = std::make_unique<MpcWrapper>();
      mpc_->setConstraints(max_vel_, max_acc_);
      RCLCPP_INFO(get_logger(), "acados MPC solver initialized (N=%d)", N_HORIZON);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize MPC solver: %s", e.what());
      throw;
    }

    // Control loop timer
    int period_ms = static_cast<int>(1000.0 / mpc_rate);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&MpcTrajectoryNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "MPC trajectory controller initialized (%.0f Hz)", mpc_rate);
    RCLCPP_INFO(get_logger(), "  max_vel=%.1f m/s, max_acc=%.1f m/s^2, yaw_kp=%.1f",
                max_vel_, max_acc_, yaw_kp_);
    RCLCPP_INFO(get_logger(), "  Subscribing to: %s", t_poly.c_str());
    RCLCPP_INFO(get_logger(), "  Publishing to:  %s", t_mpc_setpoint.c_str());
  }

private:
  // ── Callbacks ────────────────────────────────────────────────

  void poly_traj_cb(const super_planner::msg::PolynomialTrajectory::SharedPtr msg)
  {
    // Update timestamp for any message (heartbeat or trajectory)
    traj_last_time_ = now_sec();

    // Check for position trajectory data (type is a bitmask, may include HEART_BEAT too)
    if (!(msg->type & super_planner::msg::PolynomialTrajectory::POSITION_TRAJ)) {
      return;  // Heartbeat-only, no trajectory data
    }

    if (msg->piece_num_pos == 0 || msg->coef_pos_x.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Received empty polynomial trajectory");
      return;
    }

    // Store trajectory
    evaluator_.setTrajectory(msg);
    traj_last_time_ = now_sec();
    trajectory_id_ = msg->trajectory_id;

    // Frame offset calibration on first trajectory
    if (!evaluator_.isOffsetCalibrated() && has_px4_state_) {
      auto enu_start = evaluator_.evaluateStartPositionENU();
      // ENU → NED raw (before offset)
      double raw_ned[3];
      raw_ned[0] = enu_start[1];   // North = ENU_Y
      raw_ned[1] = enu_start[0];   // East = ENU_X
      raw_ned[2] = -enu_start[2];  // Down = -ENU_Z

      double ox = px4_pos_[0] - raw_ned[0];
      double oy = px4_pos_[1] - raw_ned[1];
      double oz = px4_pos_[2] - raw_ned[2];
      evaluator_.setFrameOffset(ox, oy, oz);

      RCLCPP_INFO(get_logger(),
        "Frame calibrated: offset NED = (%.2f, %.2f, %.2f)", ox, oy, oz);
    }

    if (!trajectory_active_) {
      trajectory_active_ = true;
      goal_reached_ = false;
      near_goal_start_ = 0.0;
      RCLCPP_INFO(get_logger(), "Trajectory activated (id=%u, pieces=%u, duration=%.2fs)",
                  msg->trajectory_id, msg->piece_num_pos,
                  evaluator_.duration());
    }
  }

  void local_position_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    px4_pos_[0] = msg->x;
    px4_pos_[1] = msg->y;
    px4_pos_[2] = msg->z;
    px4_vel_[0] = msg->vx;
    px4_vel_[1] = msg->vy;
    px4_vel_[2] = msg->vz;
    px4_yaw_ = msg->heading;
    has_px4_state_ = true;
  }

  // ── Control loop ─────────────────────────────────────────────

  void control_loop()
  {
    if (!trajectory_active_ || !has_px4_state_ || !evaluator_.hasTrajectory()) {
      return;
    }

    if (!evaluator_.isOffsetCalibrated()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for frame calibration...");
      return;
    }

    // Check trajectory timeout
    double now = now_sec();
    if (now - traj_last_time_ > planner_timeout_) {
      RCLCPP_WARN(get_logger(), "Trajectory timeout (%.1fs) — stopping MPC output",
                  now - traj_last_time_);
      trajectory_active_ = false;
      return;
    }

    // If goal already reached, don't publish MPC setpoints
    if (goal_reached_) {
      return;
    }

    // Get current trajectory time
    double t_now = now;
    double t_start = evaluator_.startTime();
    double t_end = evaluator_.endTime();

    // MPC horizon timestep
    constexpr double dt = 0.05;  // 50ms per step, 1.0s lookahead

    // Set MPC initial state from PX4
    StateVector x0;
    x0 << px4_pos_[0], px4_pos_[1], px4_pos_[2],
           px4_vel_[0], px4_vel_[1], px4_vel_[2];
    mpc_->setInitialState(x0);

    // Set reference trajectory for each horizon step
    TrajectoryPoint ref_first;
    for (int k = 0; k < N_HORIZON; k++) {
      double t_ref = t_now + k * dt;

      // Clamp to trajectory end (hold final position)
      TrajectoryPoint ref = evaluator_.evaluate(t_ref);

      StateVector x_ref;
      x_ref << ref.position[0], ref.position[1], ref.position[2],
               ref.velocity[0], ref.velocity[1], ref.velocity[2];

      ControlVector u_ref;
      u_ref << ref.acceleration[0], ref.acceleration[1], ref.acceleration[2];

      mpc_->setReference(k, x_ref, u_ref);

      if (k == 0) ref_first = ref;
    }

    // Terminal reference
    TrajectoryPoint ref_terminal = evaluator_.evaluate(t_now + N_HORIZON * dt);
    StateVector x_terminal;
    x_terminal << ref_terminal.position[0], ref_terminal.position[1], ref_terminal.position[2],
                  ref_terminal.velocity[0], ref_terminal.velocity[1], ref_terminal.velocity[2];
    mpc_->setTerminalReference(x_terminal);

    // Solve MPC
    int status = mpc_->solve();
    if (status != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "MPC solver returned status %d", status);
      // Still use the result — acados often returns usable solutions even with status != 0
    }

    // Extract optimal control and predicted state
    ControlVector u_opt = mpc_->getOptimalControl();
    StateVector x_pred = mpc_->getPredictedState(1);  // Next predicted state

    // Compute yaw control
    double yaw_ref = ref_first.yaw;
    double yaw_error = yaw_ref - px4_yaw_;
    // Wrap to [-pi, pi]
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
    double yawspeed = yaw_kp_ * yaw_error;
    // Clamp yawspeed
    yawspeed = std::clamp(yawspeed, -1.5, 1.5);

    // Publish TrajectorySetpoint
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // Position from reference (not predicted — reference gives better tracking)
    msg.position[0] = static_cast<float>(ref_first.position[0]);
    msg.position[1] = static_cast<float>(ref_first.position[1]);
    msg.position[2] = static_cast<float>(ref_first.position[2]);

    // Velocity from predicted state (MPC-optimal)
    msg.velocity[0] = static_cast<float>(x_pred[3]);
    msg.velocity[1] = static_cast<float>(x_pred[4]);
    msg.velocity[2] = static_cast<float>(x_pred[5]);

    // Acceleration from optimal control
    msg.acceleration[0] = static_cast<float>(u_opt[0]);
    msg.acceleration[1] = static_cast<float>(u_opt[1]);
    msg.acceleration[2] = static_cast<float>(u_opt[2]);

    msg.yaw = static_cast<float>(yaw_ref);
    msg.yawspeed = static_cast<float>(yawspeed);

    mpc_setpoint_pub_->publish(msg);

    // Check goal reached (drone near final trajectory position)
    if (t_now > t_end - 0.5) {
      TrajectoryPoint final_pt = evaluator_.evaluate(t_end);
      double dx = px4_pos_[0] - final_pt.position[0];
      double dy = px4_pos_[1] - final_pt.position[1];
      double dz = px4_pos_[2] - final_pt.position[2];
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (dist < goal_threshold_) {
        if (near_goal_start_ == 0.0) near_goal_start_ = now;
        if (now - near_goal_start_ >= goal_hold_time_) {
          goal_reached_ = true;
          RCLCPP_INFO(get_logger(), "Goal reached at (%.2f, %.2f, %.2f)",
                      px4_pos_[0], px4_pos_[1], -px4_pos_[2]);
        }
      } else {
        near_goal_start_ = 0.0;
      }
    }

    // Periodic logging
    solve_count_++;
    if (solve_count_ % 250 == 0 || solve_count_ <= 3) {
      RCLCPP_INFO(get_logger(),
        "MPC [%u]: pos=(%.2f,%.2f,%.2f) ref=(%.2f,%.2f,%.2f) acc=(%.2f,%.2f,%.2f) t=%.1fms",
        solve_count_,
        px4_pos_[0], px4_pos_[1], px4_pos_[2],
        ref_first.position[0], ref_first.position[1], ref_first.position[2],
        u_opt[0], u_opt[1], u_opt[2],
        mpc_->getSolveTime() * 1000.0);
      RCLCPP_INFO(get_logger(),
        "  t_now=%.3f t_start=%.3f t_end=%.3f elapsed=%.3f duration=%.3f",
        t_now, t_start, t_end, t_now - t_start, t_end - t_start);
    }
  }

  // ── Helpers ──────────────────────────────────────────────────

  double now_sec() const
  {
    return static_cast<double>(this->get_clock()->now().nanoseconds()) * 1e-9;
  }

  // ── Subscriptions ────────────────────────────────────────────
  rclcpp::Subscription<super_planner::msg::PolynomialTrajectory>::SharedPtr poly_traj_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

  // ── Publisher ────────────────────────────────────────────────
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr mpc_setpoint_pub_;

  // ── Timer ────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;

  // ── MPC solver ───────────────────────────────────────────────
  std::unique_ptr<MpcWrapper> mpc_;
  PolynomialEvaluator evaluator_;

  // ── Parameters ───────────────────────────────────────────────
  double max_vel_{1.0};
  double max_acc_{2.0};
  double yaw_kp_{2.0};
  double planner_timeout_{3.0};
  double goal_threshold_{0.5};
  double goal_hold_time_{3.0};

  // ── PX4 state ────────────────────────────────────────────────
  double px4_pos_[3]{0.0, 0.0, 0.0};
  double px4_vel_[3]{0.0, 0.0, 0.0};
  double px4_yaw_{0.0};
  bool has_px4_state_{false};

  // ── Trajectory state ─────────────────────────────────────────
  bool trajectory_active_{false};
  double traj_last_time_{0.0};
  uint32_t trajectory_id_{0};
  bool goal_reached_{false};
  double near_goal_start_{0.0};
  uint32_t solve_count_{0};
};

}  // namespace px4_mpc_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_mpc_controller::MpcTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
