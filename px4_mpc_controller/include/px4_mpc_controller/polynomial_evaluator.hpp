#pragma once

#include <vector>
#include <cmath>
#include <array>
#include <stdexcept>
#include <super_planner/msg/polynomial_trajectory.hpp>

namespace px4_mpc_controller {

struct TrajectoryPoint {
  std::array<double, 3> position;      // NED
  std::array<double, 3> velocity;      // NED
  std::array<double, 3> acceleration;  // NED
  double yaw;                          // NED (0=North, CW+)
  double yaw_rate;
};

class PolynomialEvaluator {
public:
  void setTrajectory(const super_planner::msg::PolynomialTrajectory::SharedPtr& msg)
  {
    traj_ = *msg;
    has_trajectory_ = true;
  }

  bool hasTrajectory() const { return has_trajectory_; }

  double startTime() const { return traj_.start_wt_pos; }

  /// Compute end time from start + sum of piece durations
  /// (SUPER doesn't set final_wt_pos, so we compute it)
  double endTime() const {
    double total = 0.0;
    for (uint32_t i = 0; i < traj_.piece_num_pos && i < traj_.time_pos.size(); i++)
      total += traj_.time_pos[i];
    return traj_.start_wt_pos + total;
  }

  double duration() const { return endTime() - traj_.start_wt_pos; }

  void setFrameOffset(double ox, double oy, double oz)
  {
    offset_[0] = ox;
    offset_[1] = oy;
    offset_[2] = oz;
    offset_calibrated_ = true;
  }

  bool isOffsetCalibrated() const { return offset_calibrated_; }
  const std::array<double, 3>& frameOffset() const { return offset_; }

  /// Evaluate trajectory at time t (world time).
  /// Returns position, velocity, acceleration in NED frame with offset applied.
  TrajectoryPoint evaluate(double t) const
  {
    TrajectoryPoint pt{};

    // Clamp to trajectory bounds
    double t_end = endTime();
    double t_pos = std::clamp(t, traj_.start_wt_pos, t_end);

    // Evaluate position polynomial (ENU)
    double enu_pos[3], enu_vel[3], enu_acc[3];
    evaluateAxis(traj_.coef_pos_x, traj_.time_pos, traj_.piece_num_pos,
                 traj_.order_pos, traj_.start_wt_pos, t_pos,
                 enu_pos[0], enu_vel[0], enu_acc[0]);
    evaluateAxis(traj_.coef_pos_y, traj_.time_pos, traj_.piece_num_pos,
                 traj_.order_pos, traj_.start_wt_pos, t_pos,
                 enu_pos[1], enu_vel[1], enu_acc[1]);
    evaluateAxis(traj_.coef_pos_z, traj_.time_pos, traj_.piece_num_pos,
                 traj_.order_pos, traj_.start_wt_pos, t_pos,
                 enu_pos[2], enu_vel[2], enu_acc[2]);

    // ENU → NED: ned_x=enu_y, ned_y=enu_x, ned_z=-enu_z
    pt.position[0] = enu_pos[1] + offset_[0];   // North
    pt.position[1] = enu_pos[0] + offset_[1];   // East
    pt.position[2] = -enu_pos[2] + offset_[2];  // Down

    pt.velocity[0] = enu_vel[1];
    pt.velocity[1] = enu_vel[0];
    pt.velocity[2] = -enu_vel[2];

    pt.acceleration[0] = enu_acc[1];
    pt.acceleration[1] = enu_acc[0];
    pt.acceleration[2] = -enu_acc[2];

    // Evaluate yaw polynomial (if available)
    if (traj_.piece_num_yaw > 0 && !traj_.coef_yaw.empty()) {
      double yaw_end = traj_.start_wt_yaw;
      for (uint32_t i = 0; i < traj_.piece_num_yaw && i < traj_.time_yaw.size(); i++)
        yaw_end += traj_.time_yaw[i];
      double t_yaw = std::clamp(t, traj_.start_wt_yaw, yaw_end);
      double enu_yaw, enu_yaw_rate, enu_yaw_acc;
      evaluateAxis(traj_.coef_yaw, traj_.time_yaw, traj_.piece_num_yaw,
                   traj_.order_yaw, traj_.start_wt_yaw, t_yaw,
                   enu_yaw, enu_yaw_rate, enu_yaw_acc);

      // ENU yaw (0=East, CCW+) → NED yaw (0=North, CW+)
      pt.yaw = M_PI_2 - enu_yaw;
      pt.yaw_rate = -enu_yaw_rate;  // Negate for CW+ convention
    } else {
      pt.yaw = 0.0;
      pt.yaw_rate = 0.0;
    }

    // Wrap yaw to [-pi, pi]
    while (pt.yaw > M_PI) pt.yaw -= 2.0 * M_PI;
    while (pt.yaw < -M_PI) pt.yaw += 2.0 * M_PI;

    return pt;
  }

  /// Evaluate position at t=0 in ENU (for frame calibration before offset is set)
  std::array<double, 3> evaluateStartPositionENU() const
  {
    double enu_pos[3], dummy_vel, dummy_acc;
    evaluateAxis(traj_.coef_pos_x, traj_.time_pos, traj_.piece_num_pos,
                 traj_.order_pos, traj_.start_wt_pos, traj_.start_wt_pos,
                 enu_pos[0], dummy_vel, dummy_acc);
    evaluateAxis(traj_.coef_pos_y, traj_.time_pos, traj_.piece_num_pos,
                 traj_.order_pos, traj_.start_wt_pos, traj_.start_wt_pos,
                 enu_pos[1], dummy_vel, dummy_acc);
    evaluateAxis(traj_.coef_pos_z, traj_.time_pos, traj_.piece_num_pos,
                 traj_.order_pos, traj_.start_wt_pos, traj_.start_wt_pos,
                 enu_pos[2], dummy_vel, dummy_acc);
    return {enu_pos[0], enu_pos[1], enu_pos[2]};
  }

private:
  /// Evaluate a single axis of the piecewise polynomial at world time t.
  /// Outputs position (0th derivative), velocity (1st), acceleration (2nd).
  static void evaluateAxis(
    const std::vector<double>& coefs,
    const std::vector<double>& time_durations,
    uint32_t piece_num,
    uint32_t order,
    double start_wt,
    double t,
    double& pos_out,
    double& vel_out,
    double& acc_out)
  {
    uint32_t n_coef = order + 1;

    // Find active piece
    double elapsed = t - start_wt;
    uint32_t piece = 0;
    double accumulated = 0.0;

    for (uint32_t i = 0; i < piece_num; i++) {
      if (i < time_durations.size() && elapsed <= accumulated + time_durations[i] + 1e-9) {
        piece = i;
        break;
      }
      if (i < time_durations.size())
        accumulated += time_durations[i];
      piece = i;
    }

    // Local time within the piece
    double t_local = elapsed - accumulated;
    if (t_local < 0.0) t_local = 0.0;

    // Coefficient offset for this piece
    uint32_t coef_offset = piece * n_coef;

    // Evaluate polynomial and its derivatives.
    // SUPER stores coefficients in DESCENDING order:
    //   coefs[offset+0] = t^(order) coeff, ..., coefs[offset+order] = constant
    // So: p(t) = coefs[0]*t^7 + coefs[1]*t^6 + ... + coefs[7]*t^0
    // Using Horner's method: p(t) = (...((c0*t + c1)*t + c2)*t + ... + c7)
    pos_out = 0.0;
    vel_out = 0.0;
    acc_out = 0.0;

    for (uint32_t i = 0; i < n_coef && (coef_offset + i) < coefs.size(); i++) {
      double c = coefs[coef_offset + i];
      uint32_t power = order - i;  // Descending: first coef has highest power
      double t_pow = std::pow(t_local, static_cast<double>(power));

      pos_out += c * t_pow;

      if (power >= 1) {
        vel_out += power * c * std::pow(t_local, static_cast<double>(power - 1));
      }
      if (power >= 2) {
        acc_out += power * (power - 1) * c * std::pow(t_local, static_cast<double>(power - 2));
      }
    }
  }

  super_planner::msg::PolynomialTrajectory traj_;
  bool has_trajectory_{false};
  std::array<double, 3> offset_{0.0, 0.0, 0.0};
  bool offset_calibrated_{false};
};

}  // namespace px4_mpc_controller
