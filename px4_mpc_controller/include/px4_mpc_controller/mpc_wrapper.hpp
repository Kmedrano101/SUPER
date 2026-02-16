#pragma once

#include <Eigen/Dense>
#include <string>
#include <stdexcept>

// acados generated solver headers (included after generation)
#include "acados_solver_quadrotor.h"

namespace px4_mpc_controller {

// State dimension: [px, py, pz, vx, vy, vz]
static constexpr int NX = QUADROTOR_NX;
// Control dimension: [ax, ay, az]
static constexpr int NU = QUADROTOR_NU;
// Prediction horizon
static constexpr int N_HORIZON = QUADROTOR_N;

using StateVector = Eigen::Matrix<double, NX, 1>;
using ControlVector = Eigen::Matrix<double, NU, 1>;

class MpcWrapper {
public:
  MpcWrapper()
  {
    capsule_ = quadrotor_acados_create_capsule();
    if (!capsule_) {
      throw std::runtime_error("Failed to create acados capsule");
    }

    int status = quadrotor_acados_create(capsule_);
    if (status != 0) {
      throw std::runtime_error("Failed to create acados solver, status: " + std::to_string(status));
    }

    nlp_config_ = quadrotor_acados_get_nlp_config(capsule_);
    nlp_dims_ = quadrotor_acados_get_nlp_dims(capsule_);
    nlp_in_ = quadrotor_acados_get_nlp_in(capsule_);
    nlp_out_ = quadrotor_acados_get_nlp_out(capsule_);
    nlp_solver_ = quadrotor_acados_get_nlp_solver(capsule_);
  }

  ~MpcWrapper()
  {
    if (capsule_) {
      quadrotor_acados_free(capsule_);
      quadrotor_acados_free_capsule(capsule_);
    }
  }

  // Non-copyable
  MpcWrapper(const MpcWrapper&) = delete;
  MpcWrapper& operator=(const MpcWrapper&) = delete;

  /// Set initial state constraint x(0) = x0
  /// Note: stage 0 has NBX0=6 (all states constrained)
  void setInitialState(const StateVector& x0)
  {
    // ocp_nlp_constraints_model_set(config, dims, in, out, stage, field, value)
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                  0, "lbx", const_cast<double*>(x0.data()));
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                  0, "ubx", const_cast<double*>(x0.data()));
  }

  /// Set reference for a given stage (0 to N-1)
  /// y_ref = [x_ref (6), u_ref (3)] for intermediate stages
  void setReference(int stage, const StateVector& x_ref, const ControlVector& u_ref)
  {
    if (stage < 0 || stage >= N_HORIZON) return;

    // Intermediate reference: y = [x; u]
    Eigen::Matrix<double, NX + NU, 1> y_ref;
    y_ref.head<NX>() = x_ref;
    y_ref.tail<NU>() = u_ref;
    // ocp_nlp_cost_model_set(config, dims, in, stage, field, value)
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, stage,
                           "yref", y_ref.data());
  }

  /// Set terminal reference (stage N)
  void setTerminalReference(const StateVector& x_ref)
  {
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_HORIZON,
                           "yref", const_cast<double*>(x_ref.data()));
  }

  /// Update velocity and acceleration constraints for all stages
  void setConstraints(double max_vel, double max_acc)
  {
    // State constraints: velocity bounds (NBX=3, indices 3,4,5 of state)
    Eigen::Vector3d lbx, ubx;
    lbx << -max_vel, -max_vel, -max_vel;
    ubx <<  max_vel,  max_vel,  max_vel;

    // Control constraints: acceleration bounds (NBU=3)
    ControlVector lbu, ubu;
    lbu << -max_acc, -max_acc, -max_acc;
    ubu <<  max_acc,  max_acc,  max_acc;

    for (int i = 1; i < N_HORIZON; i++) {
      ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                    i, "lbx", lbx.data());
      ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                    i, "ubx", ubx.data());
    }

    for (int i = 0; i < N_HORIZON; i++) {
      ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                    i, "lbu", lbu.data());
      ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                    i, "ubu", ubu.data());
    }
  }

  /// Solve the OCP. Returns acados status (0 = success).
  int solve()
  {
    return quadrotor_acados_solve(capsule_);
  }

  /// Get optimal control at stage 0: u*[0]
  ControlVector getOptimalControl() const
  {
    ControlVector u;
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u.data());
    return u;
  }

  /// Get predicted state at a given stage
  StateVector getPredictedState(int stage) const
  {
    StateVector x;
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());
    return x;
  }

  /// Get solver computation time (seconds)
  double getSolveTime() const
  {
    double time;
    // ocp_nlp_get(solver, field, value)
    ocp_nlp_get(nlp_solver_, "time_tot", &time);
    return time;
  }

private:
  quadrotor_solver_capsule* capsule_{nullptr};
  ocp_nlp_config* nlp_config_{nullptr};
  ocp_nlp_dims* nlp_dims_{nullptr};
  ocp_nlp_in* nlp_in_{nullptr};
  ocp_nlp_out* nlp_out_{nullptr};
  ocp_nlp_solver* nlp_solver_{nullptr};
};

}  // namespace px4_mpc_controller
