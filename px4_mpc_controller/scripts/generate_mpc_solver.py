#!/usr/bin/env python3
"""
Generate acados MPC solver for quadrotor trajectory tracking.

Model: 3D Double Integrator (6 states, 3 controls)
  State:   x = [px, py, pz, vx, vy, vz]  (NED frame)
  Control: u = [ax, ay, az]               (NED frame)
  Dynamics: x_dot = [vx, vy, vz, ax, ay, az]

Run once to generate C solver code in the 'generated/' directory.

Usage:
  cd ~/super_ws/src/SUPER/px4_mpc_controller
  python3 scripts/generate_mpc_solver.py
"""

import os
import sys
import shutil
import numpy as np
from casadi import SX, vertcat

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel


def create_quadrotor_model():
    """Create the 3D double integrator model."""
    model = AcadosModel()
    model.name = "quadrotor"

    # States: [px, py, pz, vx, vy, vz]
    px = SX.sym("px")
    py = SX.sym("py")
    pz = SX.sym("pz")
    vx = SX.sym("vx")
    vy = SX.sym("vy")
    vz = SX.sym("vz")
    x = vertcat(px, py, pz, vx, vy, vz)

    # Controls: [ax, ay, az]
    ax = SX.sym("ax")
    ay = SX.sym("ay")
    az = SX.sym("az")
    u = vertcat(ax, ay, az)

    # State derivatives: x_dot = [vx, vy, vz, ax, ay, az]
    px_dot = SX.sym("px_dot")
    py_dot = SX.sym("py_dot")
    pz_dot = SX.sym("pz_dot")
    vx_dot = SX.sym("vx_dot")
    vy_dot = SX.sym("vy_dot")
    vz_dot = SX.sym("vz_dot")
    x_dot = vertcat(px_dot, py_dot, pz_dot, vx_dot, vy_dot, vz_dot)

    # Explicit dynamics
    f_expl = vertcat(vx, vy, vz, ax, ay, az)

    # Implicit dynamics: x_dot - f(x, u) = 0
    f_impl = x_dot - f_expl

    model.x = x
    model.u = u
    model.xdot = x_dot
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl

    return model


def create_ocp():
    """Create the acados OCP formulation."""
    ocp = AcadosOcp()

    # Model
    model = create_quadrotor_model()
    ocp.model = model

    # Dimensions
    nx = 6   # states
    nu = 3   # controls
    N = 20   # prediction horizon steps
    dt = 0.05  # timestep (1.0s total lookahead)

    ocp.dims.N = N

    # Cost: Quadratic tracking
    # Intermediate: ||y - y_ref||_W  where y = [x; u]
    # Terminal:     ||y_e - y_e_ref||_W_e  where y_e = x
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    # y = Vx * x + Vu * u
    ny = nx + nu  # 9
    ny_e = nx     # 6

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :nu] = np.eye(nu)

    ocp.cost.Vx_e = np.eye(ny_e, nx)

    # Weight matrices
    Q_pos = 50.0
    Q_vel = 10.0
    R_acc = 1.0

    Q = np.diag([Q_pos, Q_pos, Q_pos, Q_vel, Q_vel, Q_vel])
    R = np.diag([R_acc, R_acc, R_acc])
    W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])
    W_e = 2.0 * Q  # Terminal cost (stability margin)

    ocp.cost.W = W
    ocp.cost.W_e = W_e

    # Reference (will be set at runtime)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # Constraints: initial state (set at runtime)
    ocp.constraints.x0 = np.zeros(nx)

    # State constraints: velocity bounds (indices 3,4,5)
    max_vel = 1.0  # m/s (default, overridden at runtime)
    ocp.constraints.idxbx = np.array([3, 4, 5])
    ocp.constraints.lbx = np.array([-max_vel, -max_vel, -max_vel])
    ocp.constraints.ubx = np.array([max_vel, max_vel, max_vel])

    # Control constraints: acceleration bounds
    max_acc = 2.0  # m/s^2 (default, overridden at runtime)
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.lbu = np.array([-max_acc, -max_acc, -max_acc])
    ocp.constraints.ubu = np.array([max_acc, max_acc, max_acc])

    # Solver options
    ocp.solver_options.tf = N * dt  # Total prediction horizon time
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # Real-Time Iteration for speed
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"  # Explicit Runge-Kutta
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_max_iter = 1  # RTI: single iteration
    ocp.solver_options.qp_solver_iter_max = 50

    return ocp


def main():
    # Output directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    generated_dir = os.path.join(pkg_dir, "generated")

    # Clean previous generation
    if os.path.exists(generated_dir):
        shutil.rmtree(generated_dir)
    os.makedirs(generated_dir)

    # Change to generated dir so acados outputs there
    original_dir = os.getcwd()
    os.chdir(generated_dir)

    print(f"Generating MPC solver in: {generated_dir}")

    # Create and generate solver
    ocp = create_ocp()

    try:
        solver = AcadosOcpSolver(ocp, json_file="acados_ocp_quadrotor.json")
        print("Solver generated successfully!")
    except Exception as e:
        print(f"ERROR: Solver generation failed: {e}")
        os.chdir(original_dir)
        sys.exit(1)

    # Verify generated files
    expected_files = [
        "acados_solver_quadrotor.h",
        "acados_solver_quadrotor.c",
    ]
    for f in expected_files:
        path = os.path.join(generated_dir, f)
        if os.path.exists(path):
            print(f"  OK: {f}")
        else:
            print(f"  MISSING: {f}")

    # Quick test: solve with zero reference
    print("\nRunning quick test solve...")
    x0 = np.zeros(6)
    solver.set(0, "lbx", x0)
    solver.set(0, "ubx", x0)
    status = solver.solve()
    print(f"  Solve status: {status} (0=success)")
    solve_time = solver.get_stats("time_tot")
    print(f"  Solve time:   {solve_time*1000:.3f} ms")

    os.chdir(original_dir)
    print(f"\nGenerated files in: {generated_dir}")
    print("You can now build the px4_mpc_controller package.")


if __name__ == "__main__":
    main()
