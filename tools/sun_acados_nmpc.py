"""acados SQP-RTI NMPC for the Sun et al. quadrotor benchmark.

The model follows the MATLAB benchmark convention:
    x = [p(3), q_wxyz(4), v(3), omega(3)]
    u = four rotor thrusts [N]

The least-squares residual matches the Agilicious/Sun tilt-yaw quaternion
residual used in main.m.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/private/tmp/matplotlib")
ACADOS_SOURCE_DIR = os.environ.get("ACADOS_SOURCE_DIR", "/private/tmp/acados")
os.environ.setdefault("ACADOS_SOURCE_DIR", ACADOS_SOURCE_DIR)
os.environ.setdefault("ACADOS_INSTALL_DIR", ACADOS_SOURCE_DIR)

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


_SOLVER: "SunAcadosNMPC | None" = None


def _q_mul(a, b):
    return ca.vertcat(
        a[0] * b[0] - ca.dot(a[1:4], b[1:4]),
        a[0] * b[1:4] + b[0] * a[1:4] + ca.cross(a[1:4], b[1:4]),
    )


def _q_conj(q):
    return ca.vertcat(q[0], -q[1], -q[2], -q[3])


def _q_normalize(q):
    return q / ca.sqrt(ca.dot(q, q) + 1e-12)


def _q_rotate(q, axis):
    qn = _q_normalize(q)
    rotated = _q_mul(_q_mul(qn, ca.vertcat(0, axis)), _q_conj(qn))
    return rotated[1:4]


def _attitude_residual(q, q_ref):
    qn = _q_normalize(q)
    qrn = _q_normalize(q_ref)
    qe = _q_normalize(_q_mul(_q_conj(qn), qrn))
    den = ca.sqrt(qe[0] * qe[0] + qe[3] * qe[3] + 1e-3)
    return ca.vertcat(
        qe[0] * qe[1] - qe[2] * qe[3],
        qe[0] * qe[2] + qe[1] * qe[3],
        qe[3],
    ) / den


def _allocation_matrix(kappa=0.022):
    t_bm = np.array(
        [
            [0.075, -0.075, -0.075, 0.075],
            [-0.100, 0.100, -0.100, 0.100],
            [0.0, 0.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    return np.vstack(
        (
            np.ones((1, 4)),
            t_bm[1:2, :],
            -t_bm[0:1, :],
            kappa * np.array([[-1.0, -1.0, 1.0, 1.0]]),
        )
    )


def _make_model():
    model = AcadosModel()
    model.name = "sun_quadrotor_nmpc"

    p = ca.MX.sym("p", 3)
    q = ca.MX.sym("q", 4)
    v = ca.MX.sym("v", 3)
    omega = ca.MX.sym("omega", 3)
    x = ca.vertcat(p, q, v, omega)

    xdot = ca.MX.sym("xdot", 13)
    u = ca.MX.sym("u", 4)

    p_ref = ca.MX.sym("p_ref", 3)
    q_ref = ca.MX.sym("q_ref", 4)
    v_ref = ca.MX.sym("v_ref", 3)
    omega_ref = ca.MX.sym("omega_ref", 3)
    u_ref = ca.MX.sym("u_ref", 4)
    param = ca.vertcat(p_ref, q_ref, v_ref, omega_ref, u_ref)

    mass = 0.752
    gravity = 9.81
    inertia = ca.diag(ca.DM([0.0025, 0.0021, 0.0043]))
    inv_inertia = ca.diag(ca.DM([1.0 / 0.0025, 1.0 / 0.0021, 1.0 / 0.0043]))
    e3 = ca.DM([0.0, 0.0, 1.0])
    G = ca.DM(_allocation_matrix())

    wrench = G @ u
    thrust = wrench[0]
    tau = wrench[1:4]

    q_dot = 0.5 * _q_mul(_q_normalize(q), ca.vertcat(0, omega))
    v_dot = gravity * e3 - thrust / mass * _q_rotate(q, e3)
    omega_dot = inv_inertia @ (tau - ca.cross(omega, inertia @ omega))
    f_expl = ca.vertcat(v, q_dot, v_dot, omega_dot)

    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = param
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl

    model.cost_y_expr = ca.vertcat(
        p - p_ref,
        v - v_ref,
        _attitude_residual(q, q_ref),
        omega - omega_ref,
        u - u_ref,
    )
    model.cost_y_expr_e = ca.vertcat(
        p - p_ref,
        v - v_ref,
        _attitude_residual(q, q_ref),
        omega - omega_ref,
    )

    return model


def _as_stage_array(value, cols, stages):
    arr = np.asarray(value, dtype=float)
    if arr.ndim == 1:
        arr = arr.reshape((-1, cols))
    if arr.shape == (cols, stages):
        arr = arr.T
    if arr.shape != (stages, cols):
        raise ValueError(f"expected reference shape {(stages, cols)}, got {arr.shape}")
    return np.ascontiguousarray(arr, dtype=float)


def _as_vector(value, length):
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != length:
        raise ValueError(f"expected vector length {length}, got {arr.size}")
    return np.ascontiguousarray(arr, dtype=float)


class SunAcadosNMPC:
    def __init__(self, n_horizon=20, dt=0.05, code_export_dir=None):
        self.N = int(n_horizon)
        self.dt = float(dt)
        self.nx = 13
        self.nu = 4
        self.np = 17
        self.initialized = False

        if code_export_dir is None:
            code_export_dir = "/private/tmp/uav_sun_acados_codegen"
        self.code_export_dir = str(Path(code_export_dir).expanduser())
        Path(self.code_export_dir).mkdir(parents=True, exist_ok=True)

        ocp = AcadosOcp()
        ocp.model = _make_model()
        ocp.code_export_directory = self.code_export_dir
        ocp.parameter_values = np.zeros(self.np)

        q_xi = np.diag([200.0, 200.0, 500.0])
        q_v = np.eye(3)
        q_q = np.diag([5.0, 5.0, 200.0])
        q_omega = np.eye(3)
        q_u = 6.0 * np.eye(4)
        q_terminal = np.block(
            [
                [q_xi, np.zeros((3, 9))],
                [np.zeros((3, 3)), q_v, np.zeros((3, 6))],
                [np.zeros((3, 6)), q_q, np.zeros((3, 3))],
                [np.zeros((3, 9)), q_omega],
            ]
        )

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.block(
            [
                [q_xi, np.zeros((3, 13))],
                [np.zeros((3, 3)), q_v, np.zeros((3, 10))],
                [np.zeros((3, 6)), q_q, np.zeros((3, 7))],
                [np.zeros((3, 9)), q_omega, np.zeros((3, 4))],
                [np.zeros((4, 12)), q_u],
            ]
        )
        ocp.cost.W_e = q_terminal
        ocp.cost.yref = np.zeros(16)
        ocp.cost.yref_e = np.zeros(12)

        ocp.constraints.x0 = np.zeros(self.nx)
        ocp.constraints.lbu = np.zeros(self.nu)
        ocp.constraints.ubu = 8.5 * np.ones(self.nu)
        ocp.constraints.idxbu = np.arange(self.nu)
        ocp.constraints.lbx = -np.array([10.0, 10.0, 4.0])
        ocp.constraints.ubx = np.array([10.0, 10.0, 4.0])
        ocp.constraints.idxbx = np.array([10, 11, 12])
        ocp.constraints.lbx_e = ocp.constraints.lbx
        ocp.constraints.ubx_e = ocp.constraints.ubx
        ocp.constraints.idxbx_e = ocp.constraints.idxbx

        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.N * self.dt
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.qp_solver_cond_N = 5
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.print_level = 0

        json_file = str(Path(self.code_export_dir) / "sun_quadrotor_nmpc_ocp.json")
        self.solver = AcadosOcpSolver(ocp, json_file=json_file, verbose=False)

    def _set_references(self, p_ref, q_ref, v_ref, omega_ref, u_ref):
        for k in range(self.N):
            param = np.concatenate((p_ref[k], q_ref[k], v_ref[k], omega_ref[k], u_ref[k]))
            self.solver.set(k, "p", param)
        param_e = np.concatenate((p_ref[self.N], q_ref[self.N], v_ref[self.N], omega_ref[self.N], u_ref[self.N]))
        self.solver.set(self.N, "p", param_e)

    def _initialize_guess(self, x0, p_ref, q_ref, v_ref, omega_ref, u_ref):
        if self.initialized:
            return
        for k in range(self.N + 1):
            x_ref = np.concatenate((p_ref[k], q_ref[k], v_ref[k], omega_ref[k]))
            self.solver.set(k, "x", x_ref)
        self.solver.set(0, "x", x0)
        for k in range(self.N):
            self.solver.set(k, "u", np.clip(u_ref[k], 0.0, 8.5))
        self.initialized = True

    def solve(self, x0, p_ref, q_ref, v_ref, omega_ref, u_ref):
        stages = self.N + 1
        x0 = _as_vector(x0, self.nx)
        p_ref = _as_stage_array(p_ref, 3, stages)
        q_ref = _as_stage_array(q_ref, 4, stages)
        v_ref = _as_stage_array(v_ref, 3, stages)
        omega_ref = _as_stage_array(omega_ref, 3, stages)
        u_ref = _as_stage_array(u_ref, 4, stages)

        self._set_references(p_ref, q_ref, v_ref, omega_ref, u_ref)
        self._initialize_guess(x0, p_ref, q_ref, v_ref, omega_ref, u_ref)
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)

        start = time.perf_counter()
        status = int(self.solver.solve())
        solve_time = time.perf_counter() - start
        u0 = np.asarray(self.solver.get(0, "u"), dtype=float).reshape(-1)
        return {
            "u0": u0,
            "status": status,
            "solve_time": solve_time,
            "cost": float(self.solver.get_cost()),
        }


def reset():
    global _SOLVER
    _SOLVER = None


def reset_warm_start():
    if _SOLVER is not None:
        _SOLVER.initialized = False


def solve(x0, p_ref, q_ref, v_ref, omega_ref, u_ref):
    global _SOLVER
    if _SOLVER is None:
        _SOLVER = SunAcadosNMPC()
    return _SOLVER.solve(x0, p_ref, q_ref, v_ref, omega_ref, u_ref)


def allocation_matrix():
    return _allocation_matrix()
