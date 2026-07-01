"""acados SQP-RTI NMPC for the Sun et al. quadrotor benchmark.

The model follows the MATLAB benchmark convention:
    x = [p(3), q_wxyz(4), v(3), omega(3)]
    u = four rotor thrusts [N]

The OCP follows the Sun formulation: single-rotor thrust inputs, full nonlinear
rigid-body dynamics, bounded rates/thrusts, and the Eq. (12) quaternion-vector
attitude residual. Sun Eq. (9) is used for aerodynamic force; d_tau is not
predicted.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/private/tmp/matplotlib")
ACADOS_SOURCE_DIR = os.environ.get(
    "ACADOS_SOURCE_DIR",
    str(Path(__file__).resolve().parents[1] / ".acados" / "acados"),
)
os.environ.setdefault("ACADOS_SOURCE_DIR", ACADOS_SOURCE_DIR)
os.environ.setdefault("ACADOS_INSTALL_DIR", ACADOS_SOURCE_DIR)
ACADOS_TEMPLATE_PATH = Path(ACADOS_SOURCE_DIR) / "interfaces" / "acados_template"
if ACADOS_TEMPLATE_PATH.is_dir():
    sys.path.insert(0, str(ACADOS_TEMPLATE_PATH))

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


_SOLVER: "SunAcadosNMPC | None" = None
_CONFIG: dict | None = None


def _q_mul(a, b):
    return ca.vertcat(
        a[0] * b[0] - ca.dot(a[1:4], b[1:4]),
        a[0] * b[1:4] + b[0] * a[1:4] + ca.cross(a[1:4], b[1:4]),
    )


def _q_conj(q):
    return ca.vertcat(q[0], -q[1], -q[2], -q[3])


def _q_normalize(q):
    return q / ca.sqrt(ca.dot(q, q))


def _q_rotate(q, axis):
    qn = _q_normalize(q)
    rotated = _q_mul(_q_mul(qn, ca.vertcat(0, axis)), _q_conj(qn))
    return rotated[1:4]


def _attitude_residual(q, q_ref):
    qn = _q_normalize(q)
    qrn = _q_normalize(q_ref)
    # Sun Eq. (12), with the benchmark convention q_e = q^{-1} * q_ref.
    qe = _q_mul(_q_conj(qn), qrn)
    return qe[1:4]


def _default_allocation_matrix():
    ct = 1.51e-6
    cq = 2.37e-8
    kappa = cq / ct
    pos = np.array(
        [
            [0.13, 0.22, -0.023],
            [-0.13, -0.20, -0.023],
            [0.13, -0.22, -0.023],
            [-0.13, 0.20, -0.023],
        ],
        dtype=float,
    )
    axis = np.array([0.0, 0.0, -1.0])
    spin = np.array([1.0, 1.0, -1.0, -1.0])
    km = kappa * spin

    b2 = np.zeros((6, 4))
    for i in range(4):
        moment = np.cross(pos[i, :], axis) - km[i] * axis
        force = axis
        b2[:, i] = np.concatenate((moment, force))

    return np.vstack((-b2[5:6, :], b2[0:3, :]))


def _config_vector(value, length, default):
    arr = np.asarray(default if value is None else value, dtype=float).reshape(-1)
    if arr.size != length:
        raise ValueError(f"expected vector length {length}, got {arr.size}")
    return np.ascontiguousarray(arr, dtype=float)


def _as_square(value, size, default):
    arr = np.asarray(default if value is None else value, dtype=float)
    if arr.ndim == 1:
        arr = np.diag(arr.reshape(-1))
    arr = arr.reshape((size, size))
    return np.ascontiguousarray(arr, dtype=float)


def _as_matrix(value, shape, default):
    arr = np.asarray(default if value is None else value, dtype=float).reshape(shape)
    return np.ascontiguousarray(arr, dtype=float)


def _default_config():
    return {
        "n_horizon": 20,
        "dt": 0.05,
        "mass": 0.75,
        "gravity": 9.81,
        "inertia_diag": np.array([0.0025, 0.0021, 0.0043], dtype=float),
        # Positive-thrust convention: [T; tau] = G * u.
        "allocation_matrix": _default_allocation_matrix(),
        "u_min": np.zeros(4),
        "u_max": 8.5 * np.ones(4),
        "omega_max": np.array([10.0, 10.0, 4.0], dtype=float),
        "aero_enabled": True,
        "aero_kd": np.array([0.26, 0.28, 0.42], dtype=float),
        "aero_kh": 0.01,
        "q_xi": np.diag([200.0, 200.0, 500.0]),
        "q_v": np.eye(3),
        "q_q": np.diag([5.0, 5.0, 200.0]),
        "q_omega": np.eye(3),
        "q_u": 6.0 * np.eye(4),
        "code_export_dir": "/private/tmp/uav_sun_acados_codegen",
    }


def _normalize_config(config=None):
    cfg = _default_config()
    if config:
        for key, value in config.items():
            if value is not None:
                cfg[key] = value

    cfg["n_horizon"] = int(cfg["n_horizon"])
    cfg["dt"] = float(cfg["dt"])
    cfg["mass"] = float(cfg["mass"])
    cfg["gravity"] = float(cfg["gravity"])
    cfg["inertia_diag"] = _config_vector(
        cfg.get("inertia_diag"), 3, cfg["inertia_diag"]
    )
    cfg["allocation_matrix"] = _as_matrix(
        cfg.get("allocation_matrix"), (4, 4), cfg["allocation_matrix"]
    )
    cfg["u_min"] = _config_vector(cfg.get("u_min"), 4, cfg["u_min"])
    cfg["u_max"] = _config_vector(cfg.get("u_max"), 4, cfg["u_max"])
    cfg["omega_max"] = _config_vector(cfg.get("omega_max"), 3, cfg["omega_max"])
    cfg["aero_enabled"] = bool(cfg["aero_enabled"])
    cfg["aero_kd"] = _config_vector(cfg.get("aero_kd"), 3, cfg["aero_kd"])
    cfg["aero_kh"] = float(cfg["aero_kh"])
    cfg["q_xi"] = _as_square(cfg.get("q_xi"), 3, cfg["q_xi"])
    cfg["q_v"] = _as_square(cfg.get("q_v"), 3, cfg["q_v"])
    cfg["q_q"] = _as_square(cfg.get("q_q"), 3, cfg["q_q"])
    cfg["q_omega"] = _as_square(cfg.get("q_omega"), 3, cfg["q_omega"])
    cfg["q_u"] = _as_square(cfg.get("q_u"), 4, cfg["q_u"])
    cfg["code_export_dir"] = str(cfg["code_export_dir"])
    return cfg


def _aero_force_body(q, v, kd, kh, enabled):
    if not enabled:
        return ca.DM.zeros(3)

    v_body = _q_rotate(_q_conj(_q_normalize(q)), v)
    lateral_speed_sq = v_body[0] * v_body[0] + v_body[1] * v_body[1]
    return ca.vertcat(
        -kd[0] * v_body[0],
        -kd[1] * v_body[1],
        -kd[2] * v_body[2] - kh * lateral_speed_sq,
    )


def _make_model(config=None):
    cfg = _normalize_config(config)

    model = AcadosModel()
    model.name = "sun_quadrotor_nmpc"

    p = ca.MX.sym("p", 3)
    q = ca.MX.sym("q", 4)
    v = ca.MX.sym("v", 3)
    omega = ca.MX.sym("omega", 3)
    x = ca.vertcat(p, q, v, omega)

    xdot = ca.MX.sym("xdot", 13)
    u = ca.MX.sym("u", 4)

    q_ref = ca.MX.sym("q_ref", 4)
    param = q_ref

    mass = cfg["mass"]
    gravity = cfg["gravity"]
    inertia_diag = cfg["inertia_diag"].tolist()
    inertia = ca.diag(ca.DM(inertia_diag))
    inv_inertia = ca.diag(ca.DM([1.0 / x for x in inertia_diag]))
    e3 = ca.DM([0.0, 0.0, 1.0])
    G = ca.DM(cfg["allocation_matrix"])
    aero_kd = ca.DM(cfg["aero_kd"])
    aero_kh = cfg["aero_kh"]

    wrench = G @ u
    thrust = wrench[0]
    tau = wrench[1:4]

    # Sun Eq. (translate) with aerodynamic force from Eq. (9). The body-torque
    # disturbance d_tau is not predicted; INDI handles it through measurements.
    q_dot = 0.5 * _q_mul(_q_normalize(q), ca.vertcat(0, omega))
    aero_body = _aero_force_body(q, v, aero_kd, aero_kh, cfg["aero_enabled"])
    v_dot = gravity * e3 - thrust / mass * _q_rotate(q, e3) \
        + _q_rotate(q, aero_body) / mass
    omega_dot = inv_inertia @ (tau - ca.cross(omega, inertia @ omega))
    f_expl = ca.vertcat(v, q_dot, v_dot, omega_dot)

    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = param
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl

    model.cost_y_expr = ca.vertcat(
        p,
        _attitude_residual(q, q_ref),
        v,
        omega,
        u,
    )
    model.cost_y_expr_e = ca.vertcat(
        p,
        _attitude_residual(q, q_ref),
        v,
        omega,
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


def _np_q_mul(a, b):
    return np.array(
        [
            a[0] * b[0] - np.dot(a[1:4], b[1:4]),
            a[0] * b[1] + b[0] * a[1] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] + b[0] * a[2] + a[3] * b[1] - a[1] * b[3],
            a[0] * b[3] + b[0] * a[3] + a[1] * b[2] - a[2] * b[1],
        ],
        dtype=float,
    )


def _np_q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def _np_q_normalize(q):
    q = np.asarray(q, dtype=float).reshape(4)
    return q / np.linalg.norm(q)


def _np_q_rotate(q, axis):
    qn = _np_q_normalize(q)
    rotated = _np_q_mul(_np_q_mul(qn, np.r_[0.0, axis]), _np_q_conj(qn))
    return rotated[1:4]


def _np_aero_force_body(q, v, cfg):
    if not cfg["aero_enabled"]:
        return np.zeros(3)
    v_body = _np_q_rotate(_np_q_conj(q), v)
    kd = cfg["aero_kd"]
    lateral_speed_sq = v_body[0] * v_body[0] + v_body[1] * v_body[1]
    return np.array(
        [
            -kd[0] * v_body[0],
            -kd[1] * v_body[1],
            -kd[2] * v_body[2] - cfg["aero_kh"] * lateral_speed_sq,
        ],
        dtype=float,
    )


def _np_dynamics(x, u, cfg):
    q = _np_q_normalize(x[3:7])
    v = x[7:10]
    omega = x[10:13]

    wrench = cfg["allocation_matrix"] @ u
    thrust = wrench[0]
    tau = wrench[1:4]
    inertia_diag = cfg["inertia_diag"]
    inertia_omega = inertia_diag * omega

    q_dot = 0.5 * _np_q_mul(q, np.r_[0.0, omega])
    aero_body = _np_aero_force_body(q, v, cfg)
    e3 = np.array([0.0, 0.0, 1.0])
    v_dot = (
        cfg["gravity"] * e3
        - thrust / cfg["mass"] * _np_q_rotate(q, e3)
        + _np_q_rotate(q, aero_body) / cfg["mass"]
    )
    omega_dot = (tau - np.cross(omega, inertia_omega)) / inertia_diag

    return np.r_[v, q_dot, v_dot, omega_dot]


def _np_rk4_step(x, u, dt, cfg):
    k1 = _np_dynamics(x, u, cfg)
    k2 = _np_dynamics(x + 0.5 * dt * k1, u, cfg)
    k3 = _np_dynamics(x + 0.5 * dt * k2, u, cfg)
    k4 = _np_dynamics(x + dt * k3, u, cfg)
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    x_next[3:7] = _np_q_normalize(x_next[3:7])
    return x_next


class SunAcadosNMPC:
    def __init__(self, n_horizon=None, dt=None, code_export_dir=None, config=None):
        cfg = _normalize_config(config)
        if n_horizon is not None:
            cfg["n_horizon"] = int(n_horizon)
        if dt is not None:
            cfg["dt"] = float(dt)
        if code_export_dir is not None:
            cfg["code_export_dir"] = str(code_export_dir)

        self.config = cfg
        self.N = int(cfg["n_horizon"])
        self.dt = float(cfg["dt"])
        self.nx = 13
        self.nu = 4
        self.np = 4
        self.initialized = False

        code_export_dir = cfg["code_export_dir"]
        self.code_export_dir = str(Path(code_export_dir).expanduser())
        Path(self.code_export_dir).mkdir(parents=True, exist_ok=True)

        ocp = AcadosOcp()
        ocp.model = _make_model(cfg)
        ocp.code_export_directory = self.code_export_dir
        ocp.parameter_values = np.array([1.0, 0.0, 0.0, 0.0])

        q_xi = cfg["q_xi"]
        q_v = cfg["q_v"]
        q_q = cfg["q_q"]
        q_omega = cfg["q_omega"]
        q_u = cfg["q_u"]
        q_terminal = np.block(
            [
                [q_xi, np.zeros((3, 9))],
                [np.zeros((3, 3)), q_q, np.zeros((3, 6))],
                [np.zeros((3, 6)), q_v, np.zeros((3, 3))],
                [np.zeros((3, 9)), q_omega],
            ]
        )

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.block(
            [
                [q_xi, np.zeros((3, 13))],
                [np.zeros((3, 3)), q_q, np.zeros((3, 10))],
                [np.zeros((3, 6)), q_v, np.zeros((3, 7))],
                [np.zeros((3, 9)), q_omega, np.zeros((3, 4))],
                [np.zeros((4, 12)), q_u],
            ]
        )
        ocp.cost.W_e = q_terminal
        ocp.cost.yref = np.zeros(16)
        ocp.cost.yref_e = np.zeros(12)

        ocp.constraints.x0 = np.zeros(self.nx)
        ocp.constraints.lbu = cfg["u_min"]
        ocp.constraints.ubu = cfg["u_max"]
        ocp.constraints.idxbu = np.arange(self.nu)
        ocp.constraints.lbx = -cfg["omega_max"]
        ocp.constraints.ubx = cfg["omega_max"]
        ocp.constraints.idxbx = np.array([10, 11, 12])

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
            self.solver.set(k, "p", q_ref[k])
            y_ref = np.concatenate(
                (p_ref[k], np.zeros(3), v_ref[k], omega_ref[k], u_ref[k])
            )
            self.solver.set(k, "yref", y_ref)

        self.solver.set(self.N, "p", q_ref[self.N])
        y_ref_e = np.concatenate(
            (p_ref[self.N], np.zeros(3), v_ref[self.N], omega_ref[self.N])
        )
        self.solver.set(self.N, "yref", y_ref_e)

    def _initialize_guess(self, x0, p_ref, q_ref, v_ref, omega_ref, u_ref):
        if self.initialized:
            return
        x_guess = x0.copy()
        for k in range(self.N + 1):
            self.solver.set(k, "x", x_guess)
            if k < self.N:
                u_guess = np.clip(
                    u_ref[k], self.config["u_min"], self.config["u_max"]
                )
                self.solver.set(k, "u", u_guess)
                x_guess = _np_rk4_step(x_guess, u_guess, self.dt, self.config)
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


def configure(
    n_horizon=20,
    dt=0.05,
    mass=0.75,
    gravity=9.81,
    inertia_diag=None,
    allocation_matrix=None,
    u_min=None,
    u_max=None,
    omega_max=None,
    aero_enabled=True,
    aero_kd=None,
    aero_kh=0.01,
    q_xi=None,
    q_v=None,
    q_q=None,
    q_omega=None,
    q_u=None,
    code_export_dir=None,
):
    global _CONFIG, _SOLVER

    config = {
        "n_horizon": n_horizon,
        "dt": dt,
        "mass": mass,
        "gravity": gravity,
        "inertia_diag": inertia_diag,
        "allocation_matrix": allocation_matrix,
        "u_min": u_min,
        "u_max": u_max,
        "omega_max": omega_max,
        "aero_enabled": aero_enabled,
        "aero_kd": aero_kd,
        "aero_kh": aero_kh,
        "q_xi": q_xi,
        "q_v": q_v,
        "q_q": q_q,
        "q_omega": q_omega,
        "q_u": q_u,
    }
    if code_export_dir is not None:
        config["code_export_dir"] = code_export_dir

    _CONFIG = _normalize_config(config)
    _SOLVER = None


def reset():
    global _SOLVER
    _SOLVER = None


def reset_warm_start():
    if _SOLVER is not None:
        _SOLVER.initialized = False


def solve(x0, p_ref, q_ref, v_ref, omega_ref, u_ref):
    global _SOLVER
    if _SOLVER is None:
        _SOLVER = SunAcadosNMPC(config=_CONFIG)
    return _SOLVER.solve(x0, p_ref, q_ref, v_ref, omega_ref, u_ref)


def allocation_matrix():
    return _normalize_config(_CONFIG)["allocation_matrix"]
