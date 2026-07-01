UAV Algorithm Benchmark
=======================

A MATLAB-based UAV simulation framework for rapid validation and benchmarking
of control and planning algorithms.

Coordinate convention: simulation states, controller references, and 3D plots
use NED coordinates (`x` north, `y` east, `z` down). 3D figures reverse the
z-axis direction and include a small NED reference triad so positive `z_NED`
appears visually downward.

Quick Start
-----------

Run the default single-case simulation:

```matlab
main
```

Run the disturbance benchmark:

```matlab
main_disturbance_benchmark
```

`main.m` currently defaults to:

```matlab
par.controllerName = "px4_iris";
```

To run Sun et al. NMPC in a single case, set:

```matlab
par.controllerName = "sun_nmpc";
```

`main_trajectory_sweep.m` is configured to sweep `sun_nmpc` by default.
The Sun NMPC path is acados-backed; it is not a pure MATLAB implementation.

Sun NMPC Dependency Setup
-------------------------

The strict Sun et al. NMPC reproduction uses MATLAB as the simulation host and
Python/acados as the nonlinear MPC solver generator/runtime:

```text
MATLAB main.m
  -> controllerSunNMPC
  -> MATLAB py.* bridge
  -> tools/sun_acados_nmpc.py
  -> acados SQP-RTI generated C solver
  -> rotor thrusts u1..u4
  -> MATLAB plant step
```

Required external tools:

- MATLAB with Python support.
- Command-line tools: `git`, `cmake`, and a C/C++ compiler.

Recommended one-command acados install from MATLAB:

```matlab
install_acados
```

This clones and builds acados, creates `.venv`, installs Python packages
(`casadi`, `scipy`, `matplotlib`, `cython`, `Deprecated`, `acados_template`),
downloads `bin/t_renderer`, and runs a short `sun_nmpc` smoke test to verify
that this repository can call the generated acados solver. To skip that final
algorithm-level check:

```matlab
install_acados("RunSunSmokeTest", false)
```

The default install is repository-local and ignored by git:

```text
.acados/acados   acados source/build/install tree
.venv            Python environment used by MATLAB
```

The installer chooses platform-specific CMake options automatically. On Apple
Silicon it uses `BLASFEO_TARGET=ARMV8A_APPLE_M1` and
`CMAKE_OSX_ARCHITECTURES=arm64`; on x86_64 it uses acados'
`X64_AUTOMATIC` target.

The runtime code looks for Python in this order:

1. `ACADOS_PYTHON`, when explicitly set.
2. Legacy `SUN_NMPC_PYTHON`, when explicitly set.
3. `.venv/bin/python3` under this repository.
4. MATLAB's current/default Python, only if no project Python exists.

Manual install, if you do not want to use `install_acados`:

```bash
mkdir -p .acados
git clone --depth 1 --recurse-submodules https://github.com/acados/acados.git .acados/acados
cmake -S .acados/acados -B .acados/acados/build -DACADOS_WITH_QPOASES=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=.acados/acados
cmake --build .acados/acados/build --target install -j4
```

On Apple Silicon, add these CMake options to avoid accidentally building x86_64
libraries:

```bash
-DBLASFEO_TARGET=ARMV8A_APPLE_M1 -DCMAKE_OSX_ARCHITECTURES=arm64
```

For a non-default acados path, set it inside MATLAB before setup, especially
when MATLAB is opened from the macOS Dock:

```matlab
setenv("ACADOS_SOURCE_DIR", "/path/to/acados")
```

Then start MATLAB, `cd` to this repository, and run:

```matlab
setup_acados_python
```

The setup script creates `.venv` when needed, installs the Python packages into
that environment, installs `acados_template` from your acados checkout, and
downloads `bin/t_renderer` when it is missing.

If you want MATLAB to use a specific Python instead of `.venv`, set this before
running MATLAB or before calling `setup_acados_python`:

```matlab
setenv("ACADOS_PYTHON", "/path/to/python3")
```

If you see an error like `No module named 'casadi'` and MATLAB reports
`/Applications/Xcode.app/.../python3`, MATLAB has already loaded the wrong
Python. Run `install_acados`, restart MATLAB, `cd` back to this
repository, and run `main` or `main_trajectory_sweep` again. Parallel workers
also use the same `.venv/bin/python3` path after restart.

Current Sun NMPC Architecture
-----------------------------

The implemented NMPC is in [tools/sun_acados_nmpc.py](tools/sun_acados_nmpc.py).
It follows the Sun/Agilicious Kingfisher platform parameters used in `main.m`:

- State: `x = [p(3); q_wxyz(4); v(3); Omega(3)]`.
- Input: four rotor thrusts `u = [u1; u2; u3; u4]`.
- Allocation map: `G*u = [T; tau_x; tau_y; tau_z]`.
- Horizon: `N = 20`, `dt = 0.05 s`.
- Solver: acados `SQP_RTI`, ERK integration, Gauss-Newton Hessian.
- Input bounds: `0 <= ui <= 8.5 N`.
- Body-rate bounds: `|Omega| <= [10, 10, 4] rad/s`.
- Stage cost:
  - position weights `[200, 200, 500]`;
  - velocity weights `[1, 1, 1]`;
  - tilt/yaw attitude residual weights `[5, 5, 200]`;
  - body-rate weights `[1, 1, 1]`;
  - rotor thrust weights `6*I`.

The public Sun NMPC controller names are:

- `sun_nmpc`: the Sun et al. Eq. (10) nonlinear MPC OCP, solved through
  acados/SQP-RTI internally.
- `sun_nmpc_indi`: the same NMPC outer loop combined with the Sun et al. INDI
  inner loop for disturbance robustness.

acados is an implementation detail of `sun_nmpc`, not a different algorithm.
For NMPC, the internal prediction reference is allowed to continue past
`par.Tend` by the controller horizon, while the MATLAB simulation still stops
exactly at `par.Tend`. The disturbance benchmark can exclude the last
prediction horizon from statistics with:

```matlab
cfg.errorEvalMode = "sun_prediction_horizon";
```

Notes on Other MATLAB NMPC Repositories
---------------------------------------

Several public MATLAB NMPC repositories were checked as possible alternatives.
They are useful references, but most do not solve the core issue for strict Sun
et al. reproduction:

- `DeathstrokeN/matlab-drone-ekf-nmpc`: direct-shooting `fmincon`; useful for
  simple MATLAB structure, not real-time Sun NMPC.
- `FrancescoZ83/UAV-Trajectory-Tracking-Adaptive-NMPC-vs.-Observer-based-MPC`:
  reports a `quadprog`-based nonlinear/adaptive MPC style; not the Sun acados
  OCP.
- `industoai/Nonlinear-Model-Predictive-Control`: educational `fmincon`
  examples.
- `Giapducnguyen/NMPC-Multiple-Shooting`: CasADi + IPOPT multiple shooting;
  better modeling style than `fmincon`, but still not SQP-RTI/acados.
- `kul-optec/nmpc-codegen-matlab`: code-generation/PANOC approach and the most
  relevant pure MATLAB-adjacent alternative, but it targets older CasADi and
  would require a separate port.
- `mlazar04/sNMPC`: YALMIP/CasADi/IPOPT style stochastic NMPC toolbox; not a
  direct quadrotor agile-flight reproduction.
- `Chanho-Ko/NMPC-planning-simulink`: MATLAB `nlmpc`/Simulink workflow; useful
  for planning examples, not real-time agile-flight NMPC.
- `Murad275/nmpc_CarSim`: CasADi + IPOPT CarSim block; vehicle-focused.
- `HybridRobotics/NMPC-DCLF-DCBF`: YALMIP + IPOPT safety-critical MPC/CBF
  research code; not quadrotor Sun NMPC.
- `CindiFeng/NMPC-SlungLoadQuad`: MATLAB `nlmpc` and `nlmpcMultistage`; useful
  MATLAB toolbox reference, but not fast enough for this reproduction target.

For the Sun et al. result, acados remains the closest match because the
Agilicious implementation also uses generated acados solvers with SQP-RTI-like
operation.
