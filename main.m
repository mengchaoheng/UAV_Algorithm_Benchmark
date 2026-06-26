%% main.m
% Simple quadrotor simulation with modular reference trajectories.
% Internal coordinate: NED, z_NED points downward.
%
% State:
%   p : position in NED
%   v : velocity in NED
%   R : body-to-NED rotation matrix
%   Omega : body angular velocity expressed in body frame
%
% Input:
%   T   : total thrust force
%   tau : body-frame moment
%
% Reference interface:
%   ref.p   position
%   ref.v   velocity
%   ref.a   acceleration
%   ref.psi yaw

clear; clc; close all;

%% ========================================================================
%% 0. Parameters
par.g = 9.81;
par.e3 = [0;0;1];
par.m = 1.0;
par.J = diag([0.07, 0.07, 0.12]);

par.dt = 0.01;          % 100 Hz
par.Tend = 15.0;
par.integratorName = "ode45";  % "ode45" or "lie_rk4"

% Reference time scaling.
% scale > 1 slows the reference; scale < 1 speeds it up and may saturate control.
par.progress.mode = "scale_range";      % "scale_fixed" or "scale_range"
par.progress.scale = 2.5;               % scale_fixed: constant time scale
par.progress.scaleRange = [2, 0.5];   % scale_range: start/end scale over the simulation

% Available choices:
%   "figure8_horizontal"
%   "figure8_vertical"
%   "helix_flip"
%   "flip_loop_sine"
%   "fast_circle"
par.trajName = "helix_flip";

% controller
% "geometric", "lee", "johnson_beard"
% "sun_dfbc", "sun_dfbc_indi", "faessler"
% "on_manifold_mpc", "sun_nmpc", "sun_nmpc_full", "sun_nmpc_indi"
% "geometric_indi", "tal_karaman"
par.controllerName = "on_manifold_mpc";  
% Simple controller gains
par.Kp = diag([20, 20, 25]);
par.Kv = diag([9, 9, 10]);
par.KR = 35*eye(3);
par.KOmega = 35*par.J;

% Controller-specific gain namespaces. The base gains above remain as
% convenient defaults, but controller code should use its own namespace so
% paper implementations can be tuned independently.
par.geometric.Kp = par.Kp;
par.geometric.Kv = par.Kv;
par.geometric.KR = par.KR;
par.geometric.KOmega = par.KOmega;

par.lee.Kp = par.Kp;
par.lee.Kv = par.Kv;
par.lee.KR = par.KR;
par.lee.KOmega = par.KOmega;

par.johnsonBeard.Kp = par.Kp;
par.johnsonBeard.Kv = par.Kv;
par.johnsonBeard.KR = par.KR;
par.johnsonBeard.KOmega = par.KOmega;

% On-manifold finite-horizon controller.
% State error: [p-pd; v-vd; Log(Rd'R)], input: [aT-aTd; Omega-OmegaD].
par.mpc.N = 16; % Lu et al. use N=8; use longer horizon for the simulated rate loop.
par.mpc.Q = diag([450, 450, 650, ...
                  70, 70, 100, ...
                  140, 140, 80]);
par.mpc.R = diag([1.0, 0.55, 0.55, 0.75]);
par.mpc.P = par.mpc.Q;
par.mpc.omegaMax = deg2rad(800);
par.mpc.KOmega = par.KOmega;

% Sun/Foehn/Agilicious MPC defaults. These are the C++ defaults matching the
% paper's Table I for the state cost; the rotor-thrust input weight R=6 is
% projected to this benchmark's collective-thrust channel where possible.
par.sun.N = 20;
par.sun.dt = 0.05;
par.sun.Qpos = diag([200, 200, 500]);
par.sun.Qatt = diag([5, 5, 200]);
par.sun.Qvel = eye(3);
par.sun.Qomega = eye(3);
par.sun.Rrotor = 6*eye(4);
par.sun.Rcollective = par.sun.Rrotor(1,1)/4;
% The simplified Sun NMPC uses virtual inputs [collective acceleration; body
% rate], so only the collective channel maps directly from rotor R.
par.sun.Rvirtual = diag([par.sun.Rcollective, 1, 1, 1]);
par.sun.Q = blkdiag(par.sun.Qpos, par.sun.Qvel, par.sun.Qatt);
par.sun.P = par.sun.Q;
par.sun.omegaMax = par.mpc.omegaMax;
par.sun.Kp = par.Kp;
par.sun.Kv = par.Kv;
par.sun.KR = par.KR;
par.sun.KOmega = par.KOmega;

% Full nonlinear MPC replica, using MATLAB fmincon instead of ACADO/acados.
% Foehn/Sun use N=20, dt=0.05, single-rotor thrust inputs, and RTI SQP.
% Here the dynamics are full nonlinear, while control allocation is simplified
% to total thrust and body moments [T; tau]. MATLAB fmincon is far slower
% than ACADO/acados, so the default full-NMPC path uses single shooting and
% falls back to sun_nmpc if the nonlinear solve fails.
par.sunFull.N = par.sun.N;
par.sunFull.dt = par.sun.dt;
par.sunFull.solvePeriod = 0.10;
par.sunFull.maxIterations = 2;
par.sunFull.maxFunctionEvaluations = 400;
par.sunFull.Qpos = par.sun.Qpos;
par.sunFull.Qatt = par.sun.Qatt;
par.sunFull.Qvel = par.sun.Qvel;
par.sunFull.Qomega = par.sun.Qomega;
% Agilicious uses R=6 for each rotor thrust; equal-rotor collective gives 6/4.
% Moment channels have no exact equivalent after skipping allocation, so they
% keep a dimensionless regularizer derived from the C++ omega weight.
par.sunFull.R = diag([par.sun.Rcollective, 1, 1, 1]);
par.sunFull.RtauNorm = 0.5;

% Geometric INDI gains.
par.indi.Kp = par.Kp;
par.indi.Kv = par.Kv;
par.indi.KR = par.KR;
par.indi.KOmega = par.KOmega;
par.indi.Ktheta = 55*eye(3);
par.indi.Komega = 14*eye(3);

% Faessler et al. rotor-drag flatness controller.
% Keep the rotor-drag interface, but default to zero because the current
% benchmark plant has no external drag force or drag torque model.
% Paper example coefficients: diag([0.544, 0.386, 0.0]).
par.faessler.D = zeros(3);
% Paper Eq. (5) thrust disturbance coefficient. The benchmark input is direct
% force, not a motor thrust command, so the framework-adapted default is zero.
par.faessler.kh = 0.0;
par.faessler.Kp = par.Kp;
par.faessler.Kv = par.Kv;
par.faessler.KR = par.KR;
par.faessler.KOmega = par.KOmega;

% Tal and Karaman INDI + differential-flatness controller.
% Use controller-specific gains even when their numerical defaults are close
% to the baseline controller; this keeps each paper controller independently
% tunable.
par.tal.Kp = diag([20, 20, 25]);         % Eq. (17), position term
par.tal.Kv = diag([9, 9, 10]);           % Eq. (17), velocity term
par.tal.Ka = 0.3*eye(3);                 % Eq. (17), acceleration term
% Eq. (28), tuned for this direct force/moment plant. The paper closes the
% loop through motor dynamics and allocation; without that layer, lower gains
% let the vehicle lag almost 180 deg during flips.
par.tal.Ktheta = 200*eye(3);
par.tal.Komega = 45*eye(3);
% Paper Fig. 4 applies identical LPFs to IMU acceleration/rate derivatives
% and motor-output-derived thrust/moment signals. The benchmark has no motor
% model or sensor noise, but the finite-difference signals still need the
% same phase-consistent filtering for INDI. Use a shorter time constant than
% a hardware implementation to avoid adding artificial phase lag in the ideal
% direct-actuation simulation.
par.tal.filterTau = 0.01;
% Eq. (14)-(15) can become numerically ill-conditioned near the yaw
% parametrization singularity during flip trajectories. Beyond this bound the
% code falls back to SO(3) finite-difference feed-forward and documents it at
% the call site below.
par.tal.alphaRefMax = 500;
par.tal.omegaRefMax = 80;
par.tal.flatnessRcondMin = 1e-6;

% Actuator limits
par.Tmax = 4*9.81;
par.tauMax = [8; 8; 8];

% Initial condition
par.startOnReference = true;

% 3D attitude sampling visualization
par.poseEvery = 0.10;       % seconds
par.bodyAxisScale = 0.5;   % meters
par.poseSource = "actual";  % "actual" or "desired"

% Post-simulation 3D animation
par.enableAnimation = true;
par.animationSpeed = 1;       % 1.0 = real time
par.animationFrameDt = 0.02;    % seconds

%% ========================================================================
%% 1. Build trajectory
traj = makeTrajectory(par);
par.Tend = traj.Tend;

%% ========================================================================
%% 2. Initial state
ref0 = traj.eval(0);
[R0, ~] = desiredAttitudeFromAccel(ref0.a, ref0.psi, par);

if par.startOnReference
    x.p = ref0.p;
    x.v = ref0.v;
    x.R = R0;
    x.Omega = zeros(3,1);
else
    x.p = [0;0;0];
    x.v = [0;0;0];
    x.R = eye(3);
    x.Omega = zeros(3,1);
end

%% ========================================================================
%% 3. Logs
time = 0:par.dt:par.Tend;
N = numel(time);

log.p = zeros(3,N);
log.v = zeros(3,N);
log.pd = zeros(3,N);
log.vd = zeros(3,N);
log.ad = zeros(3,N);

log.R = zeros(3,3,N);
log.Rd = zeros(3,3,N);
log.Omega = zeros(3,N);

log.euler = zeros(3,N);
log.eulerD = zeros(3,N);

log.T = zeros(1,N);
log.tau = zeros(3,N);
log.sunFullUsedFallback = false(1,N);
log.sunFullSolved = false(1,N);
log.sunFullFallbackCode = zeros(1,N);
log.sunFullExitflag = nan(1,N);
log.sunFullCostRatio = nan(1,N);

%% ========================================================================
%% 4. Simulation loop
for k = 1:N
    t = time(k);

    ref = traj.eval(t);
    u = controller(x, ref, traj, t, par);

    log.p(:,k) = x.p;
    log.v(:,k) = x.v;
    log.pd(:,k) = ref.p;
    log.vd(:,k) = ref.v;
    log.ad(:,k) = ref.a;

    log.R(:,:,k) = x.R;
    log.Rd(:,:,k) = u.Rd;
    log.Omega(:,k) = x.Omega;

    log.euler(:,k) = rotm2eulZYX(x.R);
    log.eulerD(:,k) = rotm2eulZYX(u.Rd);

    log.T(k) = u.T;
    log.tau(:,k) = u.tau;

    if isfield(u, 'sunFullUsedFallback')
        log.sunFullUsedFallback(k) = u.sunFullUsedFallback;
        log.sunFullSolved(k) = u.sunFullSolved;
        log.sunFullFallbackCode(k) = u.sunFullFallbackCode;
        log.sunFullExitflag(k) = u.sunFullExitflag;
        log.sunFullCostRatio(k) = u.sunFullCostRatio;
    end

    x = stepModel(x, u, par);
end

if par.controllerName == "sun_nmpc_full"
    nFallback = nnz(log.sunFullUsedFallback);
    nScheduled = nnz(log.sunFullFallbackCode == 2);
    nSolverFallback = nnz(log.sunFullFallbackCode == 1 ...
        | log.sunFullFallbackCode == 3 ...
        | log.sunFullFallbackCode == 4);
    fprintf(['sun_nmpc_full fallback: %d/%d total, ' ...
        '%d scheduled skips, %d solver-triggered. Optimized commands: %d.\n'], ...
        nFallback, N, nScheduled, nSolverFallback, nnz(log.sunFullSolved));
end

%% ========================================================================
%% 5. Plot
plotResults(time, log, par, traj);

if par.enableAnimation
    animateTrajectory3D(time, log, par, traj);
end

%% ========================================================================
%% Trajectory factory
function traj = makeTrajectory(par)

    switch par.trajName

        case "figure8_horizontal"
            traj.name = "figure8_horizontal";
            traj.Tend = 12.0;
            traj.eval = @(t) evalFigure8Horizontal(t);

        case "figure8_vertical"
            traj.name = "figure8_vertical";
            traj.Tend = par.Tend;
            traj.eval = @(t) evalFigure8Vertical(t);

        case "helix_flip"
            traj.name = "helix_flip";
            traj.Tend = par.Tend;
            traj.eval = @(t) evalHelixFlip(t);

        case "flip_loop_sine"
            traj.name = "flip_loop_sine";
            traj.Tend = par.Tend;
            traj.eval = @(t) evalFlipLoopSine(t);

        case "fast_circle"
            traj.name = "fast_circle";
            traj.Tend = par.Tend;
            traj.eval = @(t) evalFastCircle(t);

        otherwise
            error("Unknown trajectory name.");
    end

    traj = applyTrajectoryProgress(traj, par);
end

function traj = applyTrajectoryProgress(traj, par)

    baseEval = traj.eval;
    baseTend = traj.Tend;

    switch par.progress.mode
        case "scale_fixed"
            scale = par.progress.scale;

            if scale <= 0
                error("Trajectory time scale must be positive.");
            end

            traj.Tend = scale*baseTend;
            traj.eval = @(t) evalProgressTrajectory(baseEval, t/scale, 1/scale, 0, baseTend);

            if abs(scale - 1) >= 1e-12
                traj.name = traj.name + "_timeScale_" + string(scale);
            end

        case "scale_range"
            scaleRange = par.progress.scaleRange;

            if any(scaleRange <= 0)
                error("Trajectory time scale must be positive.");
            end

            traj.Tend = par.Tend;
            traj.name = traj.name + "_scaleRange_" + string(scaleRange(1)) ...
                      + "_" + string(scaleRange(2));
            traj.eval = @(t) evalScaleRangeTrajectory(baseEval, baseTend, t, traj.Tend, scaleRange);

        otherwise
            error("Unknown progress mode.");
    end
end

function ref = evalScaleRangeTrajectory(baseEval, baseTend, t, simTend, scaleRange)

    alpha = clampScalar(t/simTend, 0, 1);
    scale0 = scaleRange(1);
    scale1 = scaleRange(2);

    tClip = alpha*simTend;
    scaleDot = (scale1 - scale0)/simTend;
    scale = scale0 + scaleDot*tClip;

    % The scale is instantaneous: ds/dt = 1/scale(t).
    if abs(scaleDot) < 1e-12
        s = tClip/scale0;
    else
        s = log(scale/scale0)/scaleDot;
    end

    sDot = 1/scale;
    sDDot = -scaleDot/scale^2;

    ref = evalProgressTrajectory(baseEval, s, sDot, sDDot, baseTend);
end

function ref = evalProgressTrajectory(baseEval, s, sDot, sDDot, baseTend)

    s = clampScalar(s, 0, baseTend);
    ref = baseEval(s);

    vBase = ref.v;
    aBase = ref.a;
    ref.v = vBase*sDot;
    ref.a = aBase*sDot^2 + vBase*sDDot;
end

function y = clampScalar(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

%% ========================================================================
%% Analytic horizontal figure-eight
function ref = evalFigure8Horizontal(t)

    Ax = 4.0;
    Ay = 2.5;
    h0 = 3.0;
    Tfig = 12.0;
    Om = 2*pi/Tfig;

    ref.p = [Ax*sin(Om*t);
             Ay*sin(2*Om*t);
            -h0];

    ref.v = [Ax*Om*cos(Om*t);
             2*Ay*Om*cos(2*Om*t);
             0];

    ref.a = [-Ax*Om^2*sin(Om*t);
             -4*Ay*Om^2*sin(2*Om*t);
             0];

    ref.psi = atan2(ref.v(2), ref.v(1));
end

%% ========================================================================
%% Analytic vertical figure-eight
function ref = evalFigure8Vertical(t)

    Ay = 1.15;
    Az = 1.00;
    hLow = 1.35;
    hCenter = hLow + Az;
    Tfig = 5.50;
    tHover = 1.0;
    tRamp = 1.50;
    Om = 2*pi/Tfig;
    theta0 = -pi/4;

    if t <= tHover
        ref.p = [0; -Ay/sqrt(2); -hLow];
        ref.v = [0; 0; 0];
        ref.a = [0; 0; 0];
        ref.psi = 0;
        return;
    end

    tau = t - tHover;
    [q, qDot, qDDot] = rampedTime(tau, tRamp);

    theta = theta0 + Om*q;
    thetaDot = Om*qDot;
    thetaDDot = Om*qDDot;

    h = hCenter + Az*sin(2*theta);

    ref.p = [0;
             Ay*sin(theta);
            -h];

    ref.v = [0;
             Ay*cos(theta)*thetaDot;
            -2*Az*cos(2*theta)*thetaDot];

    ref.a = [0;
             Ay*(-sin(theta)*thetaDot^2 + cos(theta)*thetaDDot);
             4*Az*sin(2*theta)*thetaDot^2 - 2*Az*cos(2*theta)*thetaDDot];

    ref.psi = 0;
end

%% ========================================================================
%% Analytic helix with flips
function ref = evalHelixFlip(t)

    vx = 0.30;
    Ay = 0.80;
    Az = 0.80;
    hHover = 1.30;
    hCenter = hHover + Az;
    Tturn = 1.65;
    tHover = 1.0;
    tRamp = 1.50;
    Om = 2*pi/Tturn;

    if t <= tHover
        ref.p = [0; 0; -hHover];
        ref.v = [0; 0; 0];
        ref.a = [0; 0; 0];
        ref.psi = 0;
        return;
    end

    tau = t - tHover;
    [q, qDot, qDDot] = rampedTime(tau, tRamp);

    theta = pi + Om*q;
    thetaDot = Om*qDot;
    thetaDDot = Om*qDDot;

    h = hCenter + Az*cos(theta);

    ref.p = [vx*q;
             Ay*sin(theta);
            -h];

    ref.v = [vx*qDot;
             Ay*cos(theta)*thetaDot;
             Az*sin(theta)*thetaDot];

    ref.a = [vx*qDDot;
             Ay*(-sin(theta)*thetaDot^2 + cos(theta)*thetaDDot);
             Az*(cos(theta)*thetaDot^2 + sin(theta)*thetaDDot)];

    ref.psi = 0;
end

%% ========================================================================
%% Analytic vertical flip loop
function ref = evalFlipLoopSine(t)

    Ay = 1.0;
    Az = 1.5;
    hHover = 1.5;
    hCenter = hHover + Az;
    Tloop = 1.90;
    tHover = 1.0;
    tRamp = 1.50;
    Om = 2*pi/Tloop;

    if t <= tHover
        ref.p = [0; 0; -hHover];
        ref.v = [0; 0; 0];
        ref.a = [0; 0; 0];
        ref.psi = 0;
        return;
    end

    tau = t - tHover;
    [q, qDot, qDDot] = rampedTime(tau, tRamp);

    theta = pi + Om*q;
    thetaDot = Om*qDot;
    thetaDDot = Om*qDDot;

    h = hCenter + Az*cos(theta);

    ref.p = [0;
             Ay*sin(theta);
            -h];

    ref.v = [0;
             Ay*cos(theta)*thetaDot;
             Az*sin(theta)*thetaDot];

    ref.a = [0;
             Ay*(-sin(theta)*thetaDot^2 + cos(theta)*thetaDDot);
             Az*(cos(theta)*thetaDot^2 + sin(theta)*thetaDDot)];

    ref.psi = 0;
end

function [q, qDot, qDDot] = rampedTime(t, tRamp)

    if t <= 0
        q = 0;
        qDot = 0;
        qDDot = 0;
        return;
    end

    if t >= tRamp
        q = t - 0.5*tRamp;
        qDot = 1;
        qDDot = 0;
        return;
    end

    s = t/tRamp;

    q = 0.5*t - 0.5*tRamp/pi*sin(pi*s);
    qDot = 0.5*(1 - cos(pi*s));
    qDDot = 0.5*pi/tRamp*sin(pi*s);
end

%% ========================================================================
%% Analytic fast horizontal circle
function ref = evalFastCircle(t)

    radius = 5.0;
    Tcircle = 2.5;
    h0 = 5.0;
    Om = 2*pi/Tcircle;

    ref.p = [radius*cos(Om*t);
             radius*sin(Om*t);
            -h0];

    ref.v = [-radius*Om*sin(Om*t);
              radius*Om*cos(Om*t);
              0];

    ref.a = [-radius*Om^2*cos(Om*t);
             -radius*Om^2*sin(Om*t);
              0];

    ref.psi = atan2(ref.v(2), ref.v(1));
end

%% Controller layer
function u = controller(x, ref, traj, t, par)

    switch par.controllerName
        case "geometric"
            u = controllerPDGeometric(x, ref, par);
        case "lee"
            u = controllerLee(x, ref, t, par);
        case "johnson_beard"
            u = controllerJohnsonBeard(x, ref, t, par);
        case "sun_nmpc"
            u = controllerSunNMPC(x, ref, traj, t, par);
        case "sun_nmpc_full"
            u = controllerSunNMPCFull(x, ref, traj, t, par);
        case "sun_dfbc"
            u = controllerSunDFBC(x, ref, traj, t, par);
        case "sun_nmpc_indi"
            u = controllerSunNMPCINDI(x, ref, traj, t, par);
        case "sun_dfbc_indi"
            u = controllerSunDFBCINDI(x, ref, traj, t, par);
        case "on_manifold_mpc"
            u = controllerOnManifoldMPC(x, ref, traj, t, par);
        case "geometric_indi"
            u = controllerGeometricINDI(x, ref, t, par);
        case "faessler"
            u = controllerFaessler(x, ref, traj, t, par);
        case "tal_karaman"
            u = controllerTalKaraman(x, ref, traj, t, par);
        otherwise
            error("Unknown controllerName.");
    end
end

function u = controllerPDGeometric(x, ref, par)

    ep = ref.p - x.p;
    ev = ref.v - x.v;

    aCmd = ref.a + par.geometric.Kp*ep + par.geometric.Kv*ev;

    [Rd, T] = desiredAttitudeFromAccel(aCmd, ref.psi, par, x.R);

    rErr = LogSO3(x.R' * Rd);

    tau = cross(x.Omega, par.J*x.Omega) ...
        + par.geometric.KR*rErr + par.geometric.KOmega*(zeros(3,1)-x.Omega);

    u.T = min(max(T, 0), par.Tmax);
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = Rd;
end

function u = controllerLee(x, ref, t, par)

    persistent st

    ex = x.p - ref.p;
    ev = x.v - ref.v;

    aCmd = ref.a - par.lee.Kp*ex - par.lee.Kv*ev;
    thrustAxisForce = par.m*(par.g*par.e3 - aCmd);
    [Rc, ~] = desiredAttitudeFromThrustVector(thrustAxisForce, ref.psi, par);

    if isempty(st) || t <= par.dt/2 || t <= st.t
        OmegaC = zeros(3,1);
        OmegaCDot = zeros(3,1);
    else
        h = max(t - st.t, par.dt);
        OmegaC = -LogSO3(Rc' * st.Rc)/h;
        OmegaCPrev = Rc' * st.Rc * st.OmegaC;
        OmegaCDot = (OmegaC - OmegaCPrev)/h;
    end

    eR = 0.5*vee(Rc' * x.R - x.R' * Rc);
    eOmega = x.Omega - x.R' * Rc * OmegaC;

    tau = -par.lee.KR*eR - par.lee.KOmega*eOmega ...
        + cross(x.Omega, par.J*x.Omega) ...
        - par.J*(hat(x.Omega)*x.R' * Rc * OmegaC - x.R' * Rc * OmegaCDot);

    T = dot(thrustAxisForce, x.R*par.e3);

    u.T = min(max(T, 0), par.Tmax);
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = Rc;

    st.Rc = Rc;
    st.OmegaC = OmegaC;
    st.t = t;
end

function u = controllerJohnsonBeard(x, ref, t, par)

    persistent st

    ep = x.p - ref.p;
    ev = x.v - ref.v;

    aCmd = ref.a - par.johnsonBeard.Kp*ep - par.johnsonBeard.Kv*ev;
    desiredForce = par.m*(aCmd - par.g*par.e3);
    thrustAxisForce = -desiredForce;
    [Rd, ~] = desiredAttitudeFromThrustVector(thrustAxisForce, ref.psi, par);

    if isempty(st) || t <= par.dt/2 || t <= st.t
        OmegaD = zeros(3,1);
        OmegaDDot = zeros(3,1);
    else
        h = max(t - st.t, par.dt);
        OmegaD = -LogSO3(Rd' * st.Rd)/h;
        OmegaDPrev = Rd' * st.Rd * st.OmegaD;
        OmegaDDot = (OmegaD - OmegaDPrev)/h;
    end

    Rbd = x.R' * Rd;
    r = LogSO3(Rbd);
    omegaDInBody = Rbd * OmegaD;
    omegaErr = omegaDInBody - x.Omega;
    omegaDDotInBody = Rbd * OmegaDDot - hat(x.Omega)*omegaDInBody;

    Jlinv = leftJacobianSO3Inv(r);
    tau = cross(x.Omega, par.J*x.Omega) ...
        + par.J*omegaDDotInBody ...
        + Jlinv' * par.johnsonBeard.KR*r ...
        + par.johnsonBeard.KOmega*omegaErr;

    u.T = min(norm(desiredForce), par.Tmax);
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = Rd;

    st.Rd = Rd;
    st.OmegaD = OmegaD;
    st.t = t;
end

function u = controllerFaessler(x, ref, traj, t, par)

    % Faessler et al. 2018, Section V, adapted to this benchmark:
    % - Paper frame: z_W is up and the thrust axis is +z_B.
    % - This code: NED z points down and thrust acceleration is
    %   -T/m * R*e3. The force-axis vector below is therefore c*b3_down.
    % - Paper output: [c_cmd, omega_des] or [c_cmd, tau] if the platform
    %   exposes moments. This framework outputs direct [T; tau].
    % - Paper Eq. (17)-(30) compute omega_ref and omegadot_ref from jerk and
    %   snap. The trajectory interface here exposes p/v/a/psi only, so the
    %   same feed-forward quantities are obtained by finite-differencing the
    %   flatness attitude map. This is the MATLAB-framework adaptation.

    ff = faesslerFlatnessReference(traj, t, par);

    % Paper Eq. (31)-(32), rewritten for NED. The reference drag acceleration
    % is a_rd = -R_ref*D*R_ref'*v_ref, so the body-z-down force axis is
    % g*e3 - (a_ref + a_fb) + a_rd.
    aFb = par.faessler.Kp*(ref.p - x.p) + par.faessler.Kv*(ref.v - x.v);
    aDragRef = -ff.R * par.faessler.D * ff.R' * ref.v;
    forceAxis = par.g*par.e3 - (ref.a + aFb) + aDragRef;

    [Rd, ~] = desiredAttitudeFromThrustVector(par.m*forceAxis, ref.psi, par);

    % Paper Eq. (36), with c projected onto the actual body z axis. The kh
    % term is kept for the paper thrust model but defaults to zero because
    % this benchmark applies force directly.
    cCmd = dot(forceAxis, x.R*par.e3);
    vh = dot(x.v, x.R(:,1) + x.R(:,2));
    cCmd = cCmd - par.faessler.kh*vh^2;

    % Paper Eq. (37)-(38): omega_des = omega_fb + omega_ref and
    % omegadot_des = omegadot_ref. Since we output moments, the body-rate
    % feedback is written as an angular-acceleration command.
    refToDesired = Rd' * ff.R;
    omegaRef = refToDesired * ff.Omega;
    alphaRef = refToDesired * ff.alpha;

    eR = LogSO3(x.R' * Rd);
    alphaCmd = alphaRef ...
        + (par.J \ par.faessler.KR)*eR ...
        + (par.J \ par.faessler.KOmega)*(omegaRef - x.Omega);

    tau = par.J*alphaCmd + cross(x.Omega, par.J*x.Omega);

    u.T = min(max(par.m*cCmd, 0), par.Tmax);
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = Rd;
end

function ff = faesslerFlatnessReference(traj, t, par)

    ref = traj.eval(t);
    [R, c] = faesslerFlatnessAttitude(ref, par);
    [Omega, alpha] = faesslerFlatnessRates(traj, t, par);

    ff.R = R;
    ff.c = c;
    ff.Omega = Omega;
    ff.alpha = alpha;
end

function [R, c] = faesslerFlatnessAttitude(ref, par)

    % Faessler Eq. (7)-(14), rewritten for the current NED model:
    % a = g*e3 - c*b3 - R*D*R'*v.
    % Therefore b1 is orthogonal to alpha = g*e3 - a - dx*v, b2 is
    % orthogonal to beta = g*e3 - a - dy*v, and
    % c = b3'*(g*e3 - a - dz*v).
    D = par.faessler.D;
    dx = D(1,1);
    dy = D(2,2);
    dz = D(3,3);

    yC = [-sin(ref.psi); cos(ref.psi); 0];

    alpha = par.g*par.e3 - ref.a - dx*ref.v;
    beta = par.g*par.e3 - ref.a - dy*ref.v;

    b1Raw = cross(yC, alpha);
    if norm(b1Raw) < 1e-9
        [R, c] = desiredAttitudeFromAccel(ref.a, ref.psi, par);
        c = c/par.m;
        return;
    end
    b1 = b1Raw/norm(b1Raw);

    b2Raw = cross(beta, b1);
    if norm(b2Raw) < 1e-9
        [R, c] = desiredAttitudeFromAccel(ref.a, ref.psi, par);
        c = c/par.m;
        return;
    end
    b2 = b2Raw/norm(b2Raw);
    b3 = cross(b1, b2);
    b3 = b3/norm(b3);

    R = projectSO3([b1, b2, b3]);
    c = dot(par.g*par.e3 - ref.a - dz*ref.v, R*par.e3);
end

function [Omega, alpha] = faesslerFlatnessRates(traj, t, par)

    % Paper Eq. (17)-(30) are the analytic flatness derivatives using jerk,
    % snap, psi_dot, and psi_ddot. The current trajectory interface only
    % exposes p/v/a/psi, so use the same SO(3) finite-difference convention
    % as the other paper controllers in this benchmark.
    h = par.dt;
    tPrev = max(t - h, 0);
    tNext = min(t + h, par.Tend);

    RNow = faesslerReferenceAttitudeOnly(traj.eval(t), par);

    if tNext > t
        RNext = faesslerReferenceAttitudeOnly(traj.eval(tNext), par);
        Omega = LogSO3(RNow' * RNext)/(tNext - t);
    else
        Omega = zeros(3,1);
    end

    if t > tPrev && tNext > t
        RPrev = faesslerReferenceAttitudeOnly(traj.eval(tPrev), par);
        OmegaPrev = LogSO3(RPrev' * RNow)/(t - tPrev);
        OmegaPrevAtNow = RNow' * RPrev * OmegaPrev;
        alpha = (Omega - OmegaPrevAtNow)/(0.5*(tNext - tPrev));
    else
        alpha = zeros(3,1);
    end
end

function R = faesslerReferenceAttitudeOnly(ref, par)

    [R, ~] = faesslerFlatnessAttitude(ref, par);
end

function u = controllerTalKaraman(x, ref, traj, t, par)

    persistent st

    % Tal and Karaman 2021, adapted to this MATLAB benchmark:
    % - Paper model uses NED and v_dot = g*i_z + tau*b_z + f_ext/m, where
    %   tau is the signed specific thrust. This benchmark instead outputs a
    %   positive force T and uses v_dot = g*e3 - T/m*b_z, so tau = -T/m.
    % - Paper Eq. (17), (20), (28), and (31) are kept. IMU acceleration and
    %   angular-acceleration measurements are represented by finite
    %   differences of the simulated state, then LPF-filtered as in Fig. 4.
    %   Eq. (20)'s filtered specific-thrust vector is represented by the
    %   previous saturated force command st.T, converted from N to m/s^2 and
    %   passed through the same LPF. Eq. (31)'s mu_f is handled similarly for
    %   the direct equivalent moment.
    % - Paper Eq. (22)-(26) builds an incremental quaternion attitude command.
    %   This framework stores absolute attitude commands, so we compute the
    %   equivalent Rd from the INDI thrust vector and yaw; x.R'*Rd is the
    %   incremental attitude used in Eq. (28).
    % - Paper Eq. (33)-(36) motor speed inversion is intentionally omitted:
    %   this benchmark has no control allocation layer and directly accepts
    %   force/moment commands. Eq. (31)'s filtered moment mu_f is therefore
    %   represented by the previous saturated equivalent moment st.tau.

    ff = talFlatnessReference(traj, t, par);

    if isempty(st) || t <= par.dt/2 || t <= st.t
        aFilt = ref.a;
        omegaDotF = zeros(3,1);
        thrustAccelF = -ff.c * ff.R*par.e3;
        tauF = zeros(3,1);
    else
        h = max(t - st.t, par.dt);
        rawAFilt = (x.v - st.v)/h;
        rawOmegaDotF = (x.Omega - st.Omega)/h;
        % Previous applied force T [N] -> paper's (tau*b_z)_f [m/s^2].
        rawThrustAccelF = -st.T/par.m * st.R*par.e3;
        rawTauF = st.tau;

        aFilt = firstOrderLPF(rawAFilt, st.aFilt, h, par.tal.filterTau);
        omegaDotF = firstOrderLPF(rawOmegaDotF, st.omegaDotF, h, par.tal.filterTau);
        thrustAccelF = firstOrderLPF(rawThrustAccelF, st.thrustAccelF, h, par.tal.filterTau);
        tauF = firstOrderLPF(rawTauF, st.tauF, h, par.tal.filterTau);
    end

    % Eq. (17): commanded acceleration with acceleration feedback. The paper
    % uses LPF IMU acceleration a_f; here aFilt is the finite-difference
    % simulated acceleration.
    aCmd = par.tal.Kp*(ref.p - x.p) ...
         + par.tal.Kv*(ref.v - x.v) ...
         + par.tal.Ka*(ref.a - aFilt) ...
         + ref.a;

    % Eq. (20): INDI linear acceleration control. thrustAccelCmd is the
    % paper's vector (tau*b_z)_c. In this NED framework tau = -T/m, so the
    % thrust acceleration vector is -T/m*b_z.
    thrustAccelCmd = thrustAccelF + aCmd - aFilt;
    [Rd, xiE, T] = talIncrementalAttitudeCommand(x.R, thrustAccelCmd, ref.psi, par);

    % Eq. (14)-(15): the paper solves a 4x4 flatness matrix for
    % [Omega_ref; tau_dot_ref] and [OmegaDot_ref; tau_ddot_ref] from jerk,
    % snap, yaw rate, and yaw acceleration. That matrix uses the yaw
    % parametrization in Eq. (11)-(13) and becomes ill-conditioned when the
    % projected b_x direction is near zero during flips. In that case the raw
    % paper formula can produce nonphysical spikes, e.g. O(1e4) rad/s^2 in
    % yaw acceleration. When the condition/magnitude guard below trips, keep
    % the same reference attitude but compute Omega_ref/OmegaDot_ref by SO(3)
    % finite differences; this is a framework adaptation, not the printed
    % Eq. (14)-(15).
    refDer = talReferenceDerivatives(traj, t, par);
    [omegaRef, alphaRef] = talReferenceFeedforward(traj, t, ff, refDer, par);

    % Eq. (28): attitude/rate controller. xiE is Eq. (27), computed from the
    % incremental quaternion command of Eq. (22)-(26).
    omegaDotCmd = par.tal.Ktheta*xiE ...
                + par.tal.Komega*(omegaRef - x.Omega) ...
                + alphaRef;

    % Eq. (31): INDI angular acceleration control. The paper uses filtered
    % motor-speed-derived moment mu_f; with direct moment actuation, tauF is
    % the LPF output of previous saturated equivalent moments [N*m].
    if isempty(st) || t <= par.dt/2 || t <= st.t
        tau = par.J*omegaDotCmd + cross(x.Omega, par.J*x.Omega);
    else
        tau = tauF + par.J*(omegaDotCmd - omegaDotF);
    end

    % Saturate first; the plant then applies these force/moment values
    % directly, and the next INDI update reuses the saturated applied values.
    u.T = min(max(T, 0), par.Tmax);
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = Rd;

    st.v = x.v;
    st.R = x.R;
    st.Omega = x.Omega;
    st.T = u.T;
    st.tau = u.tau;
    st.aFilt = aFilt;
    st.omegaDotF = omegaDotF;
    st.thrustAccelF = thrustAccelF;
    st.tauF = tauF;
    st.Rd = Rd;
    st.t = t;
end

function [Rd, xiE, T] = talIncrementalAttitudeCommand(R, thrustAccelCmd, psiRef, par)

    % Tal Eq. (21)-(27). thrustAccelCmd is (tau*b_z)_c in inertial NED.
    % Since tau is negative for upward thrust, the commanded body z axis is
    % b_z,c = -normalize((tau*b_z)_c).
    if norm(thrustAccelCmd) < 1e-9
        thrustAccelCmd = -par.g*par.e3;
    end

    thrustDir = thrustAccelCmd/norm(thrustAccelCmd);
    thrustDirBody = R' * thrustDir;       % Eq. (22): (-b_z)_c in body frame.

    qTilt = talQuatAlignMinusE3(thrustDirBody);
    RTilt = quatToRotmWXYZ(qTilt);
    RIntermediate = R * RTilt;

    nPsi = [sin(psiRef); -cos(psiRef); 0];
    nBody = RIntermediate' * nPsi;        % Eq. (24).

    if abs(nBody(2)) < 1e-9
        qYaw = [1; 0; 0; 0];
    else
        k = -nBody(1)/nBody(2);
        qYaw = [1; 0; 0; k/(1 + sqrt(1 + k^2))];
        qYaw = normalizeQuatWXYZ(qYaw);
    end

    qCmd = quatMultiplyWXYZ(qTilt, qYaw); % Eq. (26): current to command.
    qCmd = normalizeQuatWXYZ(qCmd);
    RCmd = quatToRotmWXYZ(qCmd);

    Rd = projectSO3(R * RCmd);
    xiE = talQuatErrorVector(qCmd);       % Eq. (27).
    % Paper Eq. (21) is T_c = -m*||(tau*b_z)_c|| because the motor/allocation
    % layer is assumed to realize the commanded attitude/thrust vector. In
    % this benchmark the plant immediately applies a scalar force along the
    % current body z-axis R*e3. Directly using the paper norm here can apply
    % full thrust along the wrong current axis while the attitude loop is
    % catching up, which is a direct-actuation artifact rather than a reference
    % trajectory error. Therefore the framework conversion projects the
    % commanded specific-thrust vector onto the current thrust axis:
    %   -T/m * R*e3 ~= (tau*b_z)_c  =>  T = -m*((tau*b_z)_c)'*R*e3.
    T = -par.m*dot(thrustAccelCmd, R*par.e3);
end

function q = talQuatAlignMinusE3(vBody)

    vBody = vBody/norm(vBody);
    e3 = [0; 0; 1];
    c = dot(e3, vBody);
    axis = -cross(e3, vBody);

    % Tal Eq. (23) aligns current -b_z with (tau*b_z)_c. It is singular when
    % i_z = (-b_z)_c^b, i.e. a 180 deg tilt with arbitrary rotation axis.
    % The paper explicitly resolves this by selecting any direction of
    % rotation. Use a fixed body-x axis near that singularity instead of
    % normalizing a near-zero cross product, which otherwise jitters just as a
    % flip starts. If vBody ~= -i_z, the required tilt is zero.
    if 1 - c < 1e-8
        q = [0; 1; 0; 0];
    elseif 1 + c < 1e-8
        q = [1; 0; 0; 0];
    else
        q = [1 - c; axis];
        q = normalizeQuatWXYZ(q);
    end
end

function xiE = talQuatErrorVector(q)

    % Tal Eq. (27) is the quaternion logarithm written as
    %   xi_e = 2*acos(q_w)/sqrt(1-q_w^2) * q_v.
    % Directly evaluating that form has two numerical problems:
    %   1) q_w -> 1 gives a 0/0 expression;
    %   2) d acos(q_w)/dq_w blows up as q_w -> +/-1, making it sensitive to
    %      normalization/roundoff during small corrections before flips.
    % Sola 2017 Eq. (105a)-(105b) writes the same Log map with atan2:
    %   Log(q) = 2*atan2(||q_v||, q_w) * q_v/||q_v||.
    % Use that equivalent form plus the small-angle series, which is the
    % normalized, numerically stable version of Tal Eq. (27).
    xiE = quatLogVectorWXYZ(q);
end

function [omegaRef, alphaRef] = talPaperFlatnessFeedforward(R, tauSpec, refDer, par)

    % Tal Eq. (14)-(15). The 4x4 matrix solves for [tau_dot; Omega_ref] and
    % [tau_ddot; Omega_dot_ref] from the reference jerk, snap, yaw rate, and
    % yaw acceleration. This is the paper's differential-flatness feed-forward
    % path, evaluated on the reference attitude and specific thrust.
    A = talFlatnessMatrix(R, tauSpec);

    yJerk = [refDer.j; refDer.psiDot];
    solJerk = talSolveFlatnessSystem(A, yJerk);
    tauDotRef = solJerk(1);
    omegaRef = solJerk(2:4);

    omegaHat = hat(omegaRef);
    knownSnap = R*(2*tauDotRef*omegaHat + tauSpec*omegaHat*omegaHat)*par.e3;
    sDotOmega = talYawSdotOmega(R, omegaRef);
    ySnap = [refDer.s - knownSnap;
             refDer.psiDDot - sDotOmega];

    solSnap = talSolveFlatnessSystem(A, ySnap);
    alphaRef = solSnap(2:4);

    if ~isfinite(tauDotRef)
        omegaRef = zeros(3,1);
    end
end

function [omegaRef, alphaRef] = talReferenceFeedforward(traj, t, ff, refDer, par)

    [omegaRef, alphaRef] = talPaperFlatnessFeedforward(ff.R, -ff.c, refDer, par);

    A = talFlatnessMatrix(ff.R, -ff.c);
    badPaperFeedforward = rcond(A) < par.tal.flatnessRcondMin ...
        || any(~isfinite(omegaRef)) ...
        || any(~isfinite(alphaRef)) ...
        || norm(omegaRef) > par.tal.omegaRefMax ...
        || norm(alphaRef) > par.tal.alphaRefMax;

    if badPaperFeedforward
        % Framework adaptation for Eq. (14)-(15):
        % The printed equations are kept when well-conditioned. Near the yaw
        % flatness singularity, use the same desired-attitude construction but
        % differentiate directly on SO(3). This avoids injecting artificial
        % yaw-acceleration spikes from the 4x4 flatness solve.
        [omegaRef, alphaRef] = talSO3ReferenceRates(traj, t, par);
    end
end

function [Omega, alpha] = talSO3ReferenceRates(traj, t, par)

    h = par.dt;
    tPrev = max(t - h, 0);
    tNext = min(t + h, par.Tend);

    RNow = talReferenceAttitudeOnly(traj.eval(t), par);

    if tNext > t
        RNext = talReferenceAttitudeOnly(traj.eval(tNext), par);
        Omega = LogSO3(RNow' * RNext)/(tNext - t);
    else
        Omega = zeros(3,1);
    end

    if t > tPrev && tNext > t
        RPrev = talReferenceAttitudeOnly(traj.eval(tPrev), par);
        OmegaPrev = LogSO3(RPrev' * RNow)/(t - tPrev);
        OmegaPrevAtNow = RNow' * RPrev * OmegaPrev;
        alpha = (Omega - OmegaPrevAtNow)/(0.5*(tNext - tPrev));
    else
        alpha = zeros(3,1);
    end
end

function R = talReferenceAttitudeOnly(ref, par)

    [R, ~] = desiredAttitudeFromAccel(ref.a, ref.psi, par);
end

function A = talFlatnessMatrix(R, tauSpec)

    b3 = R*e3Local();
    S = talYawSRow(R);
    A = [b3, -tauSpec*R*hat(e3Local());
         0,  S];
end

function x = talSolveFlatnessSystem(A, y)

    if rcond(A) < 1e-8
        x = pinv(A)*y;
    else
        x = A\y;
    end
end

function S = talYawSRow(R)

    bx = R(:,1);
    den = bx(1)^2 + bx(2)^2;

    if den < 1e-9
        S = [0, 0, 1];
        return;
    end

    S = zeros(1,3);
    for i = 1:3
        e = zeros(3,1);
        e(i) = 1;
        bxDot = R*hat(e)*e1Local();
        S(i) = (bx(1)*bxDot(2) - bx(2)*bxDot(1))/den;
    end
end

function y = talYawSdotOmega(R, omega)

    epsT = 1e-5;
    Rp = R*expm(epsT*hat(omega));
    Rm = R*expm(-epsT*hat(omega));
    Sdot = (talYawSRow(Rp) - talYawSRow(Rm))/(2*epsT);
    y = Sdot*omega;
end

function refDer = talReferenceDerivatives(traj, t, par)

    h = par.dt;
    t0 = clampScalar(t, 0, par.Tend);
    tm = clampScalar(t0 - h, 0, par.Tend);
    tp = clampScalar(t0 + h, 0, par.Tend);

    ref0 = traj.eval(t0);
    refM = traj.eval(tm);
    refP = traj.eval(tp);

    if tp > t0 && tm < t0
        hp = tp - t0;
        hm = t0 - tm;
        refDer.j = (hm^2*(refP.a - ref0.a) + hp^2*(ref0.a - refM.a))/(hp*hm*(hp + hm));
        refDer.s = 2*(hm*refP.a - (hp + hm)*ref0.a + hp*refM.a)/(hp*hm*(hp + hm));
        refDer.psiDot = (hm^2*angleDiff(refP.psi, ref0.psi) ...
            + hp^2*angleDiff(ref0.psi, refM.psi))/(hp*hm*(hp + hm));
        refDer.psiDDot = 2*(hm*angleDiff(refP.psi, ref0.psi) ...
            - hp*angleDiff(ref0.psi, refM.psi))/(hp*hm*(hp + hm));
    elseif tp > t0
        hp = tp - t0;
        refDer.j = (refP.a - ref0.a)/hp;
        refDer.s = zeros(3,1);
        refDer.psiDot = angleDiff(refP.psi, ref0.psi)/hp;
        refDer.psiDDot = 0;
    elseif tm < t0
        hm = t0 - tm;
        refDer.j = (ref0.a - refM.a)/hm;
        refDer.s = zeros(3,1);
        refDer.psiDot = angleDiff(ref0.psi, refM.psi)/hm;
        refDer.psiDDot = 0;
    else
        refDer.j = zeros(3,1);
        refDer.s = zeros(3,1);
        refDer.psiDot = 0;
        refDer.psiDDot = 0;
    end
end

function d = angleDiff(a, b)

    d = atan2(sin(a - b), cos(a - b));
end

function e = e1Local()

    e = [1; 0; 0];
end

function e = e3Local()

    e = [0; 0; 1];
end

function ff = talFlatnessReference(traj, t, par)

    ref = traj.eval(t);
    [R, T] = desiredAttitudeFromAccel(ref.a, ref.psi, par);

    ff.R = R;
    ff.c = T/par.m;
end

function u = controllerSunNMPC(x, ref, traj, t, par)

    % Version map for the fast Sun NMPC variant:
    % - Paper: Eq. (10) is a nonlinear finite-horizon OCP over
    %   x = [xi; xidot; q; Omega] and rotor thrusts.
    % - Agilicious C++: implements that OCP with acados, state order
    %   [p; q; v; omega], rotor thrust inputs, and a generated tilt-yaw
    %   quaternion residual in the cost.
    % - This MATLAB function: paper-first reduced adaptation for speed. It
    %   keeps the paper's tracking objective/reference idea, uses the C++
    %   residual/weights where they clarify implementation details, and
    %   replaces the nonlinear rotor-thrust OCP with a local finite-horizon
    %   LQR over virtual inputs [collective acceleration; body rate]. The
    %   output is converted to this framework's [T; tau].
    cmd = sunNMPCCommand(x, ref, traj, t, par);
    u = sunDirectMomentControl(x, cmd, par);
end

function u = controllerSunNMPCFull(x, ref, traj, t, par)

    persistent st

    % Version map for the full Sun NMPC replica:
    % - Paper target: Eq. (10)-(12), nonlinear horizon, reference
    %   x_r/u_r, body-rate and thrust constraints, rotor thrust input.
    % - Agilicious C++ reference: acados-generated solver, state order
    %   [p; q; v; omega], q_ref as an online parameter, tilt-yaw attitude
    %   residual, and default Table-I-like weights.
    % - This MATLAB adaptation: single-shooting fmincon over this benchmark's
    %   direct force/moment input [T; tau]. Rotor allocation is intentionally
    %   skipped; the paper's u_r is represented by the equivalent
    %   [T_ref; tau_ref]. If the solve is unavailable or worse than the fast
    %   paper-adapted controller above, the function reports and uses fallback.

    fallback = controllerSunNMPC(x, ref, traj, t, par);
    fallbackRaw = [fallback.T; fallback.tau];
    fallback.sunFullUsedFallback = true;
    fallback.sunFullSolved = false;
    fallback.sunFullFallbackCode = 0;
    fallback.sunFullExitflag = nan;
    fallback.sunFullCostRatio = nan;
    % Codes: 0 optimizer command, 1 missing fmincon, 2 scheduled skip,
    % 3 invalid solve or bad exitflag, 4 optimizer cost worse than fallback.

    if exist('fmincon', 'file') ~= 2
        warning('fmincon is unavailable; falling back to sun_nmpc.');
        fallback.sunFullFallbackCode = 1;
        u = fallback;
        return;
    end

    cfg = par.sunFull;
    N = cfg.N;
    h = cfg.dt;
    refs = sunFullBuildReferences(traj, t, N, h, par);
    x0 = sunFullStateVector(x);

    solveDue = isempty(st) || ~isfield(st, 'nextSolveTime') ...
        || t <= par.dt/2 || t <= st.t || t >= st.nextSolveTime;
    if ~solveDue
        fallback.sunFullFallbackCode = 2;
        u = fallback;
        return;
    end

    z0 = sunFullInputInitialGuess(refs, st, fallbackRaw, par);
    [lb, ub] = sunFullInputBounds(N, par);
    opts = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...
        'Display', 'off', ...
        'MaxIterations', cfg.maxIterations, ...
        'MaxFunctionEvaluations', cfg.maxFunctionEvaluations, ...
        'StepTolerance', 1e-6, ...
        'OptimalityTolerance', 1e-4);

    % Paper Eq. (10), MATLAB-adapted:
    % min sum ||x_k - x_r,k||_Q + ||u_k - u_r,k||_R over a nonlinear rollout.
    % The solver is fmincon rather than ACADO/acados. The state order follows
    % the C++ generated model, [p; q; v; omega], which is only a permutation
    % of the paper's [xi; xidot; q; Omega]. The input is [T; tau] because this
    % benchmark outputs force/moment instead of rotor thrusts.
    objective = @(z) sunFullSingleShootingObjective(z, x0, refs, cfg, par);
    fallbackZ = repmat(fallbackRaw, N, 1);
    fallbackCost = objective(fallbackZ);
    costRatio = nan;

    try
        [zOpt, fval, exitflag] = fmincon(objective, z0, [], [], [], [], lb, ub, [], opts);
    catch
        zOpt = z0;
        fval = inf;
        exitflag = -1;
    end

    if isfinite(fval) && isfinite(fallbackCost) && abs(fallbackCost) > eps
        costRatio = fval / fallbackCost;
    end

    badSolve = any(~isfinite(zOpt)) || ~isfinite(fval) || exitflag < 0;
    worseThanFallback = ~badSolve && fval > fallbackCost;
    if badSolve || worseThanFallback
        if worseThanFallback
            fallback.sunFullFallbackCode = 4;
        else
            fallback.sunFullFallbackCode = 3;
        end
        fallback.sunFullExitflag = exitflag;
        fallback.sunFullCostRatio = costRatio;
        u = fallback;
        st.nextSolveTime = t + cfg.solvePeriod;
        st.t = t;
        exitflag = -1;
        st.lastExitflag = exitflag;
        return;
    end

    UOpt = reshape(zOpt, 4, N);
    cmdRaw = UOpt(:,1);
    u.T = min(max(cmdRaw(1), 0), par.Tmax);
    u.tau = saturateVector(cmdRaw(2:4), par.tauMax);
    u.Rd = refs.R(:,:,1);
    u.sunFullUsedFallback = false;
    u.sunFullSolved = true;
    u.sunFullFallbackCode = 0;
    u.sunFullExitflag = exitflag;
    u.sunFullCostRatio = costRatio;

    st.U = UOpt;
    st.nextSolveTime = t + cfg.solvePeriod;
    st.t = t;
    st.lastExitflag = exitflag;
end

function z0 = sunFullInputInitialGuess(refs, st, fallbackRaw, par)

    N = size(refs.p, 2) - 1;
    U0 = repmat(fallbackRaw, 1, N);
    U0 = 0.5*U0 + 0.5*refs.u(:,1:N);

    hasWarmStart = ~isempty(st) && isfield(st, 'U') && size(st.U, 2) == N;
    if hasWarmStart
        U0(:,1:N-1) = st.U(:,2:N);
        U0(:,N) = st.U(:,N);
    end

    U0(1,:) = min(max(U0(1,:), 0), par.Tmax);
    U0(2:4,:) = min(max(U0(2:4,:), -par.tauMax), par.tauMax);
    z0 = U0(:);
end

function [lb, ub] = sunFullInputBounds(N, par)

    lb = repmat([0; -par.tauMax], N, 1);
    ub = repmat([par.Tmax; par.tauMax], N, 1);
end

function Jcost = sunFullSingleShootingObjective(z, x0, refs, cfg, par)

    U = reshape(z, 4, cfg.N);
    y = x0;
    Jcost = 0;
    tauWeight = cfg.RtauNorm;

    for k = 1:cfg.N
        if any(~isfinite(y)) || any(~isfinite(U(:,k)))
            Jcost = 1e12;
            return;
        end

        Jcost = Jcost + sunFullStateCost(y, refs, k, cfg);

        % Sun Eq. (10): ||u_k - u_r,k||_R. The paper/C++ use rotor thrusts;
        % here u_r is the force/moment equivalent [T_ref; tau_ref].
        uErr = U(:,k) - refs.u(:,k);
        Jcost = Jcost + uErr' * cfg.R * uErr;

        tauNorm = U(2:4,k)./max(par.tauMax, 1e-6);
        Jcost = Jcost + tauWeight*(tauNorm' * tauNorm);

        y = sunFullStepState(y, U(:,k), cfg.dt, par);
    end

    if any(~isfinite(y))
        Jcost = 1e12;
        return;
    end

    Jcost = Jcost + sunFullStateCost(y, refs, cfg.N+1, cfg);
end

function Jk = sunFullStateCost(y, refs, k, cfg)

    ep = y(1:3) - refs.p(:,k);
    eR = sunAgiliciousAttitudeResidual(y(4:7), refs.q(:,k));
    ev = y(8:10) - refs.v(:,k);
    eOmega = y(11:13) - refs.Omega(:,k);

    Jk = ep' * cfg.Qpos * ep ...
       + eR' * cfg.Qatt * eR ...
       + ev' * cfg.Qvel * ev ...
       + eOmega' * cfg.Qomega * eOmega;
end

function eAtt = sunAgiliciousAttitudeResidual(q, qRef)

    % Agilicious generated acados cost_y_fun: q_ref is an online parameter,
    % yref attitude is zero, and the attitude residual is the tilt-yaw split
    % vector from q_e = q^{-1} * q_ref with the generated 1e-3 regularization.
    q = normalizeQuatWXYZ(q);
    qRef = normalizeQuatWXYZ(qRef);
    qe = quatMultiplyWXYZ(quatConjugateWXYZ(q), qRef);
    qe = normalizeQuatWXYZ(qe);

    den = sqrt(qe(1)^2 + qe(4)^2 + 1e-3);
    eAtt = [qe(1)*qe(2) - qe(3)*qe(4);
            qe(1)*qe(3) + qe(2)*qe(4);
            qe(4)]/den;
end

function yNext = sunFullStepState(y, u, h, par)

    k1 = sunFullStateDerivative(y, u, par);
    k2 = sunFullStateDerivative(y + 0.5*h*k1, u, par);
    k3 = sunFullStateDerivative(y + 0.5*h*k2, u, par);
    k4 = sunFullStateDerivative(y + h*k3, u, par);

    yNext = y + h/6*(k1 + 2*k2 + 2*k3 + k4);
    yNext(4:7) = normalizeQuatWXYZ(yNext(4:7));
end

function yDot = sunFullStateDerivative(y, u, par)

    q = normalizeQuatWXYZ(y(4:7));
    Omega = y(11:13);
    R = quatToRotmWXYZ(q);
    T = u(1);
    tau = u(2:4);

    % Foehn Eq. (15) and Sun Eq. (1)-(3), converted to this NED model:
    % p_dot = v, v_dot = g*e3 - T/m*R*e3, q_dot = 1/2 q*[0;Omega],
    % J*Omega_dot = tau - Omega x J*Omega.
    qDot = 0.5*quatMultiplyWXYZ(q, [0; Omega]);
    vDot = par.g*par.e3 - T/par.m*R*par.e3;
    OmegaDot = par.J \ (tau - cross(Omega, par.J*Omega));

    yDot = [y(8:10); qDot; vDot; OmegaDot];
end

function refs = sunFullBuildReferences(traj, t, N, h, par)

    % Reference handling, paper vs C++ vs this MATLAB adaptation:
    % - Paper Eq. (10): supplies x_r = [xi_r; xidot_r; q_r; Omega_r] and
    %   u_r as rotor thrusts from the planner/flatness map.
    % - Agilicious C++: stores p/v/omega/u in yref, while q_ref is passed as
    %   an online parameter to the generated cost.
    % - This MATLAB code: stores q_ref directly in refs.q and evaluates the
    %   same cost explicitly. Since allocation is skipped, u_r is represented
    %   by the force/moment equivalent [T_ref; tau_ref].
    refs.p = zeros(3, N + 1);
    refs.v = zeros(3, N + 1);
    refs.R = zeros(3, 3, N + 1);
    refs.q = zeros(4, N + 1);
    refs.Omega = zeros(3, N + 1);
    refs.alpha = zeros(3, N + 1);
    refs.tau = zeros(3, N + 1);
    refs.T = zeros(1, N + 1);
    refs.u = zeros(4, N + 1);

    for k = 1:N+1
        tk = min(t + (k-1)*h, par.Tend);
        ref = traj.eval(tk);
        [Rk, Tk] = desiredAttitudeFromAccel(ref.a, ref.psi, par);

        refs.p(:,k) = ref.p;
        refs.v(:,k) = ref.v;
        refs.R(:,:,k) = Rk;
        refs.q(:,k) = rotmToQuatWXYZ(Rk);
        refs.T(k) = min(max(Tk, 0), par.Tmax);
    end

    for k = 1:N
        refs.Omega(:,k) = LogSO3(refs.R(:,:,k)' * refs.R(:,:,k+1))/h;
    end
    refs.Omega(:,N+1) = refs.Omega(:,N);

    for k = 1:N-1
        omegaNextAtK = refs.R(:,:,k)' * refs.R(:,:,k+1) * refs.Omega(:,k+1);
        refs.alpha(:,k) = (omegaNextAtK - refs.Omega(:,k))/h;
    end
    refs.alpha(:,N) = refs.alpha(:,max(N-1, 1));
    refs.alpha(:,N+1) = refs.alpha(:,N);

    for k = 1:N+1
        refs.tau(:,k) = par.J*refs.alpha(:,k) ...
            + cross(refs.Omega(:,k), par.J*refs.Omega(:,k));
        refs.tau(:,k) = saturateVector(refs.tau(:,k), par.tauMax);
        refs.u(:,k) = [refs.T(k); refs.tau(:,k)];
    end
end

function xVec = sunFullStateVector(x)

    xVec = [x.p;
            rotmToQuatWXYZ(x.R);
            x.v;
            x.Omega];
end

function u = controllerSunDFBC(x, ref, traj, t, par)

    cmd = sunDFBCCommand(x, ref, traj, t, par);
    u = sunDirectMomentControl(x, cmd, par);
end

function u = controllerSunNMPCINDI(x, ref, traj, t, par)

    persistent st

    cmd = sunNMPCCommand(x, ref, traj, t, par);
    [u, st] = sunINDIMomentControl(x, cmd, t, par, st);
end

function u = controllerSunDFBCINDI(x, ref, traj, t, par)

    persistent st

    cmd = sunDFBCCommand(x, ref, traj, t, par);
    [u, st] = sunINDIMomentControl(x, cmd, t, par, st);
end

function cmd = sunNMPCCommand(x, ref, traj, t, par)

    % Fast Sun NMPC details:
    % Paper Eq. (10) would optimize the nonlinear model over
    % x = [xi; xidot; q; Omega] and rotor thrusts. That exact problem is the
    % job of controllerSunNMPCFull below. This fast variant keeps the paper's
    % MPC structure but uses a local model-predictive LQR so it can run inside
    % the simple MATLAB simulation loop.
    %
    % Differences from the paper:
    % - input is virtual [a_T; Omega_cmd], not four rotor thrusts;
    % - dynamics are the local error model from this benchmark, not the full
    %   nonlinear rotor model;
    % - output is converted to [T; tau].
    %
    % C++ details intentionally borrowed:
    % - Table-I/C++ state weights in par.sun;
    % - generated tilt-yaw attitude residual, because it is the actual
    %   Agilicious cost implementation behind the paper-level description.
    [Rd, aTd, OmegaD] = referenceInputOnManifold(ref, traj, t, par, par.sun.dt);

    % Paper Eq. (11)-(12) attitude error, using the C++ generated residual:
    % q_e = q^{-1}*q_ref. The local linear model below expects angle-scale
    % Log(Rd'*R), so the C++ residual is negated and doubled to match the
    % first-order convention used by this benchmark's error dynamics.
    eAtt = -2*sunAgiliciousAttitudeResidual( ...
        rotmToQuatWXYZ(x.R), rotmToQuatWXYZ(Rd));
    e = [x.p - ref.p;
         x.v - ref.v;
         eAtt];

    [Ad, Bd] = linearizedQuadrotorErrorModel(Rd, aTd, par, par.sun.dt);
    K = finiteHorizonLQR(Ad, Bd, par.sun.Q, par.sun.Rvirtual, ...
        par.sun.P, par.sun.N);

    du = -K*e;
    aTCmd = aTd + du(1);
    OmegaCmd = OmegaD + du(2:4);

    aTCmd = min(max(aTCmd, 0), par.Tmax/par.m);
    OmegaCmd = saturateVector(OmegaCmd, par.sun.omegaMax);

    cmd.T = par.m*aTCmd;
    cmd.alpha = par.J \ (par.sun.KOmega*(OmegaCmd - x.Omega));
    cmd.Rd = Rd;
end

function cmd = sunDFBCCommand(x, ref, traj, t, par)

    % Sun et al. Eq. (13): desired acceleration from PD position feedback.
    xiErr = ref.p - x.p;
    vErr = ref.v - x.v;
    accD = par.sun.Kp*xiErr + par.sun.Kv*vErr + ref.a;

    % Sun et al. Eq. (14)-(17), converted from the paper's ENU convention
    % where z_B is the thrust direction to this NED model where R*e3 is
    % opposite thrust: a = g*e3 - T/m*R*e3.
    thrustAxisForce = par.m*(par.g*par.e3 - accD);
    [Rd, T] = desiredAttitudeFromThrustVector(thrustAxisForce, ref.psi, par);

    % Sun et al. Eq. (18)-(24): differential-flatness feed-forward body
    % rates and angular acceleration. The paper's printed frame convention
    % is ENU and effectively uses B-to-I R=[x_B,y_B,z_B]. Here the NED
    % flatness attitude already includes the sign change in Eq. (14), so
    % Omega_r and alpha_r are finite-differenced from that B-to-NED attitude.
    [OmegaR, alphaR] = sunFlatnessReferenceRates(traj, t, par);

    % Sun et al. Eq. (25): for B-to-I attitude, use q_e = q^{-1} \otimes q_d
    % so the reduced/yaw tangent vectors below are body-local errors.
    qd = rotmToQuatWXYZ(Rd);
    q = rotmToQuatWXYZ(x.R);
    qe = quatMultiplyWXYZ(quatConjugateWXYZ(q), qd);
    qe = qe/norm(qe);

    % Sun et al. Eq. (26)-(27): split reduced-attitude and yaw errors.
    % Agilicious' GeometricController multiplies this residual by 2 as a gain
    % convention; here the paper form is kept and gains absorb that scaling.
    den = sqrt(qe(1)^2 + qe(4)^2);
    if den < 1e-8
        qRed = [qe(2); qe(3); 0];
        qYaw = zeros(3,1);
    else
        qRed = [qe(1)*qe(2) - qe(3)*qe(4);
                qe(1)*qe(3) + qe(2)*qe(4);
                0]/den;
        qYaw = [0; 0; qe(4)]/den;
    end

    % Sun et al. Eq. (28): tilt-prioritized attitude control. The paper's
    % K_Omega is an angular-acceleration gain, so convert the framework's
    % moment-shaped gain KOmega by J^{-1}.
    Kq = par.J \ par.sun.KR;
    KOmegaAlpha = par.J \ par.sun.KOmega;
    yawSign = 1;
    if qe(1) < 0
        yawSign = -1;
    end

    alphaD = Kq*qRed + Kq(3,3)*yawSign*qYaw ...
           + KOmegaAlpha*(OmegaR - x.Omega) + alphaR;

    cmd.T = T;
    cmd.alpha = alphaD;
    cmd.Rd = Rd;
end

function u = sunDirectMomentControl(x, cmd, par)

    % Rigid-body rotational dynamics in Sun et al. Eq. (3), simplified
    % after skipping rotor allocation: tau = J*alpha_d + Omega x J*Omega.
    tau = par.J*cmd.alpha + cross(x.Omega, par.J*x.Omega);

    u.T = min(max(cmd.T, 0), par.Tmax);
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = cmd.Rd;
end

function [u, st] = sunINDIMomentControl(x, cmd, t, par, st)

    % Sun et al. Eq. (32)-(35): tau_indi = tau_f +
    % J*(alpha_cmd - omega_dot_f). Since control allocation is skipped here,
    % tau_f is represented by the previous saturated moment. Agilicious'
    % IndiController additionally replaces yaw with an NDI value; that is a
    % C++ implementation patch, so it is only noted here, not used.

    if isempty(st) || t <= par.dt/2 || t <= st.t
        omegaDotF = zeros(3,1);
        tauF = zeros(3,1);
    else
        h = max(t - st.t, par.dt);
        omegaDotF = (x.Omega - st.Omega)/h;
        tauF = st.tau;
    end

    tau = tauF + par.J*(cmd.alpha - omegaDotF);

    u.T = min(max(cmd.T, 0), par.Tmax);
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = cmd.Rd;

    st.Omega = x.Omega;
    st.tau = u.tau;
    st.t = t;
end

function [OmegaR, alphaR] = sunFlatnessReferenceRates(traj, t, par)

    h = par.dt;
    tPrev = max(t - h, 0);
    tNext = min(t + h, par.Tend);

    RNow = sunReferenceAttitudeOnly(traj.eval(t), par);

    if tNext > t
        RNext = sunReferenceAttitudeOnly(traj.eval(tNext), par);
        OmegaR = LogSO3(RNow' * RNext)/(tNext - t);
    else
        OmegaR = zeros(3,1);
    end

    if t > tPrev && tNext > t
        RPrev = sunReferenceAttitudeOnly(traj.eval(tPrev), par);
        OmegaPrev = LogSO3(RPrev' * RNow)/(t - tPrev);
        OmegaPrevAtNow = RNow' * RPrev * OmegaPrev;
        alphaR = (OmegaR - OmegaPrevAtNow)/(0.5*(tNext - tPrev));
    else
        alphaR = zeros(3,1);
    end
end

function R = sunReferenceAttitudeOnly(ref, par)

    [R, ~] = desiredAttitudeFromAccel(ref.a, ref.psi, par);
end

function u = controllerOnManifoldMPC(x, ref, traj, t, par)

    [Rd, aTd, OmegaD] = referenceInputOnManifold(ref, traj, t, par);

    e = [x.p - ref.p;
         x.v - ref.v;
         LogSO3(Rd' * x.R)];

    [Ad, Bd] = linearizedQuadrotorErrorModel(Rd, aTd, par);
    K = finiteHorizonLQR(Ad, Bd, par.mpc.Q, par.mpc.R, par.mpc.P, par.mpc.N);

    du = -K*e;
    aTCmd = aTd + du(1);
    OmegaCmd = OmegaD + du(2:4);

    aTCmd = min(max(aTCmd, 0), par.Tmax/par.m);
    OmegaCmd = saturateVector(OmegaCmd, par.mpc.omegaMax);

    tau = cross(x.Omega, par.J*x.Omega) ...
        + par.mpc.KOmega*(OmegaCmd - x.Omega);

    u.T = par.m*aTCmd;
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = Rd;
end

function u = controllerGeometricINDI(x, ref, t, par)

    persistent st

    ep = ref.p - x.p;
    ev = ref.v - x.v;
    aCmd = par.indi.Kp*ep + par.indi.Kv*ev + ref.a;

    if isempty(st) || t <= par.dt/2
        [Rd, T] = desiredAttitudeFromAccel(aCmd, ref.psi, par, x.R);
        rErr = LogSO3(x.R' * Rd);

        u.T = min(max(T, 0), par.Tmax);
        u.tau = saturateVector(par.indi.KR*rErr - par.indi.KOmega*x.Omega, par.tauMax);
        u.Rd = Rd;

        st = updateINDIState(x, u, Rd, zeros(3,1), t);
        return;
    end

    h = max(t - st.t, par.dt);
    vDot0 = (x.v - st.v)/h;
    OmegaDot0 = (x.Omega - st.Omega)/h;

    T_b_z0 = st.T * st.R*par.e3;
    T_b_z = T_b_z0 - par.m*(aCmd - vDot0);
    [Rd, T] = desiredAttitudeFromThrustVector(T_b_z, ref.psi, par);

    OmegaR = LogSO3(st.Rd' * Rd)/h;
    OmegaDotR = (OmegaR - st.OmegaR)/h;

    rErr = LogSO3(x.R' * Rd);
    OmegaDotCmd = par.indi.Ktheta*rErr ...
                + par.indi.Komega*(OmegaR - x.Omega) ...
                + OmegaDotR;

    tau = st.tau + par.J*(OmegaDotCmd - OmegaDot0);

    u.T = min(max(T, 0), par.Tmax);
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = Rd;

    st = updateINDIState(x, u, Rd, OmegaR, t);
end

function st = updateINDIState(x, u, Rd, OmegaR, t)

    st.v = x.v;
    st.R = x.R;
    st.Omega = x.Omega;
    st.T = u.T;
    st.tau = u.tau;
    st.Rd = Rd;
    st.OmegaR = OmegaR;
    st.t = t;
end

function [Rd, aT, OmegaD] = referenceInputOnManifold(ref, traj, t, par, h)

    if nargin < 5
        h = par.dt;
    end

    [Rd, T] = desiredAttitudeFromAccel(ref.a, ref.psi, par, eye(3));
    aT = T/par.m;

    tNext = min(t + h, par.Tend);
    refNext = traj.eval(tNext);
    [RdNext, ~] = desiredAttitudeFromAccel(refNext.a, refNext.psi, par, eye(3));

    if tNext > t
        OmegaD = LogSO3(Rd' * RdNext)/(tNext - t);
    else
        OmegaD = zeros(3,1);
    end
end

function [Ad, Bd] = linearizedQuadrotorErrorModel(Rd, aTd, par, h)

    if nargin < 4
        h = par.dt;
    end

    Ac = zeros(9,9);
    Bc = zeros(9,4);

    Ac(1:3,4:6) = eye(3);
    Ac(4:6,7:9) = aTd*Rd*hat(par.e3);

    Bc(4:6,1) = -Rd*par.e3;
    Bc(7:9,2:4) = eye(3);

    Ad = eye(9) + h*Ac;
    Bd = h*Bc;
end

function K = finiteHorizonLQR(A, B, Q, R, P, N)

    K = zeros(size(B,2), size(A,1));

    for k = N:-1:1
        S = R + B'*P*B;
        K = S\(B'*P*A);
        P = Q + A'*P*A - A'*P*B*K;
    end
end

%% ========================================================================
%% Flatness attitude map layer
function [Rd, T] = desiredAttitudeFromAccel(aCmd, psi, par, RCurrent)

    T_b_z = -par.m*(aCmd - par.g*par.e3); % -f_d. desired force along body z_B

    if norm(T_b_z) < 1e-9
        T_b_z = par.m*par.g*par.e3;
    end

    % T = dot(T_b_z, RCurrent*par.e3); % option 1: current-attitude projection
    T = norm(T_b_z);                 % option 2: desired-force magnitude

    Rd = attitudeFromThrustDirection(T_b_z/norm(T_b_z), psi);
end

function [Rd, T] = desiredAttitudeFromThrustVector(T_b_z, psi, par)

    if norm(T_b_z) < 1e-9
        T_b_z = par.m*par.g*par.e3;
    end

    T = norm(T_b_z);
    Rd = attitudeFromThrustDirection(T_b_z/T, psi);
end

function Rd = attitudeFromThrustDirection(b3d, psi)

    headingAxis = [cos(psi); sin(psi); 0];

    b2raw = cross(b3d, headingAxis);

    if norm(b2raw) < 1e-8
        headingAxis = [0;1;0];
        b2raw = cross(b3d, headingAxis);
    end

    b2d = b2raw/norm(b2raw);
    b1d = cross(b2d, b3d);

    Rd = [b1d, b2d, b3d];
end

%% ========================================================================
%% Quadrotor model layer
function xNext = stepModel(x, u, par)

    switch par.integratorName
        case "ode45"
            xNext = stepModelODE45(x, u, par);
        case "lie_rk4"
            xNext = stepModelLieRK4(x, u, par);
        otherwise
            error("Unknown integratorName.");
    end
end

function xNext = stepModelODE45(x, u, par)

    y0 = [x.p; x.v; reshape(x.R, 9, 1); x.Omega];
    opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9);
    [~, yHist] = ode45(@(t,y) quadrotorOde(t, y, u, par), [0 par.dt], y0, opts);

    y = yHist(end,:)';
    xNext.p = y(1:3);
    xNext.v = y(4:6);
    xNext.R = projectSO3(reshape(y(7:15), 3, 3));
    xNext.Omega = y(16:18);
end

function yDot = quadrotorOde(~, y, u, par)

    v = y(4:6);
    R = reshape(y(7:15), 3, 3);
    Omega = y(16:18);

    [a, OmegaDot] = rigidBodyRates(R, Omega, u, par);

    yDot = [v;
            a;
            reshape(R*hat(Omega), 9, 1);
            OmegaDot];
end

function xNext = stepModelLieRK4(x, u, par)

    h = par.dt;

    Om1 = x.Omega;
    [a1, OmDot1] = rigidBodyRates(x.R, Om1, u, par);

    v2 = x.v + 0.5*h*a1;
    R2 = x.R*expm(0.5*h*hat(Om1));
    Om2 = x.Omega + 0.5*h*OmDot1;
    [a2, OmDot2] = rigidBodyRates(R2, Om2, u, par);

    v3 = x.v + 0.5*h*a2;
    R3 = x.R*expm(0.5*h*hat(Om2));
    Om3 = x.Omega + 0.5*h*OmDot2;
    [a3, OmDot3] = rigidBodyRates(R3, Om3, u, par);

    v4 = x.v + h*a3;
    R4 = x.R*expm(h*hat(Om3));
    Om4 = x.Omega + h*OmDot3;
    [a4, OmDot4] = rigidBodyRates(R4, Om4, u, par);

    OmegaBar = (Om1 + 2*Om2 + 2*Om3 + Om4)/6;

    xNext.p = x.p + h/6*(x.v + 2*v2 + 2*v3 + v4);
    xNext.v = x.v + h/6*(a1 + 2*a2 + 2*a3 + a4);
    xNext.R = x.R*expm(h*hat(OmegaBar));
    xNext.Omega = x.Omega + h/6*(OmDot1 + 2*OmDot2 + 2*OmDot3 + OmDot4);
end

function [a, OmegaDot] = rigidBodyRates(R, Omega, u, par)

    a = par.g*par.e3 - u.T/par.m*R*par.e3;
    OmegaDot = par.J \ (u.tau - cross(Omega, par.J*Omega));
end

function R = projectSO3(R)

    [U, ~, V] = svd(R);
    R = U*diag([1, 1, det(U*V')])*V';
end

%% ========================================================================
%% Plot layer
function plotResults(time, log, par, traj)

    % Bounded Euler-angle display.
    % Each angle is displayed in [-180 deg, 180 deg].
    eul = wrapToPiLocal(log.euler);
    eulD = wrapToPiLocal(log.eulerD);

    figure('Name','3D trajectory with sampled attitude');

    hActual = plot3(log.p(1,:), log.p(2,:), log.p(3,:), ...
        'LineWidth', 1.6); 
    hold on;

    hRef = plot3(log.pd(1,:), log.pd(2,:), log.pd(3,:), ...
        '--', 'LineWidth', 1.6);

    switch par.poseSource
        case "actual"
            poseP = log.p;
            poseR = log.R;
        case "desired"
            poseP = log.pd;
            poseR = log.Rd;
        otherwise
            poseP = log.p;
            poseR = log.R;
    end

    [hx, hy, hz] = drawSampledBodyAxes(time, poseP, poseR, par);

    grid on; axis equal;
    view(35, 25);
    set(gca, 'ZDir', 'reverse');
    xlabel('x_{NED} north (m)');
    ylabel('y_{NED} east (m)');
    zlabel('z_{NED} down (m)');
    title("3D trajectory with sampled body axes: " + traj.name);

    legend([hActual, hRef, hx, hy, hz], ...
        {'actual trajectory','reference trajectory','x_B','y_B','z_B'}, ...
        'Location','best');

    figure('Name','position and attitude response');

    labels = {'x (m)', 'y (m)', 'z_{NED} (m)', ...
              'roll (deg)', 'pitch (deg)', 'yaw (deg)'};

    actual = [log.p; rad2deg(eul)];
    desired = [log.pd; rad2deg(eulD)];

    for i = 1:6
        subplot(6,1,i);
        plot(time, actual(i,:), 'LineWidth', 1.1); hold on;
        plot(time, desired(i,:), '--', 'LineWidth', 1.1);
        grid on;
        ylabel(labels{i});

        if i == 1
            title("Reference tracking: " + traj.name);
        end

        if i == 6
            xlabel('time (s)');
        end
    end

    legend('actual','reference/command');
end

%% ========================================================================
%% Draw sampled body-frame axes on 3D trajectory
function [hx, hy, hz] = drawSampledBodyAxes(time, pLog, RLog, par)

    step = max(1, round(par.poseEvery/par.dt));
    idxList = unique([1:step:numel(time), numel(time)]);

    L = par.bodyAxisScale;

    hx = gobjects(1);
    hy = gobjects(1);
    hz = gobjects(1);

    for s = 1:numel(idxList)

        idx = idxList(s);

        pNED = pLog(:,idx);
        R = RLog(:,:,idx);

        if s == 1
            hx = quiver3(pNED(1), pNED(2), pNED(3), ...
                    L*R(1,1), L*R(2,1), L*R(3,1), ...
                    0, 'r', 'LineWidth', 1.0, 'MaxHeadSize', 0.8);

            hy = quiver3(pNED(1), pNED(2), pNED(3), ...
                    L*R(1,2), L*R(2,2), L*R(3,2), ...
                    0, 'g', 'LineWidth', 1.0, 'MaxHeadSize', 0.8);

            hz = quiver3(pNED(1), pNED(2), pNED(3), ...
                    L*R(1,3), L*R(2,3), L*R(3,3), ...
                    0, 'b', 'LineWidth', 1.0, 'MaxHeadSize', 0.8);
        else
            quiver3(pNED(1), pNED(2), pNED(3), ...
                    L*R(1,1), L*R(2,1), L*R(3,1), ...
                    0, 'r', 'LineWidth', 1.0, 'MaxHeadSize', 0.8, ...
                    'HandleVisibility','off');

            quiver3(pNED(1), pNED(2), pNED(3), ...
                    L*R(1,2), L*R(2,2), L*R(3,2), ...
                    0, 'g', 'LineWidth', 1.0, 'MaxHeadSize', 0.8, ...
                    'HandleVisibility','off');

            quiver3(pNED(1), pNED(2), pNED(3), ...
                    L*R(1,3), L*R(2,3), L*R(3,3), ...
                    0, 'b', 'LineWidth', 1.0, 'MaxHeadSize', 0.8, ...
                    'HandleVisibility','off');
        end
    end
end

%% SO(3) utility functions
function S = hat(w)

    S = [0, -w(3), w(2);
         w(3), 0, -w(1);
        -w(2), w(1), 0];
end

function v = vee(S)

    v = [S(3,2); S(1,3); S(2,1)];
end

function phi = LogSO3(R)

    % Robust SO(3) Log for aggressive flips:
    % The usual trace/acos formula,
    %   theta/(2*sin(theta))*vee(R - R'),
    % is sensitive near theta = pi and has the same acos derivative issue as
    % Tal Eq. (27). so3_log_map and manif both use the more robust path
    % matrix -> unit quaternion -> atan2 quaternion Log. The sign at exactly
    % pi is inherently ambiguous; qw >= 0 selects the principal branch.
    q = rotmToQuatWXYZ(projectSO3(R));
    phi = quatLogVectorWXYZ(q);
end

function Jinv = leftJacobianSO3Inv(phi)

    theta = norm(phi);
    Phi = hat(phi);

    if theta < 1e-6
        Jinv = eye(3) - 0.5*Phi + (1/12)*Phi*Phi;
        return;
    end

    x = 0.5*theta;
    sincX = sin(x)/x;
    A = (1 - cos(x)/sincX)/(theta^2);

    Jinv = eye(3) - 0.5*Phi + A*Phi*Phi;
end

function q = rotmToQuatWXYZ(R)

    tr = trace(R);

    if tr > 0
        s = sqrt(max(tr + 1.0, 0))*2;
        qw = 0.25*s;
        qx = (R(3,2) - R(2,3))/s;
        qy = (R(1,3) - R(3,1))/s;
        qz = (R(2,1) - R(1,2))/s;
    elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
        s = sqrt(max(1.0 + R(1,1) - R(2,2) - R(3,3), 0))*2;
        qw = (R(3,2) - R(2,3))/s;
        qx = 0.25*s;
        qy = (R(1,2) + R(2,1))/s;
        qz = (R(1,3) + R(3,1))/s;
    elseif R(2,2) > R(3,3)
        s = sqrt(max(1.0 + R(2,2) - R(1,1) - R(3,3), 0))*2;
        qw = (R(1,3) - R(3,1))/s;
        qx = (R(1,2) + R(2,1))/s;
        qy = 0.25*s;
        qz = (R(2,3) + R(3,2))/s;
    else
        s = sqrt(max(1.0 + R(3,3) - R(1,1) - R(2,2), 0))*2;
        qw = (R(2,1) - R(1,2))/s;
        qx = (R(1,3) + R(3,1))/s;
        qy = (R(2,3) + R(3,2))/s;
        qz = 0.25*s;
    end

    q = [qw; qx; qy; qz];
    q = normalizeQuatWXYZ(q);
end

function R = quatToRotmWXYZ(q)

    q = normalizeQuatWXYZ(q);
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    R = [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w);
         2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w);
         2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)];
end

function q = normalizeQuatWXYZ(q)

    nq = norm(q);
    if nq < 1e-12
        q = [1; 0; 0; 0];
    else
        q = q/nq;
    end
end

function phi = quatLogVectorWXYZ(q)

    q = normalizeQuatWXYZ(q);
    if q(1) < 0
        q = -q;
    end

    v = q(2:4);
    nv = norm(v);
    qw = min(1, max(-1, q(1)));

    if nv < 1e-8
        nv2 = nv^2;
        scale = 2*(1 + nv2/6 + 3*nv2^2/40);
    else
        scale = 2*atan2(nv, qw)/nv;
    end

    phi = scale*v;
end

function qc = quatConjugateWXYZ(q)

    qc = [q(1); -q(2:4)];
end

function q = quatMultiplyWXYZ(a, b)

    aw = a(1);
    av = a(2:4);
    bw = b(1);
    bv = b(2:4);

    q = [aw*bw - dot(av, bv);
         aw*bv + bw*av + cross(av, bv)];
end

function y = firstOrderLPF(raw, prev, h, tau)

    if tau <= 0
        y = raw;
        return;
    end

    beta = h/(tau + h);
    y = prev + beta*(raw - prev);
end

function y = saturateVector(x, xmax)

    if isscalar(xmax)
        nx = norm(x);

        if nx > xmax
            y = x * xmax/nx;
        else
            y = x;
        end
    else
        y = min(max(x, -xmax), xmax);
    end
end

function eul = rotm2eulZYX(R)

    yaw = atan2(R(2,1), R(1,1));

    s = -R(3,1);
    s = min(1, max(-1, s));
    pitch = asin(s);

    roll = atan2(R(3,2), R(3,3));

    eul = [roll; pitch; yaw];
end
function ang = wrapToPiLocal(ang)
    ang = atan2(sin(ang), cos(ang));
end
