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

if exist('UAV_BENCHMARK_BATCH', 'var') && UAV_BENCHMARK_BATCH
    if exist('UAV_BENCHMARK_PAR_OVERRIDE', 'var')
        parOverride__ = UAV_BENCHMARK_PAR_OVERRIDE;
    else
        parOverride__ = struct();
    end
else
    clear; clc; close all;
    parOverride__ = struct();
end

%% ========================================================================
%% 0. Parameters
par.g = 9.81;
par.e3 = [0;0;1];
% Sun et al. / Agilicious Kingfisher platform, Table II.
par.m = 0.752;
par.J = diag([0.0025, 0.0021, 0.0043]);

par.dt = 0.01;          % 100 Hz
par.Tend = 16.0;
par.integratorName = "ode45";  % "ode45" or "lie_rk4"

% Reference time scaling.
% scale > 1 slows the reference; scale < 1 speeds it up and may saturate control.
par.progress.mode = "scale_range";      % "scale_fixed" or "scale_range"
par.progress.scale = 0.8;               % scale_fixed: constant time scale
par.progress.scaleRange = [2, 0.5];   % scale_range: start/end scale over the simulation

% Available choices:
%   "figure8_horizontal"
%   "figure8_vertical"
%   "helix_flip"
%   "flip_loop_sine"
%   "fast_circle"
par.trajName = "helix_flip";

% One knob for all trajectory shapes. The factory below converts it into
% periods/radii using m, J, Tmax, tauMax, and progress.scaleRange.
par.trajIntensity = 1;  % 0 = gentle, 1 = near the actuator envelope
par.flipTurns = 3;         % flip trajectories: turns during the second half

% controller
% "geometric", "faessler", "lee", "johnson_beard"
% "sun_dfbc", "sun_dfbc_indi"
% "lu_on_manifold_lqr", "sun_nmpc", "sun_nmpc_indi"
% "geometric_indi", "tal_karaman"
par.controllerName = "geometric_indi";
% Simple controller gains
par.Kp = diag([20, 20, 25]);
par.Kv = diag([9, 9, 10]);
% Attitude gains are moment gains. Keep them inertia-scaled so changing the
% platform does not silently change the angular closed-loop dynamics.
par.KR = 600*par.J;
par.KOmega = 50*par.J;

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

% Lu et al. on-manifold finite-horizon LQR approximation.
% State error: [p-pd; v-vd; Log(Rd'R)], input: [aT-aTd; Omega-OmegaD].
par.mpc.N = 16; % Lu et al. use N=8; use longer horizon for the simulated rate loop.
par.mpc.Q = diag([450, 450, 650, ...
                  70, 70, 100, ...
                  140, 140, 80]);
par.mpc.R = diag([1.0, 0.55, 0.55, 0.75]);
par.mpc.P = par.mpc.Q;
par.mpc.omegaMax = deg2rad(800);
par.mpc.KOmega = par.KOmega;

% Sun et al. 2022, Table I and Table II controller/platform parameters.
% The benchmark plant above uses the same Kingfisher mass/inertia, and Sun
% control allocation/NMPC inputs are single-rotor thrusts u = [u1;u2;u3;u4],
% as in Eq. (4)-(12).
par.sun.N = 20;
par.sun.dt = 0.05;
par.sun.Qxi = diag([200, 200, 500]);
par.sun.Qv = eye(3);
par.sun.Qq = diag([5, 5, 200]);
par.sun.QOmega = eye(3);
par.sun.Qu = 6*eye(4);
par.sun.QN = blkdiag(par.sun.Qxi, par.sun.Qv, par.sun.Qq, par.sun.QOmega);
par.sun.Kxi = diag([10, 10, 10]);
par.sun.Kv = diag([6, 6, 6]);
par.sun.KqRed = diag([150, 150, 0]);
par.sun.kqYaw = 3;
par.sun.KOmega = diag([20, 20, 8]);
par.sun.W = diag([0.001, 10, 10, 0.1]);
par.sun.wlsAllocPath = "/Users/mchmini/Proj/control_allocation/control_allocation_lib/qcat/QCAT/qcat";
par.sun.tBM = [ 0.075, -0.075, -0.075,  0.075;
               -0.100,  0.100, -0.100,  0.100;
                0.000,  0.000,  0.000,  0.000];
par.sun.kappa = 0.022;
par.sun.uMin = zeros(4,1);
par.sun.uMax = 8.5*ones(4,1);
par.sun.omegaMax = [10; 10; 4];
acadosSourceDir = string(getenv("ACADOS_SOURCE_DIR"));
if strlength(acadosSourceDir) == 0
    acadosSourceDir = "/private/tmp/acados";
end
par.sun.acadosSourceDir = acadosSourceDir;
par.sun.acadosToolsPath = fullfile(pwd, "tools");
sunPython = string(getenv("SUN_NMPC_PYTHON"));
if strlength(sunPython) == 0 ...
        && exist("/Users/mchmini/.pyenv/versions/3.12.8/bin/python3", "file") == 2
    sunPython = "/Users/mchmini/.pyenv/versions/3.12.8/bin/python3";
end
par.sun.pythonExecutable = sunPython;
par.sun.solvePeriod = par.sun.dt;
par.sun.printSolverTiming = false;

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

% Actuator limits from four Kingfisher rotors, u_i in [0, 8.5] N.
par.Tmax = 4*8.5;
par.tauMax = [1.70; 1.275; 0.374];

% Additive plant disturbances. The force disturbance is expressed in inertial
% NED coordinates [N]; the moment disturbance is expressed in the body frame
% [N*m], matching the plant translational and rotational equations below.
% The default is disabled, so normal single-run behavior is unchanged.
par.disturbance.enabled = false;
par.disturbance.type = "none";       % "none", "constant", or "sin"
par.disturbance.forceAmp = 0;        % scalar or 3x1 per-axis amplitude [N]
par.disturbance.momentAmp = 0;       % scalar or 3x1 per-axis amplitude [N*m]
par.disturbance.forceFreq = [0.31; 0.47; 0.61];   % sinusoid frequencies [Hz]
par.disturbance.momentFreq = [0.43; 0.59; 0.73];  % sinusoid frequencies [Hz]
par.disturbance.forcePhase = [0; 2*pi/3; 4*pi/3];
par.disturbance.momentPhase = [pi/4; 3*pi/4; 5*pi/4];
par.disturbance.startTime = 0;
par.disturbance.endTime = inf;

% Initial condition
par.startOnReference = true;

% 3D attitude sampling visualization
par.poseEvery = 0.10;       % seconds
par.bodyAxisScale = 0.5;   % meters
par.poseSource = "actual";  % "actual" or "desired"

% Post-simulation 3D animation
par.enablePlots = true;
par.enableAnimation = true;
par.animationSpeed = 1;       % 1.0 = real time
par.animationFrameDt = 0.02;    % seconds

if ~isempty(fieldnames(parOverride__))
    par = mergeStructRecursive(par, parOverride__);
end

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
log.forceDist = zeros(3,N);
log.momentDist = zeros(3,N);
log.sunNMPCCached = false(1,N);
log.sunNMPCSolved = false(1,N);
log.sunNMPCStatusCode = zeros(1,N);
log.sunNMPCExitflag = nan(1,N);
log.sunNMPCSolveTime = nan(1,N);

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
    [log.forceDist(:,k), log.momentDist(:,k)] = disturbanceAtTime(t, par);

    if isfield(u, 'sunNMPCSolved')
        log.sunNMPCCached(k) = u.sunNMPCCached;
        log.sunNMPCSolved(k) = u.sunNMPCSolved;
        log.sunNMPCStatusCode(k) = u.sunNMPCStatusCode;
        log.sunNMPCExitflag(k) = u.sunNMPCExitflag;
        log.sunNMPCSolveTime(k) = u.sunNMPCSolveTime;
    end

    x = stepModel(x, u, par, t);
end

if any(par.controllerName == ["sun_nmpc", "sun_nmpc_indi"])
    nCached = nnz(log.sunNMPCCached);
    nSolverFailures = nnz(log.sunNMPCStatusCode ~= 0 & ~log.sunNMPCCached);
    fprintf(['sun_nmpc solve status: %d optimized solves, ' ...
        '%d cached steps, %d nonzero statuses, mean solve %.3f s.\n'], ...
        nnz(log.sunNMPCSolved), nCached, nSolverFailures, ...
        mean(log.sunNMPCSolveTime(log.sunNMPCSolved), 'omitnan'));
end

%% ========================================================================
%% 5. Plot
if par.enablePlots
    plotResults(time, log, par, traj);
end

if par.enableAnimation
    animateTrajectory3D(time, log, par, traj);
end

%% ========================================================================
%% Trajectory factory
function traj = makeTrajectory(par)

    shape = trajectoryShape(par);

    switch par.trajName

        case "figure8_horizontal"
            traj.name = "figure8_horizontal";
            traj.Tend = par.Tend;
            cfg = makeFigure8HorizontalParams(shape);
            traj.eval = @(t) evalFigure8Horizontal(t, cfg);

        case "figure8_vertical"
            traj.name = "figure8_vertical";
            traj.Tend = par.Tend;
            cfg = makeFigure8VerticalParams(shape);
            traj.eval = @(t) evalFigure8Vertical(t, cfg);

        case "helix_flip"
            traj.name = "helix_flip";
            traj.Tend = par.Tend;
            cfg = makeFlipLoopParams(shape, 0.30, 1.30, 0.85);
            traj.eval = @(t) evalFlipLoop(t, cfg);

        case "flip_loop_sine"
            traj.name = "flip_loop_sine";
            traj.Tend = par.Tend;
            cfg = makeFlipLoopParams(shape, 0.00, 1.50, 1.00);
            traj.eval = @(t) evalFlipLoop(t, cfg);

        case "fast_circle"
            traj.name = "fast_circle";
            traj.Tend = par.Tend;
            cfg = makeFastCircleParams(shape);
            traj.eval = @(t) evalFastCircle(t, cfg);

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
            traj.eval = @(t) evalProgressTrajectory( ...
                baseEval, t/scale, 1/scale, 0, baseTend);
            traj.evalPredict = @(t) evalProgressTrajectory( ...
                baseEval, t/scale, 1/scale, 0, baseTend, true);

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
            traj.eval = @(t) evalScaleRangeTrajectory( ...
                baseEval, baseTend, t, traj.Tend, scaleRange);
            traj.evalPredict = @(t) evalScaleRangeTrajectory( ...
                baseEval, baseTend, t, traj.Tend, scaleRange, true);

        otherwise
            error("Unknown progress mode.");
    end
end

function ref = evalScaleRangeTrajectory( ...
        baseEval, baseTend, t, simTend, scaleRange, allowPredict)

    if nargin < 6
        allowPredict = false;
    end

    scale0 = scaleRange(1);
    scale1 = scaleRange(2);
    scaleDot = (scale1 - scale0)/simTend;

    if allowPredict && t > simTend
        tClip = simTend;
        scale = scale1;
    else
        alpha = clampScalar(t/simTend, 0, 1);
        tClip = alpha*simTend;
        scale = scale0 + scaleDot*tClip;
    end

    if scale <= 0
        error("Trajectory time scale became non-positive.");
    end

    % The scale is instantaneous: ds/dt = 1/scale(t).
    if abs(scaleDot) < 1e-12
        s = tClip/scale0;
    else
        s = log(scale/scale0)/scaleDot;
    end

    if allowPredict && t > simTend
        s = s + (t - simTend)/scale1;
        sDot = 1/scale1;
        sDDot = 0;
    else
        sDot = 1/scale;
        sDDot = -scaleDot/scale^2;
    end

    ref = evalProgressTrajectory( ...
        baseEval, s, sDot, sDDot, baseTend, allowPredict);
end

function ref = evalProgressTrajectory( ...
        baseEval, s, sDot, sDDot, baseTend, allowPredict)

    if nargin < 6
        allowPredict = false;
    end

    if allowPredict
        s = max(s, 0);
    else
        s = clampScalar(s, 0, baseTend);
    end

    ref = baseEval(s);

    vBase = ref.v;
    aBase = ref.a;
    ref.v = vBase*sDot;
    ref.a = aBase*sDot^2 + vBase*sDDot;
end

function y = clampScalar(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

function scale = trajectoryScaleAtFraction(par, fraction)

    fraction = clampScalar(fraction, 0, 1);

    if ~isfield(par, 'progress') || ~isfield(par.progress, 'mode')
        scale = 1;
        return;
    end

    switch string(par.progress.mode)
        case "scale_range"
            scaleRange = par.progress.scaleRange;
            scale = scaleRange(1) + (scaleRange(2) - scaleRange(1))*fraction;
        case "scale_fixed"
            scale = par.progress.scale;
        otherwise
            scale = 1;
    end

    scale = max(scale, eps);
end

function s = trajectoryBaseTimeAtFraction(par, fraction)

    fraction = clampScalar(fraction, 0, 1);

    if ~isfield(par, 'progress') || ~isfield(par.progress, 'mode')
        s = fraction*par.Tend;
        return;
    end

    switch string(par.progress.mode)
        case "scale_range"
            scale0 = par.progress.scaleRange(1);
            scale1 = par.progress.scaleRange(2);
            scaleDot = (scale1 - scale0)/par.Tend;
            scale = scale0 + (scale1 - scale0)*fraction;

            if abs(scaleDot) < 1e-12
                s = fraction*par.Tend/scale0;
            else
                s = log(scale/scale0)/scaleDot;
            end

        case "scale_fixed"
            s = fraction*par.Tend/par.progress.scale;

        otherwise
            s = fraction*par.Tend;
    end
end

function shape = trajectoryShape(par)

    intensity = clampScalar(getStructField(par, 'trajIntensity', 0.75), 0, 1);
    scaleHalf = trajectoryScaleAtFraction(par, 0.5);
    scaleEnd = trajectoryScaleAtFraction(par, 1.0);
    sHalf = trajectoryBaseTimeAtFraction(par, 0.5);
    sEnd = trajectoryBaseTimeAtFraction(par, 1.0);
    flipTurns = max(double(getStructField(par, 'flipTurns', 3)), 0.5);

    thrustAccel = max(par.Tmax/max(par.m, eps) - par.g, 0.5*par.g);
    alphaMax = angularAccelLimit(par);

    shape.g = par.g;
    shape.scaleEnd = scaleEnd;
    shape.regularAccel = max((0.18 + 0.20*intensity)*thrustAccel*scaleEnd^2, ...
        0.10*par.g*scaleEnd^2);

    frontFlipMax = (0.93 + 0.04*intensity)*par.g*scaleHalf^2;
    rearFlipMin = (1.02 + 0.10*intensity)*par.g*scaleEnd^2;
    rearFlipTarget = (1.08 + 0.35*intensity ...
                    + 0.80*max(flipTurns - 1, 0))*par.g*scaleEnd^2;
    thrustFlipMax = ((0.70 + 0.18*intensity)*par.Tmax/max(par.m, eps) ...
                   - par.g)*scaleEnd^2;
    flipCap = min([frontFlipMax, rearFlipTarget, thrustFlipMax]);
    shape.flipAccel = min(frontFlipMax, max(rearFlipMin, flipCap));

    shape.loopOmega = clampScalar((0.22 + 0.08*intensity)*sqrt(alphaMax), ...
        1.80, 3.00);
    shape.rampTime = clampScalar(0.5*pi*shape.loopOmega ...
        / max((0.12 + 0.08*intensity)*alphaMax, eps), 1.80, 3.20);
    shape.flipTurns = flipTurns;
    shape.flipSpan = max(sEnd - sHalf, eps);
end

function alphaMax = angularAccelLimit(par)

    Jdiag = abs(diag(par.J));
    tauMax = abs(par.tauMax(:));
    n = min(numel(Jdiag), numel(tauMax));

    if n == 0
        alphaMax = 80;
        return;
    end

    alpha = tauMax(1:n)./max(Jdiag(1:n), eps);
    alpha = alpha(isfinite(alpha) & alpha > 0);

    if isempty(alpha)
        alphaMax = 80;
    else
        alphaMax = min(alpha);
    end

    alphaMax = max(alphaMax, eps);
end

function dst = mergeStructRecursive(dst, src)

    names = fieldnames(src);

    for i = 1:numel(names)
        name = names{i};

        if isstruct(src.(name)) ...
                && isfield(dst, name) ...
                && isstruct(dst.(name))
            dst.(name) = mergeStructRecursive(dst.(name), src.(name));
        else
            dst.(name) = src.(name);
        end
    end
end

function [forceDist, momentDist] = disturbanceAtTime(t, par)

    forceDist = zeros(3,1);
    momentDist = zeros(3,1);

    if ~isfield(par, 'disturbance') || ~par.disturbance.enabled
        return;
    end

    d = par.disturbance;
    distType = string(getStructField(d, 'type', "none"));

    startTime = double(getStructField(d, 'startTime', 0));
    endTime = double(getStructField(d, 'endTime', inf));
    if t < startTime || t > endTime
        return;
    end

    forceAmp = vector3(getStructField(d, 'forceAmp', 0));
    momentAmp = vector3(getStructField(d, 'momentAmp', 0));

    switch distType
        case "constant"
            forceDist = forceAmp;
            momentDist = momentAmp;

        case "sin"
            forceFreq = vector3(getStructField(d, 'forceFreq', [0.31; 0.47; 0.61]));
            momentFreq = vector3(getStructField(d, 'momentFreq', [0.43; 0.59; 0.73]));
            forcePhase = vector3(getStructField(d, 'forcePhase', zeros(3,1)));
            momentPhase = vector3(getStructField(d, 'momentPhase', zeros(3,1)));

            forceDist = forceAmp .* sin(2*pi*forceFreq*t + forcePhase);
            momentDist = momentAmp .* sin(2*pi*momentFreq*t + momentPhase);

        case "none"
            return;

        otherwise
            error("Unknown disturbance type.");
    end
end

function value = getStructField(s, name, defaultValue)

    if isfield(s, name)
        value = s.(name);
    else
        value = defaultValue;
    end
end

function v = vector3(x)

    if isscalar(x)
        v = repmat(x, 3, 1);
    else
        v = x(:);
    end

    if numel(v) ~= 3
        error("Disturbance amplitude/frequency/phase must be scalar or 3x1.");
    end
end

%% ========================================================================
%% Analytic horizontal figure-eight
function cfg = makeFigure8HorizontalParams(shape)

    cfg.Ax = 4.0;
    cfg.Ay = 2.5;
    cfg.h0 = 3.0;
    cfg.Tfig = periodForAccel(max(cfg.Ax, 4*cfg.Ay), ...
        shape.regularAccel, 7.0, 13.0);
end

function ref = evalFigure8Horizontal(t, cfg)

    Om = 2*pi/cfg.Tfig;

    ref.p = [cfg.Ax*sin(Om*t);
             cfg.Ay*sin(2*Om*t);
            -cfg.h0];

    ref.v = [cfg.Ax*Om*cos(Om*t);
             2*cfg.Ay*Om*cos(2*Om*t);
             0];

    ref.a = [-cfg.Ax*Om^2*sin(Om*t);
             -4*cfg.Ay*Om^2*sin(2*Om*t);
             0];

    ref.psi = atan2(ref.v(2), ref.v(1));
end

%% ========================================================================
%% Analytic vertical figure-eight
function cfg = makeFigure8VerticalParams(shape)

    cfg.Ay = 1.15;
    cfg.Az = 1.00;
    cfg.hLow = 1.35;
    cfg.tHover = 1.0;
    cfg.tRamp = 1.50;
    cfg.theta0 = -pi/4;
    accelLimit = min(shape.regularAccel, 0.80*shape.g*shape.scaleEnd^2);
    cfg.Tfig = periodForAccel(max(cfg.Ay, 4*cfg.Az), ...
        accelLimit, 4.8, 8.0);
end

function ref = evalFigure8Vertical(t, cfg)

    hCenter = cfg.hLow + cfg.Az;
    Om = 2*pi/cfg.Tfig;

    if t <= cfg.tHover
        ref.p = [0; -cfg.Ay/sqrt(2); -cfg.hLow];
        ref.v = [0; 0; 0];
        ref.a = [0; 0; 0];
        ref.psi = 0;
        return;
    end

    tau = t - cfg.tHover;
    [q, qDot, qDDot] = rampedTime(tau, cfg.tRamp);

    theta = cfg.theta0 + Om*q;
    thetaDot = Om*qDot;
    thetaDDot = Om*qDDot;

    h = hCenter + cfg.Az*sin(2*theta);

    ref.p = [0;
             cfg.Ay*sin(theta);
            -h];

    ref.v = [0;
             cfg.Ay*cos(theta)*thetaDot;
            -2*cfg.Az*cos(2*theta)*thetaDot];

    ref.a = [0;
             cfg.Ay*(-sin(theta)*thetaDot^2 + cos(theta)*thetaDDot);
             4*cfg.Az*sin(2*theta)*thetaDot^2 ...
             - 2*cfg.Az*cos(2*theta)*thetaDDot];

    ref.psi = 0;
end

%% ========================================================================
%% Analytic helix with flips
function cfg = makeFlipLoopParams(shape, vx, hHover, yRadiusRatio)

    loopOmega = max(shape.loopOmega, 2*pi*shape.flipTurns/shape.flipSpan);

    cfg.vx = vx;
    cfg.hHover = hHover;
    cfg.tHover = 1.0;
    cfg.tRamp = shape.rampTime;
    cfg.Az = clampScalar(shape.flipAccel/loopOmega^2, 0.25, 5.00);
    cfg.Ay = clampScalar(yRadiusRatio*cfg.Az, 0.20, 5.00);
    cfg.Tturn = 2*pi/loopOmega;
end

function ref = evalFlipLoop(t, cfg)

    vx = cfg.vx;
    Ay = cfg.Ay;
    Az = cfg.Az;
    hHover = cfg.hHover;
    hCenter = hHover + Az;
    tHover = cfg.tHover;
    tRamp = cfg.tRamp;
    Om = 2*pi/cfg.Tturn;

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
function cfg = makeFastCircleParams(shape)

    cfg.radius = 5.0;
    cfg.h0 = 5.0;
    cfg.Tcircle = periodForAccel(cfg.radius, shape.regularAccel, 5.0, 10.0);
end

function ref = evalFastCircle(t, cfg)

    Om = 2*pi/cfg.Tcircle;

    ref.p = [cfg.radius*cos(Om*t);
             cfg.radius*sin(Om*t);
            -cfg.h0];

    ref.v = [-cfg.radius*Om*sin(Om*t);
              cfg.radius*Om*cos(Om*t);
              0];

    ref.a = [-cfg.radius*Om^2*cos(Om*t);
             -cfg.radius*Om^2*sin(Om*t);
             0];

    ref.psi = atan2(ref.v(2), ref.v(1));
end

function T = periodForAccel(lengthCoeff, accelLimit, Tmin, Tmax)

    T = 2*pi*sqrt(max(lengthCoeff, eps)/max(accelLimit, eps));
    T = clampScalar(T, Tmin, Tmax);
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
        case "sun_dfbc"
            u = controllerSunDFBC(x, ref, traj, t, par);
        case "sun_nmpc_indi"
            u = controllerSunNMPCINDI(x, ref, traj, t, par);
        case "sun_dfbc_indi"
            u = controllerSunDFBCINDI(x, ref, traj, t, par);
        case "lu_on_manifold_lqr"
            u = controllerLuOnManifoldLQR(x, ref, traj, t, par);
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

    persistent st

    cfg = par.sun;
    N = cfg.N;
    h = cfg.dt;
    refs = sunNMPCBuildReferences(traj, t, N, h, par);
    x0 = sunNMPCStateVector(x);

    solverReset = isempty(st) || t <= par.dt/2 || t <= st.t;
    if solverReset
        st = struct;
        st.pySolver = sunEnsureAcadosPython(par);
        st.pySolver.reset_warm_start();
        st.nextSolveTime = -inf;
        st.lastExitflag = nan;
    end

    solveDue = ~isfield(st, 'lastRotorThrusts') ...
        || t + 0.5*par.dt >= st.nextSolveTime;
    if ~solveDue
        u = sunRotorThrustToControl(st.lastRotorThrusts, refs.R(:,:,1), par);
        u.sunNMPCCached = true;
        u.sunNMPCSolved = false;
        u.sunNMPCStatusCode = 2;
        u.sunNMPCExitflag = st.lastExitflag;
        u.sunNMPCSolveTime = 0;
        return;
    end

    pyResult = st.pySolver.solve( ...
        py.numpy.array(x0'), ...
        py.numpy.array(refs.p'), ...
        py.numpy.array(refs.q'), ...
        py.numpy.array(refs.v'), ...
        py.numpy.array(refs.Omega'), ...
        py.numpy.array(refs.u'));

    status = double(pyResult{'status'});
    rotorThrusts = double(pyResult{'u0'});
    rotorThrusts = rotorThrusts(:);
    solveTime = double(pyResult{'solve_time'});

    if any(~isfinite(rotorThrusts)) || numel(rotorThrusts) ~= 4
        error("sun_nmpc acados returned an invalid rotor-thrust vector.");
    end

    if status ~= 0
        warning("sun_nmpc acados returned status %d at t = %.3f s.", ...
            status, t);
    end

    u = sunRotorThrustToControl(rotorThrusts, refs.R(:,:,1), par);
    u.sunNMPCCached = false;
    u.sunNMPCSolved = true;
    u.sunNMPCStatusCode = status;
    u.sunNMPCExitflag = status;
    u.sunNMPCSolveTime = solveTime;

    if cfg.printSolverTiming
        fprintf('sun_nmpc t=%.3f solve=%.4fs status=%d\n', ...
            t, solveTime, status);
    end

    st.lastRotorThrusts = rotorThrusts;
    st.nextSolveTime = t + cfg.solvePeriod;
    st.t = t;
    st.lastExitflag = status;
end

function pySolver = sunEnsureAcadosPython(par)

    persistent pyModule

    if isempty(pyModule)
        sunConfigurePythonForAcados(par);

        setenv("ACADOS_SOURCE_DIR", char(par.sun.acadosSourceDir));
        setenv("ACADOS_INSTALL_DIR", char(par.sun.acadosSourceDir));

        toolsPath = char(par.sun.acadosToolsPath);
        if exist(toolsPath, 'dir') ~= 7
            mainPath = which('main');
            if strlength(mainPath) > 0
                toolsPath = fullfile(fileparts(mainPath), 'tools');
            end
        end

        pyPath = py.sys.path;
        pyPath.insert(int32(0), toolsPath);
        try
            py.importlib.import_module('casadi');
            py.importlib.import_module('acados_template');
            pyModule = py.importlib.import_module('sun_acados_nmpc');
        catch err
            pe = pyenv;
            error(['sun_nmpc cannot import its Python/acados dependencies.\n' ...
                   'MATLAB is currently using Python:\n  %s\n' ...
                   'Run setup_sun_acados_python once, then restart MATLAB ' ...
                   'if pyenv Status is Loaded, and run main again.\n' ...
                   'Original import error:\n%s'], ...
                  char(pe.Executable), err.message);
        end
    end

    pySolver = pyModule;
end

function sunConfigurePythonForAcados(par)

    if ~isfield(par.sun, 'pythonExecutable')
        return;
    end

    desiredPython = char(par.sun.pythonExecutable);
    if exist(desiredPython, 'file') ~= 2
        return;
    end

    pe = pyenv;
    if string(pe.Status) == "NotLoaded"
        pyenv('Version', desiredPython, 'ExecutionMode', 'InProcess');
        return;
    end

    currentPython = char(pe.Executable);
    if strcmp(currentPython, desiredPython)
        return;
    end

    try
        py.importlib.import_module('casadi');
        py.importlib.import_module('acados_template');
    catch err
        error(['MATLAB has already loaded a Python environment that cannot ' ...
               'import Sun NMPC dependencies.\n' ...
               'Current Python:\n  %s\n' ...
               'Prepared Python:\n  %s\n' ...
               'Restart MATLAB, then before running main execute:\n' ...
               '  pyenv(''Version'', ''%s'', ''ExecutionMode'', ''InProcess'')\n' ...
               'Or run setup_sun_acados_python to install dependencies into ' ...
               'the current Python.\nOriginal import error:\n%s'], ...
              currentPython, desiredPython, desiredPython, err.message);
    end
end

function refs = sunNMPCBuildReferences(traj, t, N, h, par)

    % Paper Eq. (10): supplies x_r = [xi_r; xidot_r; q_r; Omega_r] and u_r
    % as four rotor thrusts. The prediction reference may run past par.Tend
    % by N*h, but the simulation and logged tracking error still stop exactly
    % at par.Tend.
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
        tk = t + (k-1)*h;
        if isfield(traj, 'evalPredict')
            ref = traj.evalPredict(tk);
        else
            ref = traj.eval(min(tk, par.Tend));
        end
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
        refs.u(:,k) = sunAllocateRotorThrusts( ...
            refs.T(k), refs.tau(:,k), par);
    end
end

function xVec = sunNMPCStateVector(x)

    xVec = [x.p;
            rotmToQuatWXYZ(x.R);
            x.v;
            x.Omega];
end

function G = sunAllocationMatrix(par)

    tBM = par.sun.tBM;
    kappa = par.sun.kappa;
    G = [ones(1,4);
         tBM(2,:);
        -tBM(1,:);
         kappa*[-1, -1, 1, 1]];
end

function uRotor = sunAllocateRotorThrusts(T, tau, par)

    mu = [T; tau(:)];
    uRotor = sunBoundedAllocation(mu, par);
end

function uRotor = sunBoundedAllocation(mu, par)

    sunEnsureWLSAllocPath(par);

    G = sunAllocationMatrix(par);
    Wv = diag(sqrt(diag(par.sun.W)));
    lb = par.sun.uMin(:);
    ub = par.sun.uMax(:);
    u0 = min(max(G\mu(:), lb), ub);

    Wu = zeros(4);
    ud = zeros(4,1);
    gamma = 1;
    W0 = zeros(4,1);
    uRotor = wls_alloc(G, mu(:), lb, ub, Wv, Wu, ud, gamma, u0, W0, 50);
    uRotor = min(max(uRotor(:), lb), ub);
end

function sunEnsureWLSAllocPath(par)

    wlsPath = char(par.sun.wlsAllocPath);
    if exist('wls_alloc', 'file') == 2
        return;
    end

    if exist(wlsPath, 'dir') ~= 7
        error("QCAT wls_alloc path does not exist: %s", wlsPath);
    end

    addpath(wlsPath);
    if exist('wls_alloc', 'file') ~= 2
        error("wls_alloc.m was not found after adding path: %s", wlsPath);
    end
end

function u = sunRotorThrustToControl(uRotor, Rd, par)

    uRotor = min(max(uRotor(:), par.sun.uMin), par.sun.uMax);
    mu = sunAllocationMatrix(par)*uRotor;

    u.T = min(max(mu(1), 0), par.Tmax);
    u.tau = saturateVector(mu(2:4), par.tauMax);
    u.Rd = Rd;
    u.rotorThrusts = uRotor;
end

function u = controllerSunDFBC(x, ref, traj, t, par)

    cmd = sunDFBCCommand(x, ref, traj, t, par);
    u = sunRotorThrustToControl(cmd.rotorThrusts, cmd.Rd, par);
end

function u = controllerSunDFBCINDI(x, ref, traj, t, par)

    persistent st

    cmd = sunDFBCCommand(x, ref, traj, t, par);
    [u, st] = sunINDIRotorControl(x, cmd, t, par, st);
end

function u = controllerSunNMPCINDI(x, ref, traj, t, par)

    persistent st

    uMpc = controllerSunNMPC(x, ref, traj, t, par);
    cmd = sunCommandFromRotorThrusts(uMpc.rotorThrusts, uMpc.Rd, x, par);
    [u, st] = sunINDIRotorControl(x, cmd, t, par, st);

    u.sunNMPCCached = uMpc.sunNMPCCached;
    u.sunNMPCSolved = uMpc.sunNMPCSolved;
    u.sunNMPCStatusCode = uMpc.sunNMPCStatusCode;
    u.sunNMPCExitflag = uMpc.sunNMPCExitflag;
    u.sunNMPCSolveTime = uMpc.sunNMPCSolveTime;
end

function cmd = sunDFBCCommand(x, ref, traj, t, par)

    % Sun et al. Eq. (13): desired acceleration from PD position feedback.
    xiErr = ref.p - x.p;
    vErr = ref.v - x.v;
    accD = par.sun.Kxi*xiErr + par.sun.Kv*vErr + ref.a;

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

    % Sun et al. Eq. (28): tilt-prioritized attitude control.
    yawSign = 1;
    if qe(1) < 0
        yawSign = -1;
    end

    alphaD = par.sun.KqRed*qRed + par.sun.kqYaw*yawSign*qYaw ...
           + par.sun.KOmega*(OmegaR - x.Omega) + alphaR;

    tauDesired = par.J*alphaD + cross(x.Omega, par.J*x.Omega);
    rotorThrusts = sunAllocateRotorThrusts(T, tauDesired, par);

    cmd.rotorThrusts = rotorThrusts;
    cmd.T = T;
    cmd.alpha = alphaD;
    cmd.Rd = Rd;
end

function cmd = sunCommandFromRotorThrusts(rotorThrusts, Rd, x, par)

    rotorThrusts = min(max(rotorThrusts(:), par.sun.uMin), par.sun.uMax);
    mu = sunAllocationMatrix(par)*rotorThrusts;
    cmd.rotorThrusts = rotorThrusts;
    cmd.T = mu(1);
    cmd.alpha = par.J \ (mu(2:4) - cross(x.Omega, par.J*x.Omega));
    cmd.Rd = Rd;
end

function [u, st] = sunINDIRotorControl(x, cmd, t, par, st)

    % Sun et al. Eq. (32)-(35), with the Agilicious implementation detail
    % that yaw torque uses the NDI value to avoid yaw oscillation. The MATLAB
    % plant has no motor-speed sensor, so the previous commanded rotor
    % thrusts stand in for filtered rotor-thrust feedback.

    if isempty(st) || t <= par.dt/2 || t <= st.t
        omegaDotF = zeros(3,1);
        thrustsF = cmd.rotorThrusts;
    else
        h = max(t - st.t, par.dt);
        omegaDotF = (x.Omega - st.Omega)/h;
        thrustsF = st.rotorThrusts;
    end

    muF = sunAllocationMatrix(par)*thrustsF;
    tauF = muF(2:4);

    mu = zeros(4,1);
    mu(1) = sum(cmd.rotorThrusts);
    mu(2:4) = tauF + par.J*(cmd.alpha - omegaDotF);

    muNdi = zeros(4,1);
    muNdi(1) = mu(1);
    muNdi(2:4) = par.J*cmd.alpha + cross(x.Omega, par.J*x.Omega);
    mu(4) = muNdi(4);

    rotorThrusts = sunBoundedAllocation(mu, par);

    u = sunRotorThrustToControl(rotorThrusts, cmd.Rd, par);

    st.Omega = x.Omega;
    st.rotorThrusts = rotorThrusts;
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

function u = controllerLuOnManifoldLQR(x, ref, traj, t, par)

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
function xNext = stepModel(x, u, par, t0)

    if nargin < 4
        t0 = 0;
    end

    switch par.integratorName
        case "ode45"
            xNext = stepModelODE45(x, u, par, t0);
        case "lie_rk4"
            xNext = stepModelLieRK4(x, u, par, t0);
        otherwise
            error("Unknown integratorName.");
    end
end

function xNext = stepModelODE45(x, u, par, t0)

    y0 = [x.p; x.v; reshape(x.R, 9, 1); x.Omega];
    opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9);
    [~, yHist] = ode45(@(t,y) quadrotorOde(t, y, u, par), ...
        [t0, t0 + par.dt], y0, opts);

    y = yHist(end,:)';
    xNext.p = y(1:3);
    xNext.v = y(4:6);
    xNext.R = projectSO3(reshape(y(7:15), 3, 3));
    xNext.Omega = y(16:18);
end

function yDot = quadrotorOde(t, y, u, par)

    v = y(4:6);
    R = reshape(y(7:15), 3, 3);
    Omega = y(16:18);

    [a, OmegaDot] = rigidBodyRates(R, Omega, u, par, t);

    yDot = [v;
            a;
            reshape(R*hat(Omega), 9, 1);
            OmegaDot];
end

function xNext = stepModelLieRK4(x, u, par, t0)

    h = par.dt;

    Om1 = x.Omega;
    [a1, OmDot1] = rigidBodyRates(x.R, Om1, u, par, t0);

    v2 = x.v + 0.5*h*a1;
    R2 = x.R*expm(0.5*h*hat(Om1));
    Om2 = x.Omega + 0.5*h*OmDot1;
    [a2, OmDot2] = rigidBodyRates(R2, Om2, u, par, t0 + 0.5*h);

    v3 = x.v + 0.5*h*a2;
    R3 = x.R*expm(0.5*h*hat(Om2));
    Om3 = x.Omega + 0.5*h*OmDot2;
    [a3, OmDot3] = rigidBodyRates(R3, Om3, u, par, t0 + 0.5*h);

    v4 = x.v + h*a3;
    R4 = x.R*expm(h*hat(Om3));
    Om4 = x.Omega + h*OmDot3;
    [a4, OmDot4] = rigidBodyRates(R4, Om4, u, par, t0 + h);

    OmegaBar = (Om1 + 2*Om2 + 2*Om3 + Om4)/6;

    xNext.p = x.p + h/6*(x.v + 2*v2 + 2*v3 + v4);
    xNext.v = x.v + h/6*(a1 + 2*a2 + 2*a3 + a4);
    xNext.R = x.R*expm(h*hat(OmegaBar));
    xNext.Omega = x.Omega + h/6*(OmDot1 + 2*OmDot2 + 2*OmDot3 + OmDot4);
end

function [a, OmegaDot] = rigidBodyRates(R, Omega, u, par, t)

    if nargin < 5
        t = 0;
    end

    [forceDist, momentDist] = disturbanceAtTime(t, par);

    a = par.g*par.e3 - u.T/par.m*R*par.e3 + forceDist/par.m;
    OmegaDot = par.J \ (u.tau + momentDist - cross(Omega, par.J*Omega));
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

    plotDerivativeTracking(time, log, par, traj);
end

function plotDerivativeTracking(time, log, par, traj)

    accActual = loggedLinearAcceleration(log, par);
    [omegaRef, alphaRef] = rotationLogRates(log.Rd, time);
    alphaActual = loggedAngularAcceleration(log, par);

    figure('Name','velocity and acceleration tracking');

    labels = {'v_x (m/s)', 'v_y (m/s)', 'v_z (m/s)', ...
              'a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)'};
    actual = [log.v; accActual];
    desired = [log.vd; log.ad];

    for i = 1:6
        subplot(6,1,i);
        plot(time, actual(i,:), 'LineWidth', 1.1); hold on;
        plot(time, desired(i,:), '--', 'LineWidth', 1.1);
        grid on;
        ylabel(labels{i});

        if i == 1
            title("Velocity/acceleration tracking: " + traj.name);
        end

        if i == 6
            xlabel('time (s)');
        end
    end

    legend('actual','reference');

    figure('Name','angular velocity and angular acceleration tracking');

    labels = {'Omega x (rad/s)', 'Omega y (rad/s)', 'Omega z (rad/s)', ...
              'Omega dot x (rad/s^2)', 'Omega dot y (rad/s^2)', ...
              'Omega dot z (rad/s^2)'};
    actual = [log.Omega; alphaActual];
    desired = [omegaRef; alphaRef];

    for i = 1:6
        subplot(6,1,i);
        plot(time, actual(i,:), 'LineWidth', 1.1); hold on;
        plot(time, desired(i,:), '--', 'LineWidth', 1.1);
        grid on;
        ylabel(labels{i});

        if i == 1
            title("Angular-rate/acceleration tracking: " + traj.name);
        end

        if i == 6
            xlabel('time (s)');
        end
    end

    legend('actual','reference');
end

function acc = loggedLinearAcceleration(log, par)

    N = size(log.v, 2);
    acc = zeros(3, N);

    for k = 1:N
        acc(:,k) = par.g*par.e3 ...
            - log.T(k)/par.m*log.R(:,:,k)*par.e3 ...
            + log.forceDist(:,k)/par.m;
    end
end

function alpha = loggedAngularAcceleration(log, par)

    N = size(log.Omega, 2);
    alpha = zeros(3, N);

    for k = 1:N
        Omega = log.Omega(:,k);
        alpha(:,k) = par.J \ (log.tau(:,k) + log.momentDist(:,k) ...
            - cross(Omega, par.J*Omega));
    end
end

function [omega, alpha] = rotationLogRates(RLog, time)

    N = numel(time);
    omega = zeros(3, N);
    alpha = zeros(3, N);

    if N < 2
        return;
    end

    for k = 1:N-1
        h = time(k+1) - time(k);
        omega(:,k) = LogSO3(RLog(:,:,k)' * RLog(:,:,k+1))/h;
    end

    omega(:,N) = omega(:,N-1);

    for k = 1:N-1
        h = time(k+1) - time(k);
        omegaNextAtK = RLog(:,:,k)' * RLog(:,:,k+1) * omega(:,k+1);
        alpha(:,k) = (omegaNextAtK - omega(:,k))/h;
    end

    alpha(:,N) = alpha(:,N-1);
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
