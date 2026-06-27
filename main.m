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
%   ref.j   jerk
%   ref.s   snap
%   ref.psi yaw
%   ref.psiDot yaw rate
%   ref.psiDDot yaw acceleration

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
% "lu_on_manifold_mpc", "sun_nmpc", "sun_nmpc_indi"
% "geometric_indi", "tal_karaman"
par.controllerName = "lee";
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

% Unified actuator model and control allocation.
% All controllers produce a desired wrench mu = [T; tau], then this layer
% maps it to rotor thrusts and back to the actual wrench applied by the
% benchmark plant. Choose "wls" for bound-aware weighted least squares or
% "pinv" for pseudoinverse followed by clipping.
par.allocation.method = "wls";
par.allocation.tBM = [ 0.075, -0.075, -0.075,  0.075;
                      -0.100,  0.100, -0.100,  0.100;
                       0.000,  0.000,  0.000,  0.000];
par.allocation.kappa = 0.022;
par.allocation.uMin = zeros(4,1);
par.allocation.uMax = 8.5*ones(4,1);
par.allocation.W = diag([0.001, 10, 10, 0.1]);
par.Tmax = sum(par.allocation.uMax);
par.tauMax = allocationMomentLimits(par);

% Lu et al. on-manifold MPC, Eq. (6)-(16).
% State error: [p-pd; v-vd; Log(Rd'R)], input: [aT-aTd; Omega-OmegaD].
par.mpc.N = 8; % Lu et al. use N=8 and run the MPC at 100 Hz.
par.mpc.Q = diag([450, 450, 650, ...
                  70, 70, 100, ...
                  140, 140, 80]);
par.mpc.R = diag([1.0, 0.55, 0.55, 0.75]);
par.mpc.P = par.mpc.Q;
par.mpc.maxQPIt = 120;
par.mpc.qpTol = 1e-7;
par.mpc.omegaMax = deg2rad(800);
par.mpc.rateController = "p"; % "p", "pid", or "indi".
% Lu's MPC input is [thrust acceleration; body rate]. The benchmark plant
% accepts force/moment, so a body-rate inner loop converts Omega_cmd to tau.
par.mpc.rate.Kp = par.KOmega;      % P/PID moment gain.
par.mpc.rate.Ki = zeros(3);        % PID only.
par.mpc.rate.Kd = zeros(3);        % PID only.
par.mpc.rate.integralLimit = deg2rad(120)*ones(3,1);
par.mpc.rate.indiK = 55*eye(3);    % INDI rate error -> angular acceleration.

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
par.sun.W = par.allocation.W;
par.sun.tBM = par.allocation.tBM;
par.sun.kappa = par.allocation.kappa;
par.sun.uMin = par.allocation.uMin;
par.sun.uMax = par.allocation.uMax;
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
par = finalizeActuatorModel(par);
par = finalizeMPCConfig(par);

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
                baseEval, t/scale, 1/scale, 0, 0, 0, baseTend);
            traj.evalPredict = @(t) evalProgressTrajectory( ...
                baseEval, t/scale, 1/scale, 0, 0, 0, baseTend, true);

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
        sDDDot = 0;
        sDDDDot = 0;
    else
        sDot = 1/scale;
        sDDot = -scaleDot/scale^2;
        sDDDot = 2*scaleDot^2/scale^3;
        sDDDDot = -6*scaleDot^3/scale^4;
    end

    ref = evalProgressTrajectory( ...
        baseEval, s, sDot, sDDot, sDDDot, sDDDDot, ...
        baseTend, allowPredict);
end

function ref = evalProgressTrajectory( ...
        baseEval, s, sDot, sDDot, sDDDot, sDDDDot, ...
        baseTend, allowPredict)

    if nargin < 8
        allowPredict = false;
    end

    if allowPredict
        s = max(s, 0);
    else
        s = clampScalar(s, 0, baseTend);
    end

    ref = baseEval(s);
    ref = completeReferenceDerivatives(ref);

    vBase = ref.v;
    aBase = ref.a;
    jBase = ref.j;
    sBase = ref.s;
    psiDotBase = ref.psiDot;
    psiDDotBase = ref.psiDDot;

    ref.v = vBase*sDot;
    ref.a = aBase*sDot^2 + vBase*sDDot;
    ref.j = jBase*sDot^3 + 3*aBase*sDot*sDDot + vBase*sDDDot;
    ref.s = sBase*sDot^4 + 6*jBase*sDot^2*sDDot ...
        + 3*aBase*sDDot^2 + 4*aBase*sDot*sDDDot ...
        + vBase*sDDDDot;
    ref.psiDot = psiDotBase*sDot;
    ref.psiDDot = psiDDotBase*sDot^2 + psiDotBase*sDDot;
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

function par = finalizeActuatorModel(par)

    required = ["tBM", "kappa", "uMin", "uMax", "W"];
    for i = 1:numel(required)
        name = required(i);
        if ~isfield(par.allocation, name)
            error("par.allocation.%s is required.", name);
        end
    end

    par.allocation.method = lower(string(par.allocation.method));
    if ~any(par.allocation.method == ["wls", "pinv"])
        error("par.allocation.method must be ""wls"" or ""pinv"".");
    end

    par.allocation.uMin = par.allocation.uMin(:);
    par.allocation.uMax = par.allocation.uMax(:);
    if numel(par.allocation.uMin) ~= 4 || numel(par.allocation.uMax) ~= 4
        error("The benchmark actuator model expects four rotor thrusts.");
    end

    par.Tmax = sum(par.allocation.uMax);
    par.tauMax = allocationMomentLimits(par);

    % Keep Sun NMPC/DFBC parameters on the same actuator model. This is a
    % compatibility mirror; par.allocation is the source of truth.
    par.sun.tBM = par.allocation.tBM;
    par.sun.kappa = par.allocation.kappa;
    par.sun.uMin = par.allocation.uMin;
    par.sun.uMax = par.allocation.uMax;
    par.sun.W = par.allocation.W;
end

function par = finalizeMPCConfig(par)

    if ~isfield(par, 'mpc')
        return;
    end

    if ~isfield(par.mpc, 'rateController')
        par.mpc.rateController = "p";
    end
    par.mpc.rateController = lower(string(par.mpc.rateController));
    if ~any(par.mpc.rateController == ["p", "pid", "indi"])
        error('par.mpc.rateController must be "p", "pid", or "indi".');
    end

    if ~isfield(par.mpc, 'rate') || ~isstruct(par.mpc.rate)
        par.mpc.rate = struct();
    end

    par.mpc.rate.Kp = matrix3(getStructField(par.mpc.rate, 'Kp', par.KOmega), ...
        'par.mpc.rate.Kp');
    par.mpc.rate.Ki = matrix3(getStructField(par.mpc.rate, 'Ki', zeros(3)), ...
        'par.mpc.rate.Ki');
    par.mpc.rate.Kd = matrix3(getStructField(par.mpc.rate, 'Kd', zeros(3)), ...
        'par.mpc.rate.Kd');
    par.mpc.rate.indiK = matrix3(getStructField(par.mpc.rate, 'indiK', 55*eye(3)), ...
        'par.mpc.rate.indiK');
    par.mpc.rate.integralLimit = vector3( ...
        getStructField(par.mpc.rate, 'integralLimit', deg2rad(120)*ones(3,1)));
end

function tauMax = allocationMomentLimits(par)

    G = allocationMatrix(par);
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    tauMax = zeros(3,1);

    for i = 1:3
        row = G(i+1,:)';
        tauHi = sum(max(row.*lb, row.*ub));
        tauLo = sum(min(row.*lb, row.*ub));
        tauMax(i) = max(abs([tauLo, tauHi]));
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

function M = matrix3(x, name)

    if isscalar(x)
        M = x*eye(3);
    elseif isvector(x) && numel(x) == 3
        M = diag(x(:));
    else
        M = x;
    end

    if ~isequal(size(M), [3, 3])
        error("%s must be scalar, 3-vector, or 3x3.", name);
    end
end

function v = vector3(x)

    if isscalar(x)
        v = repmat(x, 3, 1);
    else
        v = x(:);
    end

    if numel(v) ~= 3
        error("Value must be scalar or 3x1.");
    end
end

function ref = completeReferenceDerivatives(ref)

    if ~isfield(ref, 'j')
        ref.j = zeros(3,1);
    end

    if ~isfield(ref, 's')
        ref.s = zeros(3,1);
    end

    if ~isfield(ref, 'psiDot')
        ref.psiDot = 0;
    end

    if ~isfield(ref, 'psiDDot')
        ref.psiDDot = 0;
    end
end

function ref = setHeadingFromVelocity(ref, defaultPsi)

    speed2 = ref.v(1)^2 + ref.v(2)^2;

    if speed2 < 1e-10
        % Special handling: yaw is undefined at zero horizontal speed, e.g.
        % hover segments. Keep the requested default heading exactly.
        ref.psi = defaultPsi;
        ref.psiDot = 0;
        ref.psiDDot = 0;
        return;
    end

    num = ref.v(1)*ref.a(2) - ref.v(2)*ref.a(1);
    denDot = 2*(ref.v(1)*ref.a(1) + ref.v(2)*ref.a(2));
    numDot = ref.v(1)*ref.j(2) - ref.v(2)*ref.j(1);

    ref.psi = atan2(ref.v(2), ref.v(1));
    ref.psiDot = num/speed2;
    ref.psiDDot = (numDot*speed2 - num*denDot)/speed2^2;
end

function ref = setConstantHeading(ref, psi)

    ref.psi = psi;
    ref.psiDot = 0;
    ref.psiDDot = 0;
end

function [q, qDot, qDDot, q3, q4] = rampedTime(t, tRamp)

    if t <= 0
        q = 0;
        qDot = 0;
        qDDot = 0;
        q3 = 0;
        q4 = 0;
        return;
    end

    if t >= tRamp
        q = t - 0.5*tRamp;
        qDot = 1;
        qDDot = 0;
        q3 = 0;
        q4 = 0;
        return;
    end

    sigma = t/tRamp;

    q = 0.5*t - 0.5*tRamp/pi*sin(pi*sigma);
    qDot = 0.5*(1 - cos(pi*sigma));
    qDDot = 0.5*pi/tRamp*sin(pi*sigma);
    q3 = 0.5*pi^2/tRamp^2*cos(pi*sigma);
    q4 = -0.5*pi^3/tRamp^3*sin(pi*sigma);
end

function [y, yd, ydd, y3, y4] = trigDerivatives( ...
        amplitude, theta, thetaDot, thetaDDot, theta3, theta4, kind)

    s = sin(theta);
    c = cos(theta);

    switch string(kind)
        case "sin"
            y = amplitude*s;
            yd = amplitude*c*thetaDot;
            ydd = amplitude*(c*thetaDDot - s*thetaDot^2);
            y3 = amplitude*(c*theta3 - 3*s*thetaDot*thetaDDot ...
                - c*thetaDot^3);
            y4 = amplitude*(c*theta4 - 4*s*thetaDot*theta3 ...
                - 3*s*thetaDDot^2 - 6*c*thetaDot^2*thetaDDot ...
                + s*thetaDot^4);

        case "cos"
            y = amplitude*c;
            yd = -amplitude*s*thetaDot;
            ydd = amplitude*(-c*thetaDot^2 - s*thetaDDot);
            y3 = amplitude*(s*thetaDot^3 - 3*c*thetaDot*thetaDDot ...
                - s*theta3);
            y4 = amplitude*(c*thetaDot^4 + 6*s*thetaDot^2*thetaDDot ...
                - 3*c*thetaDDot^2 - 4*c*thetaDot*theta3 ...
                - s*theta4);

        otherwise
            error("Unknown trigonometric derivative kind.");
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

    [x, vx, ax, jx, sx] = trigDerivatives( ...
        cfg.Ax, Om*t, Om, 0, 0, 0, "sin");
    [y, vy, ay, jy, sy] = trigDerivatives( ...
        cfg.Ay, 2*Om*t, 2*Om, 0, 0, 0, "sin");

    ref.p = [x; y; -cfg.h0];
    ref.v = [vx; vy; 0];
    ref.a = [ax; ay; 0];
    ref.j = [jx; jy; 0];
    ref.s = [sx; sy; 0];
    ref = setHeadingFromVelocity(ref, 0);
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
        ref.j = [0; 0; 0];
        ref.s = [0; 0; 0];
        ref = setConstantHeading(ref, 0);
        return;
    end

    tau = t - cfg.tHover;
    [q, qDot, qDDot, q3, q4] = rampedTime(tau, cfg.tRamp);

    theta = cfg.theta0 + Om*q;
    thetaDot = Om*qDot;
    thetaDDot = Om*qDDot;
    theta3 = Om*q3;
    theta4 = Om*q4;

    [y, vy, ay, jy, sy] = trigDerivatives( ...
        cfg.Ay, theta, thetaDot, thetaDDot, theta3, theta4, "sin");
    [zOsc, vz, az, jz, sz] = trigDerivatives( ...
        -cfg.Az, 2*theta, 2*thetaDot, 2*thetaDDot, ...
        2*theta3, 2*theta4, "sin");

    ref.p = [0; y; -hCenter + zOsc];
    ref.v = [0; vy; vz];
    ref.a = [0; ay; az];
    ref.j = [0; jy; jz];
    ref.s = [0; sy; sz];
    ref = setConstantHeading(ref, 0);
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
        ref.j = [0; 0; 0];
        ref.s = [0; 0; 0];
        ref = setConstantHeading(ref, 0);
        return;
    end

    tau = t - tHover;
    [q, qDot, qDDot, q3, q4] = rampedTime(tau, tRamp);

    theta = pi + Om*q;
    thetaDot = Om*qDot;
    thetaDDot = Om*qDDot;
    theta3 = Om*q3;
    theta4 = Om*q4;

    [y, vy, ay, jy, sy] = trigDerivatives( ...
        Ay, theta, thetaDot, thetaDDot, theta3, theta4, "sin");
    [zOsc, vz, az, jz, sz] = trigDerivatives( ...
        -Az, theta, thetaDot, thetaDDot, theta3, theta4, "cos");

    ref.p = [vx*q; y; -hCenter + zOsc];
    ref.v = [vx*qDot; vy; vz];
    ref.a = [vx*qDDot; ay; az];
    ref.j = [vx*q3; jy; jz];
    ref.s = [vx*q4; sy; sz];
    ref = setConstantHeading(ref, 0);
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

    [x, vx, ax, jx, sx] = trigDerivatives( ...
        cfg.radius, Om*t, Om, 0, 0, 0, "cos");
    [y, vy, ay, jy, sy] = trigDerivatives( ...
        cfg.radius, Om*t, Om, 0, 0, 0, "sin");

    ref.p = [x; y; -cfg.h0];
    ref.v = [vx; vy; 0];
    ref.a = [ax; ay; 0];
    ref.j = [jx; jy; 0];
    ref.s = [sx; sy; 0];
    ref = setHeadingFromVelocity(ref, 0);
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
            u = controllerLee(x, ref, par);
        case "johnson_beard"
            u = controllerJohnsonBeard(x, ref, par);
        case "sun_nmpc"
            u = controllerSunNMPC(x, ref, traj, t, par);
        case "sun_dfbc"
            u = controllerSunDFBC(x, ref, traj, t, par);
        case "sun_nmpc_indi"
            u = controllerSunNMPCINDI(x, ref, traj, t, par);
        case "sun_dfbc_indi"
            u = controllerSunDFBCINDI(x, ref, traj, t, par);
        case "lu_on_manifold_mpc"
            u = controllerLuOnManifoldMPC(x, ref, traj, t, par);
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

    u = wrenchToControl(T, tau, Rd, par);
end

function u = controllerLee(x, ref, par)

    ex = x.p - ref.p;
    ev = x.v - ref.v;

    aCmd = ref.a - par.lee.Kp*ex - par.lee.Kv*ev;
    thrustAxisForce = par.m*(par.g*par.e3 - aCmd);
    [Rc, ~] = desiredAttitudeFromThrustVector(thrustAxisForce, ref.psi, par);
    [OmegaC, OmegaCDot] = geometricFeedforwardInDesiredFrame(ref, Rc, par);

    eR = 0.5*vee(Rc' * x.R - x.R' * Rc);
    eOmega = x.Omega - x.R' * Rc * OmegaC;

    % Lee et al. Eq. (12)-(13), rewritten for the NED plant.
    tau = -par.lee.KR*eR - par.lee.KOmega*eOmega ...
        + cross(x.Omega, par.J*x.Omega) ...
        - par.J*(hat(x.Omega)*x.R' * Rc * OmegaC - x.R' * Rc * OmegaCDot);

    T = dot(thrustAxisForce, x.R*par.e3);

    u = wrenchToControl(T, tau, Rc, par);
end

function u = controllerJohnsonBeard(x, ref, par)

    ep = x.p - ref.p;
    ev = x.v - ref.v;

    aCmd = ref.a - par.johnsonBeard.Kp*ep - par.johnsonBeard.Kv*ev;
    desiredForce = par.m*(aCmd - par.g*par.e3);
    thrustAxisForce = -desiredForce;
    [Rd, ~] = desiredAttitudeFromThrustVector(thrustAxisForce, ref.psi, par);
    [OmegaD, OmegaDDot] = geometricFeedforwardInDesiredFrame(ref, Rd, par);

    Rbd = x.R' * Rd;
    r = LogSO3(Rbd);
    omegaDInBody = Rbd * OmegaD;
    omegaErr = omegaDInBody - x.Omega;
    omegaDDotInBody = Rbd * OmegaDDot - hat(x.Omega)*omegaDInBody;

    Jl = leftJacobianSO3(r);
    % Johnson and Beard Eq. (21)-(23), (29)-(32). The paper uses J_l(r)' in
    % Eq. (32), not J_l(r)^(-T).
    tau = cross(x.Omega, par.J*x.Omega) ...
        + par.J*omegaDDotInBody ...
        + Jl' * par.johnsonBeard.KR*r ...
        + par.johnsonBeard.KOmega*omegaErr;

    u = wrenchToControl(norm(desiredForce), tau, Rd, par);
end

function u = controllerFaessler(x, ref, traj, t, par)

    % Faessler et al. 2018, Section V, adapted to this benchmark:
    % - Paper frame: z_W is up and the thrust axis is +z_B.
    % - This code: NED z points down and thrust acceleration is
    %   -T/m * R*e3. The force-axis vector below is therefore c*b3_down.
    % - Paper output: [c_cmd, omega_des] or [c_cmd, tau] if the platform
    %   exposes moments. This framework outputs direct [T; tau].
    % - Paper Eq. (17)-(30) compute omega_ref and omegadot_ref from jerk and
    %   snap; this implementation obtains them by analytically differentiating
    %   the same Eq. (11)-(14) flatness attitude map below.

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

    u = wrenchToControl(par.m*cCmd, tau, Rd, par);
end

function ff = faesslerFlatnessReference(traj, t, par)

    ref = completeReferenceDerivatives(traj.eval(t));
    [R, c, Omega, alpha] = faesslerFlatnessAttitudeDerivatives(ref, par);

    ff.R = R;
    ff.c = c;
    ff.Omega = Omega;
    ff.alpha = alpha;
end

function [R, c, Omega, alpha] = faesslerFlatnessAttitudeDerivatives(ref, par)

    % Faessler Eq. (11)-(14), plus analytic time derivatives equivalent to
    % Eq. (17)-(30). The signs are rewritten for the benchmark NED model:
    % a = g*e3 - c*b3 - R*D*R'*v.
    D = par.faessler.D;
    dx = D(1,1);
    dy = D(2,2);
    dz = D(3,3);

    xC = [cos(ref.psi); sin(ref.psi); 0];
    yC = [-sin(ref.psi); cos(ref.psi); 0];
    yCDot = -ref.psiDot*xC;
    yCDD = -ref.psiDDot*xC - ref.psiDot^2*yC;

    alphaVec = par.g*par.e3 - ref.a - dx*ref.v;
    alphaDot = -ref.j - dx*ref.a;
    alphaDDot = -ref.s - dx*ref.j;

    betaVec = par.g*par.e3 - ref.a - dy*ref.v;
    betaDot = -ref.j - dy*ref.a;
    betaDDot = -ref.s - dy*ref.j;

    b1Raw = cross(yC, alphaVec);
    b1RawDot = cross(yCDot, alphaVec) + cross(yC, alphaDot);
    b1RawDDot = cross(yCDD, alphaVec) ...
        + 2*cross(yCDot, alphaDot) + cross(yC, alphaDDot);

    if norm(b1Raw) < 1e-9
        % Special handling: Faessler Eq. (11) is singular when y_C is
        % parallel to alpha. Fall back to the no-drag flatness map.
        ff = geometricFlatnessReference(ref, par);
        R = ff.R;
        c = ff.c;
        Omega = ff.Omega;
        alpha = ff.alpha;
        return;
    end

    [b1, b1Dot, b1DDot] = normalizedVectorDerivatives( ...
        b1Raw, b1RawDot, b1RawDDot);

    b2Raw = cross(betaVec, b1);
    b2RawDot = cross(betaDot, b1) + cross(betaVec, b1Dot);
    b2RawDDot = cross(betaDDot, b1) ...
        + 2*cross(betaDot, b1Dot) + cross(betaVec, b1DDot);

    if norm(b2Raw) < 1e-9
        % Special handling: Faessler Eq. (12) is singular when beta is
        % parallel to x_B. Fall back to the no-drag flatness map.
        ff = geometricFlatnessReference(ref, par);
        R = ff.R;
        c = ff.c;
        Omega = ff.Omega;
        alpha = ff.alpha;
        return;
    end

    [b2, b2Dot, b2DDot] = normalizedVectorDerivatives( ...
        b2Raw, b2RawDot, b2RawDDot);

    b3 = cross(b1, b2);
    b3Dot = cross(b1Dot, b2) + cross(b1, b2Dot);
    b3DDot = cross(b1DDot, b2) ...
        + 2*cross(b1Dot, b2Dot) + cross(b1, b2DDot);

    R = [b1, b2, b3];
    RDot = [b1Dot, b2Dot, b3Dot];
    RDDot = [b1DDot, b2DDot, b3DDot];

    c = dot(par.g*par.e3 - ref.a - dz*ref.v, R*par.e3);
    Omega = vee(R' * RDot);
    alpha = vee(R' * RDDot - hat(Omega)*hat(Omega));
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
    %   this benchmark has a unified thrust-allocation layer but no rotor
    %   speed dynamics. Eq. (31)'s filtered moment mu_f is therefore
    %   represented by the previous allocated equivalent moment st.tau.

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
    u = wrenchToControl(T, tau, Rd, par);

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

    ref = completeReferenceDerivatives(traj.eval(clampScalar(t, 0, par.Tend)));

    refDer.j = ref.j;
    refDer.s = ref.s;
    refDer.psiDot = ref.psiDot;
    refDer.psiDDot = ref.psiDDot;
end

function e = e1Local()

    e = [1; 0; 0];
end

function e = e3Local()

    e = [0; 0; 1];
end

function ff = talFlatnessReference(traj, t, par)

    ff = geometricFlatnessReference(traj.eval(t), par);
end

function u = controllerSunNMPC(x, ~, traj, t, par)

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
        ff = geometricFlatnessReference(ref, par);

        refs.p(:,k) = ref.p;
        refs.v(:,k) = ref.v;
        refs.R(:,:,k) = ff.R;
        refs.q(:,k) = rotmToQuatWXYZ(ff.R);
        refs.Omega(:,k) = ff.Omega;
        refs.alpha(:,k) = ff.alpha;
        refs.T(k) = min(max(ff.T, 0), par.Tmax);
    end

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

function G = allocationMatrix(par)

    tBM = par.allocation.tBM;
    kappa = par.allocation.kappa;
    G = [ones(1,4);
         tBM(2,:);
        -tBM(1,:);
         kappa*[-1, -1, 1, 1]];
end

function u = wrenchToControl(T, tau, Rd, par)

    uRotor = allocateRotorThrusts(T, tau, par);
    u = rotorThrustsToControl(uRotor, Rd, par);
end

function uRotor = allocateRotorThrusts(T, tau, par)

    mu = [T; tau(:)];
    uRotor = allocateWrench(mu, par);
end

function uRotor = allocateWrench(mu, par)

    switch par.allocation.method
        case "wls"
            uRotor = allocateWrenchWLS(mu, par);
        case "pinv"
            uRotor = allocateWrenchPinv(mu, par);
        otherwise
            error("Unknown allocation method.");
    end
end

function uRotor = allocateWrenchPinv(mu, par)

    G = allocationMatrix(par);
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    uRotor = min(max(pinv(G)*mu(:), lb), ub);
end

function uRotor = allocateWrenchWLS(mu, par)

    % Unified WLS allocation, QCAT wls_alloc.m active-set form:
    %   min ||Wu*(u-ud)||^2 + gamma*||Wv*(G*u-mu)||^2
    %   s.t. uMin <= u <= uMax.
    %
    % Source mapping:
    %   B=G is this benchmark's force/moment effectiveness matrix,
    %   v=mu=[T;tau] is the commanded virtual control,
    %   u is the 4x1 rotor thrust vector.
    %
    % Special handling: Wv is sqrt(par.allocation.W), because QCAT writes
    % the cost as a squared norm while par.allocation.W is stored as the
    % quadratic virtual-control weight. Wu=0 keeps the allocator focused on
    % matching the wrench; actuator regularization can be added here later
    % without changing controller code.
    G = allocationMatrix(par);
    Wv = diag(sqrt(diag(par.allocation.W)));
    Wu = zeros(4);
    ud = zeros(4,1);
    gamma = 1;
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    u0 = min(max(pinv(G)*mu(:), lb), ub);
    W0 = zeros(4,1);

    if exist('wls_alloc', 'file') == 2
        uRotor = wls_alloc(G, mu(:), lb, ub, Wv, Wu, ud, gamma, ...
            u0, W0, 50);
    else
        % Fallback copy of the same active-set algorithm, kept only so the
        % benchmark remains runnable if wls_alloc.m is not on the MATLAB path.
        uRotor = boundedWLSActiveSet(G, mu(:), lb, ub, Wv, Wu, ud, gamma, ...
            u0, W0, 50);
    end
end

function u = boundedWLSActiveSet(B, v, umin, umax, Wv, Wu, ud, gamma, u, W, imax)

    A = [sqrt(gamma)*Wv*B; Wu];
    b = [sqrt(gamma)*Wv*v; Wu*ud];
    d = b - A*u;
    free = W == 0;

    for iter = 1:imax
        AFree = A(:,free);
        pFree = AFree\d;
        p = zeros(size(u));
        p(free) = pFree;
        uCandidate = u + p;

        infeasible = (uCandidate < umin) | (uCandidate > umax);
        if ~any(infeasible(free))
            u = uCandidate;
            d = d - AFree*pFree;

            lambda = W.*(A'*d);
            if all(lambda >= -eps)
                u = min(max(u, umin), umax);
                return;
            end

            [~, iRemove] = min(lambda);
            W(iRemove) = 0;
            free(iRemove) = true;
        else
            dist = ones(size(u));
            movingLow = free & p < 0;
            movingHigh = free & p > 0;
            dist(movingLow) = (umin(movingLow) - u(movingLow)) ./ p(movingLow);
            dist(movingHigh) = (umax(movingHigh) - u(movingHigh)) ./ p(movingHigh);

            [alpha, iBound] = min(dist);
            u = u + alpha*p;
            d = d - AFree*(alpha*pFree);
            W(iBound) = sign(p(iBound));
            free(iBound) = false;
        end
    end

    u = min(max(u, umin), umax);
end

function mu = rotorThrustsToWrench(uRotor, par)

    uRotor = min(max(uRotor(:), par.allocation.uMin), par.allocation.uMax);
    mu = allocationMatrix(par)*uRotor;
end

function u = rotorThrustsToControl(uRotor, Rd, par)

    uRotor = min(max(uRotor(:), par.allocation.uMin), par.allocation.uMax);
    mu = rotorThrustsToWrench(uRotor, par);

    u.T = mu(1);
    u.tau = mu(2:4);
    u.Rd = Rd;
    u.rotorThrusts = uRotor;
end

function G = sunAllocationMatrix(par)

    G = allocationMatrix(par);
end

function uRotor = sunAllocateRotorThrusts(T, tau, par)

    uRotor = allocateRotorThrusts(T, tau, par);
end

function uRotor = sunBoundedAllocation(mu, par)

    uRotor = allocateWrench(mu, par);
end

function u = sunRotorThrustToControl(uRotor, Rd, par)

    u = rotorThrustsToControl(uRotor, Rd, par);
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

function cmd = sunDFBCCommand(x, ref, ~, ~, par)

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
    % rates and angular acceleration from reference jerk/snap. The NED
    % flatness attitude already includes the sign change in Eq. (14).
    [OmegaR, alphaR] = sunFlatnessReferenceRates(ref, Rd, par);

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
    muAllocated = rotorThrustsToWrench(rotorThrusts, par);

    cmd.rotorThrusts = rotorThrusts;
    % Sun Eq. (32): after constrained allocation, retrieve the actually
    % achievable collective thrust and angular acceleration for INDI.
    cmd.T = muAllocated(1);
    cmd.alpha = par.J \ (muAllocated(2:4) ...
        - cross(x.Omega, par.J*x.Omega));
    cmd.Rd = Rd;
end

function cmd = sunCommandFromRotorThrusts(rotorThrusts, Rd, x, par)

    rotorThrusts = min(max(rotorThrusts(:), ...
        par.allocation.uMin), par.allocation.uMax);
    mu = rotorThrustsToWrench(rotorThrusts, par);
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

function [OmegaR, alphaR] = sunFlatnessReferenceRates(ref, Rd, par)

    [OmegaR, alphaR] = geometricFeedforwardInDesiredFrame(ref, Rd, par);
end

function u = controllerLuOnManifoldMPC(x, ref, traj, t, par)

    % Lu et al. on-manifold MPC, quadrotor case.
    %
    % Paper-to-code convention:
    %   Lu Eq. (14)-(16) uses state x=(p,v,R) in M=R3 x R3 x SO(3)
    %   and input u=[aT; omega]. In this benchmark R maps body to NED, so
    %   the same model form is
    %       p_dot = v,
    %       v_dot = g*e3 - aT*R*e3,
    %       R_dot = R*hat(omega).
    %
    %   The MPC variable is the input error delta u = u - u_d from Lu
    %   Eq. (13). Therefore the command below is
    %       [aT_cmd; Omega_cmd] = [aT_d; Omega_d] + delta u_0.
    %
    %   Special handling: Lu's experiment sends aT and body-rate omega to a
    %   PX4 rate controller. This benchmark plant accepts force/moment, so
    %   luMpcRateLoop adapts Omega_cmd to tau before the unified allocator.
    [Rd, aTd, OmegaD] = referenceInputOnManifold(ref, par);

    refs = onManifoldMPCReferences(ref, traj, t, par);
    du = solveOnManifoldMPC(x, refs, par);

    aTCmd = aTd + du(1);
    OmegaCmd = OmegaD + du(2:4);
    aTCmd = min(max(aTCmd, 0), par.Tmax/par.m);
    OmegaCmd = saturateVector(OmegaCmd, par.mpc.omegaMax);

    % Lu Eq. (14)-(16) outputs thrust acceleration and body rate. This
    % benchmark plant accepts force/moment, so Omega_cmd is adapted through
    % the configured body-rate loop before unified actuator allocation.
    tau = luMpcRateLoop("compute", x, OmegaCmd, t, par, []);
    u = wrenchToControl(par.m*aTCmd, tau, Rd, par);
    luMpcRateLoop("commit", x, OmegaCmd, t, par, u);
end

function tau = luMpcRateLoop(action, x, OmegaCmd, t, par, uApplied)

    persistent st

    tau = zeros(3,1);

    switch string(action)
        case "compute"
            resetState = isempty(st) || ~isfield(st, 't') ...
                || t <= par.dt/2 || t <= st.t;
            if resetState
                st.t = t - par.dt;
                st.Omega = x.Omega;
                st.eOmega = zeros(3,1);
                st.eInt = zeros(3,1);
                st.tau = cross(x.Omega, par.J*x.Omega);
            end

            h = max(t - st.t, par.dt);
            eOmega = OmegaCmd - x.Omega;
            eInt = min(max(st.eInt + h*eOmega, ...
                -par.mpc.rate.integralLimit), par.mpc.rate.integralLimit);
            eDot = (eOmega - st.eOmega)/h;
            gyro = cross(x.Omega, par.J*x.Omega);

            switch par.mpc.rateController
                case "p"
                    % Lu MPC gives body-rate input. P mode is the minimal
                    % paper-to-plant adaptation: rate error directly to moment.
                    tau = gyro + par.mpc.rate.Kp*eOmega;

                case "pid"
                    % Special handling: derivative is taken on rate error
                    % because the reference body-rate derivative is not an MPC
                    % decision variable in Lu Eq. (14)-(16).
                    tau = gyro + par.mpc.rate.Kp*eOmega ...
                        + par.mpc.rate.Ki*eInt + par.mpc.rate.Kd*eDot;

                case "indi"
                    % Special handling: INDI increments from the previous
                    % allocated moment, so actuator saturation in the unified
                    % allocation layer is reflected on the next update.
                    omegaDotCmd = par.mpc.rate.indiK*eOmega;
                    if resetState
                        tau = gyro + par.J*omegaDotCmd;
                    else
                        omegaDotEst = (x.Omega - st.Omega)/h;
                        tau = st.tau + par.J*(omegaDotCmd - omegaDotEst);
                    end
            end

            st.pendingEOmega = eOmega;
            st.pendingEInt = eInt;

        case "commit"
            if isempty(st) || isempty(uApplied)
                return;
            end

            st.Omega = x.Omega;
            st.eOmega = getStructField(st, 'pendingEOmega', OmegaCmd - x.Omega);
            st.eInt = getStructField(st, 'pendingEInt', st.eInt);
            st.tau = uApplied.tau;
            st.t = t;

        otherwise
            error("Unknown Lu MPC rate-loop action.");
    end
end

function u = controllerGeometricINDI(x, ref, t, par)

    persistent st

    ep = ref.p - x.p;
    ev = ref.v - x.v;
    aCmd = par.indi.Kp*ep + par.indi.Kv*ev + ref.a;

    if isempty(st) || t <= par.dt/2
        [Rd, T] = desiredAttitudeFromAccel(aCmd, ref.psi, par, x.R);
        rErr = LogSO3(x.R' * Rd);

        tau = par.indi.KR*rErr - par.indi.KOmega*x.Omega;
        u = wrenchToControl(T, tau, Rd, par);

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

    u = wrenchToControl(T, tau, Rd, par);

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

function [Rd, aT, OmegaD] = referenceInputOnManifold(ref, par)

    % Reference input u_d=[aT_d; Omega_d] for Lu Eq. (15b).
    % aT_d is collective thrust acceleration, and Omega_d is the body-rate
    % feed-forward recovered from the analytic flat-output derivatives.
    ff = geometricFlatnessReference(ref, par);

    Rd = ff.R;
    aT = ff.T/par.m;
    OmegaD = ff.Omega;
end

function refs = onManifoldMPCReferences(ref, traj, t, par)

    % Build the reference sequence x_d(k), u_d(k) used in Lu Eq. (6) and
    % Eq. (13). The reference trajectory is evaluated analytically where the
    % factory provides derivatives; no finite-difference reference rates are
    % introduced here.
    N = par.mpc.N;
    refs.p = zeros(3, N + 1);
    refs.v = zeros(3, N + 1);
    refs.R = zeros(3, 3, N + 1);
    refs.aT = zeros(1, N + 1);
    refs.Omega = zeros(3, N + 1);

    ff0 = geometricFlatnessReference(ref, par);
    refs.p(:,1) = ref.p;
    refs.v(:,1) = ref.v;
    refs.R(:,:,1) = ff0.R;
    refs.aT(1) = ff0.c;
    refs.Omega(:,1) = ff0.Omega;

    for k = 2:N+1
        tk = t + (k-1)*par.dt;
        if isfield(traj, 'evalPredict')
            refK = traj.evalPredict(tk);
        else
            refK = traj.eval(min(tk, par.Tend));
        end

        ff = geometricFlatnessReference(refK, par);
        refs.p(:,k) = refK.p;
        refs.v(:,k) = refK.v;
        refs.R(:,:,k) = ff.R;
        refs.aT(k) = ff.c;
        refs.Omega(:,k) = ff.Omega;
    end
end

function du0 = solveOnManifoldMPC(x, refs, par)

    % Lu Eq. (13): standard Euclidean MPC over error coordinates.
    %
    % Error-state definition, matching Eq. (13):
    %   delta x = x boxminus x_d
    %           = [p-p_d; v-v_d; Log(R_d' R)] in this benchmark.
    % The right attitude error is consistent with the SO(3) boxplus
    % R = R_d Exp(delta theta) used by the linearization below.
    %
    % Input-error definition:
    %   delta u = u - u_d = [aT-aT_d; Omega-Omega_d].
    %
    % Linearized error dynamics, Lu Eq. (9):
    %   delta x_{k+1} = A_k delta x_k + B_k delta u_k.
    %
    % Condensed prediction:
    %   delta X = Hx*delta x_0 + Mu*delta U,
    % where delta X stacks k=1..N and delta U stacks k=0..N-1.
    %
    % Condensed cost from Lu Eq. (13):
    %   sum ||delta x_k||_Q^2 + ||delta u_k||_R^2 + ||delta x_N||_P^2
    % becomes
    %   min_deltaU 0.5*deltaU' H deltaU + f' deltaU
    % with the irrelevant global factor of two dropped:
    %   H = Mu' Qbar Mu + Rbar,
    %   f = Mu' Qbar Hx delta x_0.
    %
    % Bounds are exactly Lu Eq. (13):
    %   u_min - u_d(k) <= delta u_k <= u_max - u_d(k).
    N = par.mpc.N;
    nx = 9;
    nu = 4;
    h = par.dt;

    dx0 = [x.p - refs.p(:,1);
           x.v - refs.v(:,1);
           LogSO3(refs.R(:,:,1)' * x.R)];

    Hx = zeros(nx*N, nx);
    Mu = zeros(nx*N, nu*N);
    Aseq = zeros(nx, nx, N);
    Bseq = zeros(nx, nu, N);

    for k = 1:N
        [Aseq(:,:,k), Bseq(:,:,k)] = onManifoldMPCLinearization( ...
            refs.R(:,:,k), refs.aT(k), refs.Omega(:,k), h, par);
    end

    Aprod = eye(nx);
    for i = 1:N
        Aprod = Aseq(:,:,i)*Aprod;
        Hx((i-1)*nx+1:i*nx,:) = Aprod;

        for j = 1:i
            Aj = eye(nx);
            for l = j+1:i
                Aj = Aseq(:,:,l)*Aj;
            end
            Mu((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = Aj*Bseq(:,:,j);
        end
    end

    Qbar = kron(eye(N), par.mpc.Q);
    Qbar(end-nx+1:end, end-nx+1:end) = par.mpc.P;
    Rbar = kron(eye(N), par.mpc.R);

    H = Mu'*Qbar*Mu + Rbar;
    f = Mu'*Qbar*Hx*dx0;

    [lb, ub] = onManifoldMPCInputBounds(refs, par);
    du = solveBoxQP(H, f, lb, ub, par.mpc.maxQPIt, par.mpc.qpTol);
    du0 = du(1:nu);
end

function [Ad, Bd] = onManifoldMPCLinearization(Rd, aTd, OmegaD, h, par)

    % Lu Eq. (10)-(12): A_k = Gx + dt*Gf*df/d(delta x),
    %                    B_k = dt*Gf*df/d(delta u).
    %
    % Manifold-specific part for M=R3 x R3 x SO(3):
    %   R3 position and velocity blocks have Gx=Gf=I.
    %   SO(3) uses Appendix Eq. (23) with v=dt*Omega_d:
    %       Gx_R = Exp(-v),
    %       Gf_R = A(v)' = leftJacobianSO3(v)'.
    %
    % System-specific part from Lu Eq. (16), adapted to this benchmark's
    % body-to-NED R and v_dot = g*e3 - aT*R*e3:
    %   df/d(delta x) =
    %       [0 I 0;
    %        0 0 aT*R*hat(e3);
    %        0 0 0],
    %   df/d(delta u) =
    %       [0 0;
    %        -R*e3 0;
    %        0 I].
    %
    % The sign aT*R*hat(e3) follows from R=R_d Exp(delta theta):
    % d[-aT*R*e3]/d(delta theta) at zero = aT*R*hat(e3).
    nx = 9;
    phi = h*OmegaD;
    Gx = eye(nx);
    Gf = eye(nx);
    Gx(7:9,7:9) = expm(-hat(phi));
    Gf(7:9,7:9) = leftJacobianSO3(phi)';

    dfdx = zeros(nx, nx);
    dfdu = zeros(nx, 4);
    dfdx(1:3,4:6) = eye(3);
    dfdx(4:6,7:9) = aTd*Rd*hat(par.e3);
    dfdu(4:6,1) = -Rd*par.e3;
    dfdu(7:9,2:4) = eye(3);

    Ad = Gx + h*Gf*dfdx;
    Bd = h*Gf*dfdu;
end

function [lb, ub] = onManifoldMPCInputBounds(refs, par)

    % Lu Eq. (13) input-error set delta U_k. The physical command bounds
    % are on u=[aT;Omega], so the QP bounds are shifted by u_d(k).
    N = par.mpc.N;
    nu = 4;
    lb = zeros(nu*N,1);
    ub = zeros(nu*N,1);
    omegaMax = vector3(par.mpc.omegaMax);
    uMin = [0; -omegaMax];
    uMax = [par.Tmax/par.m; omegaMax];

    for k = 1:N
        uRef = [refs.aT(k); refs.Omega(:,k)];
        idx = (k-1)*nu+1:k*nu;
        lb(idx) = uMin - uRef;
        ub(idx) = uMax - uRef;
    end
end

function x = solveBoxQP(H, f, lb, ub, maxIt, tol)

    % Solver for the bound-constrained QP in Lu Eq. (13). quadprog is used
    % when available; the projected-gradient fallback is deterministic and
    % keeps the benchmark runnable without extra toolboxes.
    H = 0.5*(H + H') + 1e-9*eye(size(H));
    if exist('quadprog', 'file') == 2
        opts = optimoptions('quadprog', 'Display', 'off');
        [x, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, [], opts);
        if exitflag > 0
            return;
        end
    end

    L = max(eig(H));
    if ~isfinite(L) || L <= 0
        L = 1;
    end
    step = 1/L;
    x = min(max(-(H\f), lb), ub);

    for k = 1:maxIt
        grad = H*x + f;
        xNext = min(max(x - step*grad, lb), ub);
        if norm(xNext - x, inf) < tol
            x = xNext;
            return;
        end
        x = xNext;
    end
end

%% ========================================================================
%% Flatness attitude map layer
function ff = geometricFlatnessReference(ref, par)

    ref = completeReferenceDerivatives(ref);

    % Lee Eq. (14), Johnson-Beard Eq. (19), Sun Eq. (14)-(17), in this
    % benchmark's NED coordinates: F_b3 = m*(g*e3 - a_ref).
    F = par.m*(par.g*par.e3 - ref.a);
    FDot = -par.m*ref.j;
    FDDot = -par.m*ref.s;

    [R, T, Omega, alpha] = attitudeFromThrustDerivatives( ...
        F, FDot, FDDot, ref.psi, ref.psiDot, ref.psiDDot, par);

    ff.R = R;
    ff.T = T;
    ff.c = T/par.m;
    ff.Omega = Omega;
    ff.alpha = alpha;
end

function [Omega, alpha] = geometricFeedforwardInDesiredFrame(ref, Rd, par)

    ff = geometricFlatnessReference(ref, par);

    % Special handling for cascaded controllers: the paper feed-forward
    % rates come from the flat-output reference, while Rd may include
    % position feedback. Rotate the analytic reference rates into the
    % commanded desired frame and let attitude feedback close the remainder.
    RRefToDesired = Rd' * ff.R;
    Omega = RRefToDesired * ff.Omega;
    alpha = RRefToDesired * ff.alpha;
end

function [R, T, Omega, alpha] = attitudeFromThrustDerivatives( ...
        thrustAxisForce, thrustAxisForceDot, thrustAxisForceDDot, ...
        psi, psiDot, psiDDot, par)

    if norm(thrustAxisForce) < 1e-9
        % Special handling: the flatness attitude is undefined for zero
        % thrust-axis force. Use level hover and zero feed-forward rates.
        thrustAxisForce = par.m*par.g*par.e3;
        thrustAxisForceDot = zeros(3,1);
        thrustAxisForceDDot = zeros(3,1);
    end

    [b3, b3Dot, b3DDot] = normalizedVectorDerivatives( ...
        thrustAxisForce, thrustAxisForceDot, thrustAxisForceDDot);
    T = norm(thrustAxisForce);

    headingAxis = [cos(psi); sin(psi); 0];
    headingAxisDot = psiDot*[-sin(psi); cos(psi); 0];
    headingAxisDDot = psiDDot*[-sin(psi); cos(psi); 0] ...
                    - psiDot^2*headingAxis;

    b2Raw = cross(b3, headingAxis);
    b2RawDot = cross(b3Dot, headingAxis) + cross(b3, headingAxisDot);
    b2RawDDot = cross(b3DDot, headingAxis) ...
        + 2*cross(b3Dot, headingAxisDot) + cross(b3, headingAxisDDot);

    if norm(b2Raw) < 1e-8
        % Special handling: yaw heading is parallel to the thrust axis.
        % Match attitudeFromThrustDirection's fixed fallback heading and
        % drop yaw derivatives because the yaw constraint is singular here.
        headingAxis = [0; 1; 0];
        headingAxisDot = zeros(3,1);
        headingAxisDDot = zeros(3,1);
        b2Raw = cross(b3, headingAxis);
        b2RawDot = cross(b3Dot, headingAxis) + cross(b3, headingAxisDot);
        b2RawDDot = cross(b3DDot, headingAxis) ...
            + 2*cross(b3Dot, headingAxisDot) + cross(b3, headingAxisDDot);
    end

    if norm(b2Raw) < 1e-8
        R = attitudeFromThrustDirection(b3, psi);
        Omega = zeros(3,1);
        alpha = zeros(3,1);
        return;
    end

    [b2, b2Dot, b2DDot] = normalizedVectorDerivatives( ...
        b2Raw, b2RawDot, b2RawDDot);
    b1 = cross(b2, b3);
    b1Dot = cross(b2Dot, b3) + cross(b2, b3Dot);
    b1DDot = cross(b2DDot, b3) ...
        + 2*cross(b2Dot, b3Dot) + cross(b2, b3DDot);

    R = [b1, b2, b3];
    RDot = [b1Dot, b2Dot, b3Dot];
    RDDot = [b1DDot, b2DDot, b3DDot];

    Omega = vee(R' * RDot);
    alpha = vee(R' * RDDot - hat(Omega)*hat(Omega));
end

function [u, uDot, uDDot] = normalizedVectorDerivatives(x, xDot, xDDot)

    r = norm(x);

    if r < 1e-12
        u = [1; 0; 0];
        uDot = zeros(3,1);
        uDDot = zeros(3,1);
        return;
    end

    u = x/r;
    rDot = dot(u, xDot);
    uDot = (xDot - u*rDot)/r;
    rDDot = dot(uDot, xDot) + dot(u, xDDot);
    uDDot = (xDDot - u*rDDot - 2*rDot*uDot)/r;
end

function [Rd, T] = desiredAttitudeFromAccel(aCmd, psi, par, ~)

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

function Jl = leftJacobianSO3(phi)

    theta = norm(phi);
    Phi = hat(phi);

    if theta < 1e-6
        Jl = eye(3) + 0.5*Phi + (1/6)*Phi*Phi;
        return;
    end

    Jl = eye(3) ...
        + (1 - cos(theta))/theta^2*Phi ...
        + (theta - sin(theta))/theta^3*Phi*Phi;
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
