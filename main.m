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
% Iris Gazebo Classic plant, actuator, and geometry parameters, matching
% iris.m and gazebo_iris_model.m.
par.m = 0.75;
par.J = diag([2.5, 2.1, 4.3])*1e-3;
par.ct = 1.51e-6;                 % motorConstant [N/(rad/s)^2]
par.cq = 2.37e-8;                 % momentConstant coefficient
par.kappa = par.cq/par.ct;        % yaw moment / thrust [m]
par.omegaMax = 2.3726e3;          % max rotor speed [rad/s]
par.CT = 8.5;                     % max thrust per normalized actuator [N]
par.pos = [ ...
     0.13,  0.22, -0.023;
    -0.13, -0.20, -0.023;
     0.13, -0.22, -0.023;
    -0.13,  0.20, -0.023];
par.axis = [0; 0; -1];
par.spin = [1; 1; -1; -1];

par.dt = 1/300;         % 300 Hz controller/INDI update rate
par.Tend = 16.0;
par.integratorName = "ode45";  % "ode45" or "lie_rk4"

% Reference time scaling.
% scale > 1 slows the reference; scale < 1 speeds it up and may saturate control.
par.progress.mode = "scale_range";      % "scale_fixed" or "scale_range"
par.progress.scale = 1;               % scale_fixed: constant time scale
par.progress.scaleRange = [1, 0.3];   % scale_range: start/end scale over the simulation

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
par.helixTurns = 5;     % geometric turns; par.progress controls timing

% controller
% "geometric", "lee", "johnson_beard", "px4_iris"
% "sun_dfbc", "sun_dfbc_indi"
% "lu_on_manifold_mpc", "sun_nmpc", "sun_nmpc_indi"
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

par.johnsonBeard.We = diag([22.0, 22.0, 355.0, ...
                             2.3, 2.3, 27.0, ...
                             1e-3, 1e-3, 0.1]);
par.johnsonBeard.Wf = diag([0.1, 0.1, 1.0]);
par.johnsonBeard.positionGainMode = "lqr";  % "lqr" or "lee_pd"
par.johnsonBeard.Kp = par.lee.Kp;
par.johnsonBeard.Kv = par.lee.Kv;
par.johnsonBeard.Ki = zeros(3);
par.johnsonBeard.Kr = par.KR;
par.johnsonBeard.Komega = par.KOmega;

% Unified actuator model and control allocation.
% All non-PX4 controllers produce the physical wrench mu = [T; tau].
% PX4 produces normalized actuator commands and is converted separately.
par.allocation.method = "wls";  % "wls" for Eq. (29), or "pinv"
par.allocation.W = diag([0.001, 10, 10, 0.1]);
par.allocation.uMin = zeros(4,1);
par.allocation.uMax = par.CT*ones(4,1);

% Sun Eq. (9) aerodynamic model.
% The body frame here is FRD/NED: body +z is opposite the collective thrust.
par.aero.enabled = true;
par.aero.kd = [0.26; 0.28; 0.42];
par.aero.kh = 0.01;

% PX4 Iris controller: position -> attitude -> rate -> allocation.
par.px4iris.hoverThrust = 0.216;
par.px4iris.posP = [2.0; 2.0; 1.5];
par.px4iris.velP = [4.5; 4.5; 10.0];
par.px4iris.velI = [0.0; 0.0; 0.0];
par.px4iris.velD = [0.0; 0.0; 0.0];
par.px4iris.attP = [12.0; 12.0; 5.0];
par.px4iris.rateP = [0.03; 0.03; 0.05];
par.px4iris.rateI = [0.0; 0.0; 0.0];
par.px4iris.rateD = [0.0; 0.0; 0.0];
par.px4iris.rateFF = [0.0; 0.0; 0.0];
par.px4iris.rateIntLimit = [0.3; 0.3; 0.3];
par.px4iris.useAccelerationFeedforward = true;
par.px4iris.useAttitudeRateFeedforward = true;

% Lu et al. on-manifold MPC, quadrotor experiment, Eq. (6)-(16).
% State error: [p-pd; v-vd; Log(Rd'R)], input: [aT-aTd; Omega-OmegaD].
par.mpc.N = 8;
par.mpc.dt = 0.01;  % Paper UAV experiment: MPC at 100 Hz.
par.mpc.Q = diag([15000, 15000, 15000, ...
                  40, 40, 40, ...
                  80, 80, 80]);
par.mpc.R = diag([0.5, 0.6, 0.6, 0.6]);
par.mpc.P = par.mpc.Q;
par.mpc.maxQPIt = 120;
par.mpc.qpTol = 1e-7;
par.mpc.omegaMax = deg2rad(800)*ones(3,1);
par.mpc.rateController = "p"; % "p", "pid", or "indi".
% Lu's MPC input is [thrust acceleration; body rate]. The benchmark plant
% accepts force/moment, so a body-rate inner loop converts Omega_cmd to tau.
par.mpc.rate.Kp = par.KOmega;      % P/PID moment gain.
par.mpc.rate.Ki = zeros(3);        % PID only.
par.mpc.rate.Kd = zeros(3);        % PID only.
par.mpc.rate.integralLimit = deg2rad(120)*ones(3,1);
par.mpc.rate.indiK = 55*eye(3);    % INDI rate error -> angular acceleration.

% Sun et al. 2022, Table I controller parameters. The plant/allocation layer
% uses the benchmark geometry; Sun NMPC/DFBC inputs are individual rotor
% thrusts u = [u1;u2;u3;u4], as in Eq. (4)-(12).
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
par.sun.omegaMax = [10; 10; 4];
acadosSourceDir = string(getenv("ACADOS_SOURCE_DIR"));
if strlength(acadosSourceDir) == 0
    acadosSourceDir = fullfile(getenv("HOME"), ".local", "src", "acados");
end
par.sun.acadosSourceDir = acadosSourceDir;
par.sun.acadosToolsPath = fullfile(pwd, "tools");
sunPython = string(getenv("SUN_NMPC_PYTHON"));
if strlength(sunPython) == 0 ...
        && exist("/Users/mchmini/.pyenv/versions/3.12.8/bin/python3", "file") == 2
    sunPython = "/Users/mchmini/.pyenv/versions/3.12.8/bin/python3";
end
par.sun.pythonExecutable = sunPython;
par.sun.solvePeriod = 0.01;  % 100 Hz NMPC solve rate; horizon dt remains 50 ms.
par.sun.printSolverTiming = false;
par.sun.filterCutoffHz = 12;
% d_tau is not modeled as a known input; Sun's INDI loop absorbs it through
% filtered angular-acceleration and rotor-thrust feedback.

% Geometric INDI gains.
par.indi.Kp = par.Kp;
par.indi.Kv = par.Kv;
par.indi.KR = par.KR;
par.indi.KOmega = par.KOmega;
par.indi.Ktheta = 55*eye(3);
par.indi.Komega = 14*eye(3);

% Tal and Karaman INDI + differential-flatness controller.
% Use controller-specific gains even when their numerical defaults are close
% to the baseline controller; this keeps each paper controller independently
% tunable.
par.tal.Kp = diag([20, 20, 25]);         % Eq. (17), position term
par.tal.Kv = diag([9, 9, 10]);           % Eq. (17), velocity term
par.tal.Ka = 0.3*eye(3);                 % Eq. (17), acceleration term
% Eq. (28), attitude error and angular-rate terms.
par.tal.Ktheta = 200*eye(3);
par.tal.Komega = 45*eye(3);
% Fig. 4: identical second-order Butterworth LPFs, 30 Hz cutoff.
par.tal.filterCutoffHz = 30;

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
par = finalizeAeroConfig(par);
par = finalizeActuatorModel(par);
par = finalizeMPCConfig(par);

Aa = [zeros(3), eye(3), zeros(3);
      zeros(3), zeros(3), zeros(3);
      eye(3),  zeros(3), zeros(3)];
Ba = [zeros(3); eye(3)/par.m; zeros(3)];
Ha = [Aa, -Ba*(par.johnsonBeard.Wf\Ba');
      -par.johnsonBeard.We, -Aa'];
[Va, Da] = eig(Ha);
Va = Va(:, real(diag(Da)) < 0);
Pa = real(Va(10:18,:)/Va(1:9,:));
Pa = 0.5*(Pa + Pa');
par.johnsonBeard.Klqr = par.johnsonBeard.Wf\(Ba'*Pa);

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
log.OmegaD = nan(3,N);
log.alphaD = nan(3,N);
log.OmegaDProvided = false(1,N);
log.alphaDProvided = false(1,N);

log.euler = zeros(3,N);
log.eulerD = zeros(3,N);

log.T = zeros(1,N);
log.tau = zeros(3,N);
log.actuator = nan(numel(par.allocation.uMin),N);
log.aeroForce = zeros(3,N);
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
    if isfield(u, 'OmegaD')
        log.OmegaD(:,k) = u.OmegaD;
        log.OmegaDProvided(k) = true;
    end
    if isfield(u, 'alphaD')
        log.alphaD(:,k) = u.alphaD;
        log.alphaDProvided(k) = true;
    end

    log.euler(:,k) = rotm2eulZYX(x.R);
    log.eulerD(:,k) = rotm2eulZYX(u.Rd);

    log.T(k) = u.T;
    log.tau(:,k) = u.tau;
    if isfield(u, 'actuator')
        log.actuator(:,k) = u.actuator(:);
    end
    log.aeroForce(:,k) = sunAeroForceWorld(x.R, x.v, par);
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
            cfg = makeHelixFlipParams(shape);
            traj.eval = @(t) evalHelixFlip(t, cfg);

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

            traj.Tend = scaleRangeDuration(baseTend, scaleRange);
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

function simTend = scaleRangeDuration(baseTend, scaleRange)

    scale0 = scaleRange(1);
    scale1 = scaleRange(2);

    if abs(scale1 - scale0) < 1e-12
        simTend = scale0*baseTend;
    else
        simTend = baseTend*(scale0 - scale1)/log(scale0/scale1);
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

    sRaw = s;

    if allowPredict
        s = max(sRaw, 0);
    else
        s = clampScalar(sRaw, 0, baseTend);

        if sRaw < 0 || sRaw > baseTend
            sDot = 0;
            sDDot = 0;
            sDDDot = 0;
            sDDDDot = 0;
        end
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

function accSp = referenceAcceleration(ref, p)

    if p.useAccelerationFeedforward
        accSp = ref.a;
    else
        accSp = zeros(3,1);
    end
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
            simTend = scaleRangeDuration(par.Tend, par.progress.scaleRange);
            scaleDot = (scale1 - scale0)/simTend;
            scale = scale0 + (scale1 - scale0)*fraction;

            if abs(scaleDot) < 1e-12
                s = fraction*simTend/scale0;
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
    sEnd = trajectoryBaseTimeAtFraction(par, 1.0);
    flipTurns = max(double(getStructField(par, 'flipTurns', 1)), 0.5);

    thrustAccel = max(par.Tmax/max(par.m, eps) - par.g, 0.5*par.g);
    alphaMax = angularAccelLimit(par);

    shape.g = par.g;
    shape.baseTend = par.Tend;
    shape.scaleEnd = scaleEnd;
    shape.helixTurns = max(double(getStructField(par, 'helixTurns', 1)), eps);
    shape.helixAccel = (0.12 + 0.10*intensity)*min(thrustAccel, par.g);
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
    shape.flipDuration = max(sEnd - 1.0 - 0.5*shape.rampTime, eps);
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

function par = finalizeAeroConfig(par)

    if ~isfield(par, 'aero')
        par.aero = struct();
    end
    par.aero.enabled = logical(getStructField(par.aero, 'enabled', true));
    par.aero.kd = getStructField(par.aero, 'kd', [0.26; 0.28; 0.42]);
    par.aero.kd = par.aero.kd(:);
    if numel(par.aero.kd) ~= 3
        error("par.aero.kd must be a 3-vector.");
    end
    par.aero.kh = double(getStructField(par.aero, 'kh', 0.01));

    % Keep Sun NMPC/DFBC on the same identified drag model.
    par.sun.aero = par.aero;
end

function par = finalizeActuatorModel(par)

    par.allocation.method = lower(string( ...
        getStructField(par.allocation, 'method', "wls")));
    if ~any(par.allocation.method == ["wls", "pinv"])
        error('par.allocation.method must be "wls" or "pinv".');
    end

    if ~isfield(par.allocation, 'W')
        par.allocation.W = diag([0.001, 10, 10, 0.1]);
    elseif isvector(par.allocation.W) && numel(par.allocation.W) == 4
        par.allocation.W = diag(par.allocation.W(:));
    end
    if ~isequal(size(par.allocation.W), [4, 4])
        error("par.allocation.W must be a 4-vector or 4x4 matrix.");
    end

    par.allocation.uMin = par.allocation.uMin(:);
    par.allocation.uMax = par.allocation.uMax(:);

    [par.allocation.B, par.allocation.B_px4, par.allocation.B_px4_norm] = ...
        irisAllocationMatrices(par);

    par.Tmax = allocationForceLimit(par);
    par.tauMax = allocationMomentLimits(par);

    % Keep Sun NMPC/DFBC parameters on the same actuator model. This is a
    % compatibility mirror; par.allocation is the source of truth.
    par.sun.uMin = par.allocation.uMin;
    par.sun.uMax = par.allocation.uMax;
    par.sun.W = par.allocation.W;
end

function par = finalizeMPCConfig(par)

    par.mpc.rateController = lower(string(par.mpc.rateController));
    if ~isfield(par.mpc, 'dt') || par.mpc.dt <= 0
        error("par.mpc.dt must be positive for Lu on-manifold MPC.");
    end
end

function tauMax = allocationMomentLimits(par)

    B = par.allocation.B;
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    tauMax = zeros(3,1);

    for i = 1:3
        row = B(i+1,:)';
        tauHi = sum(max(row.*lb, row.*ub));
        tauLo = sum(min(row.*lb, row.*ub));
        tauMax(i) = max(abs([tauLo, tauHi]));
    end
end

function Tmax = allocationForceLimit(par)

    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    row = -par.allocation.B(1,:)';
    Tmax = sum(max(row.*lb, row.*ub));
end

function [B, B_px4, B_px4_norm] = irisAllocationMatrices(par)

    % This is the control-allocation construction from iris.m, kept local so
    % main.m derives standard and PX4 matrices from the same Gazebo Iris model.
    CT = par.CT;
    kappa = par.kappa;
    pos = par.pos;
    axis = par.axis;
    KM = kappa*par.spin(:);

    B2 = zeros(6,4);

    for i = 1:4
        r = pos(i,:)';
        moment = cross(r, axis) - KM(i)*axis;
        force = axis;
        B2(:,i) = [moment; force];
    end

    B3 = zeros(6,4);

    for i = 1:4
        r = pos(i,:)';
        moment = CT*cross(r, axis) - CT*KM(i)*axis;
        force = CT*axis;
        B3(:,i) = [moment; force];
    end

    [~, B3_norm] = px4_normalize_B(B3, true);

    B = [B2(6,:); B2(1:3,:)];
    B_px4 = [B3(6,:); B3(1:3,:)];
    B_px4_norm = [B3_norm(6,:); B3_norm(1:3,:)];
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
%% Analytic x-forward helix_flip
function cfg = makeHelixFlipParams(shape)

    cfg.turns = shape.helixTurns;
    cfg.T = shape.baseTend;
    cfg.omega = 2*pi*cfg.turns/cfg.T;
    cfg.radius = shape.helixAccel/cfg.omega^2;
    cfg.length = cfg.radius*cfg.turns;
    cfg.hStart = max(1.50, 0.25*cfg.radius);
    cfg.hCenter = cfg.hStart + cfg.radius;
end

function ref = evalHelixFlip(t, cfg)

    theta = cfg.omega*t;

    [y, vy, ay, jy, sy] = trigDerivatives( ...
        cfg.radius, theta, cfg.omega, 0, 0, 0, "sin");
    [zOsc, vz, az, jz, sz] = trigDerivatives( ...
        cfg.radius, theta, cfg.omega, 0, 0, 0, "cos");

    vx = cfg.length/cfg.T;

    ref.p = [vx*t; y; -cfg.hCenter + zOsc];
    ref.v = [vx; vy; vz];
    ref.a = [0; ay; az];
    ref.j = [0; jy; jz];
    ref.s = [0; sy; sz];
    ref = setConstantHeading(ref, 0);
end

%% ========================================================================
%% Analytic vertical loop
function cfg = makeFlipLoopParams(shape, vx, hHover, yRadiusRatio)

    loopOmega = 2*pi*shape.flipTurns/shape.flipDuration;

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

function fWorld = sunAeroForceWorld(R, v, par)

    fWorld = R*sunAeroForceBody(R, v, par);
end

function fBody = sunAeroForceBody(R, v, par)

    if ~isfield(par, 'aero') || ~par.aero.enabled
        fBody = zeros(3,1);
        return;
    end

    vBody = R'*v;
    kd = par.aero.kd(:);
    kh = par.aero.kh;

    % Sun Eq. (9), transformed to this benchmark's FRD body frame. The paper's
    % +k_h term is along the thrust-aligned body z; here body +z is opposite
    % thrust, so the induced-drag term enters with a minus sign.
    lateralSpeedSq = vBody(1)^2 + vBody(2)^2;
    fBody = [-kd(1)*vBody(1);
             -kd(2)*vBody(2);
             -kd(3)*vBody(3) - kh*lateralSpeedSq];
end

%% Controller layer
function u = controller(x, ref, traj, t, par)

    switch par.controllerName
        case "geometric"
            u = controllerPDGeometric(x, ref, par);
        case "lee"
            u = controllerLee(x, ref, par);
        case "px4_iris"
            u = controllerPX4Iris(x, ref, t, par);
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
        case "lu_on_manifold_mpc"
            u = controllerLuOnManifoldMPC(x, ref, traj, t, par);
        case "geometric_indi"
            u = controllerGeometricINDI(x, ref, t, par);
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

    u = controlAllocation([T; tau], Rd, par);
end

function u = controllerLee(x, ref, par)

    cmd = leePositionCommand(x, ref, par);
    Rc = cmd.Rc;
    OmegaC = cmd.OmegaC;
    OmegaCDot = cmd.OmegaCDot;

    eR = 0.5*vee(Rc' * x.R - x.R' * Rc);
    eOmega = x.Omega - x.R' * Rc * OmegaC;

    % Lee et al. Eq. (12)-(13), rewritten for the NED plant.
    tau = -par.lee.KR*eR - par.lee.KOmega*eOmega ...
        + cross(x.Omega, par.J*x.Omega) ...
        - par.J*(hat(x.Omega)*x.R' * Rc * OmegaC - x.R' * Rc * OmegaCDot);

    u = controlAllocation([cmd.T; tau], Rc, par);
    u.OmegaD = OmegaC;
    u.alphaD = OmegaCDot;
end

function cmd = leePositionCommand(x, ref, par)

    % Lee et al. position-controlled flight mode, Eq. (19)-(23) in the
    % 2011 complex-maneuver paper and Eq. (12)-(14) in the 2010 paper,
    % rewritten for this NED/FRD plant where a = g*e3 - T/m*R*e3.
    ref = completeReferenceDerivatives(ref);

    ex = x.p - ref.p;
    ev = x.v - ref.v;

    thrustAxisForce = par.m*(par.lee.Kp*ex + par.lee.Kv*ev ...
        + par.g*par.e3 - ref.a);
    T = dot(thrustAxisForce, x.R*par.e3);

    [forceDot, forceDDot] = leeThrustAxisForceDerivatives( ...
        x, ref, ev, thrustAxisForce, T, par);
    [b1d, b1dDot, b1dDDot] = leeDesiredFirstBodyAxis(ref);
    [Rc, RcDot, RcDDot] = leeComputedAttitude( ...
        thrustAxisForce, forceDot, forceDDot, ...
        b1d, b1dDot, b1dDDot);

    OmegaC = vee(Rc' * RcDot);
    OmegaCDot = vee(Rc' * RcDDot - hat(OmegaC)*hat(OmegaC));

    cmd.Rc = Rc;
    cmd.T = T;
    cmd.OmegaC = OmegaC;
    cmd.OmegaCDot = OmegaCDot;
end

function [forceDot, forceDDot] = leeThrustAxisForceDerivatives( ...
        x, ref, ev, thrustAxisForce, T, par)

    b3 = x.R*par.e3;
    b3Dot = x.R*hat(x.Omega)*par.e3;

    accel = par.g*par.e3 - T/par.m*b3;
    forceDot = par.m*(par.lee.Kp*ev ...
        + par.lee.Kv*(accel - ref.a) - ref.j);

    TDot = dot(forceDot, b3) + dot(thrustAxisForce, b3Dot);
    accelDot = -TDot/par.m*b3 - T/par.m*b3Dot;

    forceDDot = par.m*(par.lee.Kp*(accel - ref.a) ...
        + par.lee.Kv*(accelDot - ref.j) - ref.s);
end

function [b1d, b1dDot, b1dDDot] = leeDesiredFirstBodyAxis(ref)

    psi = ref.psi;
    psiDot = ref.psiDot;
    psiDDot = ref.psiDDot;

    b1d = [cos(psi); sin(psi); 0];
    b1dDot = psiDot*[-sin(psi); cos(psi); 0];
    b1dDDot = psiDDot*[-sin(psi); cos(psi); 0] - psiDot^2*b1d;
end

function [Rc, RcDot, RcDDot] = leeComputedAttitude( ...
        thrustAxisForce, thrustAxisForceDot, thrustAxisForceDDot, ...
        b1d, b1dDot, b1dDDot)

    [b3c, b3cDot, b3cDDot] = leeNormalizeWithDerivatives( ...
        thrustAxisForce, thrustAxisForceDot, thrustAxisForceDDot);

    projection = b1d - b3c*dot(b3c, b1d);
    projectionDot = b1dDot ...
        - b3cDot*dot(b3c, b1d) ...
        - b3c*(dot(b3cDot, b1d) + dot(b3c, b1dDot));
    projectionDDot = b1dDDot ...
        - b3cDDot*dot(b3c, b1d) ...
        - 2*b3cDot*(dot(b3cDot, b1d) + dot(b3c, b1dDot)) ...
        - b3c*(dot(b3cDDot, b1d) ...
            + 2*dot(b3cDot, b1dDot) + dot(b3c, b1dDDot));

    [b1c, b1cDot, b1cDDot] = leeNormalizeWithDerivatives( ...
        projection, projectionDot, projectionDDot);

    b2c = cross(b3c, b1c);
    b2cDot = cross(b3cDot, b1c) + cross(b3c, b1cDot);
    b2cDDot = cross(b3cDDot, b1c) ...
        + 2*cross(b3cDot, b1cDot) + cross(b3c, b1cDDot);

    Rc = [b1c, b2c, b3c];
    RcDot = [b1cDot, b2cDot, b3cDot];
    RcDDot = [b1cDDot, b2cDDot, b3cDDot];
end

function [u, uDot, uDDot] = leeNormalizeWithDerivatives(x, xDot, xDDot)

    r = norm(x);

    u = x/r;
    rDot = dot(u, xDot);
    uDot = (xDot - u*rDot)/r;
    rDDot = dot(uDot, xDot) + dot(u, xDDot);
    uDDot = (xDDot - u*rDDot - 2*rDot*uDot)/r;
end

function u = controllerPX4Iris(x, ref, t, par)

    persistent st

    p = par.px4iris;
    if isempty(st) || t <= par.dt/2 || t <= st.t
        st.velInt = zeros(3,1);
        st.prevVel = x.v;
        st.rateInt = zeros(3,1);
        st.prevOmega = x.Omega;
        st.t = t - par.dt;
    end

    dt = par.dt;
    velDot = (x.v - st.prevVel)/max(dt, eps);
    angularAccel = (x.Omega - st.prevOmega)/max(dt, eps);

    [thrSp, Rd, st] = px4PositionControl(x, ref, velDot, par, st);

    if p.useAttitudeRateFeedforward
        [omegaFF, ~] = geometricFeedforwardInDesiredFrame(ref, Rd, par);
        ratesSp = px4AttitudeControl(x.R, Rd, p) + omegaFF;
    else
        ratesSp = px4AttitudeControl(x.R, Rd, p) ...
            + x.R' * [0; 0; 1] * ref.psiDot;
    end

    [torqueNorm, st] = px4RateControl( ...
        x.Omega, ratesSp, angularAccel, par, p, st);

    thrustBodyZ = -norm(thrSp);
    muNormCmd = [thrustBodyZ; torqueNorm];
    u = controlAllocationPX4Normalized(muNormCmd, Rd, par);

    st.prevVel = x.v;
    st.prevOmega = x.Omega;
    st.t = t;
end

function [thrSp, Rd, st] = px4PositionControl(x, ref, velDot, par, st)

    p = par.px4iris;
    dt = par.dt;

    velSp = ref.v + (ref.p - x.p) .* p.posP;
    velError = velSp - x.v;
    accSp = referenceAcceleration(ref, p) + velError .* p.velP ...
        + st.velInt - velDot .* p.velD;

    thrSp = px4AccelerationControl(accSp, p, par);
    st.velInt = st.velInt + velError .* p.velI * dt;

    bodyZ = -thrSp;
    Rd = px4BodyZToAttitude(bodyZ, ref.psi);
end

function thrSp = px4AccelerationControl(accSp, p, par)

    zSpecificForce = -par.g + accSp(3);
    bodyZ = [-accSp(1); -accSp(2); -zSpecificForce];
    bodyZ = bodyZ/norm(bodyZ);

    thrustNedZ = accSp(3) * (p.hoverThrust/par.g) - p.hoverThrust;
    cosNedBody = dot(par.e3, bodyZ);
    collectiveThrust = thrustNedZ/cosNedBody;
    thrSp = bodyZ * collectiveThrust;
end

function Rd = px4BodyZToAttitude(bodyZ, yawSp)

    bodyZ = bodyZ/norm(bodyZ);

    yC = [-sin(yawSp); cos(yawSp); 0];
    bodyX = cross(yC, bodyZ);
    bodyX = bodyX/norm(bodyX);

    bodyY = cross(bodyZ, bodyX);
    Rd = [bodyX, bodyY, bodyZ];
end

function ratesSp = px4AttitudeControl(R, Rd, p)

    q = rotmToQuatWXYZ(R);
    qd = rotmToQuatWXYZ(Rd);
    qError = normalizeQuatWXYZ(quatMultiplyWXYZ(quatConjugateWXYZ(q), qd));
    if qError(1) < 0
        qError = -qError;
    end

    ratesSp = 2*qError(2:4) .* p.attP;
end

function [torqueNorm, st] = px4RateControl( ...
        rates, ratesSp, angularAccel, par, p, st)

    rateError = ratesSp - rates;
    torqueNorm = p.rateP .* rateError + st.rateInt ...
        - p.rateD .* angularAccel + p.rateFF .* ratesSp;

    dt = par.dt;
    for i = 1:3
        iFactor = rateError(i)/deg2rad(400);
        iFactor = max(0, 1 - iFactor^2);
        nextInt = st.rateInt(i) + iFactor*p.rateI(i)*rateError(i)*dt;
        st.rateInt(i) = clampScalar(nextInt, ...
            -p.rateIntLimit(i), p.rateIntLimit(i));
    end
end

function u = controllerJohnsonBeard(x, ref, t, par)

    persistent st

    ep = x.p - ref.p;
    ev = x.v - ref.v;

    if isempty(st) || t <= par.dt/2 || t <= st.t
        st.eInt = zeros(3,1);
        st.t = t - par.dt;
    end

    h = max(t - st.t, par.dt);
    st.eInt = st.eInt + h*ep;

    ea = [ep; ev; st.eInt];
    cmd = johnsonBeardCommand(ea, ref, par);
    Rd = cmd.Rid;
    omegaD = cmd.omegaD;
    omegaDotD = cmd.omegaDotD;

    Rbd = x.R' * Rd;
    r = johnsonBeardLogSO3(Rbd);
    omegaDInBody = Rbd * omegaD;
    omegaErr = omegaDInBody - x.Omega;
    omegaDotDInBody = Rbd * omegaDotD - hat(x.Omega)*omegaDInBody;

    Jl = johnsonBeardLeftJacobianSO3(r);
    % Johnson and Beard Eq. (21)-(23), (29)-(32).
    tau = cross(x.Omega, par.J*x.Omega) ...
        + par.J*omegaDotDInBody ...
        + Jl' * par.johnsonBeard.Kr*r ...
        + par.johnsonBeard.Komega*omegaErr;

    u = controlAllocation([cmd.T; tau], Rd, par);
    u.OmegaD = omegaD;
    u.alphaD = omegaDotD;
    st.t = t;
end

function cmd = johnsonBeardCommand(ea, ref, par)

    % Johnson and Beard Eq. (13), (18)-(21): the LQR block computes f_d,
    % then the desired-rotation block aligns the body k-axis with -f_d.
    ref = completeReferenceDerivatives(ref);

    K = johnsonBeardPositionGain(par);
    Aa = [zeros(3), eye(3), zeros(3);
          zeros(3), zeros(3), zeros(3);
          eye(3),  zeros(3), zeros(3)];
    Ba = [zeros(3); eye(3)/par.m; zeros(3)];

    fEq = par.m*(ref.a - par.g*par.e3);
    fTilde = -K*ea;
    fd = fTilde + fEq;
    T = norm(fd);

    eaDot = Aa*ea + Ba*fTilde;
    fEqDot = par.m*ref.j;
    fdDot = -K*eaDot + fEqDot;

    fTildeDot = -K*eaDot;
    eaDDot = Aa*eaDot + Ba*fTildeDot;
    fEqDDot = par.m*ref.s;
    fdDDot = -K*eaDDot + fEqDDot;

    [Rid, RidDot, RidDDot] = johnsonBeardDesiredRotation( ...
        fd, fdDot, fdDDot, ref.psi, ref.psiDot, ref.psiDDot);

    omegaD = vee(Rid' * RidDot);
    omegaDotD = vee(Rid' * RidDDot - hat(omegaD)*hat(omegaD));

    cmd.fd = fd;
    cmd.T = T;
    cmd.Rid = Rid;
    cmd.omegaD = omegaD;
    cmd.omegaDotD = omegaDotD;
end

function K = johnsonBeardPositionGain(par)

    mode = lower(string(getStructField(par.johnsonBeard, ...
        'positionGainMode', "lqr")));

    switch mode
        case "lqr"
            K = par.johnsonBeard.Klqr;
        case "lee_pd"
            Kp = getStructField(par.johnsonBeard, 'Kp', par.lee.Kp);
            Kv = getStructField(par.johnsonBeard, 'Kv', par.lee.Kv);
            Ki = getStructField(par.johnsonBeard, 'Ki', zeros(3));
            K = par.m*[Kp, Kv, Ki];
        otherwise
            error('Unknown par.johnsonBeard.positionGainMode "%s".', mode);
    end
end

function [Rid, RidDot, RidDDot] = johnsonBeardDesiredRotation( ...
        fd, fdDot, fdDDot, psi, psiDot, psiDDot)

    [kd, kdDot, kdDDot] = johnsonBeardNormalizeWithDerivatives( ...
        -fd, -fdDot, -fdDDot);

    sd = [cos(psi); sin(psi); 0];
    sdDot = psiDot*[-sin(psi); cos(psi); 0];
    sdDDot = psiDDot*[-sin(psi); cos(psi); 0] - psiDot^2*sd;

    jdRaw = cross(kd, sd);
    jdRawDot = cross(kdDot, sd) + cross(kd, sdDot);
    jdRawDDot = cross(kdDDot, sd) ...
        + 2*cross(kdDot, sdDot) + cross(kd, sdDDot);
    [jd, jdDot, jdDDot] = johnsonBeardNormalizeWithDerivatives( ...
        jdRaw, jdRawDot, jdRawDDot);

    id = cross(jd, kd);
    idDot = cross(jdDot, kd) + cross(jd, kdDot);
    idDDot = cross(jdDDot, kd) ...
        + 2*cross(jdDot, kdDot) + cross(jd, kdDDot);

    Rid = [id, jd, kd];
    RidDot = [idDot, jdDot, kdDot];
    RidDDot = [idDDot, jdDDot, kdDDot];
end

function [u, uDot, uDDot] = johnsonBeardNormalizeWithDerivatives( ...
        x, xDot, xDDot)

    r = norm(x);
    u = x/r;
    rDot = dot(u, xDot);
    uDot = (xDot - u*rDot)/r;
    rDDot = dot(uDot, xDot) + dot(u, xDDot);
    uDDot = (xDDot - u*rDDot - 2*rDot*uDot)/r;
end

function phiVec = johnsonBeardLogSO3(R)

    % Johnson and Beard Eq. (5a)-(5b).
    phi = acos((trace(R) - 1)/2);

    if abs(abs(phi) - pi) < 1e-10
        [V, D] = eig(R);
        [~, i] = min(abs(diag(D) - 1));
        u = real(V(:,i));
        u = u/norm(u);
        phiVec = phi*u;
        return;
    end

    phiVec = 1/(2*johnsonBeardSinc(phi/2)*cos(phi/2)) ...
        * vee(R - R');
end

function Jl = johnsonBeardLeftJacobianSO3(phiVec)

    % Johnson and Beard Eq. (6).
    phi = norm(phiVec);

    if phi == 0
        Jl = eye(3);
        return;
    end

    uHat = hat(phiVec/phi);
    Jl = eye(3) ...
        + sin(phi/2)*johnsonBeardSinc(phi/2)*uHat ...
        + (1 - johnsonBeardSinc(phi))*uHat*uHat;
end

function y = johnsonBeardSinc(x)

    if x == 0
        y = 1;
    else
        y = sin(x)/x;
    end
end

function ff = talFlatnessReference(ref, par)

    % Tal Eq. (8)-(13): flat outputs are x_ref and psi_ref. In this benchmark
    % v_dot = g*e3 - T/m*b_z, while Tal writes v_dot = g*i_z + tau*b_z with
    % signed specific thrust tau. Therefore tau_ref*b_z,ref = a_ref - g*e3.
    ref = completeReferenceDerivatives(ref);

    tauVector = ref.a - par.g*par.e3;
    tauMag = norm(tauVector);
    b3 = -tauVector/tauMag;

    b1Yaw = [cos(ref.psi); sin(ref.psi); 0];
    b2 = cross(b3, b1Yaw);
    b2 = b2/norm(b2);
    b1 = cross(b2, b3);

    ff.R = [b1, b2, b3];
    ff.tau = -tauMag;
    ff.c = tauMag;
    ff.T = par.m*tauMag;
end

function u = controllerTalKaraman(x, ref, ~, t, par)

    persistent st

    % Tal and Karaman 2021, adapted to this MATLAB benchmark:
    % - Paper model uses NED and v_dot = g*i_z + tau*b_z + f_ext/m, where
    %   tau is the signed specific thrust. This benchmark instead outputs a
    %   positive force T and uses v_dot = g*e3 - T/m*b_z, so tau = -T/m.
    % - Paper Eq. (16), (17), (20), (28), and (31) are kept. IMU acceleration,
    %   angular rate, and angular acceleration are represented by simulated
    %   state differences, then filtered with the Fig. 4 second-order
    %   Butterworth LPF. Eq. (20)'s filtered specific-thrust vector is
    %   represented by the previous saturated force command st.T, converted
    %   from N to m/s^2 and passed through the same LPF. Eq. (31)'s mu_f is
    %   handled similarly for the direct equivalent moment.
    % - Paper Eq. (22)-(26) builds an incremental quaternion attitude command.
    %   This framework stores absolute attitude commands, so we compute the
    %   equivalent Rd from the INDI thrust vector and yaw; x.R'*Rd is the
    %   incremental attitude used in Eq. (28).
    % - Paper Eq. (32), (33), and (36) need rotor-speed dynamics and ESC
    %   throttle states, which this benchmark does not have. Eq. (35) reduces
    %   to direct inverse allocation because the benchmark actuator variable is
    %   already proportional to omega^2. Eq. (31)'s filtered moment mu_f is
    %   represented by the previous allocated equivalent moment st.tau.

    ff = talFlatnessReference(ref, par);

    filterReset = isempty(st) || t <= par.dt/2 || t <= st.t;
    if filterReset
        aFilt = ref.a;
        omegaF = x.Omega;
        omegaDotF = zeros(3,1);
        thrustAccelF = ff.tau * ff.R*par.e3;
        muF = zeros(3,1);

        st.aFilter = initSecondOrderLPF(aFilt);
        st.omegaFilter = initSecondOrderLPF(omegaF);
        st.omegaDotFilter = initSecondOrderLPF(omegaDotF);
        st.thrustAccelFilter = initSecondOrderLPF(thrustAccelF);
        st.muFilter = initSecondOrderLPF(muF);
    else
        h = max(t - st.t, par.dt);
        rawAFilt = (x.v - st.v)/h;
        rawOmegaF = x.Omega;
        rawOmegaDotF = (x.Omega - st.Omega)/h;
        % Previous applied force T [N] -> paper's (tau*b_z)_f [m/s^2].
        rawThrustAccelF = -st.T/par.m * st.R*par.e3;
        rawMuF = st.tau;

        [aFilt, st.aFilter] = secondOrderButterworthLPF( ...
            rawAFilt, st.aFilter, h, par.tal.filterCutoffHz);
        [omegaF, st.omegaFilter] = secondOrderButterworthLPF( ...
            rawOmegaF, st.omegaFilter, h, par.tal.filterCutoffHz);
        [omegaDotF, st.omegaDotFilter] = secondOrderButterworthLPF( ...
            rawOmegaDotF, st.omegaDotFilter, h, par.tal.filterCutoffHz);
        [thrustAccelF, st.thrustAccelFilter] = secondOrderButterworthLPF( ...
            rawThrustAccelF, st.thrustAccelFilter, h, par.tal.filterCutoffHz);
        [muF, st.muFilter] = secondOrderButterworthLPF( ...
            rawMuF, st.muFilter, h, par.tal.filterCutoffHz);
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

    % Eq. (14)-(15): solve Tal's 4x4 flatness matrix directly from jerk,
    % snap, yaw rate, and yaw acceleration. No SO(3) finite-difference
    % replacement is used when Eq. (11)-(13)'s yaw parametrization is
    % singular or ill-conditioned.
    refDer = completeReferenceDerivatives(ref);
    [omegaRef, alphaRef] = talPaperFlatnessFeedforward(ff.R, ff.tau, refDer, par);

    % Eq. (28): attitude/rate controller. xiE is Eq. (27), computed from the
    % incremental quaternion command of Eq. (22)-(26).
    omegaDotCmd = par.tal.Ktheta*xiE ...
                + par.tal.Komega*(omegaRef - omegaF) ...
                + alphaRef;

    % Eq. (31): INDI angular acceleration control. The paper uses filtered
    % motor-speed-derived moment mu_f; with direct moment actuation, muF is
    % the LPF output of previous saturated equivalent moments [N*m].
    tau = muF + par.J*(omegaDotCmd - omegaDotF);

    u = talControlAllocation(T, tau, Rd, par);

    st.v = x.v;
    st.R = x.R;
    st.Omega = x.Omega;
    st.T = u.T;
    st.tau = u.tau;
    u.OmegaD = omegaRef;
    u.alphaD = alphaRef;

    st.omegaDotF = omegaDotF;
    st.omegaF = omegaF;
    st.aFilt = aFilt;
    st.thrustAccelF = thrustAccelF;
    st.muF = muF;
    st.Rd = Rd;
    st.t = t;
end

function [Rd, xiE, T] = talIncrementalAttitudeCommand(R, thrustAccelCmd, psiRef, par)

    % Tal Eq. (21)-(27). thrustAccelCmd is (tau*b_z)_c in inertial NED.
    % Since tau is negative for upward thrust, the commanded body z axis is
    % b_z,c = -normalize((tau*b_z)_c).
    thrustDir = thrustAccelCmd/norm(thrustAccelCmd);
    thrustDirBody = R' * thrustDir;       % Eq. (22): (-b_z)_c in body frame.

    qTilt = talQuatAlignMinusE3(thrustDirBody);
    RTilt = quatToRotmWXYZ(qTilt);
    RIntermediate = R * RTilt;

    nPsi = [sin(psiRef); -cos(psiRef); 0];
    nBody = RIntermediate' * nPsi;        % Eq. (24).

    if nBody(2) == 0
        qYaw = [1; 0; 0; 0];
    else
        k = -nBody(1)/nBody(2);
        qYaw = [1; 0; 0; k/(1 + sqrt(1 + k^2))];
        qYaw = normalizeQuatWXYZ(qYaw);
    end

    qCmd = quatMultiplyWXYZ(qTilt, qYaw); % Eq. (26): current to command.
    qCmd = normalizeQuatWXYZ(qCmd);
    RCmd = quatToRotmWXYZ(qCmd);

    Rd = R * RCmd;
    xiE = talQuatErrorVector(qCmd);       % Eq. (27).
    % Eq. (21): paper uses signed T_c = -m*||(tau*b_z)_c||. The benchmark
    % actuator interface expects positive force magnitude, so only the sign
    % convention is converted here.
    T = par.m*norm(thrustAccelCmd);
end

function q = talQuatAlignMinusE3(vBody)

    vBody = vBody/norm(vBody);
    e3 = [0; 0; 1];
    c = dot(e3, vBody);
    axis = -cross(e3, vBody);

    % Tal Eq. (23) aligns current -b_z with (tau*b_z)_c. It is singular when
    % i_z = (-b_z)_c^b, i.e. a 180 deg tilt with arbitrary rotation axis.
    % The paper explicitly resolves this by selecting any direction of
    % rotation. Use a fixed body-x axis near that singularity. If
    % vBody == -i_z, the required tilt is zero.
    if c == 1
        q = [0; 1; 0; 0];
    elseif c == -1
        q = [1; 0; 0; 0];
    else
        q = [1 - c; axis];
        q = normalizeQuatWXYZ(q);
    end
end

function xiE = talQuatErrorVector(q)

    % Tal Eq. (27) is the quaternion logarithm written as
    %   xi_e = 2*acos(q_w)/sqrt(1-q_w^2) * q_v.
    xiE = 2*acos(q(1))/sqrt(1 - q(1)^2) * q(2:4);
end

function [omegaRef, alphaRef] = talPaperFlatnessFeedforward(R, tauSpec, refDer, par)

    % Tal Eq. (14)-(15). The 4x4 matrix solves for [tau_dot; omega_ref] and
    % [tau_ddot; omega_dot_ref] from reference jerk, snap, yaw rate, and yaw
    % acceleration, using Eq. (11)-(13)'s yaw-rate row.
    A = talFlatnessMatrix(R, tauSpec);

    yJerk = [refDer.j; refDer.psiDot];
    solJerk = A\yJerk;
    tauDotRef = solJerk(1);
    omegaRef = solJerk(2:4);

    omegaHat = hat(omegaRef);
    knownSnap = R*(2*tauDotRef*omegaHat + tauSpec*omegaHat*omegaHat)*par.e3;
    sDotOmega = talYawSdotOmega(R, omegaRef);
    ySnap = [refDer.s - knownSnap;
             refDer.psiDDot - sDotOmega];

    solSnap = A\ySnap;
    alphaRef = solSnap(2:4);
end

function A = talFlatnessMatrix(R, tauSpec)

    % Tal Eq. (14): j = R*[tau_dot*e3 + tau*hat(omega)*e3],
    % psi_dot = S(R)*omega.
    b3 = R*[0; 0; 1];
    S = talYawSRow(R);
    A = [b3, -tauSpec*R*hat([0; 0; 1]);
         0,  S];
end

function S = talYawSRow(R)

    % Tal Eq. (11)-(13): psi = atan2(b_x,y, b_x,x), psi_dot = S(R)*omega.
    % The denominator is intentionally not regularized; when the horizontal
    % projection of b_x vanishes, the paper's yaw parametrization is singular.
    bx = R(:,1);
    den = bx(1)^2 + bx(2)^2;

    S = zeros(1,3);
    for i = 1:3
        e = zeros(3,1);
        e(i) = 1;
        bxDot = R*hat(e)*[1; 0; 0];
        S(i) = (bx(1)*bxDot(2) - bx(2)*bxDot(1))/den;
    end
end

function y = talYawSdotOmega(R, omega)

    % Tal Eq. (15): psi_ddot = S(R)*omega_dot + S_dot(R,omega)*omega.
    % Differentiate Eq. (11)-(13) analytically along R_dot = R*hat(omega).
    bx = R(:,1);
    bxDot = R*hat(omega)*[1; 0; 0];
    den = bx(1)^2 + bx(2)^2;
    denDot = 2*bx(1)*bxDot(1) + 2*bx(2)*bxDot(2);

    Sdot = zeros(1,3);
    for i = 1:3
        e = zeros(3,1);
        e(i) = 1;
        bxOmegaBasis = R*hat(e)*[1; 0; 0];
        bxOmegaBasisDot = R*hat(omega)*hat(e)*[1; 0; 0];

        num = bx(1)*bxOmegaBasis(2) - bx(2)*bxOmegaBasis(1);
        numDot = bxDot(1)*bxOmegaBasis(2) ...
               + bx(1)*bxOmegaBasisDot(2) ...
               - bxDot(2)*bxOmegaBasis(1) ...
               - bx(2)*bxOmegaBasisDot(1);
        Sdot(i) = (numDot*den - num*denDot)/den^2;
    end

    y = Sdot*omega;
end

function u = talControlAllocation(T, mu, Rd, par)

    % Tal Eq. (34)-(35), adapted to the benchmark actuator variable. The
    % benchmark input is already per-rotor thrust proportional to omega^2 and
    % par.allocation.B maps it to [-T; mu], so Eq. (35) is a direct inverse.
    % When the inverse is infeasible, follow Eq. (34)'s priority: keep roll
    % and pitch moments, adjust yaw moment first, then collective thrust.
    [TAlloc, muAllocCmd] = talResolveAllocationSaturation(T, mu, par);
    muCmd = [-TAlloc; muAllocCmd(:)];
    u.Rd = Rd;
    u.actuator = min(max(par.allocation.B\muCmd, ...
        par.allocation.uMin(:)), par.allocation.uMax(:));
    u.muAllocated = par.allocation.B*u.actuator(:);
    u.T = -u.muAllocated(1);
    u.tau = u.muAllocated(2:4);
end

function [TAlloc, muAlloc] = talResolveAllocationSaturation(T, mu, par)

    mu = mu(:);
    TAlloc = T;
    muAlloc = mu;

    [yawLo, yawHi, feasible] = talYawMomentInterval(TAlloc, muAlloc(1:2), par);
    if feasible
        muAlloc(3) = clampScalar(mu(3), yawLo, yawHi);
        return;
    end

    % If yaw-only adjustment is infeasible, preserve roll/pitch and search the
    % collective thrust that gives the nearest feasible Eq. (35) inverse.
    Tmax = sum(par.allocation.uMax(:));
    TGrid = linspace(0, Tmax, 1201);
    bestCost = inf;
    bestT = clampScalar(T, 0, Tmax);
    bestYaw = mu(3);

    for i = 1:numel(TGrid)
        Ti = TGrid(i);
        [lo, hi, ok] = talYawMomentInterval(Ti, mu(1:2), par);
        if ~ok
            continue;
        end

        yawI = clampScalar(mu(3), lo, hi);
        cost = ((Ti - T)/max(Tmax, eps))^2 ...
             + ((yawI - mu(3))/max(par.tauMax(3), eps))^2;
        if cost < bestCost
            bestCost = cost;
            bestT = Ti;
            bestYaw = yawI;
        end
    end

    if isfinite(bestCost)
        TAlloc = bestT;
        muAlloc(3) = bestYaw;
        return;
    end

    % No exact Eq. (35) solution can preserve roll/pitch within bounds. Leave
    % the command unchanged and let the final actuator clamp expose the
    % remaining infeasibility instead of inventing another priority rule.
    TAlloc = clampScalar(T, 0, Tmax);
    muAlloc = mu;
end

function [yawLo, yawHi, feasible] = talYawMomentInterval(T, muXY, par)

    invB = par.allocation.B\eye(4);
    fixedWrench = [-T; muXY(:); 0];
    actuatorAtZeroYaw = invB*fixedWrench;
    yawColumn = invB(:,4);

    yawLo = -inf;
    yawHi = inf;
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);

    for i = 1:numel(yawColumn)
        c = yawColumn(i);
        if c == 0
            if actuatorAtZeroYaw(i) < lb(i) || actuatorAtZeroYaw(i) > ub(i)
                feasible = false;
                yawLo = inf;
                yawHi = -inf;
                return;
            end
            continue;
        end

        y1 = (lb(i) - actuatorAtZeroYaw(i))/c;
        y2 = (ub(i) - actuatorAtZeroYaw(i))/c;
        yawLo = max(yawLo, min(y1, y2));
        yawHi = min(yawHi, max(y1, y2));
    end

    feasible = yawLo <= yawHi;
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

    solveDue = ~isfield(st, 'lastActuator') ...
        || t + 0.5*par.dt >= st.nextSolveTime;
    if ~solveDue
        u = sunActuatorToControl(st.lastActuator, refs.R(:,:,1), par);
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
    actuator = double(pyResult{'u0'});
    actuator = actuator(:);
    solveTime = double(pyResult{'solve_time'});

    u = sunActuatorToControl(actuator, refs.R(:,:,1), par);
    u.sunNMPCCached = false;
    u.sunNMPCSolved = true;
    u.sunNMPCStatusCode = status;
    u.sunNMPCExitflag = status;
    u.sunNMPCSolveTime = solveTime;

    if cfg.printSolverTiming
        fprintf('sun_nmpc t=%.3f solve=%.4fs status=%d\n', ...
            t, solveTime, status);
    end

    st.lastActuator = actuator;
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
        setenv("MPLCONFIGDIR", char(fullfile(tempdir, "matplotlib")));

        toolsPath = char(par.sun.acadosToolsPath);
        if exist(toolsPath, 'dir') ~= 7
            mainPath = which('main');
            if strlength(mainPath) > 0
                toolsPath = fullfile(fileparts(mainPath), 'tools');
            end
        end

        pyPath = py.sys.path;
        pyPath.insert(int32(0), toolsPath);
        acadosTemplatePath = fullfile(char(par.sun.acadosSourceDir), ...
            "interfaces", "acados_template");
        if exist(acadosTemplatePath, 'dir') == 7
            pyPath.insert(int32(0), char(acadosTemplatePath));
        end
        try
            py.importlib.invalidate_caches();
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

    sunConfigureAcadosSolver(pyModule, par);
    pySolver = pyModule;
end

function sunConfigureAcadosSolver(pyModule, par)

    G1 = sunAllocationMatrix(par);

    codegenDir = fullfile(tempdir, "uav_sun_acados_codegen");
    pyModule.configure(pyargs( ...
        'n_horizon', int32(par.sun.N), ...
        'dt', double(par.sun.dt), ...
        'mass', double(par.m), ...
        'gravity', double(par.g), ...
        'inertia_diag', py.numpy.array(diag(par.J)'), ...
        'allocation_matrix', py.numpy.array(G1), ...
        'u_min', py.numpy.array(par.allocation.uMin'), ...
        'u_max', py.numpy.array(par.allocation.uMax'), ...
        'omega_max', py.numpy.array(par.sun.omegaMax'), ...
        'aero_enabled', logical(par.aero.enabled), ...
        'aero_kd', py.numpy.array(par.aero.kd(:)'), ...
        'aero_kh', double(par.aero.kh), ...
        'q_xi', py.numpy.array(par.sun.Qxi), ...
        'q_v', py.numpy.array(par.sun.Qv), ...
        'q_q', py.numpy.array(par.sun.Qq), ...
        'q_omega', py.numpy.array(par.sun.QOmega), ...
        'q_u', py.numpy.array(par.sun.Qu), ...
        'code_export_dir', char(codegenDir)));
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
    % as four actuator thrusts. The prediction reference may run past par.Tend
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
        ff = sunAeroFlatnessReference(ref, par);

        refs.p(:,k) = ref.p;
        refs.v(:,k) = ref.v;
        refs.R(:,:,k) = ff.R;
        refs.q(:,k) = rotmToQuatWXYZ(ff.R);
        refs.Omega(:,k) = ff.Omega;
        refs.alpha(:,k) = ff.alpha;
        refs.T(k) = ff.T;
    end

    for k = 1:N+1
        refs.tau(:,k) = par.J*refs.alpha(:,k) ...
            + cross(refs.Omega(:,k), par.J*refs.Omega(:,k));
        refs.u(:,k) = sunDirectAllocation([refs.T(k); refs.tau(:,k)], par);
    end
end

function xVec = sunNMPCStateVector(x)

    xVec = [x.p;
            rotmToQuatWXYZ(x.R);
            x.v;
            x.Omega];
end

function u = controlAllocation(mu, Rd, par)

    muCmd = [-mu(1); mu(2:4)];
    u.Rd = Rd;
    u.actuator = allocateActuator(muCmd, par);

    % Actuator dynamics are neglected here: after saturation, the actuator
    % vector is multiplied by the allocation matrix to obtain the wrench
    % applied to the rigid-body model.
    u.muAllocated = par.allocation.B*u.actuator(:);
    u.T = -u.muAllocated(1);
    u.tau = u.muAllocated(2:4);
end

function actuator = allocateActuator(muCmd, par)

    switch par.allocation.method
        case "pinv"
            actuator = allocateActuatorPinv(muCmd, par);
        case "wls"
            actuator = allocateActuatorWLS(muCmd, par);
        otherwise
            error("Unknown allocation method.");
    end
end

function actuator = allocateActuatorPinv(muCmd, par)

    actuator = min(max(par.allocation.B\muCmd(:), ...
        par.allocation.uMin(:)), par.allocation.uMax(:));
end

function actuator = allocateActuatorWLS(muCmd, par)

    % Sun Eq. (29), implemented as weighted bounded least squares on this
    % benchmark's internal wrench convention [-T; tau]. The sign of the first
    % channel is immaterial in the squared weighted residual. wls_alloc solves
    % ||Wv*(B*u-v)||^2, so choose Wv such that Wv'*Wv equals the paper's W.
    B = par.allocation.B;
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    Wpaper = 0.5*(par.allocation.W + par.allocation.W');
    [Wv, cholFlag] = chol(Wpaper);
    if cholFlag ~= 0
        error("par.allocation.W must be positive definite for WLS allocation.");
    end
    Wu = zeros(numel(lb), numel(lb));
    ud = zeros(numel(lb),1);
    gamma = 1;
    u0 = allocateActuatorPinv(muCmd, par);
    W0 = zeros(numel(lb),1);

    actuator = wls_alloc(B, muCmd(:), lb, ub, Wv, Wu, ud, gamma, u0, W0, 100);
    actuator = min(max(actuator(:), lb), ub);
end

function u = controlAllocationPX4Normalized(muNormCmd, Rd, par)

    % muNormCmd is [Fz; Mx; My; Mz], matching B_px4_norm.
    u.Rd = Rd;
    u.actuator = min(max(par.allocation.B_px4_norm\muNormCmd(:), 0), 1);

    % Actuator dynamics are neglected here: after saturation, the normalized
    % actuator vector is multiplied by the physical PX4 allocation matrix.
    u.muAllocated = par.allocation.B_px4*u.actuator(:);
    u.T = -u.muAllocated(1);
    u.tau = u.muAllocated(2:4);
end

function G1 = sunAllocationMatrix(par)

    % Sun uses [T; tau] = G1*u. The benchmark allocation matrix stores
    % [-T; tau], so keep the original geometry and only change the first sign.
    G1 = [-par.allocation.B(1,:); par.allocation.B(2:4,:)];
end

function uControl = sunActuatorToControl(actuator, Rd, par)

    actuator = actuator(:);
    mu = sunAllocationMatrix(par)*actuator;

    uControl.Rd = Rd;
    uControl.actuator = actuator;
    uControl.muAllocated = [-mu(1); mu(2:4)];
    uControl.T = mu(1);
    uControl.tau = mu(2:4);
end

function uControl = sunDFBCControlAllocation(mu, Rd, par)

    % Sun Eq. (31): bounded weighted QP allocation from desired collective
    % thrust and desired angular acceleration torque channel.
    actuator = sunQPAllocation(mu, par);
    uControl = sunActuatorToControl(actuator, Rd, par);
end

function actuator = sunDirectAllocation(mu, par)

    % Sun Eq. (29): direct inversion with G2/G3 omitted.
    actuator = sunAllocationMatrix(par)\mu(:);
end

function actuator = sunBoundedDirectAllocation(mu, par)

    % Sun Eq. (30): direct inversion followed by actuator bounds.
    actuator = min(max(sunDirectAllocation(mu, par), ...
        par.allocation.uMin(:)), par.allocation.uMax(:));
end

function actuator = sunQPAllocation(mu, par)

    G1 = sunAllocationMatrix(par);
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    Wpaper = 0.5*(par.allocation.W + par.allocation.W');
    [Wv, cholFlag] = chol(Wpaper);
    if cholFlag ~= 0
        error("par.allocation.W must be positive definite for Sun Eq. (31).");
    end

    Wu = zeros(numel(lb), numel(lb));
    ud = zeros(numel(lb),1);
    gamma = 1;
    u0 = sunBoundedDirectAllocation(mu, par);
    W0 = zeros(numel(lb),1);

    actuator = wls_alloc(G1, mu(:), lb, ub, Wv, Wu, ud, gamma, u0, W0, 100);
    actuator = actuator(:);
end

function u = controllerSunDFBC(x, ref, traj, t, par)

    cmd = sunDFBCCommand(x, ref, traj, t, par);
    u = sunActuatorToControl(cmd.actuator, cmd.Rd, par);
    u.OmegaD = cmd.OmegaR;
    u.alphaD = cmd.alphaR;
end

function u = controllerSunDFBCINDI(x, ref, traj, t, par)

    persistent st

    cmd = sunDFBCCommand(x, ref, traj, t, par);
    [u, st] = sunINDIActuatorControl(x, cmd, t, par, st);
    u.OmegaD = cmd.OmegaR;
    u.alphaD = cmd.alphaR;
end

function u = controllerSunNMPCINDI(x, ref, traj, t, par)

    persistent st

    uMpc = controllerSunNMPC(x, ref, traj, t, par);
    cmd = sunCommandFromActuator(uMpc.actuator, uMpc.Rd, x, par);
    [u, st] = sunINDIActuatorControl(x, cmd, t, par, st);

    u.sunNMPCCached = uMpc.sunNMPCCached;
    u.sunNMPCSolved = uMpc.sunNMPCSolved;
    u.sunNMPCStatusCode = uMpc.sunNMPCStatusCode;
    u.sunNMPCExitflag = uMpc.sunNMPCExitflag;
    u.sunNMPCSolveTime = uMpc.sunNMPCSolveTime;
end

function cmd = sunDFBCCommand(x, ref, ~, t, par)

    persistent st

    % Sun et al. Eq. (13): desired acceleration from PD position feedback.
    xiErr = ref.p - x.p;
    vErr = ref.v - x.v;
    accD = par.sun.Kxi*xiErr + par.sun.Kv*vErr + ref.a;

    % Sun et al. Eq. (14)-(17), converted from the paper's convention where
    % z_B is the thrust direction to this NED/FRD model where R*e3 is opposite
    % thrust: a = g*e3 - T/m*R*e3 + R*f_a^B/m. Thus
    % T*R*e3 = m*(g*e3 - xi_ddot_d) + R*f_a^B.
    fAeroWorld = sunAeroForceWorld(x.R, x.v, par);
    thrustAxisForce = par.m*(par.g*par.e3 - accD) + fAeroWorld;
    [Rd, T] = sunDesiredAttitudeFromThrustVector(thrustAxisForce, ref.psi);

    resetState = isempty(st) || t <= par.dt/2 || t <= st.t;
    if resetState || ~isfield(st, 'T')
        TForFlatness = T;
    else
        TForFlatness = st.T;
    end

    % Sun et al. Eq. (18)-(24): use the current attitude/angular velocity and
    % the currently applied collective thrust. In this direct-actuator
    % benchmark, the previous allocated thrust is the available thrust
    % measurement; the first sample falls back to the newly requested T.
    [OmegaR, alphaR] = sunFlatnessReferenceRates(x, ref, TForFlatness, par);

    % Sun et al. Eq. (25): for B-to-I attitude, use q_e = q^{-1} \otimes q_d
    % so the reduced/yaw tangent vectors below are body-local errors.
    qd = rotmToQuatWXYZ(Rd);
    q = rotmToQuatWXYZ(x.R);
    qe = quatMultiplyWXYZ(quatConjugateWXYZ(q), qd);
    qe = qe/norm(qe);

    % Sun et al. Eq. (26)-(27): split reduced-attitude and yaw errors.
    den = sqrt(qe(1)^2 + qe(4)^2);
    qRed = [qe(1)*qe(2) - qe(3)*qe(4);
            qe(1)*qe(3) + qe(2)*qe(4);
            0]/den;
    qYaw = [0; 0; qe(4)]/den;

    % Sun et al. Eq. (28): tilt-prioritized attitude control.
    yawSign = 1;
    if qe(1) < 0
        yawSign = -1;
    end

    alphaD = par.sun.KqRed*qRed + par.sun.kqYaw*yawSign*qYaw ...
           + par.sun.KOmega*(OmegaR - x.Omega) + alphaR;

    tauDesired = par.J*alphaD + cross(x.Omega, par.J*x.Omega);
    uAlloc = sunDFBCControlAllocation([T; tauDesired], Rd, par);

    cmd.actuator = uAlloc.actuator;
    % Sun Eq. (32): after constrained allocation, retrieve the actually
    % achievable collective thrust and angular acceleration for INDI.
    cmd.T = uAlloc.T;
    cmd.alpha = par.J \ (uAlloc.tau ...
        - cross(x.Omega, par.J*x.Omega));
    cmd.Rd = Rd;
    cmd.OmegaR = OmegaR;
    cmd.alphaR = alphaR;
    cmd.alphaD = alphaD;

    st.T = uAlloc.T;
    st.t = t;
end

function cmd = sunCommandFromActuator(actuator, Rd, x, par)

    u = sunActuatorToControl(actuator, Rd, par);
    cmd.actuator = u.actuator;
    cmd.T = u.T;
    cmd.alpha = par.J \ (u.tau - cross(x.Omega, par.J*x.Omega));
    cmd.Rd = Rd;
end

function [Rd, T] = sunDesiredAttitudeFromThrustVector(thrustAxisForce, psi)

    % Sun Eq. (14)-(17), expressed in this NED/FRD benchmark convention.
    % No fallback is applied: zero thrust or yaw-axis degeneracy is left as
    % the paper's mapping produces it.
    T = norm(thrustAxisForce);
    zBd = thrustAxisForce/T;
    xCd = [cos(psi); sin(psi); 0];
    yBd = cross(zBd, xCd)/norm(cross(zBd, xCd));
    xBd = cross(yBd, zBd);
    Rd = [xBd, yBd, zBd];
end

function [u, st] = sunINDIActuatorControl(x, cmd, t, par, st)

    % Sun et al. Eq. (32)-(35). The MATLAB plant has no motor-speed state;
    % since the benchmark actuator variable is already u=c_t*omega^2, the
    % previous allocated actuator vector stands in for filtered rotor-thrust
    % feedback. The unknown d_tau in the paper is the plant disturbance here
    % and is not predicted or canceled.

    resetState = isempty(st) || t <= par.dt/2 || t <= st.t;
    if resetState
        omegaDotF = zeros(3,1);
        actuatorF = cmd.actuator;
        st = struct;
        st.omegaFilter = initSecondOrderLPF(x.Omega);
        st.actuatorFilter = initSecondOrderLPF(cmd.actuator);
        st.omegaF = x.Omega;
    else
        h = max(t - st.t, par.dt);
        rawActuator = st.actuator;
        [omegaF, st.omegaFilter] = secondOrderButterworthLPF( ...
            x.Omega, st.omegaFilter, h, par.sun.filterCutoffHz);
        [actuatorF, st.actuatorFilter] = secondOrderButterworthLPF( ...
            rawActuator, st.actuatorFilter, h, par.sun.filterCutoffHz);
        omegaDotF = (omegaF - st.omegaF)/h;
        st.omegaF = omegaF;
    end

    muF = sunAllocationMatrix(par)*actuatorF;
    tauF = muF(2:4);

    mu = zeros(4,1);
    mu(1) = cmd.T;
    mu(2:4) = tauF + par.J*(cmd.alpha - omegaDotF);

    u = sunActuatorToControl(sunDirectAllocation(mu, par), cmd.Rd, par);

    st.actuator = u.actuator;
    st.t = t;
end

function [OmegaR, alphaR] = sunFlatnessReferenceRates(x, ref, T, par)

    ref = completeReferenceDerivatives(ref);

    b1 = x.R(:,1);
    b2 = x.R(:,2);
    b3 = x.R(:,3);       % body z-down axis; thrust acceleration is -T/m*b3.
    omegaWorld = x.R*x.Omega;

    % Sun Eq. (18)-(24), rewritten for the NED plant
    %   a = g*e3 - T/m*b3.
    % Therefore m*j = -Tdot*b3 - T*(Omega x b3).
    TDot = -par.m*dot(ref.j, b3);
    hOmega = (-par.m*ref.j - TDot*b3)/T;
    OmegaR = [-dot(hOmega, b2);
               dot(hOmega, b1);
               ref.psiDot*dot(par.e3, b3)];

    % Differentiating again:
    % m*s = -Tddot*b3 - 2*Tdot*hOmega
    %       - T*(alpha x b3 + Omega x hOmega).
    TDDot = -par.m*dot(ref.s, b3) ...
        - par.m*dot(cross(omegaWorld, b3), ref.j);
    hAlpha = -(par.m/T)*ref.s ...
        - cross(omegaWorld, hOmega) ...
        - 2*(TDot/T)*hOmega ...
        - (TDDot/T)*b3;
    alphaR = [-dot(hAlpha, b2);
               dot(hAlpha, b1);
               ref.psiDDot*dot(par.e3, b3)];
end

function u = controllerLuOnManifoldMPC(x, ref, traj, t, par)

    persistent st

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
    %   Lu's UAV experiment runs this MPC at 100 Hz. The benchmark inner
    %   body-rate adapter below still runs at par.dt, holding the latest MPC
    %   command between solves.
    %
    %   Special handling: Lu's experiment sends aT and body-rate omega to a
    %   PX4 rate controller. This benchmark plant accepts force/moment, so
    %   luMpcRateLoop adapts Omega_cmd to tau before the unified allocator.
    resetState = isempty(st) || t <= par.dt/2 || t <= st.t;
    solveDue = resetState || t + 0.5*par.dt >= st.nextSolveTime;

    if solveDue
        [Rd, aTd, OmegaD] = referenceInputOnManifold(ref, par);

        refs = onManifoldMPCReferences(ref, traj, t, par);
        du = solveOnManifoldMPC(x, refs, par);

        aTCmd = aTd + du(1);
        OmegaCmd = OmegaD + du(2:4);
        aTCmd = min(max(aTCmd, 0), par.Tmax/par.m);
        OmegaCmd = saturateVector(OmegaCmd, par.mpc.omegaMax);

        st.aTCmd = aTCmd;
        st.OmegaCmd = OmegaCmd;
        st.Rd = Rd;
        st.nextSolveTime = t + par.mpc.dt;
    else
        aTCmd = st.aTCmd;
        OmegaCmd = st.OmegaCmd;
        Rd = st.Rd;
    end

    % Lu Eq. (14)-(16) outputs thrust acceleration and body rate. This
    % benchmark plant accepts force/moment, so Omega_cmd is adapted through
    % the configured body-rate loop before unified actuator allocation.
    tau = luMpcRateLoop("compute", x, OmegaCmd, t, par, []);
    u = controlAllocation([par.m*aTCmd; tau], Rd, par);
    luMpcRateLoop("commit", x, OmegaCmd, t, par, u);

    st.t = t;
end

function tau = luMpcRateLoop(action, x, OmegaCmd, t, par, uApplied)

    persistent st

    tau = zeros(3,1);

    switch string(action)
        case "compute"
            resetState = isempty(st) || t <= par.dt/2 || t <= st.t;
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
            st.Omega = x.Omega;
            st.eOmega = st.pendingEOmega;
            st.eInt = st.pendingEInt;
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
        u = controlAllocation([T; tau], Rd, par);

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

    u = controlAllocation([T; tau], Rd, par);

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

    h = par.mpc.dt;
    for k = 2:N+1
        tk = t + (k-1)*h;
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
    h = par.mpc.dt;

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
    omegaMax = par.mpc.omegaMax(:);
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

function ff = sunAeroFlatnessReference(ref, par)

    ref = completeReferenceDerivatives(ref);

    thrustAxisForce = par.m*(par.g*par.e3 - ref.a);
    [R, T, Omega, alpha] = sunAttitudeFromThrustDerivatives( ...
        thrustAxisForce, -par.m*ref.j, -par.m*ref.s, ...
        ref.psi, ref.psiDot, ref.psiDDot);

    for i = 1:3
        if ~isfield(par, 'aero') || ~par.aero.enabled
            break;
        end

        aeroForceWorld = sunAeroForceWorld(R, ref.v, par);
        thrustAxisForce = par.m*(par.g*par.e3 - ref.a) + aeroForceWorld;
        [R, T, Omega, alpha] = sunAttitudeFromThrustDerivatives( ...
            thrustAxisForce, -par.m*ref.j, -par.m*ref.s, ...
            ref.psi, ref.psiDot, ref.psiDDot);
    end

    ff.R = R;
    ff.T = T;
    ff.c = T/par.m;
    ff.Omega = Omega;
    ff.alpha = alpha;
end

function [R, T, Omega, alpha] = sunAttitudeFromThrustDerivatives( ...
        thrustAxisForce, thrustAxisForceDot, thrustAxisForceDDot, ...
        psi, psiDot, psiDDot)

    % Sun Eq. (14)-(17), differentiated analytically for NMPC references.
    % The paper mapping is used as-is; singular force/heading cases are not
    % regularized here.
    [zBd, zBdDot, zBdDDot] = sunNormalizeWithDerivatives( ...
        thrustAxisForce, thrustAxisForceDot, thrustAxisForceDDot);
    T = norm(thrustAxisForce);

    xCd = [cos(psi); sin(psi); 0];
    xCdDot = psiDot*[-sin(psi); cos(psi); 0];
    xCdDDot = psiDDot*[-sin(psi); cos(psi); 0] - psiDot^2*xCd;

    yBdRaw = cross(zBd, xCd);
    yBdRawDot = cross(zBdDot, xCd) + cross(zBd, xCdDot);
    yBdRawDDot = cross(zBdDDot, xCd) ...
        + 2*cross(zBdDot, xCdDot) + cross(zBd, xCdDDot);
    [yBd, yBdDot, yBdDDot] = sunNormalizeWithDerivatives( ...
        yBdRaw, yBdRawDot, yBdRawDDot);

    xBd = cross(yBd, zBd);
    xBdDot = cross(yBdDot, zBd) + cross(yBd, zBdDot);
    xBdDDot = cross(yBdDDot, zBd) ...
        + 2*cross(yBdDot, zBdDot) + cross(yBd, zBdDDot);

    R = [xBd, yBd, zBd];
    RDot = [xBdDot, yBdDot, zBdDot];
    RDDot = [xBdDDot, yBdDDot, zBdDDot];

    Omega = vee(R' * RDot);
    alpha = vee(R' * RDDot - hat(Omega)*hat(Omega));
end

function [u, uDot, uDDot] = sunNormalizeWithDerivatives(x, xDot, xDDot)

    r = norm(x);
    u = x/r;
    rDot = dot(u, xDot);
    uDot = (xDot - u*rDot)/r;
    rDDot = dot(uDot, xDot) + dot(u, xDDot);
    uDDot = (xDDot - u*rDDot - 2*rDot*uDot)/r;
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

%注意：有三类奇异点
% 1.如果总推力为零，平移动力学不再约束姿态，R_d不是由平坦输出唯一决定的。——这也是算法特性，不要解决。
% 比如 Mellinger 的微分平坦性求角速度公式中 hω 就含有 m/u1，因此 u1=0 时必然退化。
% 
% 2.第二类是 yaw 构造奇异。——我们不打算解决这个问题，我们认为这是算法特性。
% 例如：定义x_C= [cos(yaw_r);sin(yaw_r);0],期望z轴 z_B x x_C 不能为0， 否则分母为零。z_B x x_C !=0时才能唯一确定 R。
% 又例如tal，如果用ψ=atan2(^1_y,b^1_x),那么当 b1的水平投影为零时，yaw 不可定义。
% 还有其他使用y_C= [-sin(yaw_r);cos(yaw_r);0]来定义yaw方向的，则在另一个方向有奇异性。
% Tal 和 Karaman 的 S 矩阵中有rψTrψ这样的分母；当 bx在水平面的投影消失时，yaw rate 映射退化。
% 
% 3.第三类是轨迹光滑性不足或动力学不可行。若要计算角速度，需要位置至少三阶可导；——这个在我们这里似乎没问题？因为都是解析的基准曲线。
% 若要计算角加速度，需要位置至少四阶可导，yaw 至少二阶可导。Tal 和 Karaman 明确要求 x_ref ∈ C^4、ψ_ref ∈ C^2。

    T_b_z = -par.m*(aCmd - par.g*par.e3); % -f_d. desired force along body z_B
    % 如果总推力为零，平移动力学不再约束姿态，R_d不是由平坦输出唯一决定的。
    if norm(T_b_z) < 1e-9
        T_b_z = par.m*par.g*par.e3; 
    end

    % T = dot(T_b_z, RCurrent*par.e3); % option 1: current-attitude projection
    T = norm(T_b_z);                 % option 2: desired-force magnitude

    Rd = attitudeFromThrustDirection(T_b_z/norm(T_b_z), psi);
    % 这里需要解决norm(T_b_z)=0的问题，参考轨迹也需要解决这个问题，Mellinger 的角速度公式中 hω 就含有 m/u1，因此 u1=0 时必然退化。
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

    [a, OmegaDot] = rigidBodyRates(R, v, Omega, u, par, t);

    yDot = [v;
            a;
            reshape(R*hat(Omega), 9, 1);
            OmegaDot];
end

function xNext = stepModelLieRK4(x, u, par, t0)

    h = par.dt;

    Om1 = x.Omega;
    [a1, OmDot1] = rigidBodyRates(x.R, x.v, Om1, u, par, t0);

    v2 = x.v + 0.5*h*a1;
    R2 = x.R*expm(0.5*h*hat(Om1));
    Om2 = x.Omega + 0.5*h*OmDot1;
    [a2, OmDot2] = rigidBodyRates(R2, v2, Om2, u, par, t0 + 0.5*h);

    v3 = x.v + 0.5*h*a2;
    R3 = x.R*expm(0.5*h*hat(Om2));
    Om3 = x.Omega + 0.5*h*OmDot2;
    [a3, OmDot3] = rigidBodyRates(R3, v3, Om3, u, par, t0 + 0.5*h);

    v4 = x.v + h*a3;
    R4 = x.R*expm(h*hat(Om3));
    Om4 = x.Omega + h*OmDot3;
    [a4, OmDot4] = rigidBodyRates(R4, v4, Om4, u, par, t0 + h);

    OmegaBar = (Om1 + 2*Om2 + 2*Om3 + Om4)/6;

    xNext.p = x.p + h/6*(x.v + 2*v2 + 2*v3 + v4);
    xNext.v = x.v + h/6*(a1 + 2*a2 + 2*a3 + a4);
    xNext.R = x.R*expm(h*hat(OmegaBar));
    xNext.Omega = x.Omega + h/6*(OmDot1 + 2*OmDot2 + 2*OmDot3 + OmDot4);
end

function [a, OmegaDot] = rigidBodyRates(R, v, Omega, u, par, t)

    if nargin < 5
        t = 0;
    end

    [forceDist, momentDist] = disturbanceAtTime(t, par);
    fAeroWorld = sunAeroForceWorld(R, v, par);

    a = par.g*par.e3 - u.T/par.m*R*par.e3 ...
        + (fAeroWorld + forceDist)/par.m;
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

    figure;

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

    figure;

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
    [omegaRef, alphaRef] = desiredAngularDerivativesForPlot(log, time);
    alphaActual = loggedAngularAcceleration(log, par);

    figure;

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

    figure;

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

function [omegaRef, alphaRef] = desiredAngularDerivativesForPlot(log, time)

    [omegaRef, alphaRef] = rotationLogRates(log.Rd, time);

    if isfield(log, 'OmegaDProvided')
        mask = log.OmegaDProvided;
        omegaRef(:,mask) = log.OmegaD(:,mask);
    end

    if isfield(log, 'alphaDProvided')
        mask = log.alphaDProvided;
        alphaRef(:,mask) = log.alphaD(:,mask);
    end
end

function acc = loggedLinearAcceleration(log, par)

    N = size(log.v, 2);
    acc = zeros(3, N);

    for k = 1:N
        acc(:,k) = par.g*par.e3 ...
            - log.T(k)/par.m*log.R(:,:,k)*par.e3 ...
            + (log.aeroForce(:,k) + log.forceDist(:,k))/par.m;
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

function st = initSecondOrderLPF(raw)

    raw = raw(:);
    st.x1 = raw;
    st.x2 = raw;
    st.y1 = raw;
    st.y2 = raw;
end

function [y, st] = secondOrderButterworthLPF(raw, st, h, cutoffHz)

    raw = raw(:);
    if cutoffHz <= 0 || h <= 0
        y = raw;
        st = initSecondOrderLPF(raw);
        return;
    end

    fs = 1/h;
    fc = min(cutoffHz, 0.45*fs);
    K = tan(pi*fc/fs);
    normFactor = 1/(1 + sqrt(2)*K + K^2);
    b0 = K^2*normFactor;
    b1 = 2*b0;
    b2 = b0;
    a1 = 2*(K^2 - 1)*normFactor;
    a2 = (1 - sqrt(2)*K + K^2)*normFactor;

    y = b0*raw + b1*st.x1 + b2*st.x2 - a1*st.y1 - a2*st.y2;

    st.x2 = st.x1;
    st.x1 = raw;
    st.y2 = st.y1;
    st.y1 = y;
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
