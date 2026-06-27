%% main_lee.m
% Simple quadrotor simulation with modular reference trajectories.
% Internal coordinate: NED, z_NED points downward.
%
% State:
%   p : position in NED
%   v : velocity in NED
%   R : body-to-NED rotation matrix
%   Omega : body angular velocity expressed in body frame
%
% Input after control allocation:
%   actuator : Lee uses rotor thrusts [N]; PX4 uses normalized commands [0,1].
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
% Iris Gazebo Classic plant, actuator, and geometry parameters, matching
% iris.m and gazebo_iris_model.m.
par.g = 9.81;
par.e3 = [0;0;1];
% Sun et al. 2022 Table II mass and inertia.
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

par.dt = 0.01;          % 100 Hz
par.Tend = 16.0;
par.integratorName = "ode45";  % "ode45" or "lie_rk4"

% Reference time scaling.
% scale > 1 slows the reference; scale < 1 speeds it up and may saturate control.
par.progress.mode = "scale_range";      % "scale_fixed" or "scale_range"
par.progress.scale = 1.0;               % scale_fixed: constant time scale
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
% Available choices:
%   "lee"       geometric Lee controller
%   "px4_iris"  PX4 Iris-style cascaded position/attitude/rate controller
par.controllerName = "px4_iris";
% Controller gains, using the DFBC gains in Sun et al. Table I as the
% quantity reference while keeping Lee's moment-control form.
par.Kp = diag([10, 10, 10]);
par.Kv = diag([6, 6, 6]);
par.KR = par.J*diag([150, 150, 3]);
par.KOmega = par.J*diag([20, 20, 8]);

% Controller-specific gain namespaces. The base gains above remain as
% convenient defaults, but controller code should use its own namespace so
% paper implementations can be tuned independently.
par.lee.Kp = par.Kp;
par.lee.Kv = par.Kv;
par.lee.KR = par.KR;
par.lee.KOmega = par.KOmega;

% PX4 Iris controller parameters. The structure follows the current PX4
% multicopter controller chain:
%   mc_pos_control -> mc_att_control -> mc_rate_control -> control allocation.
% The simplified linear plant uses the Iris airframe MPC_THR_HOVER and rate
% gain overrides from ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris.
par.px4iris.hoverThrust = 0.216;

% P-only PX4-style baseline tuned inside the PX4 parameter ranges for the
% aggressive benchmark trajectories. I/D fields are implemented below and
% can be enabled by overriding these values.
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

% Iris allocation matrices are generated below:
%   B        maps rotor thrusts [N] to [Fz; Mx; My; Mz] for Lee.
%   B_px4    maps normalized PX4 actuator commands to physical wrench.
%   B_px4_norm is the PX4-normalized control effectiveness matrix.
par.allocation.uMin = zeros(4,1);
par.allocation.uMax = par.CT*ones(4,1);
par.allocation.uNormMin = zeros(4,1);
par.allocation.uNormMax = ones(4,1);

% Additive plant disturbances. The force disturbance is expressed in inertial
% NED coordinates [N]; the moment disturbance is expressed in the body frame
% [N*m], matching the plant translational and rotational equations below.
% The default is disabled, so normal single-run behavior is unchanged.
par.disturbance.enabled = false;
par.disturbance.type = "none";       % "none", "constant", or "sin"
par.disturbance.forceAmp = zeros(3,1);   % per-axis amplitude [N]
par.disturbance.momentAmp = zeros(3,1);  % per-axis amplitude [N*m]
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

par = mergeStructRecursive(par, parOverride__);

[par.allocation.B, par.allocation.B_px4, par.allocation.B_px4_norm] = ...
    irisAllocationMatrices(par);

par.Tmax = allocationForceLimit(par);
par.tauMax = allocationMomentLimits(par);

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

controllerState = initControllerState(par, x);

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
log.actuator = zeros(4,N);
log.thrSp = nan(3,N);
log.ratesSp = nan(3,N);
log.rateError = nan(3,N);
log.torqueNorm = nan(3,N);
log.forceDist = zeros(3,N);
log.momentDist = zeros(3,N);

%% ========================================================================
%% 4. Simulation loop
for k = 1:N
    t = time(k);

    ref = traj.eval(t);
    [u, controllerState] = runController(x, ref, par, controllerState);

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

    muApplied = allocatedWrench(u, par);
    log.T(k) = muApplied(1);
    log.tau(:,k) = muApplied(2:4);
    log.actuator(:,k) = u.actuator;
    if par.controllerName == "px4_iris"
        log.thrSp(:,k) = u.thrSp;
        log.ratesSp(:,k) = u.ratesSp;
        log.rateError(:,k) = u.rateError;
        log.torqueNorm(:,k) = u.torqueNorm;
    end
    [log.forceDist(:,k), log.momentDist(:,k)] = disturbanceAtTime(t, par);


    x = stepModel(x, u, par, t);
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

%% ========================================================================
%% Trajectory factory and shared utilities

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
    end

    traj = applyTrajectoryProgress(traj, par);
end

function traj = applyTrajectoryProgress(traj, par)

    baseEval = traj.eval;
    baseTend = traj.Tend;

    switch par.progress.mode
        case "scale_fixed"
            scale = par.progress.scale;

            traj.Tend = scale*baseTend;
            traj.eval = @(t) evalProgressTrajectory( ...
                baseEval, t/scale, 1/scale, 0, 0, 0, baseTend, false);
            traj.evalPredict = @(t) evalProgressTrajectory( ...
                baseEval, t/scale, 1/scale, 0, 0, 0, baseTend, true);

            if abs(scale - 1) >= 1e-12
                traj.name = traj.name + "_timeScale_" + string(scale);
            end

        case "scale_range"
            scaleRange = par.progress.scaleRange;

            traj.Tend = par.Tend;
            traj.name = traj.name + "_scaleRange_" + string(scaleRange(1)) ...
                      + "_" + string(scaleRange(2));
            traj.eval = @(t) evalScaleRangeTrajectory( ...
                baseEval, baseTend, t, traj.Tend, scaleRange, false);
            traj.evalPredict = @(t) evalScaleRangeTrajectory( ...
                baseEval, baseTend, t, traj.Tend, scaleRange, true);
    end
end

function ref = evalScaleRangeTrajectory( ...
        baseEval, baseTend, t, simTend, scaleRange, allowPredict)

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

    if allowPredict
        s = max(s, 0);
    else
        s = clampScalar(s, 0, baseTend);
    end

    ref = baseEval(s);

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

    switch string(par.progress.mode)
        case "scale_range"
            scaleRange = par.progress.scaleRange;
            scale = scaleRange(1) + (scaleRange(2) - scaleRange(1))*fraction;
        case "scale_fixed"
            scale = par.progress.scale;
    end
end

function s = trajectoryBaseTimeAtFraction(par, fraction)

    fraction = clampScalar(fraction, 0, 1);

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
    end
end

function shape = trajectoryShape(par)

    intensity = clampScalar(par.trajIntensity, 0, 1);
    scaleHalf = trajectoryScaleAtFraction(par, 0.5);
    scaleEnd = trajectoryScaleAtFraction(par, 1.0);
    sHalf = trajectoryBaseTimeAtFraction(par, 0.5);
    sEnd = trajectoryBaseTimeAtFraction(par, 1.0);
    flipTurns = max(double(par.flipTurns), 0.5);

    thrustAccel = max(par.Tmax/par.m - par.g, 0.5*par.g);
    alphaMax = angularAccelLimit(par);

    shape.g = par.g;
    shape.scaleEnd = scaleEnd;
    shape.regularAccel = max((0.18 + 0.20*intensity)*thrustAccel*scaleEnd^2, ...
        0.10*par.g*scaleEnd^2);

    frontFlipMax = (0.93 + 0.04*intensity)*par.g*scaleHalf^2;
    rearFlipMin = (1.02 + 0.10*intensity)*par.g*scaleEnd^2;
    rearFlipTarget = (1.08 + 0.35*intensity ...
                    + 0.80*max(flipTurns - 1, 0))*par.g*scaleEnd^2;
    thrustFlipMax = ((0.70 + 0.18*intensity)*par.Tmax/par.m ...
                   - par.g)*scaleEnd^2;
    flipCap = min([frontFlipMax, rearFlipTarget, thrustFlipMax]);
    shape.flipAccel = min(frontFlipMax, max(rearFlipMin, flipCap));

    shape.loopOmega = clampScalar((0.22 + 0.08*intensity)*sqrt(alphaMax), ...
        1.80, 3.00);
    shape.rampTime = clampScalar(0.5*pi*shape.loopOmega ...
        / ((0.12 + 0.08*intensity)*alphaMax), 1.80, 3.20);
    shape.flipTurns = flipTurns;
    shape.flipSpan = sEnd - sHalf;
end

function alphaMax = angularAccelLimit(par)

    Jdiag = abs(diag(par.J));
    tauMax = abs(par.tauMax(:));
    alphaMax = min(tauMax./Jdiag);
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

    B = par.allocation.B;
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    row = -B(1,:)';
    Tmax = sum(max(row.*lb, row.*ub));
end

function [B, B_px4, B_px4_norm] = irisAllocationMatrices(par)

    % This is the control-allocation construction from iris.m, kept local so
    % main_lee.m derives Lee and PX4 matrices from the same Gazebo Iris model.
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

    if ~par.disturbance.enabled
        return;
    end

    d = par.disturbance;
    distType = string(d.type);

    startTime = double(d.startTime);
    endTime = double(d.endTime);
    if t < startTime || t > endTime
        return;
    end

    forceAmp = d.forceAmp;
    momentAmp = d.momentAmp;

    switch distType
        case "constant"
            forceDist = forceAmp;
            momentDist = momentAmp;

        case "sin"
            forceFreq = d.forceFreq;
            momentFreq = d.momentFreq;
            forcePhase = d.forcePhase;
            momentPhase = d.momentPhase;

            forceDist = forceAmp .* sin(2*pi*forceFreq*t + forcePhase);
            momentDist = momentAmp .* sin(2*pi*momentFreq*t + momentPhase);

        case "none"
            return;
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

function state = initControllerState(par, x)

    state = struct();

    switch string(par.controllerName)
        case "px4_iris"
            state.px4.velInt = zeros(3,1);
            state.px4.prevVel = x.v;
            state.px4.rateInt = zeros(3,1);
            state.px4.prevOmega = x.Omega;

        case "lee"
            return;
    end
end

function [u, state] = runController(x, ref, par, state)

    switch string(par.controllerName)
        case "lee"
            u = controllerLee(x, ref, par);

        case "px4_iris"
            [u, state] = controllerPX4Iris(x, ref, par, state);
    end
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

    u = controlAllocation([T; tau], Rc, par);
end

function [u, state] = controllerPX4Iris(x, ref, par, state)

    p = par.px4iris;

    dt = par.dt;
    velDot = (x.v - state.px4.prevVel)/max(dt, eps);
    angularAccel = (x.Omega - state.px4.prevOmega)/max(dt, eps);

    [thrSp, Rd, state] = px4PositionControl(x, ref, velDot, par, state);

    if p.useAttitudeRateFeedforward
        [omegaFF, ~] = geometricFeedforwardInDesiredFrame(ref, Rd, par);
        ratesSp = px4AttitudeControl(x.R, Rd, nan, p) + omegaFF;
    else
        ratesSp = px4AttitudeControl(x.R, Rd, ref.psiDot, p);
    end

    [torqueNorm, rateError, state] = px4RateControl( ...
        x.Omega, ratesSp, angularAccel, par, p, state);

    thrustBodyZ = -norm(thrSp);
    muNormCmd = [thrustBodyZ; torqueNorm];
    u = controlAllocationPX4Normalized(muNormCmd, Rd, par);
    u.thrSp = thrSp;
    u.ratesSp = ratesSp;
    u.rateError = rateError;
    u.torqueNorm = torqueNorm;

    state.px4.prevVel = x.v;
    state.px4.prevOmega = x.Omega;
end

function [thrSp, Rd, state] = px4PositionControl(x, ref, velDot, par, state)

    p = par.px4iris;
    dt = par.dt;

    velSp = ref.v + (ref.p - x.p) .* p.posP;
    velError = velSp - x.v;
    accSp = referenceAcceleration(ref, p) + velError .* p.velP ...
        + state.px4.velInt - velDot .* p.velD;

    thrSp = px4AccelerationControl(accSp, p, par);
    state.px4.velInt = state.px4.velInt + velError .* p.velI * dt;

    bodyZ = -thrSp;
    Rd = px4BodyZToAttitude(bodyZ, ref.psi);
end

function thrSp = px4AccelerationControl(accSp, p, par)

    zSpecificForce = -par.g + accSp(3);
    bodyZ = [-accSp(1); -accSp(2); -zSpecificForce];

    if norm(bodyZ) < eps
        bodyZ = par.e3;
    else
        bodyZ = bodyZ/norm(bodyZ);
    end

    thrustNedZ = accSp(3) * (p.hoverThrust/par.g) - p.hoverThrust;
    cosNedBody = dot(par.e3, bodyZ);
    collectiveThrust = thrustNedZ/cosNedBody;
    thrSp = bodyZ * collectiveThrust;
end

function Rd = px4BodyZToAttitude(bodyZ, yawSp)

    if norm(bodyZ)^2 < eps
        bodyZ = [0; 0; 1];
    else
        bodyZ = bodyZ/norm(bodyZ);
    end

    yC = [-sin(yawSp); cos(yawSp); 0];
    bodyX = cross(yC, bodyZ);

    if bodyZ(3) < 0
        bodyX = -bodyX;
    end

    if abs(bodyZ(3)) < 1e-6
        bodyX = [0; 0; 1];
    end

    if norm(bodyX) < eps
        bodyX = [1; 0; 0];
    else
        bodyX = bodyX/norm(bodyX);
    end

    bodyY = cross(bodyZ, bodyX);
    Rd = [bodyX, bodyY, bodyZ];
end

function ratesSp = px4AttitudeControl(R, Rd, yawRateSp, p)

    q = rotmToQuatWXYZ(R);
    qd = rotmToQuatWXYZ(Rd);
    qError = quatCanonicalWXYZ( ...
        quatMultiplyWXYZ(quatInverseWXYZ(q), qd));
    ratesSp = 2*qError(2:4) .* p.attP;
    if isfinite(yawRateSp)
        ratesSp = ratesSp + R' * [0; 0; 1] * yawRateSp;
    end
end

function [torqueNorm, rateError, state] = px4RateControl( ...
        rates, ratesSp, angularAccel, par, p, state)

    rateError = ratesSp - rates;
    torqueNorm = p.rateP .* rateError + state.px4.rateInt ...
        - p.rateD .* angularAccel + p.rateFF .* ratesSp;

    dt = par.dt;
    for i = 1:3
        iFactor = rateError(i)/deg2rad(400);
        iFactor = max(0, 1 - iFactor^2);
        nextInt = state.px4.rateInt(i) ...
            + iFactor*p.rateI(i)*rateError(i)*dt;

        if isfinite(nextInt)
            state.px4.rateInt(i) = clampScalar(nextInt, ...
                -p.rateIntLimit(i), p.rateIntLimit(i));
        end
    end
end

function q = quatMultiplyWXYZ(q1, q2)

    w1 = q1(1);
    v1 = q1(2:4);
    w2 = q2(1);
    v2 = q2(2:4);

    q = [w1*w2 - dot(v1, v2); ...
         w1*v2 + w2*v1 + cross(v1, v2)];
    q = normalizeQuatWXYZ(q);
end

function qInv = quatInverseWXYZ(q)

    q = normalizeQuatWXYZ(q);
    qInv = [q(1); -q(2:4)];
end

function q = quatCanonicalWXYZ(q)

    q = normalizeQuatWXYZ(q);

    if q(1) < 0
        q = -q;
    elseif abs(q(1)) < eps
        for i = 2:4
            if abs(q(i)) > eps
                if q(i) < 0
                    q = -q;
                end

                break;
            end
        end
    end
end

function u = controlAllocation(mu, Rd, par)

    muCmd = [-mu(1); mu(2:4)];
    u.Rd = Rd;
    u.actuator = min(max(par.allocation.B\muCmd, ...
        par.allocation.uMin(:)), par.allocation.uMax(:));
    u.muAllocated = par.allocation.B*u.actuator(:);
end

function u = controlAllocationPX4Normalized(muNormCmd, Rd, par)

    % muNormCmd is already [Fz; Mx; My; Mz], matching B_px4_norm.
    u.Rd = Rd;
    u.muNormCmd = muNormCmd(:);
    u.actuatorNormRaw = par.allocation.B_px4_norm\u.muNormCmd;
    u.actuator = min(max(u.actuatorNormRaw, ...
        par.allocation.uNormMin(:)), par.allocation.uNormMax(:));
    u.muNormAllocated = par.allocation.B_px4_norm*u.actuator(:);
    u.muAllocated = par.allocation.B_px4*u.actuator(:);
end

function mu = allocatedWrench(u, par)

    % Actuator dynamics are neglected here. After allocation and saturation,
    % the actuator vector is multiplied by the matching allocation matrix to
    % obtain the physical wrench applied to the rigid-body model.
    mu = [-u.muAllocated(1); u.muAllocated(2:4)];
end

function ff = geometricFlatnessReference(ref, par)

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

    switch par.integratorName
        case "ode45"
            xNext = stepModelODE45(x, u, par, t0);
        case "lie_rk4"
            xNext = stepModelLieRK4(x, u, par, t0);
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

    [forceDist, momentDist] = disturbanceAtTime(t, par);
    mu = allocatedWrench(u, par);
    T = mu(1);
    tau = mu(2:4);

    a = par.g*par.e3 - T/par.m*R*par.e3 + forceDist/par.m;
    OmegaDot = par.J \ (tau + momentDist - cross(Omega, par.J*Omega));
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
    [omegaRef, alphaRef] = rotationLogRates(log.Rd, time);
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
    % is sensitive near theta = pi. The quaternion path uses atan2 and keeps
    % the principal branch by choosing qw >= 0.
    q = rotmToQuatWXYZ(projectSO3(R));
    phi = quatLogVectorWXYZ(q);
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
