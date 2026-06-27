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
%   rotorThrusts : four bounded rotor thrusts [N]
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
% PX4 x500 actuator geometry with main.m-like normalized dynamics.
% Special handling: par.m and par.J are dynamic-similarity values, chosen so
% the x500 actuator limits give the same Tmax/m and tauMax/J as main.m.
par.m = 1.73579294117647;
par.J = diag([0.0100408235294118, ...
              0.0112457223529412, ...
              0.00721848128342246]);

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
par.lee.Kp = par.Kp;
par.lee.Kv = par.Kv;
par.lee.KR = par.KR;
par.lee.KOmega = par.KOmega;

% Direct physical actuator model, matching main.m's semantics.
% uRotor is per-rotor thrust [N], and mu=[T;Mx;My;Mz] = G*uRotor.
% Special handling: motor thrust is calibrated from the 2 kg x500 SDF hover
% point, while par.m above is the dynamic-similarity mass.
par.motor.hoverMass = 2.0;
par.motor.hoverCommand = 0.5;
par.motor.maxRotVelocity = 1000.0;       % SDF full-scale speed reference.
par.motor.sdfMotorConstant = 8.54858e-06;
par.motor.motorConstant = (par.motor.hoverMass*par.g/4) ...
    /(par.motor.hoverCommand*par.motor.maxRotVelocity)^2;
par.motor.maxThrust = par.motor.motorConstant*par.motor.maxRotVelocity^2;

% Same G structure as main.m:
%   G=[1; y; -x; kappa*[-1 -1 1 1]].
par.allocation.method = "inv";
par.allocation.tBM = [ 0.174, -0.174,  0.174, -0.174;
                      -0.174,  0.174,  0.174, -0.174;
                       0.060,  0.060,  0.060,  0.060];
par.allocation.kappa = 0.016;
par.allocation.G = [ones(1,4);
                    par.allocation.tBM(2,:);
                   -par.allocation.tBM(1,:);
                    par.allocation.kappa*[-1, -1, 1, 1]];
par.allocation.uMin = zeros(4,1);
par.allocation.uMax = par.motor.maxThrust*ones(4,1);
par.Tmax = allocationForceLimit(par);
par.tauMax = allocationMomentLimits(par);

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

par.allocation.G = [ones(1,4);
                    par.allocation.tBM(2,:);
                   -par.allocation.tBM(1,:);
                    par.allocation.kappa*[-1, -1, 1, 1]];

par.allocation.method = lower(string(par.allocation.method));
if par.allocation.method ~= "inv"
    error('main_lee.m uses only par.allocation.method = "inv".');
end
if ~isequal(size(par.allocation.G), [4 4])
    error("par.allocation.G must map rotor thrusts to [T;Mx;My;Mz].");
end
par.allocation.uMin = par.allocation.uMin(:);
par.allocation.uMax = par.allocation.uMax(:);
par.allocation.Ginv = inv(par.allocation.G);
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
log.rotorThrusts = zeros(4,N);
log.forceDist = zeros(3,N);
log.momentDist = zeros(3,N);

%% ========================================================================
%% 4. Simulation loop
for k = 1:N
    t = time(k);

    ref = traj.eval(t);
    u = controllerLee(x, ref, par);

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
    log.rotorThrusts(:,k) = u.rotorThrusts;
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

function Tmax = allocationForceLimit(par)

    G = allocationMatrix(par);
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    row = G(1,:)';
    Tmax = sum(max(row.*lb, row.*ub));
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

function G = allocationMatrix(par)

    G = par.allocation.G;
end

function u = controlAllocation(mu, Rd, par)

    % Public allocation interface for main_lee:
    %   controller mu=[T;Mx;My;Mz] -> rotorThrusts=Ginv*mu -> actuator limits.
    u.Rd = Rd;
    u.muCmd = mu(:);
    u.rotorThrusts = min(max(par.allocation.Ginv*u.muCmd, ...
        par.allocation.uMin(:)), par.allocation.uMax(:));
end

function mu = allocatedWrench(u, par)

    % Plant-side effectiveness: limited rotor thrusts produce the actual
    % applied wrench mu=[T;Mx;My;Mz].
    mu = par.allocation.G*u.rotorThrusts(:);
end

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
