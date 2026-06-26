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
par.Tend = 12.0;
par.trajTimeScale = 1.0;    % >1 slows the reference trajectory
par.integratorName = "ode45";  % "ode45" or "lie_rk4"

% Available choices:
%   "figure8_horizontal"
%   "helix_flip"
%   "flip_loop_sine"
%   "fast_circle"
par.trajName = "flip_loop_sine";

% Simple controller gains
par.controllerName = "on_manifold_mpc";  % "geometric" or "on_manifold_mpc"
par.Kp = diag([20, 20, 25]);
par.Kv = diag([9, 9, 10]);
par.KR = 35*eye(3);
par.KOmega = 35*par.J;

% On-manifold finite-horizon controller.
% State error: [p-pd; v-vd; Log(Rd'R)], input: [aT-aTd; Omega-OmegaD].
par.mpc.N = 16; % Lu et al. use N=8; use longer horizon for the simulated rate loop.
par.mpc.Q = diag([450, 450, 650, ...
                  70, 70, 100, ...
                  140, 140, 80]);
par.mpc.R = diag([1.0, 0.55, 0.55, 0.75]);
par.mpc.P = par.mpc.Q;
par.mpc.omegaMax = deg2rad(800);

% Actuator limits
par.Tmax = 4*9.81;
par.tauMax = [8; 8; 8];

% Initial condition
par.startOnReference = true;

% 3D attitude sampling visualization
par.poseEvery = 0.20;       % seconds
par.bodyAxisScale = 0.25;   % meters
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

    x = stepModel(x, u, par);
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

    traj = scaleTrajectoryTime(traj, par.trajTimeScale);
end

function traj = scaleTrajectoryTime(traj, scale)

    if scale <= 0
        error("Trajectory time scale must be positive.");
    end

    if abs(scale - 1) < 1e-12
        return;
    end

    baseEval = traj.eval;
    baseTend = traj.Tend;

    traj.Tend = scale*baseTend;
    traj.name = traj.name + "_timeScale_" + string(scale);
    traj.eval = @(t) evalScaledTrajectory(baseEval, t, scale, baseTend);
end

function ref = evalScaledTrajectory(baseEval, t, scale, baseTend)

    tBase = min(max(t/scale, 0), baseTend);
    ref = baseEval(tBase);
    ref.v = ref.v/scale;
    ref.a = ref.a/(scale^2);
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
        case "on_manifold_mpc"
            u = controllerOnManifoldMPC(x, ref, traj, t, par);
        otherwise
            error("Unknown controllerName.");
    end
end

function u = controllerPDGeometric(x, ref, par)

    ep = ref.p - x.p;
    ev = ref.v - x.v;

    aCmd = ref.a + par.Kp*ep + par.Kv*ev;

    [Rd, T] = desiredAttitudeFromAccel(aCmd, ref.psi, par, x.R);

    rErr = LogSO3(x.R' * Rd);

    tau = cross(x.Omega, par.J*x.Omega) ...
        + par.KR*rErr + par.KOmega*(zeros(3,1)-x.Omega);

    u.T = min(max(T, 0), par.Tmax);
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = Rd;
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
        + par.KOmega*(OmegaCmd - x.Omega);

    u.T = par.m*aTCmd;
    u.tau = saturateVector(tau, par.tauMax);
    u.Rd = Rd;
end

function [Rd, aT, OmegaD] = referenceInputOnManifold(ref, traj, t, par)

    [Rd, T] = desiredAttitudeFromAccel(ref.a, ref.psi, par, eye(3));
    aT = T/par.m;

    tNext = min(t + par.dt, par.Tend);
    refNext = traj.eval(tNext);
    [RdNext, ~] = desiredAttitudeFromAccel(refNext.a, refNext.psi, par, eye(3));

    if tNext > t
        OmegaD = LogSO3(Rd' * RdNext)/(tNext - t);
    else
        OmegaD = zeros(3,1);
    end
end

function [Ad, Bd] = linearizedQuadrotorErrorModel(Rd, aTd, par)

    Ac = zeros(9,9);
    Bc = zeros(9,4);

    Ac(1:3,4:6) = eye(3);
    Ac(4:6,7:9) = aTd*Rd*hat(par.e3);

    Bc(4:6,1) = -Rd*par.e3;
    Bc(7:9,2:4) = eye(3);

    Ad = eye(9) + par.dt*Ac;
    Bd = par.dt*Bc;
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


    b3d = T_b_z/norm(T_b_z);

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

    c = (trace(R) - 1)/2;
    c = min(1, max(-1, c));
    theta = acos(c);

    if theta < 1e-10
        phi = 0.5*vee(R - R');
        return;
    end

    if abs(theta - pi) < 1e-5
        [V,D] = eig(R);
        [~,idx] = min(abs(diag(D) - 1));
        u = real(V(:,idx));
        u = u/norm(u);
        phi = theta*u;
        return;
    end

    phi = theta/(2*sin(theta))*vee(R - R');
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
