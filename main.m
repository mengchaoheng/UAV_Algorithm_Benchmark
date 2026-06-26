%% main.m
% Simple quadrotor simulation with modular reference trajectories.
% Internal coordinate: NED, z_NED points downward.
% 3D plots use NED axes; the display reverses z so Down is visually downward.
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

% Available choices:
%   "figure8_horizontal"
%   "minsnap_helix_flip"
%   "helix_flip"
%   "flip_loop_sine"
%   "fast_circle"
par.trajName = "helix_flip";

% Simple controller gains
par.Kp = diag([20, 20, 25]);
par.Kv = diag([9, 9, 10]);
par.KR = 35*eye(3);
par.KOmega = 35*par.J;

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
    u = controllerPDGeometric(x, ref, par);

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

    x = stepModelRigidBodyRK4(x, u, par);
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

        case "minsnap_helix_flip"
            traj.name = "minsnap_helix_flip";
            data = buildMinSnapHelixFlip();
            traj.Tend = data.ts(end);
            traj.eval = @(t) evalMinSnapTraj(t, data);

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

%% ========================================================================
%% Build minimum-snap helix flip trajectory
function data = buildMinSnapHelixFlip()

    vx = 0.30;
    Ay = 0.80;
    Az = 0.80;
    h0 = 1.30;

    Tturn = 1.65;
    Nturn = 6;
    NsegPerTurn = 8;

    Nseg = Nturn*NsegPerTurn;
    Ttotal = Nturn*Tturn;
    Om = 2*pi/Tturn;

    ts = linspace(0, Ttotal, Nseg+1);
    theta = Om*ts;

    height = h0 + Az*cos(theta);

    wp = [vx*ts;
          Ay*sin(theta);
         -height];

    dStart.x = [vx, 0, 0];
    dEnd.x   = [vx, 0, 0];

    dStart.y = [ Ay*Om*cos(theta(1)), ...
                -Ay*Om^2*sin(theta(1)), ...
                -Ay*Om^3*cos(theta(1))];

    dEnd.y   = [ Ay*Om*cos(theta(end)), ...
                -Ay*Om^2*sin(theta(end)), ...
                -Ay*Om^3*cos(theta(end))];

    dStart.z = [ Az*Om*sin(theta(1)), ...
                 Az*Om^2*cos(theta(1)), ...
                -Az*Om^3*sin(theta(1))];

    dEnd.z   = [ Az*Om*sin(theta(end)), ...
                 Az*Om^2*cos(theta(end)), ...
                -Az*Om^3*sin(theta(end))];

    data.ts = ts;
    data.cx = minSnap1D(wp(1,:), ts, dStart.x, dEnd.x);
    data.cy = minSnap1D(wp(2,:), ts, dStart.y, dEnd.y);
    data.cz = minSnap1D(wp(3,:), ts, dStart.z, dEnd.z);
end

%% ========================================================================
%% Evaluate minimum-snap trajectory
function ref = evalMinSnapTraj(t, data)

    t = min(max(t, data.ts(1)), data.ts(end));

    ref.p = [evalSnap1D(data.cx, data.ts, t, 0);
             evalSnap1D(data.cy, data.ts, t, 0);
             evalSnap1D(data.cz, data.ts, t, 0)];

    ref.v = [evalSnap1D(data.cx, data.ts, t, 1);
             evalSnap1D(data.cy, data.ts, t, 1);
             evalSnap1D(data.cz, data.ts, t, 1)];

    ref.a = [evalSnap1D(data.cx, data.ts, t, 2);
             evalSnap1D(data.cy, data.ts, t, 2);
             evalSnap1D(data.cz, data.ts, t, 2)];

    ref.psi = 0;
end

%% ========================================================================
%% Controller layer
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
function xNext = stepModelRigidBodyRK4(x, u, par)

    h = par.dt;

    k1 = rigidBodyDerivative(x, u, par);
    k2 = rigidBodyDerivative(addStateDerivative(x, k1, 0.5*h), u, par);
    k3 = rigidBodyDerivative(addStateDerivative(x, k2, 0.5*h), u, par);
    k4 = rigidBodyDerivative(addStateDerivative(x, k3, h), u, par);

    xNext.p = x.p + h/6*(k1.p + 2*k2.p + 2*k3.p + k4.p);
    xNext.v = x.v + h/6*(k1.v + 2*k2.v + 2*k3.v + k4.v);
    xNext.R = x.R + h/6*(k1.R + 2*k2.R + 2*k3.R + k4.R);
    xNext.Omega = x.Omega + h/6*(k1.Omega + 2*k2.Omega + 2*k3.Omega + k4.Omega);

    xNext.R = projectSO3(xNext.R);
end

function xTmp = addStateDerivative(x, dx, h)

    xTmp.p = x.p + h*dx.p;
    xTmp.v = x.v + h*dx.v;
    xTmp.R = x.R + h*dx.R;
    xTmp.Omega = x.Omega + h*dx.Omega;
end

function dx = rigidBodyDerivative(x, u, par)

    dx.p = x.v;
    dx.v = par.g*par.e3 - u.T/par.m*x.R*par.e3;
    dx.R = x.R*hat(x.Omega);
    dx.Omega = par.J \ (u.tau - cross(x.Omega, par.J*x.Omega));
end

function R = projectSO3(R)

    [U,~,V] = svd(R);
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
    drawWorldNEDAxes(gca);

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

        pPlot = nedPointToPlot(pNED);

        xB = nedVectorToPlot(R(:,1));
        yB = nedVectorToPlot(R(:,2));
        zB = nedVectorToPlot(R(:,3));

        if s == 1
            hx = quiver3(pPlot(1), pPlot(2), pPlot(3), ...
                    L*xB(1), L*xB(2), L*xB(3), ...
                    0, 'r', 'LineWidth', 1.0, 'MaxHeadSize', 0.8);

            hy = quiver3(pPlot(1), pPlot(2), pPlot(3), ...
                    L*yB(1), L*yB(2), L*yB(3), ...
                    0, 'g', 'LineWidth', 1.0, 'MaxHeadSize', 0.8);

            hz = quiver3(pPlot(1), pPlot(2), pPlot(3), ...
                    L*zB(1), L*zB(2), L*zB(3), ...
                    0, 'b', 'LineWidth', 1.0, 'MaxHeadSize', 0.8);
        else
            quiver3(pPlot(1), pPlot(2), pPlot(3), ...
                    L*xB(1), L*xB(2), L*xB(3), ...
                    0, 'r', 'LineWidth', 1.0, 'MaxHeadSize', 0.8, ...
                    'HandleVisibility','off');

            quiver3(pPlot(1), pPlot(2), pPlot(3), ...
                    L*yB(1), L*yB(2), L*yB(3), ...
                    0, 'g', 'LineWidth', 1.0, 'MaxHeadSize', 0.8, ...
                    'HandleVisibility','off');

            quiver3(pPlot(1), pPlot(2), pPlot(3), ...
                    L*zB(1), L*zB(2), L*zB(3), ...
                    0, 'b', 'LineWidth', 1.0, 'MaxHeadSize', 0.8, ...
                    'HandleVisibility','off');
        end
    end
end

function pPlot = nedPointToPlot(pNED)
    pPlot = pNED;
end

function vPlot = nedVectorToPlot(vNED)
    vPlot = vNED;
end

function drawWorldNEDAxes(ax)

    xl = xlim(ax);
    yl = ylim(ax);
    zl = zlim(ax);

    dx = diff(xl);
    dy = diff(yl);
    dz = diff(zl);
    L = 0.14*max([dx, dy, dz]);

    origin = [xl(1) + 0.08*dx;
              yl(1) + 0.10*dy;
              zl(1) + 0.12*dz];

    quiver3(ax, origin(1), origin(2), origin(3), L, 0, 0, ...
        0, 'Color', [0.65 0 0], 'LineWidth', 1.5, ...
        'MaxHeadSize', 0.8, 'HandleVisibility', 'off');
    quiver3(ax, origin(1), origin(2), origin(3), 0, L, 0, ...
        0, 'Color', [0 0.45 0], 'LineWidth', 1.5, ...
        'MaxHeadSize', 0.8, 'HandleVisibility', 'off');
    quiver3(ax, origin(1), origin(2), origin(3), 0, 0, L, ...
        0, 'Color', [0 0.15 0.75], 'LineWidth', 1.8, ...
        'MaxHeadSize', 0.8, 'HandleVisibility', 'off');

    text(ax, origin(1)+1.10*L, origin(2), origin(3), '+x_N', ...
        'Color', [0.65 0 0], 'FontWeight', 'bold', ...
        'HandleVisibility', 'off');
    text(ax, origin(1), origin(2)+1.10*L, origin(3), '+y_E', ...
        'Color', [0 0.45 0], 'FontWeight', 'bold', ...
        'HandleVisibility', 'off');
    text(ax, origin(1), origin(2), origin(3)+1.10*L, '+z_D', ...
        'Color', [0 0.15 0.75], 'FontWeight', 'bold', ...
        'HandleVisibility', 'off');

    xlim(ax, xl);
    ylim(ax, yl);
    zlim(ax, zl);
end

%% ========================================================================
%% Minimum-snap utility functions
function c = minSnap1D(wp, ts, dStart, dEnd)
% Equality-constrained minimum snap for one scalar trajectory.
% Polynomial on each segment: w_j(tau)=c0+c1*tau+...+c7*tau^7.

    n = 7;
    nCoef = n + 1;
    m = numel(ts) - 1;
    nVar = m*nCoef;

    H = zeros(nVar);

    for s = 1:m
        T = ts(s+1) - ts(s);
        Q = zeros(nCoef);

        for i = 4:n
            for j = 4:n
                Q(i+1,j+1) = factorial(i)/factorial(i-4) ...
                            * factorial(j)/factorial(j-4) ...
                            * T^(i+j-7)/(i+j-7);
            end
        end

        idx = segIndex(s, nCoef);
        H(idx,idx) = Q;
    end

    H = H + 1e-10*eye(nVar);

    Aeq = [];
    beq = [];

    % Position constraints at each segment boundary.
    for s = 1:m
        T = ts(s+1) - ts(s);

        row = zeros(1,nVar);
        row(segIndex(s,nCoef)) = polyBasis(n,0,0);
        Aeq = [Aeq; row];
        beq = [beq; wp(s)];

        row = zeros(1,nVar);
        row(segIndex(s,nCoef)) = polyBasis(n,0,T);
        Aeq = [Aeq; row];
        beq = [beq; wp(s+1)];
    end

    % Start derivative constraints: velocity, acceleration, jerk.
    for d = 1:3
        row = zeros(1,nVar);
        row(segIndex(1,nCoef)) = polyBasis(n,d,0);
        Aeq = [Aeq; row];
        beq = [beq; dStart(d)];
    end

    % End derivative constraints: velocity, acceleration, jerk.
    Tend = ts(end) - ts(end-1);

    for d = 1:3
        row = zeros(1,nVar);
        row(segIndex(m,nCoef)) = polyBasis(n,d,Tend);
        Aeq = [Aeq; row];
        beq = [beq; dEnd(d)];
    end

    % Interior continuity constraints: velocity, acceleration, jerk.
    for s = 1:m-1
        T = ts(s+1) - ts(s);

        for d = 1:3
            row = zeros(1,nVar);
            row(segIndex(s,nCoef)) = polyBasis(n,d,T);
            row(segIndex(s+1,nCoef)) = -polyBasis(n,d,0);
            Aeq = [Aeq; row];
            beq = [beq; 0];
        end
    end

    KKT = [H, Aeq'; Aeq, zeros(size(Aeq,1))];
    rhs = [zeros(nVar,1); beq];

    sol = KKT \ rhs;
    c = reshape(sol(1:nVar), nCoef, m);
end

function val = evalSnap1D(c, ts, tq, der)
% Evaluate derivative order der of a piecewise polynomial.

    n = size(c,1) - 1;
    m = size(c,2);

    if tq <= ts(1)
        s = 1;
        tau = 0;
    elseif tq >= ts(end)
        s = m;
        tau = ts(end) - ts(end-1);
    else
        s = find(ts <= tq, 1, 'last');

        if s == numel(ts)
            s = numel(ts)-1;
        end

        tau = tq - ts(s);
    end

    val = polyBasis(n, der, tau) * c(:,s);
end

function b = polyBasis(n, der, tau)
% Row vector for derivative der of [1,t,t^2,...,t^n].

    b = zeros(1,n+1);

    for i = der:n
        b(i+1) = factorial(i)/factorial(i-der)*tau^(i-der);
    end
end

function idx = segIndex(s, nCoef)
    idx = (s-1)*nCoef + (1:nCoef);
end

%% ========================================================================
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
