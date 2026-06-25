%% main.m
% Simple quadrotor simulation with modular reference trajectories.
% Internal coordinate: NED, z_NED points downward.
% 3D plot coordinate: height = -z_NED.
%
% State:
%   p : position in NED
%   v : velocity in NED
%   R : body-to-NED rotation matrix
%
% Input:
%   aT : thrust acceleration magnitude
%   w  : commanded body angular rate
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

par.dt = 0.01;          % 100 Hz
par.Tend = 7.2;

% Available choices:
%   "minsnap_helix_flip"
%   "helix_flip"
%   "flip_loop_sine"
%   "fast_circle"
par.trajName = "minsnap_helix_flip";

% Controller gains
par.Kp = diag([35, 45, 55]);
par.Kv = diag([14, 16, 18]);
par.kR = 35;

% Saturation
par.aTmax = 6*par.g;
par.wmax = 60;          % rad/s

% Initial condition
par.startOnReference = true;

% 3D attitude sampling visualization
par.poseEvery = 0.20;       % seconds
par.bodyAxisScale = 0.25;   % meters
par.poseSource = "actual";  % "actual" or "desired"

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
else
    x.p = [0;0;0];
    x.v = [0;0;0];
    x.R = eye(3);
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

log.euler = zeros(3,N);
log.eulerD = zeros(3,N);

log.aT = zeros(1,N);
log.wcmd = zeros(3,N);

%% ========================================================================
%% 4. Simulation loop
for k = 1:N
    t = time(k);

    ref = traj.eval(t);
    u = controllerPDFlatness(x, ref, par);

    log.p(:,k) = x.p;
    log.v(:,k) = x.v;
    log.pd(:,k) = ref.p;
    log.vd(:,k) = ref.v;
    log.ad(:,k) = ref.a;

    log.R(:,:,k) = x.R;
    log.Rd(:,:,k) = u.Rd;

    log.euler(:,k) = rotm2eulZYX(x.R);
    log.eulerD(:,k) = rotm2eulZYX(u.Rd);

    log.aT(k) = u.aT;
    log.wcmd(:,k) = u.w;

    x = stepModelBodyRate(x, u, par);
end

%% ========================================================================
%% 5. Plot
plotResults(time, log, par, traj);

%% ========================================================================
%% Trajectory factory
function traj = makeTrajectory(par)

    switch par.trajName

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
end

%% ========================================================================
%% Analytic helix with flips
function ref = evalHelixFlip(t)

    vx = 0.30;
    Ay = 0.80;
    Az = 0.80;
    h0 = 1.30;
    Tturn = 1.20;
    Om = 2*pi/Tturn;

    h = h0 + Az*cos(Om*t);

    ref.p = [vx*t;
             Ay*sin(Om*t);
            -h];

    ref.v = [vx;
             Ay*Om*cos(Om*t);
             Az*Om*sin(Om*t)];

    ref.a = [0;
            -Ay*Om^2*sin(Om*t);
             Az*Om^2*cos(Om*t)];

    ref.psi = 0;
end

%% ========================================================================
%% Analytic vertical flip loop
function ref = evalFlipLoopSine(t)

    Ay = 1.0;
    Az = 1.5;
    h0 = 1.5;
    Tloop = 1.4;
    Om = 2*pi/Tloop;

    h = h0 + Az*cos(Om*t);

    ref.p = [0;
             Ay*sin(Om*t);
            -h];

    ref.v = [0;
             Ay*Om*cos(Om*t);
             Az*Om*sin(Om*t)];

    ref.a = [0;
            -Ay*Om^2*sin(Om*t);
             Az*Om^2*cos(Om*t)];

    ref.psi = 0;
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

    Tturn = 1.20;
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
function u = controllerPDFlatness(x, ref, par)

    ep = x.p - ref.p;
    ev = x.v - ref.v;

    aCmd = ref.a - par.Kp*ep - par.Kv*ev;

    [Rd, aT] = desiredAttitudeFromAccel(aCmd, ref.psi, par);

    eR = LogSO3(x.R' * Rd);
    omegaCmd = par.kR * eR;

    omegaCmd = saturateVector(omegaCmd, par.wmax);
    aT = min(max(aT, 0), par.aTmax);

    u.aT = aT;
    u.w = omegaCmd;
    u.Rd = Rd;
end

%% ========================================================================
%% Flatness attitude map layer
function [Rd, aT] = desiredAttitudeFromAccel(aCmd, psi, par)

    c = par.g*par.e3 - aCmd;
    aT = norm(c);

    if aT < 1e-9
        c = par.g*par.e3;
        aT = norm(c);
    end

    b3d = c/aT;

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
function xNext = stepModelBodyRate(x, u, par)

    pDot = x.v;
    vDot = par.g*par.e3 - u.aT*x.R*par.e3;

    xNext.p = x.p + par.dt*pDot;
    xNext.v = x.v + par.dt*vDot;
    xNext.R = x.R * ExpSO3(u.w*par.dt);
end

%% ========================================================================
%% Plot layer
function plotResults(time, log, par, traj)

    % Bounded Euler-angle display.
    % Each angle is displayed in [-180 deg, 180 deg].
    eul = wrapToPiLocal(log.euler);
    eulD = wrapToPiLocal(log.eulerD);

    figure('Name','3D trajectory with sampled attitude');

    hActual = plot3(log.p(1,:), log.p(2,:), -log.p(3,:), ...
        'LineWidth', 1.6); 
    hold on;

    hRef = plot3(log.pd(1,:), log.pd(2,:), -log.pd(3,:), ...
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
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('height = -z_{NED} (m)');
    title("3D trajectory with sampled body axes: " + traj.name);
    view(35, 25);

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
    pPlot = [pNED(1); pNED(2); -pNED(3)];
end

function vPlot = nedVectorToPlot(vNED)
    vPlot = [vNED(1); vNED(2); -vNED(3)];
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

function R = ExpSO3(phi)

    theta = norm(phi);

    if theta < 1e-10
        R = eye(3) + hat(phi);
        return;
    end

    u = phi/theta;
    U = hat(u);

    R = eye(3) + sin(theta)*U + (1 - cos(theta))*U*U;
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

    nx = norm(x);

    if nx > xmax
        y = x * xmax/nx;
    else
        y = x;
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