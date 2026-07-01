function figureFiles = plot_main(varargin)
%PLOT_MAIN Plot or replot the latest main.m simulation result.
%
% Usage:
%   plot_main
%   plot_main("results/main/main_run.mat")
%   plot_main(time, log, par, traj)
%   plot_main(..., "PlotStateDetail", false)

    [time, log, par, traj, dataPath, args] = parsePlotMainInputs(varargin);
    opts = plotMainOptions(par, dataPath, args);

    if opts.savePlots && opts.clearOutput
        prepareFigureDir(opts.outputDir);
    end

    figs = gobjects(0,1);
    if opts.plotStateDetail
        figs(end+1,1) = plotStateDetailFigure(time, log, par); 
    end
    figs(end+1,1) = plotTrajectory3D(time, log, par, traj, opts); 

    figureFiles = strings(0,1);
    if opts.savePlots
        figureFiles = saveFigureList(figs, opts.outputDir, opts.resolution);
        fprintf('main figures saved to: %s\n', opts.outputDir);
    end

    figAnim = animateTrajectory3D(time, log, par, traj);
    if opts.savePlots && ishandle(figAnim)
        figureFiles(end+1,1) = saveFigureList(figAnim, ...
            opts.outputDir, opts.resolution);
        fprintf('main animation saved to: %s\n', opts.outputDir);
    end
end

function [time, log, par, traj, dataPath, args] = parsePlotMainInputs(argsIn)

    dataPath = "";

    if isempty(argsIn)
        dataPath = fullfile(pwd, "results", "main", "main_run.mat");
        args = {};
        data = load(dataPath, 'time', 'log', 'par', 'traj');
        time = data.time;
        log = data.log;
        par = data.par;
        traj = data.traj;
        return;
    end

    firstArg = argsIn{1};
    if ischar(firstArg) || isstring(firstArg)
        if isPlotMainOptionName(firstArg)
            dataPath = fullfile(pwd, "results", "main", "main_run.mat");
            args = argsIn;
            data = load(dataPath, 'time', 'log', 'par', 'traj');
            time = data.time;
            log = data.log;
            par = data.par;
            traj = data.traj;
            return;
        end

        dataPath = string(firstArg);
        if isfolder(dataPath)
            dataPath = fullfile(dataPath, "main_run.mat");
        end
        args = argsIn(2:end);
        data = load(dataPath, 'time', 'log', 'par', 'traj');
        time = data.time;
        log = data.log;
        par = data.par;
        traj = data.traj;
        return;
    end

    if numel(argsIn) < 4
        error('plot_main requires time, log, par, and traj.');
    end

    time = argsIn{1};
    log = argsIn{2};
    par = argsIn{3};
    traj = argsIn{4};
    args = argsIn(5:end);
end

function tf = isPlotMainOptionName(name)

    optionNames = ["saveplots", "outputdir", "resolution", ...
        "clearoutput", "plotstatedetail", ...
        "plotbodyaxes", "plotbodyaxesevery", "plotbodyaxisscale", ...
        "plotbodyaxesposesource"];
    tf = ismember(lower(string(name)), optionNames);
end

function opts = plotMainOptions(par, dataPath, args)

    opts.savePlots = strlength(dataPath) > 0;
    opts.outputDir = defaultMainFigureDir(par, dataPath);
    opts.resolution = 200;
    opts.clearOutput = true;
    opts.plotStateDetail = logical(getStructFieldLocal(par, ...
        'plotStateDetail', true));
    opts.plotBodyAxes = logical(getStructFieldLocal(par, ...
        'plotBodyAxes', false));
    opts.plotBodyAxesEvery = getStructFieldLocal(par, ...
        'plotBodyAxesEvery', 1);
    opts.plotBodyAxisScale = getStructFieldLocal(par, ...
        'plotBodyAxisScale', 0.3);
    opts.plotBodyAxesPoseSource = string(getStructFieldLocal(par, ...
        'plotBodyAxesPoseSource', "actual"));

    i = 1;
    while i <= numel(args)
        name = string(args{i});
        value = args{i+1};

        switch lower(name)
            case "saveplots"
                opts.savePlots = logical(value);
            case "outputdir"
                opts.outputDir = char(value);
            case "resolution"
                opts.resolution = double(value);
            case "clearoutput"
                opts.clearOutput = logical(value);
            case "plotstatedetail"
                opts.plotStateDetail = logical(value);
            case "plotbodyaxes"
                opts.plotBodyAxes = logical(value);
            case "plotbodyaxesevery"
                opts.plotBodyAxesEvery = double(value);
            case "plotbodyaxisscale"
                opts.plotBodyAxisScale = double(value);
            case "plotbodyaxesposesource"
                opts.plotBodyAxesPoseSource = string(value);
            otherwise
                error("Unknown plot_main option: %s.", name);
        end

        i = i + 2;
    end
end

function outputDir = defaultMainFigureDir(par, dataPath)

    if isfield(par, 'resultDir') && strlength(string(par.resultDir)) > 0
        outputDir = fullfile(char(par.resultDir), 'figures');
    elseif strlength(dataPath) > 0
        outputDir = fullfile(fileparts(char(dataPath)), 'figures');
    else
        outputDir = fullfile(pwd, 'results', 'main', 'figures');
    end
end

function fig = plotTrajectory3D(time, log, par, traj, opts)

    fig = figure('Color', 'w', 'Name', 'traj_3d');
    ax = axes(fig);
    plotTrajectory3DOnAxes(ax, time, log, par, traj, opts);
end

function fig = plotStateDetailFigure(time, log, par)

    controllerName = string(getStructFieldLocal(par, ...
        'controllerName', "controller"));
    figName = matlab.lang.makeValidName(char("state_" + controllerName));
    fig = figure('Color', 'w', 'Name', figName);
    tl = tiledlayout(fig, 6, 3, 'TileSpacing', 'compact', ...
        'Padding', 'compact');

    eul = rad2deg(wrapToPiLocal(log.euler));
    eulD = rad2deg(wrapToPiLocal(log.eulerD));
    accActual = loggedLinearAcceleration(log, par);
    [omegaRef, alphaRef] = desiredAngularDerivativesForPlot(log, time);
    alphaActual = loggedAngularAcceleration(log, par);

    plotComponentGroup(tl, time, log.p, log.pd, ...
        ["x", "y", "z"], "position", "m");
    plotComponentGroup(tl, time, eul, eulD, ...
        ["roll", "pitch", "yaw"], "attitude", "deg");
    plotComponentGroup(tl, time, log.v, log.vPlotD, ...
        ["v_x", "v_y", "v_z"], "velocity", "m/s");
    plotComponentGroup(tl, time, accActual, log.aPlotD, ...
        ["a_x", "a_y", "a_z"], "acceleration", "m/s^2");
    plotComponentGroup(tl, time, log.Omega, omegaRef, ...
        ["p", "q", "r"], "angular rate", "rad/s");
    plotComponentGroup(tl, time, alphaActual, alphaRef, ...
        ["alpha_x", "alpha_y", "alpha_z"], ...
        "angular acceleration", "rad/s^2");
end

function plotTrajectory3DOnAxes(ax, time, log, par, traj, opts)

    disableAxesToolbarLocal(ax);
    hActual = plot3(ax, log.p(1,:), log.p(2,:), log.p(3,:), ...
        'LineWidth', 1.6);
    hold(ax, 'on');
    hRef = plot3(ax, log.pd(1,:), log.pd(2,:), log.pd(3,:), ...
        '--', 'LineWidth', 1.6);

    legendHandles = [hActual, hRef];
    legendLabels = {'actual trajectory','reference trajectory'};

    if opts.plotBodyAxes
        switch opts.plotBodyAxesPoseSource
            case "desired"
                poseP = log.pd;
                poseR = log.Rd;
            otherwise
                poseP = log.p;
                poseR = log.R;
        end
        [hx, hy, hz] = drawSampledBodyAxes(time, poseP, poseR, par, ...
            opts, ax);
        legendHandles = [legendHandles, hx, hy, hz];
        legendLabels = [legendLabels, {'x_B','y_B','z_B'}];
    end

    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 35, 25);
    set(ax, 'ZDir', 'reverse');
    xlabel(ax, 'x_{NED} north (m)');
    ylabel(ax, 'y_{NED} east (m)');
    zlabel(ax, 'z_{NED} down (m)');
    title(ax, 'traj_3d', 'Interpreter', 'none');
    legend(ax, legendHandles, legendLabels, ...
        'Location', 'best', 'Interpreter', 'none');
    disableAxesToolbarLocal(ax);
end

function plotComponentGroup(tl, time, actual, desired, componentLabels, ...
        groupName, unitText)

    for i = 1:size(actual, 1)
        ax = nexttile(tl);
        plotScalarTracking(ax, time, actual(i,:), desired(i,:), ...
            string(componentLabels(i)), string(groupName), string(unitText));
    end
end

function plotScalarTracking(ax, time, actual, desired, componentLabel, ...
        groupName, unitText)

    disableAxesToolbarLocal(ax);
    plot(ax, time, actual, '-', 'LineWidth', 1.0, ...
        'DisplayName', 'actual');
    hold(ax, 'on');
    plot(ax, time, desired, '--', 'LineWidth', 1.0, ...
        'DisplayName', 'ref');

    grid(ax, 'on');
    xlabel(ax, 'time (s)');
    ylabel(ax, unitText);
    title(ax, groupName + ": " + componentLabel, 'Interpreter', 'none');
    legend(ax, 'Location', 'best', 'FontSize', 7, ...
        'Interpreter', 'none');
    disableAxesToolbarLocal(ax);
end

function disableAxesToolbarLocal(ax)

    disableDefaultInteractivity(ax);
    if isprop(ax, 'Toolbar') && ~isempty(ax.Toolbar)
        ax.Toolbar.Visible = 'off';
    end
end

function fig = plotStateTracking(time, log, traj)

    eul = wrapToPiLocal(log.euler);
    eulD = wrapToPiLocal(log.eulerD);
    actual = [log.p; rad2deg(eul)];
    desired = [log.pd; rad2deg(eulD)];
    labels = {'x (m)', 'y (m)', 'z_{NED} (m)', ...
              'roll (deg)', 'pitch (deg)', 'yaw (deg)'};

    fig = figure('Color', 'w', 'Name', 'state_tracking');
    for i = 1:6
        subplot(6,1,i);
        plot(time, actual(i,:), 'LineWidth', 1.1); hold on;
        plot(time, desired(i,:), '--', 'LineWidth', 1.1);
        grid on;
        ylabel(labels{i});
        if i == 1
            title("Reference tracking: " + string(traj.name), ...
                'Interpreter', 'none');
        end
        if i == 6
            xlabel('time (s)');
        end
    end
    legend('actual','reference/command', 'Interpreter', 'none');
end

function [figVel, figRate] = plotDerivativeTrackingMain(time, log, par, traj)

    accActual = loggedLinearAcceleration(log, par);
    [omegaRef, alphaRef] = desiredAngularDerivativesForPlot(log, time);
    alphaActual = loggedAngularAcceleration(log, par);

    figVel = figure('Color', 'w', 'Name', 'velocity_acceleration');
    labels = {'v_x (m/s)', 'v_y (m/s)', 'v_z (m/s)', ...
              'a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)'};
    actual = [log.v; accActual];
    desired = [log.vPlotD; log.aPlotD];

    for i = 1:6
        subplot(6,1,i);
        plot(time, actual(i,:), 'LineWidth', 1.1); hold on;
        plot(time, desired(i,:), '--', 'LineWidth', 1.1);
        grid on;
        ylabel(labels{i});
        if i == 1
            title("Velocity/acceleration tracking: " + string(traj.name), ...
                'Interpreter', 'none');
        end
        if i == 6
            xlabel('time (s)');
        end
    end
    legend('actual','reference/command', 'Interpreter', 'none');

    figRate = figure('Color', 'w', 'Name', 'angular_rate_acceleration');
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
            title("Angular-rate/acceleration tracking: " + string(traj.name), ...
                'Interpreter', 'none');
        end
        if i == 6
            xlabel('time (s)');
        end
    end
    legend('actual','reference/command', 'Interpreter', 'none');
end

function [omegaRef, alphaRef] = desiredAngularDerivativesForPlot(log, time)

    N = numel(time);
    omegaRef = nan(3, N);
    alphaRef = nan(3, N);

    if isfield(log, 'OmegaDProvided')
        mask = log.OmegaDProvided;
        omegaRef(:,mask) = log.OmegaD(:,mask);
    else
        mask = false(1, N);
    end

    missingOmega = ~mask;
    if any(missingOmega)
        [omegaFromRd, alphaFromRd] = rotationLogRates(log.Rd, time);
        omegaRef(:,missingOmega) = omegaFromRd(:,missingOmega);
        alphaRef(:,missingOmega) = alphaFromRd(:,missingOmega);
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
        omega(:,k) = LogSO3Local(RLog(:,:,k)' * RLog(:,:,k+1))/h;
    end

    omega(:,N) = omega(:,N-1);

    for k = 1:N-1
        h = time(k+1) - time(k);
        omegaNextAtK = RLog(:,:,k)' * RLog(:,:,k+1) * omega(:,k+1);
        alpha(:,k) = (omegaNextAtK - omega(:,k))/h;
    end

    alpha(:,N) = alpha(:,N-1);
end

function [hx, hy, hz] = drawSampledBodyAxes(time, pLog, RLog, par, opts, ax)

    poseEvery = opts.plotBodyAxesEvery;
    bodyAxisScale = opts.plotBodyAxisScale;
    dt = getStructFieldLocal(par, 'dt', time(2) - time(1));
    step = max(1, round(poseEvery/dt));
    idxList = unique([1:step:numel(time), numel(time)]);
    L = bodyAxisScale;

    hx = gobjects(1);
    hy = gobjects(1);
    hz = gobjects(1);

    for s = 1:numel(idxList)
        idx = idxList(s);
        pNED = pLog(:,idx);
        R = RLog(:,:,idx);

        if s == 1
            hx = quiver3(ax, pNED(1), pNED(2), pNED(3), ...
                    L*R(1,1), L*R(2,1), L*R(3,1), ...
                    0, 'r', 'LineWidth', 1.0, 'MaxHeadSize', 0.8);
            hy = quiver3(ax, pNED(1), pNED(2), pNED(3), ...
                    L*R(1,2), L*R(2,2), L*R(3,2), ...
                    0, 'g', 'LineWidth', 1.0, 'MaxHeadSize', 0.8);
            hz = quiver3(ax, pNED(1), pNED(2), pNED(3), ...
                    L*R(1,3), L*R(2,3), L*R(3,3), ...
                    0, 'b', 'LineWidth', 1.0, 'MaxHeadSize', 0.8);
        else
            quiver3(ax, pNED(1), pNED(2), pNED(3), ...
                    L*R(1,1), L*R(2,1), L*R(3,1), ...
                    0, 'r', 'LineWidth', 1.0, 'MaxHeadSize', 0.8, ...
                    'HandleVisibility','off');
            quiver3(ax, pNED(1), pNED(2), pNED(3), ...
                    L*R(1,2), L*R(2,2), L*R(3,2), ...
                    0, 'g', 'LineWidth', 1.0, 'MaxHeadSize', 0.8, ...
                    'HandleVisibility','off');
            quiver3(ax, pNED(1), pNED(2), pNED(3), ...
                    L*R(1,3), L*R(2,3), L*R(3,3), ...
                    0, 'b', 'LineWidth', 1.0, 'MaxHeadSize', 0.8, ...
                    'HandleVisibility','off');
        end
    end
end

function prepareFigureDir(outputDir)

    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    deleteMatching(outputDir, '*.png');
    deleteMatching(outputDir, '*.fig');
end

function deleteMatching(folder, pattern)

    files = dir(fullfile(folder, pattern));
    for i = 1:numel(files)
        delete(fullfile(files(i).folder, files(i).name));
    end
end

function figureFiles = saveFigureList(figs, outputDir, resolution)

    figureFiles = strings(numel(figs),1);
    for i = 1:numel(figs)
        fig = figs(i);
        drawnow;
        name = figureFileStem(fig);
        pngPath = fullfile(outputDir, [name, '.png']);
        figPath = fullfile(outputDir, [name, '.fig']);
        exportgraphics(fig, pngPath, 'Resolution', resolution);
        savefig(fig, figPath);
        figureFiles(i) = string(pngPath);
    end
end

function name = figureFileStem(fig)

    figName = string(get(fig, 'Name'));
    if strlength(figName) == 0
        figName = "figure";
    end

    stem = matlab.lang.makeValidName(char(figName));
    name = stem;
end

function value = getStructFieldLocal(s, fieldName, defaultValue)

    if isstruct(s) && isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end

function ang = wrapToPiLocal(ang)

    ang = atan2(sin(ang), cos(ang));
end

function phi = LogSO3Local(R)

    q = rotmToQuatWXYZLocal(projectSO3Local(R));
    phi = quatLogVectorWXYZLocal(q);
end

function R = projectSO3Local(R)

    [U, ~, V] = svd(R);
    R = U*diag([1, 1, det(U*V')])*V';
end

function q = rotmToQuatWXYZLocal(R)

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

    q = normalizeQuatWXYZLocal([qw; qx; qy; qz]);
end

function q = normalizeQuatWXYZLocal(q)

    nq = norm(q);
    if nq < 1e-12
        q = [1; 0; 0; 0];
    else
        q = q/nq;
    end
end

function phi = quatLogVectorWXYZLocal(q)

    q = normalizeQuatWXYZLocal(q);
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
