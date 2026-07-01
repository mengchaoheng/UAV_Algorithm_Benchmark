function figureFiles = plot_main_sweep(varargin)
%PLOT_MAIN_SWEEP Replot detailed trajectory-sweep results.
%
% Usage:
%   plot_main_sweep("results/main_trajectory_sweep/geometric_indi")
%   plot_main_sweep(".../main_trajectory_sweep_results.mat")
%   plot_main_sweep(results, cfg)

    [results, cfg, args] = parseSweepInputs(varargin);
    opts = sweepPlotOptions(results, cfg, args);

    data = loadSweepRunData(results);
    figureFiles = strings(0,1);
    if isempty(data)
        return;
    end

    if opts.savePlots && opts.clearOutput
        prepareFigureDir(opts.outputDir);
    end

    fig = plotSweepTrajectory3D(data);
    if opts.savePlots
        figureFiles(end+1,1) = saveSweepFigure(fig, ...
            opts.outputDir, 'sweep_traj_3d', opts.resolution);
    end
    if ~opts.keepFigureWindows && ishandle(fig)
        close(fig);
    end

    fig = animateSweepTrajectories3D(data, opts);
    if opts.savePlots && ishandle(fig)
        figureFiles(end+1,1) = saveSweepFigure(fig, ...
            opts.outputDir, 'sweep_anim_3d', opts.resolution);
    end
    if ~opts.keepFigureWindows && ishandle(fig)
        close(fig);
    end

    if opts.savePlots
        fprintf('sweep figures saved to: %s\n', opts.outputDir);
    end
end

function [results, cfg, args] = parseSweepInputs(argsIn)

    if isempty(argsIn)
        dataPath = defaultSweepResultsPath();
        S = load(dataPath, 'results', 'cfg');
        results = S.results;
        cfg = S.cfg;
        args = {};
        return;
    end

    firstArg = argsIn{1};
    if istable(firstArg)
        results = firstArg;
        if numel(argsIn) >= 2 && isstruct(argsIn{2})
            cfg = argsIn{2};
            args = argsIn(3:end);
        else
            cfg = getfield(results.Properties.UserData, 'cfg'); %#ok<GFLD>
            args = argsIn(2:end);
        end
        return;
    end

    if ischar(firstArg) || isstring(firstArg)
        if isSweepOptionName(firstArg)
            dataPath = defaultSweepResultsPath();
            args = argsIn;
        else
            dataPath = string(firstArg);
            args = argsIn(2:end);
            if isfolder(dataPath)
                dataPath = fullfile(dataPath, "main_trajectory_sweep_results.mat");
            end
        end

        S = load(dataPath, 'results', 'cfg');
        results = S.results;
        cfg = S.cfg;
        return;
    end

    error('plot_main_sweep requires a sweep result folder/file or results table.');
end

function dataPath = defaultSweepResultsPath()

    rootDir = fullfile(pwd, 'results', 'main_trajectory_sweep');
    files = dir(fullfile(rootDir, '*', 'main_trajectory_sweep_results.mat'));
    if isempty(files)
        error('No main trajectory sweep result found under %s.', rootDir);
    end

    [~, idx] = max([files.datenum]);
    dataPath = fullfile(files(idx).folder, files(idx).name);
end

function tf = isSweepOptionName(name)

    names = ["saveplots", "outputdir", "resolution", "clearoutput", ...
        "keepfigurewindows", "animationspeed", "animationframedt"];
    tf = ismember(lower(string(name)), names);
end

function opts = sweepPlotOptions(results, cfg, args)

    opts.savePlots = true;
    opts.outputDir = defaultSweepFigureDir(results, cfg);
    opts.resolution = 200;
    opts.clearOutput = true;
    opts.keepFigureWindows = true;
    opts.animationSpeed = [];
    opts.animationFrameDt = [];

    i = 1;
    while i <= numel(args)
        name = lower(string(args{i}));
        value = args{i+1};

        switch name
            case "saveplots"
                opts.savePlots = logical(value);
            case "outputdir"
                opts.outputDir = char(value);
            case "resolution"
                opts.resolution = double(value);
            case "clearoutput"
                opts.clearOutput = logical(value);
            case "keepfigurewindows"
                opts.keepFigureWindows = logical(value);
            case "animationspeed"
                opts.animationSpeed = double(value);
            case "animationframedt"
                opts.animationFrameDt = double(value);
            otherwise
                error('Unknown plot_main_sweep option: %s.', name);
        end

        i = i + 2;
    end
end

function outputDir = defaultSweepFigureDir(results, cfg)

    if isfield(cfg, 'outputDir') && strlength(string(cfg.outputDir)) > 0
        outputDir = fullfile(char(cfg.outputDir), 'figures');
    elseif isfield(results.Properties.UserData, 'outputDir')
        outputDir = fullfile(char(results.Properties.UserData.outputDir), ...
            'figures');
    else
        outputDir = fullfile(pwd, 'results', ...
            'main_trajectory_sweep', 'figures');
    end
end

function data = loadSweepRunData(results)

    data = struct('time', {}, 'log', {}, 'par', {}, 'traj', {}, 'label', {});
    for i = 1:height(results)
        matFile = char(results.MatFile(i));
        if strlength(results.ErrorMessage(i)) == 0 && isfile(matFile)
            S = load(matFile, 'time', 'log', 'par', 'traj');
            S.label = shortTrajectoryLabel(results.Trajectory(i));
            data(end+1) = S; 
        end
    end
end

function fig = plotSweepTrajectory3D(data)

    fig = figure('Color', 'w', 'Name', 'sweep_traj_3d');
    [rows, cols] = subplotGrid(numel(data));
    tl = tiledlayout(fig, rows, cols, 'TileSpacing', 'compact', ...
        'Padding', 'compact');
    title(tl, 'sweep_traj_3d', 'Interpreter', 'none');

    for i = 1:numel(data)
        ax = nexttile(tl);
        p = data(i).log.p;
        pd = data(i).log.pd;

        plot3(ax, p(1,:), p(2,:), p(3,:), ...
            'LineWidth', 1.5, 'DisplayName', 'actual');
        hold(ax, 'on');
        plot3(ax, pd(1,:), pd(2,:), pd(3,:), '--', ...
            'LineWidth', 1.2, 'DisplayName', 'ref');

        formatTrajectoryAxes(ax, [p, pd], data(i).label);
        legend(ax, 'Location', 'best', 'FontSize', 7, ...
            'Interpreter', 'none');
    end
end

function fig = animateSweepTrajectories3D(data, opts)

    fig = figure('Color', 'w', 'Name', 'sweep_anim_3d');
    [rows, cols] = subplotGrid(numel(data));
    tl = tiledlayout(fig, rows, cols, 'TileSpacing', 'compact', ...
        'Padding', 'compact');
    title(tl, 'sweep_anim_3d', 'Interpreter', 'none');

    hPath = gobjects(numel(data),1);
    hPoint = gobjects(numel(data),1);
    hx = gobjects(numel(data),1);
    hy = gobjects(numel(data),1);
    hz = gobjects(numel(data),1);
    lastIdx = ones(numel(data),1);
    posePLog = cell(numel(data),1);
    poseRLog = cell(numel(data),1);
    bodyAxisScale = zeros(numel(data),1);

    for i = 1:numel(data)
        ax = nexttile(tl);
        p = data(i).log.p;
        pd = data(i).log.pd;
        poseSource = getStructFieldLocal(data(i).par, ...
            'animationPoseSource', "actual");
        bodyAxisScale(i) = getStructFieldLocal(data(i).par, ...
            'animationBodyAxisScale', 0.3);
        [posePLog{i}, poseRLog{i}] = selectPoseLogLocal( ...
            data(i).log, poseSource);

        plot3(ax, pd(1,:), pd(2,:), pd(3,:), ':', ...
            'Color', [0.45 0.45 0.45], 'LineWidth', 1.0, ...
            'DisplayName', 'ref');
        hold(ax, 'on');
        hPath(i) = animatedline(ax, ...
            'LineWidth', 1.5, 'DisplayName', 'actual');
        addpoints(hPath(i), p(1,1), p(2,1), p(3,1));
        hPoint(i) = plot3(ax, p(1,1), p(2,1), p(3,1), 'o', ...
            'MarkerFaceColor', 'b', 'MarkerSize', 5, ...
            'DisplayName', 'vehicle');
        [origin, xB, yB, zB] = bodyAxesLocal(posePLog{i}, ...
            poseRLog{i}, 1, bodyAxisScale(i));
        hx(i) = quiver3(ax, origin(1), origin(2), origin(3), ...
            xB(1), xB(2), xB(3), 0, 'r', 'DisplayName', 'x_B');
        hy(i) = quiver3(ax, origin(1), origin(2), origin(3), ...
            yB(1), yB(2), yB(3), 0, 'g', 'DisplayName', 'y_B');
        hz(i) = quiver3(ax, origin(1), origin(2), origin(3), ...
            zB(1), zB(2), zB(3), 0, 'b', 'DisplayName', 'z_B');

        formatTrajectoryAxes(ax, [p, pd], data(i).label);
        if i == numel(data)
            legend(ax, 'Location', 'eastoutside', 'FontSize', 7, ...
                'Interpreter', 'none');
        end
    end

    basePar = data(1).par;
    frameDt = opts.animationFrameDt;
    if isempty(frameDt)
        frameDt = getStructFieldLocal(basePar, 'animationFrameDt', 0.02);
    end
    speed = opts.animationSpeed;
    if isempty(speed)
        speed = getStructFieldLocal(basePar, 'animationSpeed', 1);
    end
    frameDt = max(frameDt, eps);
    speed = max(speed, eps);
    tEnd = max(arrayfun(@(d) d.time(end), data));
    drawnow;
    t0 = tic;

    while ishandle(fig)
        frameStart = toc(t0);
        tNow = min(speed*frameStart, tEnd);

        if ~ishandle(fig)
            break;
        end

        for i = 1:numel(data)
            time = data(i).time;
            p = data(i).log.p;
            idx = find(time <= min(tNow, time(end)), 1, 'last');
            if isempty(idx)
                idx = 1;
            end

            if idx > lastIdx(i)
                addpoints(hPath(i), p(1,lastIdx(i)+1:idx), ...
                    p(2,lastIdx(i)+1:idx), p(3,lastIdx(i)+1:idx));
                lastIdx(i) = idx;
            end
            set(hPoint(i), 'XData', p(1,idx), ...
                'YData', p(2,idx), 'ZData', p(3,idx));

            [origin, xB, yB, zB] = bodyAxesLocal(posePLog{i}, ...
                poseRLog{i}, idx, bodyAxisScale(i));
            updateQuiverLocal(hx(i), origin, xB);
            updateQuiverLocal(hy(i), origin, yB);
            updateQuiverLocal(hz(i), origin, zB);
        end

        drawnow nocallbacks;

        if tNow >= tEnd
            break;
        end

        pause(max(0, frameDt/speed - (toc(t0) - frameStart)));
    end
end

function [poseP, poseR] = selectPoseLogLocal(log, poseSource)

    if string(poseSource) == "desired"
        poseP = log.pd;
        poseR = log.Rd;
    else
        poseP = log.p;
        poseR = log.R;
    end
end

function [origin, xB, yB, zB] = bodyAxesLocal(pLog, RLog, idx, scale)

    origin = pLog(:,idx);
    R = RLog(:,:,idx);
    xB = scale*R(:,1);
    yB = scale*R(:,2);
    zB = scale*R(:,3);
end

function updateQuiverLocal(h, origin, vec)

    set(h, 'XData', origin(1), 'YData', origin(2), 'ZData', origin(3), ...
        'UData', vec(1), 'VData', vec(2), 'WData', vec(3));
end

function formatTrajectoryAxes(ax, pAll, label)

    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 35, 25);
    set(ax, 'ZDir', 'reverse');
    xlabel(ax, 'x');
    ylabel(ax, 'y');
    zlabel(ax, 'z');
    title(ax, label, 'Interpreter', 'none');
    setLimitsLocal(ax, pAll);
    disableAxesToolbarLocal(ax);
end

function [rows, cols] = subplotGrid(n)

    cols = ceil(sqrt(n));
    rows = ceil(n/cols);
end

function label = shortTrajectoryLabel(labelIn)

    label = string(labelIn);
    label = regexprep(label, '_scaleRange_.*$', '');
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

function figureFile = saveSweepFigure(fig, outputDir, stem, resolution)

    drawnow;
    pngPath = fullfile(outputDir, [stem, '.png']);
    figPath = fullfile(outputDir, [stem, '.fig']);
    exportgraphics(fig, pngPath, 'Resolution', resolution);
    savefig(fig, figPath);
    figureFile = string(pngPath);
end

function setLimitsLocal(ax, pAll)

    span = max(max(pAll, [], 2) - min(pAll, [], 2));
    margin = 0.1*max(span, 1);
    xlim(ax, [min(pAll(1,:))-margin, max(pAll(1,:))+margin]);
    ylim(ax, [min(pAll(2,:))-margin, max(pAll(2,:))+margin]);
    zlim(ax, [min(pAll(3,:))-margin, max(pAll(3,:))+margin]);
end

function disableAxesToolbarLocal(ax)

    disableDefaultInteractivity(ax);
    if isprop(ax, 'Toolbar') && ~isempty(ax.Toolbar)
        ax.Toolbar.Visible = 'off';
    end
end

function value = getStructFieldLocal(s, fieldName, defaultValue)

    if isstruct(s) && isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end
