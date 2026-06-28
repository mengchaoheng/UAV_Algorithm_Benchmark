function results = run_disturbance_benchmark(cfg)
%RUN_DISTURBANCE_BENCHMARK Batch disturbance robustness comparison.
%
% Default comparison:
%   trajectories : fast_circle, figure8_horizontal, helix_flip
%   controllers  : geometric, lu_on_manifold_lqr, geometric_indi
%   disturbance  : low/medium/high additive force and moment amplitudes
%
% The script calls main.m in batch mode with parameter overrides, so main.m
% remains the single source of truth for dynamics, trajectories, and
% controllers. Results are stored as MAT/CSV files. By default the boxcharts
% use all selected tracking-error samples from each run. cfg.errorEvalMode can
% exclude the final prediction horizon so terminal-reference policy does not
% leak into a continuous-tracking benchmark. Figures are always displayed;
% cfg.savePlots only controls whether PNG copies are written to disk.

    if nargin < 1
        cfg = struct();
    end

    repoDir = fileparts(mfilename('fullpath'));
    cfg = fillDisturbanceBenchmarkDefaults(cfg, repoDir);

    runId = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outDir = fullfile(cfg.outputRoot, char(runId));
    figDir = fullfile(outDir, 'figures');
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    if cfg.savePlots && ~exist(figDir, 'dir')
        mkdir(figDir);
    end

    levels = cfg.levels;
    nTotal = numel(cfg.trajNames) * numel(cfg.controllerNames) ...
           * numel(levels);

    Trajectory = strings(nTotal,1);
    Controller = strings(nTotal,1);
    DisturbanceType = strings(nTotal,1);
    DisturbanceLevel = strings(nTotal,1);
    ForceAmpN = nan(nTotal,1);
    MomentAmpNm = nan(nTotal,1);
    RMSE = nan(nTotal,1);
    MeanError = nan(nTotal,1);
    P95Error = nan(nTotal,1);
    MaxError = nan(nTotal,1);
    FinalError = nan(nTotal,1);
    EvalStartS = nan(nTotal,1);
    EvalEndS = nan(nTotal,1);
    FullEndS = nan(nTotal,1);
    NumErrorSamples = nan(nTotal,1);
    ErrorTrace = cell(nTotal,1);
    IsFinite = false(nTotal,1);
    ErrorMessage = strings(nTotal,1);

    row = 0;

    for iTraj = 1:numel(cfg.trajNames)
        trajName = string(cfg.trajNames(iTraj));

        for iCtrl = 1:numel(cfg.controllerNames)
            controllerName = string(cfg.controllerNames(iCtrl));

            for iLevel = 1:numel(levels)
                level = levels(iLevel);

                row = row + 1;

                Trajectory(row) = trajName;
                Controller(row) = controllerName;
                DisturbanceType(row) = string(cfg.disturbanceType);
                DisturbanceLevel(row) = string(level.name);
                ForceAmpN(row) = level.forceAmp;
                MomentAmpNm(row) = level.momentAmp;

                override = makeDisturbanceOverride( ...
                    trajName, controllerName, level, cfg);

                [err, simOk, simMessage, evalInfo] = runBenchmarkSimulation( ...
                    repoDir, override, cfg);

                if simOk
                    RMSE(row) = sqrt(mean(err.^2, 'omitnan'));
                    MeanError(row) = mean(err, 'omitnan');
                    P95Error(row) = prctile(err, 95);
                    MaxError(row) = max(err);
                    FinalError(row) = err(end);
                    EvalStartS(row) = evalInfo.startTime;
                    EvalEndS(row) = evalInfo.endTime;
                    FullEndS(row) = evalInfo.fullEndTime;
                    NumErrorSamples(row) = numel(err);
                    ErrorTrace{row} = err(1:cfg.errorSampleStride:end).';
                    IsFinite(row) = all(isfinite(err));

                    fprintf('[%3d/%3d] %-18s %-16s %-6s rmse=%.4g p95=%.4g max=%.4g\n', ...
                        row, nTotal, trajName, controllerName, ...
                        string(level.name), RMSE(row), P95Error(row), ...
                        MaxError(row));

                else
                    ErrorMessage(row) = simMessage;
                    fprintf('[%3d/%3d] FAILED %s %s %s: %s\n', ...
                        row, nTotal, trajName, controllerName, ...
                        string(level.name), simMessage);
                end
            end
        end
    end

    results = table(Trajectory, Controller, DisturbanceType, ...
        DisturbanceLevel, ForceAmpN, MomentAmpNm, RMSE, MeanError, ...
        P95Error, MaxError, FinalError, EvalStartS, EvalEndS, FullEndS, ...
        NumErrorSamples, ErrorTrace, IsFinite, ErrorMessage);

    figureFiles = strings(0,1);

    if cfg.makePlots
        figureFiles = plotDisturbanceBenchmark(results, cfg, figDir);
    end

    results.Properties.UserData.outputDir = outDir;
    results.Properties.UserData.figureFiles = figureFiles;

    save(fullfile(outDir, 'disturbance_benchmark_results.mat'), ...
        'results', 'cfg');
    csvResults = removevars(results, "ErrorTrace");
    writetable(csvResults, fullfile(outDir, 'disturbance_benchmark_results.csv'));

    fprintf('Saved disturbance benchmark results to:\n  %s\n', outDir);

    if ~isempty(figureFiles)
        fprintf('Saved figure files:\n');
        for i = 1:numel(figureFiles)
            fprintf('  %s\n', figureFiles(i));
        end
    end
end

function cfg = fillDisturbanceBenchmarkDefaults(cfg, repoDir)

    if ~isfield(cfg, 'trajNames')
        cfg.trajNames = ["fast_circle", "figure8_horizontal", "helix_flip"];
    end
    if ~isfield(cfg, 'controllerNames')
        cfg.controllerNames = ["geometric", "lu_on_manifold_lqr", "geometric_indi"];
    end
    if ~isfield(cfg, 'levels')
        cfg.levels = struct( ...
            'name',     {'low', 'medium', 'high'}, ...
            'forceAmp', {0.05,  0.15,     0.30}, ...
            'momentAmp',{0.002, 0.006,    0.012});
    end
    if ~isfield(cfg, 'disturbanceType')
        cfg.disturbanceType = "sin";
    end
    if ~isfield(cfg, 'boxDataSource')
        cfg.boxDataSource = "time_error"; % "time_error" or "rmse"
    end
    if ~isfield(cfg, 'errorSampleStride')
        cfg.errorSampleStride = 1; % Use every simulated time step in boxcharts.
    else
        cfg.errorSampleStride = max(1, round(cfg.errorSampleStride));
    end
    if ~isfield(cfg, 'errorEvalMode')
        cfg.errorEvalMode = "full"; % "full", "fixed_trim", or "sun_prediction_horizon"
    end
    if ~isfield(cfg, 'errorEvalStartTime')
        cfg.errorEvalStartTime = 0;
    end
    if ~isfield(cfg, 'errorEvalEndTime')
        cfg.errorEvalEndTime = inf;
    end
    if ~isfield(cfg, 'errorTrimEndTime')
        cfg.errorTrimEndTime = 0;
    end
    if ~isfield(cfg, 'forcePhase')
        cfg.forcePhase = [0; 2*pi/3; 4*pi/3];
    end
    if ~isfield(cfg, 'momentPhase')
        cfg.momentPhase = [pi/4; 3*pi/4; 5*pi/4];
    end
    if ~isfield(cfg, 'disturbanceStartTime')
        cfg.disturbanceStartTime = 0;
    end
    if ~isfield(cfg, 'disturbanceEndTime')
        cfg.disturbanceEndTime = inf;
    end
    if ~isfield(cfg, 'integratorName')
        cfg.integratorName = "lie_rk4"; % Faster for large sweeps; use "ode45" to match main default.
    end
    if ~isfield(cfg, 'progress')
        cfg.progress = struct(); % Empty means use main.m defaults.
    end
    if ~isfield(cfg, 'makePlots')
        cfg.makePlots = true;
    end
    if ~isfield(cfg, 'savePlots')
        cfg.savePlots = false;
    end
    if ~isfield(cfg, 'outputRoot')
        cfg.outputRoot = fullfile(repoDir, 'results', 'disturbance_benchmark');
    end
end

function override = makeDisturbanceOverride(trajName, controllerName, level, cfg)

    override.trajName = trajName;
    override.controllerName = controllerName;
    override.integratorName = cfg.integratorName;
    override.enablePlots = false;
    override.enableAnimation = false;

    if ~isempty(fieldnames(cfg.progress))
        override.progress = cfg.progress;
    end

    override.disturbance.enabled = true;
    override.disturbance.type = cfg.disturbanceType;
    override.disturbance.forceAmp = directedAmplitude( ...
        level.forceAmp, cfg, 'forceDirection');
    override.disturbance.momentAmp = directedAmplitude( ...
        level.momentAmp, cfg, 'momentDirection');
    override.disturbance.startTime = cfg.disturbanceStartTime;
    override.disturbance.endTime = cfg.disturbanceEndTime;
    if isfield(cfg, 'forceFreq')
        override.disturbance.forceFreq = cfg.forceFreq;
    end
    if isfield(cfg, 'momentFreq')
        override.disturbance.momentFreq = cfg.momentFreq;
    end
    override.disturbance.forcePhase = cfg.forcePhase;
    override.disturbance.momentPhase = cfg.momentPhase;
end

function amp = directedAmplitude(nominalAmp, cfg, directionField)

    amp = nominalAmp;

    if ~isfield(cfg, directionField) || isempty(cfg.(directionField))
        return;
    end

    direction = vector3Local(cfg.(directionField));
    directionNorm = norm(direction);

    if directionNorm < eps
        amp = zeros(3,1);
    else
        amp = nominalAmp*direction/directionNorm;
    end
end

function v = vector3Local(x)

    v = x(:);

    if numel(v) ~= 3
        error("Disturbance direction must be a 3x1 vector.");
    end
end

function [trackingErr, isFinite, message, evalInfo] = runBenchmarkSimulation( ...
        repoDir, override, cfg)

    trackingErr = [];
    isFinite = false;
    message = "";
    evalInfo = struct( ...
        'startTime', nan, ...
        'endTime', nan, ...
        'fullEndTime', nan);

    try
        clear('main'); % reset script-local persistent states.
        UAV_BENCHMARK_BATCH = true; %#ok<NASGU>
        UAV_BENCHMARK_PAR_OVERRIDE = override; %#ok<NASGU>
        run(fullfile(repoDir, 'main.m'));

        fullTrackingErr = vecnorm(log.p - log.pd, 2, 1);
        [trackingErr, evalInfo] = selectTrackingErrorWindow( ...
            fullTrackingErr, time, par, cfg);
        isFinite = all(isfinite(fullTrackingErr)) ...
            && all(isfinite(log.T)) ...
            && all(isfinite(log.tau(:)));
        if ~isFinite
            message = "Simulation produced non-finite values.";
        elseif isempty(trackingErr)
            isFinite = false;
            message = "Error evaluation window is empty.";
        end

    catch ME
        message = string(ME.message);
    end
end

function [err, info] = selectTrackingErrorWindow(fullErr, time, par, cfg)

    time = time(:).';
    fullErr = fullErr(:).';

    startTime = max(0, double(cfg.errorEvalStartTime));
    fullEndTime = time(end);
    endTime = min(fullEndTime, double(cfg.errorEvalEndTime));

    switch string(cfg.errorEvalMode)
        case "full"
            trimEndTime = 0;

        case "fixed_trim"
            trimEndTime = double(cfg.errorTrimEndTime);

        case "sun_prediction_horizon"
            trimEndTime = par.sun.N*par.sun.dt;

        otherwise
            error("Unknown cfg.errorEvalMode.");
    end

    endTime = min(endTime, fullEndTime - max(trimEndTime, 0));
    mask = time >= startTime & time <= endTime;
    err = fullErr(mask);

    info.startTime = startTime;
    info.endTime = endTime;
    info.fullEndTime = fullEndTime;
end

function figureFiles = plotDisturbanceBenchmark(results, cfg, figDir)

    levelNames = string({cfg.levels.name});
    controllerNames = string(cfg.controllerNames);
    figureFiles = strings(0,1);

    for iTraj = 1:numel(cfg.trajNames)
        trajName = string(cfg.trajNames(iTraj));
        mask = results.Trajectory == trajName & results.IsFinite;

        fig = figure('Color', 'w');

        [xLabel, yData, groupLabel, yLabelText] = boxchartData(results, mask, cfg);

        x = categorical(xLabel, levelNames, levelNames);
        group = categorical(groupLabel, controllerNames, controllerNames);

        boxchart(x, yData, 'GroupByColor', group);
        grid on;
        xlabel('disturbance amplitude');
        ylabel(yLabelText);
        title("Trajectory: " + trajName, 'Interpreter', 'none');
        legend('Location', 'northwest', 'Interpreter', 'none');

        ampText = amplitudeSummaryText(cfg.levels);
        subtitle(ampText, 'Interpreter', 'none');

        if cfg.savePlots
            pngPath = fullfile(figDir, char(trajName + "_disturbance_boxplot.png"));
            exportgraphics(fig, pngPath, 'Resolution', 200);
            figureFiles(end+1,1) = string(pngPath); %#ok<AGROW>
        end

    end
end

function [xLabel, yData, groupLabel, yLabelText] = boxchartData(results, mask, cfg)

    source = string(cfg.boxDataSource);

    switch source
        case "time_error"
            rowIdx = find(mask).';
            xLabel = strings(0,1);
            groupLabel = strings(0,1);
            yData = zeros(0,1);

            for r = rowIdx
                err = results.ErrorTrace{r};
                err = err(isfinite(err));
                n = numel(err);

                xLabel = [xLabel; repmat(results.DisturbanceLevel(r), n, 1)]; %#ok<AGROW>
                groupLabel = [groupLabel; repmat(results.Controller(r), n, 1)]; %#ok<AGROW>
                yData = [yData; err(:)]; %#ok<AGROW>
            end

            yLabelText = 'position tracking error samples (m)';

        case "rmse"
            xLabel = results.DisturbanceLevel(mask);
            groupLabel = results.Controller(mask);
            yData = results.RMSE(mask);
            yLabelText = 'RMS position tracking error (m)';

        otherwise
            error("Unknown cfg.boxDataSource. Use 'time_error' or 'rmse'.");
    end
end

function txt = amplitudeSummaryText(levels)

    parts = strings(1, numel(levels));
    for i = 1:numel(levels)
        parts(i) = sprintf('%s: %.3g N / %.3g N*m', ...
            string(levels(i).name), levels(i).forceAmp, levels(i).momentAmp);
    end
    txt = strjoin(parts, ', ');
end
