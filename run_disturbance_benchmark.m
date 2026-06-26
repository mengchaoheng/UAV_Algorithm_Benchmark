function results = run_disturbance_benchmark(cfg)
%RUN_DISTURBANCE_BENCHMARK Batch disturbance robustness comparison.
%
% Default comparison:
%   trajectories : fast_circle, figure8_horizontal, helix_flip
%   controllers  : geometric, on_manifold_mpc, geometric_indi
%   disturbance  : low/medium/high additive force and moment amplitudes
%
% The script calls main.m in batch mode with parameter overrides, so main.m
% remains the single source of truth for dynamics, trajectories, and
% controllers. Results are stored as MAT/CSV files. By default the boxcharts
% use all discrete-time tracking-error samples from each run, not repeated
% Monte-Carlo trial scalars. Figures are always displayed; cfg.savePlots only
% controls whether PNG copies are written to disk.

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
           * numel(levels) * cfg.numTrials;

    Trajectory = strings(nTotal,1);
    Controller = strings(nTotal,1);
    DisturbanceType = strings(nTotal,1);
    DisturbanceLevel = strings(nTotal,1);
    ForceAmpN = nan(nTotal,1);
    MomentAmpNm = nan(nTotal,1);
    Trial = nan(nTotal,1);
    Seed = nan(nTotal,1);
    RMSE = nan(nTotal,1);
    MeanError = nan(nTotal,1);
    P95Error = nan(nTotal,1);
    MaxError = nan(nTotal,1);
    FinalError = nan(nTotal,1);
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

                for iTrial = 1:cfg.numTrials
                    row = row + 1;
                    seed = cfg.seed0 + 10000*iTraj + 1000*iCtrl ...
                         + 100*iLevel + iTrial;

                    Trajectory(row) = trajName;
                    Controller(row) = controllerName;
                    DisturbanceType(row) = cfg.disturbanceType;
                    DisturbanceLevel(row) = string(level.name);
                    ForceAmpN(row) = level.forceAmp;
                    MomentAmpNm(row) = level.momentAmp;
                    Trial(row) = iTrial;
                    Seed(row) = seed;

                    override = makeDisturbanceOverride( ...
                        trajName, controllerName, level, seed, cfg);

                    try
                        clear('main'); % reset script-local persistent states.
                        UAV_BENCHMARK_BATCH = true; %#ok<NASGU>
                        UAV_BENCHMARK_PAR_OVERRIDE = override; %#ok<NASGU>
                        run(fullfile(repoDir, 'main.m'));

                        err = vecnorm(log.p - log.pd, 2, 1);
                        RMSE(row) = sqrt(mean(err.^2, 'omitnan'));
                        MeanError(row) = mean(err, 'omitnan');
                        P95Error(row) = prctile(err, 95);
                        MaxError(row) = max(err);
                        FinalError(row) = err(end);
                        ErrorTrace{row} = err(1:cfg.errorSampleStride:end).';
                        IsFinite(row) = all(isfinite(err)) ...
                            && all(isfinite(log.T)) ...
                            && all(isfinite(log.tau(:)));

                        fprintf(['[%3d/%3d] %-18s %-16s %-6s trial=%02d ' ...
                            'rmse=%.4g p95=%.4g max=%.4g\n'], ...
                            row, nTotal, trajName, controllerName, ...
                            string(level.name), iTrial, RMSE(row), ...
                            P95Error(row), MaxError(row));

                    catch ME
                        ErrorMessage(row) = string(ME.message);
                        fprintf('[%3d/%3d] FAILED %s %s %s trial=%02d: %s\n', ...
                            row, nTotal, trajName, controllerName, ...
                            string(level.name), iTrial, ME.message);
                    end

                    clear UAV_BENCHMARK_BATCH UAV_BENCHMARK_PAR_OVERRIDE
                end
            end
        end
    end

    results = table(Trajectory, Controller, DisturbanceType, ...
        DisturbanceLevel, ForceAmpN, MomentAmpNm, Trial, Seed, ...
        RMSE, MeanError, P95Error, MaxError, FinalError, ErrorTrace, ...
        IsFinite, ErrorMessage);

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
        cfg.controllerNames = ["geometric", "on_manifold_mpc", "geometric_indi"];
    end
    if ~isfield(cfg, 'levels')
        cfg.levels = struct( ...
            'name',     {'low', 'medium', 'high'}, ...
            'forceAmp', {0.05,  0.15,     0.30}, ...
            'momentAmp',{0.002, 0.006,    0.012});
    end
    if ~isfield(cfg, 'numTrials')
        cfg.numTrials = 1;
    end
    if ~isfield(cfg, 'boxDataSource')
        cfg.boxDataSource = "time_error"; % "time_error" or "rmse"
    end
    if ~isfield(cfg, 'errorSampleStride')
        cfg.errorSampleStride = 1; % Use every simulated time step in boxcharts.
    else
        cfg.errorSampleStride = max(1, round(cfg.errorSampleStride));
    end
    if ~isfield(cfg, 'disturbanceType')
        cfg.disturbanceType = "sin"; % "sin" or "random"
    end
    if ~isfield(cfg, 'integratorName')
        cfg.integratorName = "lie_rk4"; % Faster for large sweeps; use "ode45" to match main default.
    end
    if ~isfield(cfg, 'randomHold')
        cfg.randomHold = 0.05;
    end
    if ~isfield(cfg, 'seed0')
        cfg.seed0 = 202601;
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

function override = makeDisturbanceOverride(trajName, controllerName, level, seed, cfg)

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
    override.disturbance.forceAmp = level.forceAmp;
    override.disturbance.momentAmp = level.momentAmp;
    override.disturbance.seed = seed;
    override.disturbance.randomHold = cfg.randomHold;

    if string(cfg.disturbanceType) == "sin"
        rng(seed, 'twister');
        override.disturbance.forcePhase = 2*pi*rand(3,1);
        override.disturbance.momentPhase = 2*pi*rand(3,1);
    end
end

function figureFiles = plotDisturbanceBenchmark(results, cfg, figDir)

    levelNames = string({cfg.levels.name});
    controllerNames = string(cfg.controllerNames);
    figureFiles = strings(0,1);

    for iTraj = 1:numel(cfg.trajNames)
        trajName = string(cfg.trajNames(iTraj));
        mask = results.Trajectory == trajName & results.IsFinite;

        fig = figure('Color', 'w', ...
            'Name', "disturbance benchmark: " + trajName);

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
