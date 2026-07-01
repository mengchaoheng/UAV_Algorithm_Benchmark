function results = run_disturbance_benchmark(cfg)
%RUN_DISTURBANCE_BENCHMARK Batch disturbance robustness comparison.
%
% Default comparison:
%   trajectories : fast_circle, figure8_horizontal, helix_flip
%   controllers  : geometric, lee, johnson, lu, tal, geometric_indi
%   disturbance  : low/medium/high additive force and moment amplitudes
%
% The script calls main.m in batch mode with parameter overrides, so main.m
% remains the single source of truth for dynamics, trajectories, and
% controllers. Results are stored as MAT/CSV files. cfg.numRepeats controls
% how many independent runs are made for each trajectory/controller/level.
% By default the boxcharts use all selected tracking-error samples from each
% run. Set cfg.boxDataSource="rmse" to draw boxcharts from one RMSE sample per
% repeated run. cfg.errorEvalMode can exclude the final prediction horizon so
% terminal-reference policy does not leak into a continuous-tracking benchmark.

    if nargin < 1
        cfg = struct();
    end

    repoDir = fileparts(mfilename('fullpath'));
    cfg = fillDisturbanceBenchmarkDefaults(cfg, repoDir);

    outDir = char(cfg.outputRoot);
    figDir = fullfile(outDir, 'figures');
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    if cfg.makePlots && ~exist(figDir, 'dir')
        mkdir(figDir);
    end

    levels = cfg.levels;
    nTotal = 0;
    for iCtrl = 1:numel(cfg.controllerNames)
        nTotal = nTotal + numel(cfg.trajNames) * numel(levels) ...
            * repeatCountForController(cfg, cfg.controllerNames(iCtrl));
    end

    Trajectory = strings(nTotal,1);
    Controller = strings(nTotal,1);
    DisturbanceType = strings(nTotal,1);
    DisturbanceLevel = strings(nTotal,1);
    Repeat = nan(nTotal,1);
    ForceAmpN = nan(nTotal,1);
    MomentAmpNm = nan(nTotal,1);
    ForceAmpX_N = nan(nTotal,1);
    ForceAmpY_N = nan(nTotal,1);
    ForceAmpZ_N = nan(nTotal,1);
    MomentAmpX_Nm = nan(nTotal,1);
    MomentAmpY_Nm = nan(nTotal,1);
    MomentAmpZ_Nm = nan(nTotal,1);
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

    overrides = cell(nTotal,1);
    row = 0;

    for iTraj = 1:numel(cfg.trajNames)
        trajName = string(cfg.trajNames(iTraj));

        for iCtrl = 1:numel(cfg.controllerNames)
            controllerName = string(cfg.controllerNames(iCtrl));
            nRepeats = repeatCountForController(cfg, controllerName);

            for iLevel = 1:numel(levels)
                level = levels(iLevel);

                for iRepeat = 1:nRepeats
                    row = row + 1;

                    Trajectory(row) = trajName;
                    Controller(row) = controllerName;
                    DisturbanceType(row) = string(cfg.disturbanceType);
                    DisturbanceLevel(row) = string(level.name);
                    Repeat(row) = iRepeat;
                    forceAmp = vector3Local(level.forceAmp);
                    momentAmp = vector3Local(level.momentAmp);

                    ForceAmpN(row) = norm(forceAmp);
                    MomentAmpNm(row) = norm(momentAmp);
                    ForceAmpX_N(row) = forceAmp(1);
                    ForceAmpY_N(row) = forceAmp(2);
                    ForceAmpZ_N(row) = forceAmp(3);
                    MomentAmpX_Nm(row) = momentAmp(1);
                    MomentAmpY_Nm(row) = momentAmp(2);
                    MomentAmpZ_Nm(row) = momentAmp(3);

                    override = makeDisturbanceOverride( ...
                        trajName, controllerName, level, iRepeat, cfg);
                    overrides{row} = override;
                end
            end
        end
    end

    errCell = cell(nTotal,1);
    simOk = false(nTotal,1);
    simMessage = strings(nTotal,1);
    evalInfoCell = cell(nTotal,1);

    if cfg.useParallel
        pool = startBenchmarkPool(cfg);
        fprintf('Running %d benchmark simulations in parallel with %d workers.\n', ...
            nTotal, pool.NumWorkers);
        parfor iRun = 1:nTotal
            [errCell{iRun}, simOk(iRun), simMessage(iRun), ...
                evalInfoCell{iRun}] = runBenchmarkSimulation( ...
                repoDir, overrides{iRun}, cfg);
        end
    else
        fprintf('Running %d benchmark simulations serially.\n', nTotal);
        for iRun = 1:nTotal
            [errCell{iRun}, simOk(iRun), simMessage(iRun), ...
                evalInfoCell{iRun}] = runBenchmarkSimulation( ...
                repoDir, overrides{iRun}, cfg);
        end
    end

    for iRun = 1:nTotal
        err = errCell{iRun};
        evalInfo = evalInfoCell{iRun};

        if simOk(iRun)
            RMSE(iRun) = sqrt(mean(err.^2, 'omitnan'));
            MeanError(iRun) = mean(err, 'omitnan');
            P95Error(iRun) = prctile(err, 95);
            MaxError(iRun) = max(err);
            FinalError(iRun) = err(end);
            EvalStartS(iRun) = evalInfo.startTime;
            EvalEndS(iRun) = evalInfo.endTime;
            FullEndS(iRun) = evalInfo.fullEndTime;
            NumErrorSamples(iRun) = numel(err);
            ErrorTrace{iRun} = err(1:cfg.errorSampleStride:end).';
            IsFinite(iRun) = all(isfinite(err));

            fprintf(['[%3d/%3d] %-18s %-16s %-6s r=%02d ' ...
                'rmse=%.4g p95=%.4g max=%.4g\n'], ...
                iRun, nTotal, Trajectory(iRun), Controller(iRun), ...
                DisturbanceLevel(iRun), Repeat(iRun), RMSE(iRun), ...
                P95Error(iRun), MaxError(iRun));
        else
            ErrorMessage(iRun) = simMessage(iRun);
            fprintf('[%3d/%3d] FAILED %s %s %s r=%02d: %s\n', ...
                iRun, nTotal, Trajectory(iRun), Controller(iRun), ...
                DisturbanceLevel(iRun), Repeat(iRun), simMessage(iRun));
        end
    end

    results = table(Trajectory, Controller, DisturbanceType, ...
        DisturbanceLevel, Repeat, ForceAmpN, MomentAmpNm, ...
        ForceAmpX_N, ForceAmpY_N, ForceAmpZ_N, ...
        MomentAmpX_Nm, MomentAmpY_Nm, MomentAmpZ_Nm, ...
        RMSE, MeanError, P95Error, MaxError, FinalError, ...
        EvalStartS, EvalEndS, FullEndS, NumErrorSamples, ErrorTrace, ...
        IsFinite, ErrorMessage);

    figureFiles = strings(0,1);

    if cfg.makePlots
        figureFiles = render_disturbance_benchmark( ...
            results, cfg, 'OutputDir', figDir);
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
        cfg.trajNames = ["fast_circle", "figure8_horizontal", ...
            "helix_flip", "race_track_c"];
    end
    if ~isfield(cfg, 'controllerNames')
        cfg.controllerNames = ["geometric", "lee", "johnson", ...
            "lu", "tal", "geometric_indi"];
    end
    cfg.trajNames = string(cfg.trajNames);
    cfg.controllerNames = string(cfg.controllerNames);
    if ~isfield(cfg, 'levels')
        cfg.levels = struct( ...
            'name',     {'low', 'medium', 'high'}, ...
            'forceAmp', {[0.15; 0.15; 0.08], ...
                         [0.30; 0.30; 0.15], ...
                         [0.45; 0.45; 0.22]}, ...
            'momentAmp',{[0.025; 0.025; 0.0025], ...
                         [0.050; 0.050; 0.0050], ...
                         [0.080; 0.080; 0.0080]});
    end
    if ~isfield(cfg, 'disturbanceType')
        cfg.disturbanceType = "random";
    end
    if ~isfield(cfg, 'boxDataSource')
        cfg.boxDataSource = "time_error"; % "time_error" or "rmse"
    end
    if ~isfield(cfg, 'numRepeats')
        cfg.numRepeats = 1;
    else
        cfg.numRepeats = max(1, round(double(cfg.numRepeats)));
    end
    if ~isfield(cfg, 'controllerRepeats')
        cfg.controllerRepeats = struct();
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
    if ~isfield(cfg, 'forceFreq')
        cfg.forceFreq = [0.17; 0.23; 0.31];
    end
    if ~isfield(cfg, 'momentFreq')
        cfg.momentFreq = [0.19; 0.29; 0.37];
    end
    if ~isfield(cfg, 'forcePhase')
        cfg.forcePhase = [0; 2*pi/3; 4*pi/3];
    end
    if ~isfield(cfg, 'momentPhase')
        cfg.momentPhase = [pi/4; 3*pi/4; 5*pi/4];
    end
    if ~isfield(cfg, 'forceTau')
        cfg.forceTau = [1.5; 1.5; 1.0];
    end
    if ~isfield(cfg, 'momentTau')
        cfg.momentTau = [0.35; 0.35; 0.25];
    end
    if ~isfield(cfg, 'disturbanceSeedBase')
        cfg.disturbanceSeedBase = 24001;
    end
    if ~isfield(cfg, 'feedbackNoiseSeedBase')
        cfg.feedbackNoiseSeedBase = 43001;
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
    if ~isfield(cfg, 'useParallel')
        cfg.useParallel = false;
    end
    if ~isfield(cfg, 'numWorkers')
        cfg.numWorkers = [];
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

function nRepeats = repeatCountForController(cfg, controllerName)

    nRepeats = cfg.numRepeats;
    repeatOverrides = cfg.controllerRepeats;

    if isstruct(repeatOverrides)
        field = matlab.lang.makeValidName(char(controllerName));
        if isfield(repeatOverrides, field)
            nRepeats = repeatOverrides.(field);
        end
    elseif isa(repeatOverrides, 'containers.Map')
        key = char(controllerName);
        if isKey(repeatOverrides, key)
            nRepeats = repeatOverrides(key);
        end
    end

    nRepeats = max(1, round(double(nRepeats)));
end

function override = makeDisturbanceOverride( ...
        trajName, controllerName, level, repeatIndex, cfg)

    override.trajName = trajName;
    override.controllerName = controllerName;
    override.integratorName = cfg.integratorName;
    override.enablePlots = false;
    override.enableAnimation = false;
    override.saveResults = false;
    override.sun.acadosCodegenDir = fullfile(tempdir, ...
        "uav_sun_acados_codegen_" + matlab.lang.makeValidName( ...
        trajName + "_" + controllerName + "_" + string(level.name) ...
        + "_r" + string(repeatIndex)));

    if ~isempty(fieldnames(cfg.progress))
        override.progress = cfg.progress;
    end

    override.disturbance.enabled = true;
    override.disturbance.type = cfg.disturbanceType;
    override.disturbance.forceAmp = vector3Local(level.forceAmp);
    override.disturbance.momentAmp = vector3Local(level.momentAmp);
    override.disturbance.startTime = cfg.disturbanceStartTime;
    override.disturbance.endTime = cfg.disturbanceEndTime;
    override.disturbance.forceFreq = cfg.forceFreq;
    override.disturbance.momentFreq = cfg.momentFreq;
    override.disturbance.forcePhase = cfg.forcePhase;
    override.disturbance.momentPhase = cfg.momentPhase;
    override.disturbance.forceTau = cfg.forceTau;
    override.disturbance.momentTau = cfg.momentTau;
    override.disturbance.seed = stableSeed(cfg.disturbanceSeedBase, ...
        trajName, string(level.name), "repeat", repeatIndex);

    override.feedbackNoise.seed = stableSeed(cfg.feedbackNoiseSeedBase, ...
        trajName, string(level.name), "repeat", repeatIndex);
end

function v = vector3Local(x)

    v = x(:);

    if numel(v) ~= 3
        error("Disturbance direction must be a 3x1 vector.");
    end
end

function seed = stableSeed(baseSeed, varargin)

    seed = double(baseSeed);
    for i = 1:nargin-1
        text = char(string(varargin{i}));
        for j = 1:numel(text)
            seed = mod(seed*33 + double(text(j)), 2^31 - 1);
        end
    end
    seed = mod(round(seed), 2^31 - 1);
end

function pool = startBenchmarkPool(cfg)

    pool = gcp('nocreate');
    if ~isempty(pool)
        return;
    end

    if isempty(cfg.numWorkers)
        pool = parpool;
    else
        pool = parpool(cfg.numWorkers);
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
