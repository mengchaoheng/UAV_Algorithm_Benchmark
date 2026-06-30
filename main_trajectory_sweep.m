%% main_trajectory_sweep.m
% Run one controller on multiple trajectories and keep full main.m logs.
%
% Each run is saved under:
%   results/main_trajectory_sweep/<controller>/<trajectory>/main_run.mat
%
% Replot one trajectory without rerunning simulation:
%   plot_main("results/main_trajectory_sweep/geometric_indi/helix_flip/main_run.mat")
%
% Multi-trajectory runs skip per-run state-detail windows. They draw one
% combined static 3D figure and, when enabled, one combined animation figure.

if exist('UAV_MAIN_SWEEP_BATCH', 'var') && UAV_MAIN_SWEEP_BATCH
    if exist('UAV_MAIN_SWEEP_CFG_OVERRIDE', 'var')
        sweepOverride__ = UAV_MAIN_SWEEP_CFG_OVERRIDE;
    else
        sweepOverride__ = struct();
    end
else
    clear; clc; close all;
    sweepOverride__ = struct();
end

%% ========================================================================
%% 0. What to Run

controllerName = "geometric_indi";

trajNames = ["figure8_horizontal", "figure8_vertical", ...
    "helix_flip", "flip_loop_sine", "fast_circle"];
% trajNames = "all";
% trajNames = ["helix_flip"];

%% ========================================================================
%% 1. Simulation and Output Settings

integratorName = "lie_rk4";

makePlots = true;
savePlots = true;
plotStateDetail = true;       % single-trajectory only.
keepFigureWindows = true;

outputRoot = fullfile(pwd, "results", "main_trajectory_sweep");

% Optional extra parameter overrides passed to main.m every run.
extraOverride = struct();
extraOverride.disturbance.enabled = false;

%% ========================================================================
%% 2. Build Config and Run

cfg.controllerName = controllerName;
cfg.trajNames = trajNames;
cfg.integratorName = integratorName;
cfg.makePlots = makePlots;
cfg.savePlots = savePlots;
cfg.plotStateDetail = plotStateDetail;
cfg.keepFigureWindows = keepFigureWindows;
cfg.outputRoot = outputRoot;
cfg.extraOverride = extraOverride;

if ~isempty(fieldnames(sweepOverride__))
    cfg = mergeStructLocal(cfg, sweepOverride__);
end

results = runMainTrajectorySweep(cfg);

%% ========================================================================
%% 3. Output

disp("Detailed sweep directory:");
disp(results.Properties.UserData.outputDir);

disp("Detailed trajectory runs:");
disp(results);

%% ========================================================================
%% Local functions
function results = runMainTrajectorySweep(cfg)

    repoDir = fileparts(mfilename('fullpath'));
    cfg = fillMainTrajectorySweepDefaults(cfg, repoDir);

    controllerDir = fullfile(char(cfg.outputRoot), ...
        matlab.lang.makeValidName(char(cfg.controllerName)));
    if ~exist(controllerDir, 'dir')
        mkdir(controllerDir);
    end

    trajNames = expandTrajectoryNames(cfg.trajNames);
    nRuns = numel(trajNames);
    multiInstance = nRuns > 1;

    Trajectory = strings(nRuns,1);
    Controller = strings(nRuns,1);
    RMSE = nan(nRuns,1);
    MeanError = nan(nRuns,1);
    P95Error = nan(nRuns,1);
    MaxError = nan(nRuns,1);
    FinalError = nan(nRuns,1);
    Tend = nan(nRuns,1);
    NumSamples = nan(nRuns,1);
    ResultDir = strings(nRuns,1);
    MatFile = strings(nRuns,1);
    FigureDir = strings(nRuns,1);
    IsFinite = false(nRuns,1);
    ErrorMessage = strings(nRuns,1);

    for iRun = 1:nRuns
        trajName = string(trajNames(iRun));
        runDir = fullfile(controllerDir, ...
            matlab.lang.makeValidName(char(trajName)));

        Trajectory(iRun) = trajName;
        Controller(iRun) = string(cfg.controllerName);
        ResultDir(iRun) = string(runDir);
        MatFile(iRun) = string(fullfile(runDir, 'main_run.mat'));
        FigureDir(iRun) = string(fullfile(runDir, 'figures'));

        try
            if ~exist(runDir, 'dir')
                mkdir(runDir);
            end

            override = cfg.extraOverride;
            override.trajName = trajName;
            override.controllerName = string(cfg.controllerName);
            override.integratorName = string(cfg.integratorName);
            override.enablePlots = logical(cfg.makePlots) && ~multiInstance;
            override.saveResults = true;
            override.saveMat = true;
            override.saveFigures = logical(cfg.savePlots);
            override.plotStateDetail = logical(cfg.plotStateDetail);
            override.resultDir = runDir;

            clear('main'); % reset script-local persistent states.
            UAV_BENCHMARK_BATCH = true; %#ok<NASGU>
            UAV_BENCHMARK_PAR_OVERRIDE = override; %#ok<NASGU>
            run(fullfile(repoDir, 'main.m'));

            err = vecnorm(log.p - log.pd, 2, 1);
            RMSE(iRun) = sqrt(mean(err.^2, 'omitnan'));
            MeanError(iRun) = mean(err, 'omitnan');
            P95Error(iRun) = prctile(err, 95);
            MaxError(iRun) = max(err);
            FinalError(iRun) = err(end);
            Tend(iRun) = time(end);
            NumSamples(iRun) = numel(time);
            IsFinite(iRun) = all(isfinite(err)) ...
                && all(isfinite(log.T)) ...
                && all(isfinite(log.tau(:)));

            fprintf('[%2d/%2d] %-18s %-16s rmse=%.4g p95=%.4g max=%.4g\n', ...
                iRun, nRuns, trajName, string(cfg.controllerName), ...
                RMSE(iRun), P95Error(iRun), MaxError(iRun));

            if ~cfg.keepFigureWindows && ~multiInstance
                close all;
            end

        catch ME
            ErrorMessage(iRun) = string(ME.message);
            fprintf('[%2d/%2d] FAILED %s %s: %s\n', ...
                iRun, nRuns, trajName, string(cfg.controllerName), ...
                ErrorMessage(iRun));
        end
    end

    results = table(Trajectory, Controller, RMSE, MeanError, P95Error, ...
        MaxError, FinalError, Tend, NumSamples, ResultDir, MatFile, ...
        FigureDir, IsFinite, ErrorMessage);
    results.Properties.UserData.outputDir = controllerDir;
    results.Properties.UserData.cfg = cfg;

    save(fullfile(controllerDir, 'main_trajectory_sweep_results.mat'), ...
        'results', 'cfg');
    writetable(results, ...
        fullfile(controllerDir, 'main_trajectory_sweep_results.csv'));

    if multiInstance
        plotSweepFigures(results, cfg, controllerDir);
    end
end

function cfg = fillMainTrajectorySweepDefaults(cfg, repoDir)

    if ~isfield(cfg, 'controllerName')
        cfg.controllerName = "geometric_indi";
    end
    if ~isfield(cfg, 'trajNames')
        cfg.trajNames = "all";
    end
    if ~isfield(cfg, 'integratorName')
        cfg.integratorName = "lie_rk4";
    end
    if ~isfield(cfg, 'makePlots')
        cfg.makePlots = true;
    end
    if ~isfield(cfg, 'savePlots')
        cfg.savePlots = true;
    end
    if ~isfield(cfg, 'plotStateDetail')
        cfg.plotStateDetail = true;
    end
    if ~isfield(cfg, 'keepFigureWindows')
        cfg.keepFigureWindows = true;
    end
    if ~isfield(cfg, 'outputRoot')
        cfg.outputRoot = fullfile(repoDir, 'results', 'main_trajectory_sweep');
    end
    if ~isfield(cfg, 'extraOverride')
        cfg.extraOverride = struct();
    end
end

function trajNames = expandTrajectoryNames(trajNames)

    trajNames = string(trajNames);
    if isscalar(trajNames) && trajNames == "all"
        trajNames = ["figure8_horizontal", "figure8_vertical", ...
            "helix_flip", "flip_loop_sine", "fast_circle"];
    end
end

function plotSweepFigures(results, cfg, controllerDir)

    if ~cfg.makePlots
        return;
    end

    plot_main_sweep(results, cfg, ...
        'OutputDir', fullfile(controllerDir, 'figures'), ...
        'SavePlots', cfg.savePlots, ...
        'ClearOutput', true, ...
        'KeepFigureWindows', cfg.keepFigureWindows);
end

function dst = mergeStructLocal(dst, src)

    names = fieldnames(src);
    for i = 1:numel(names)
        name = names{i};
        if isstruct(src.(name)) && isfield(dst, name) ...
                && isstruct(dst.(name))
            dst.(name) = mergeStructLocal(dst.(name), src.(name));
        else
            dst.(name) = src.(name);
        end
    end
end
