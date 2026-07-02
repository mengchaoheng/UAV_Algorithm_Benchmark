%% main_disturbance_monte_carlo.m
% Repeated disturbance benchmark. Each run produces one RMSE value; the
% figures draw RMSE boxcharts across repeated trials for each controller and
% disturbance level.

clear; clc;

%% ========================================================================
%% 0. What to Run

trajNames = ["figure8_horizontal", "helix_flip"];

% Available controllers:
% "geometric", "lee", "johnson"
% "sun_dfbc", "sun_dfbc_indi"
% "sun_nmpc", "sun_nmpc_indi"
% "lu", "tal", "geometric_indi"
controllerNames = ["lee", "johnson", "sun_dfbc_indi", ...
     "sun_nmpc_indi", "lu", "tal", "geometric_indi"];
% controllerNames = ["tal", "geometric_indi"];
% controllerNames = ["tal"];

%% ========================================================================
%% 1. Monte Carlo Settings

% Default repeats per controller. Increase for final figures.
numRepeats = 10;

% Optional per-controller overrides. Field names use MATLAB-valid controller
% names; all current controller names are already valid struct fields.
controllerRepeats = struct();
% controllerRepeats.sun_nmpc_indi = 8;
% controllerRepeats.lu = 30;

% This script plots one RMSE sample per repeated simulation.
boxDataSource = "rmse";

% A repeated trial is treated as failed, and excluded from the plotted
% boxchart, if its 3-D position RMSE exceeds this threshold. Sun et al. use
% 5 m for failure counting. Set to inf to keep every finite trial.
failureRmseThreshold = 5; % [m]

%% ========================================================================
%% 2. Disturbance Scenario

% The disturbance "scenarios" are the levels in cfg.levels: low/medium/high.
% Change this preset to compare deterministic paper cases instead.
disturbanceCase = "random_gust";
d = disturbancePreset(disturbanceCase);

%% ========================================================================
%% 3. Plot and Simulation Settings

makePlots = true;
savePlots = true;

% "lie_rk4" is faster for repeated sweeps. Use "ode45" to match main.m.
integratorName = "lie_rk4";

useParallel = true;
numWorkers = [];

errorEvalMode = "sun_prediction_horizon";

outputRoot = fullfile(pwd, "results", "disturbance_monte_carlo", ...
    char(disturbanceCase));

%% ========================================================================
%% 4. Build Config and Run

cfg.trajNames = trajNames;
cfg.controllerNames = controllerNames;

cfg.levels = d.levels;
cfg.disturbanceType = d.type;
cfg.disturbanceStartTime = d.startTime;
cfg.disturbanceEndTime = d.endTime;
cfg.forceFreq = d.forceFreq;
cfg.momentFreq = d.momentFreq;
cfg.forcePhase = d.forcePhase;
cfg.momentPhase = d.momentPhase;
cfg.forceTau = d.forceTau;
cfg.momentTau = d.momentTau;
cfg.disturbanceSeedBase = d.disturbanceSeedBase;

cfg.numRepeats = numRepeats;
cfg.controllerRepeats = controllerRepeats;
cfg.boxDataSource = boxDataSource;
cfg.failureRmseThreshold = failureRmseThreshold;

cfg.makePlots = makePlots;
cfg.savePlots = savePlots;

cfg.integratorName = integratorName;
cfg.useParallel = useParallel;
cfg.numWorkers = numWorkers;
cfg.errorEvalMode = errorEvalMode;
cfg.outputRoot = outputRoot;

results = run_disturbance_benchmark(cfg);

%% ========================================================================
%% 5. Output

disp("Monte Carlo results directory:");
disp(results.Properties.UserData.outputDir);

if ~isempty(results.Properties.UserData.figureFiles)
    disp("Saved figure files:");
    disp(results.Properties.UserData.figureFiles);
end

disp("Results table:");
disp(results);

%% ========================================================================
%% Local Functions

function d = disturbancePreset(caseName)

    d = struct();
    d.caseName = string(caseName);

    switch string(caseName)
        case "random_gust"
            d.type = "random";
            d.startTime = 0.0;
            d.endTime = inf;
            d.forceFreq = [0.17; 0.23; 0.31];
            d.momentFreq = [0.19; 0.29; 0.37];
            d.forcePhase = [0; 1*pi/3; 2*pi/3];
            d.momentPhase = [pi/4; 3*pi/4; 5*pi/4];
            d.forceTau = [1.5; 1.5; 1.0];
            d.momentTau = [0.35; 0.35; 0.25];
            d.disturbanceSeedBase = 24001;
            d.levels = struct( ...
                'name',     {'low', 'medium', 'high'}, ...
                'forceAmp', {[0.15; 0.15; 0.08], ...
                             [0.30; 0.30; 0.15], ...
                             [0.45; 0.45; 0.22]}, ...
                'momentAmp',{[0.025; 0.025; 0.0025], ...
                             [0.050; 0.050; 0.0050], ...
                             [0.080; 0.080; 0.0080]});

        case "combined_sine"
            d.type = "sin";
            d.startTime = 0.0;
            d.endTime = inf;
            d.forceFreq = [0.17; 0.23; 0.31];
            d.momentFreq = [0.19; 0.29; 0.37];
            d.forcePhase = [0; 1*pi/3; 2*pi/3];
            d.momentPhase = [pi/4; 3*pi/4; 5*pi/4];
            d.forceTau = [1.5; 1.5; 1.0];
            d.momentTau = [0.35; 0.35; 0.25];
            d.disturbanceSeedBase = 24001;
            d.levels = struct( ...
                'name',     {'low', 'medium', 'high'}, ...
                'forceAmp', {[0.15; 0.15; 0.08], ...
                             [0.30; 0.30; 0.15], ...
                             [0.45; 0.45; 0.22]}, ...
                'momentAmp',{[0.025; 0.025; 0.0025], ...
                             [0.050; 0.050; 0.0050], ...
                             [0.080; 0.080; 0.0080]});

        case "paper_force"
            d.type = "constant";
            d.startTime = 5.0;
            d.endTime = 10.0;
            d.forceFreq = [0; 0; 0];
            d.momentFreq = [0; 0; 0];
            d.forcePhase = [0; 0; 0];
            d.momentPhase = [0; 0; 0];
            d.forceTau = [1.5; 1.5; 1.0];
            d.momentTau = [0.35; 0.35; 0.25];
            d.disturbanceSeedBase = 24001;
            d.levels = struct( ...
                'name',     {'low', 'medium', 'high'}, ...
                'forceAmp', {[5; 0; 0], ...
                             [10; 0; 0], ...
                             [15; 0; 0]}, ...
                'momentAmp',{[0; 0; 0], ...
                             [0; 0; 0], ...
                             [0; 0; 0]});

        case "paper_moment"
            d.type = "constant";
            d.startTime = 5.0;
            d.endTime = 10.0;
            d.forceFreq = [0; 0; 0];
            d.momentFreq = [0; 0; 0];
            d.forcePhase = [0; 0; 0];
            d.momentPhase = [0; 0; 0];
            d.forceTau = [1.5; 1.5; 1.0];
            d.momentTau = [0.35; 0.35; 0.25];
            d.disturbanceSeedBase = 24001;
            d.levels = struct( ...
                'name',     {'low', 'medium', 'high'}, ...
                'forceAmp', {[0; 0; 0], ...
                             [0; 0; 0], ...
                             [0; 0; 0]}, ...
                'momentAmp',{[0.0707; 0.0707; 0], ...
                             [0.1414; 0.1414; 0], ...
                             [0.2121; 0.2121; 0]});

        otherwise
            error("Unknown disturbanceCase.");
    end
end
