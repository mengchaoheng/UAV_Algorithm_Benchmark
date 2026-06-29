%% main_disturbance_benchmark.m
% Run this file directly, just like main.m.
%
% This benchmark adds force/moment disturbances to the plant and compares
% absolute position tracking error, ||p - pd||.

clear; clc; close all;

%% ========================================================================
%% 0. What to Run

% Available:
%   "figure8_horizontal"
%   "figure8_vertical"
%   "helix_flip"
%   "flip_loop_sine"
%   "fast_circle"
trajNames = ["figure8_horizontal", "figure8_vertical", ...
    "helix_flip", "flip_loop_sine", "fast_circle"];
% trajNames = ["helix_flip"];

% Available controllers for this comparison:
% "geometric", "lee", "johnson"
% "sun_dfbc", "sun_dfbc_indi" 
% "sun_nmpc", "sun_nmpc_indi"
% "lu", "tal", "geometric_indi"
controllerNames = ["geometric", "lee", "johnson", ...
    "sun_dfbc", "sun_dfbc_indi", ...
    "sun_nmpc", "sun_nmpc_indi", ...
    "lu", "tal", "geometric_indi"];

% Optional focused subsets:
% controllerNames = ["tal", "geometric_indi"];
% controllerNames = ["sun_dfbc", "sun_dfbc_indi", "sun_nmpc", "sun_nmpc_indi"];
% controllerNames = ["sun_dfbc_indi", "sun_nmpc_indi", "tal", "geometric_indi"];
% controllerNames = ["sun_nmpc", "sun_nmpc_indi", "lu", "tal", "geometric_indi"];



%% ========================================================================
%% 1. Disturbance Settings

% Available presets:
%   "combined_sine" : force and moment together, zero-mean engineering test.
%   "legacy_bias"  : old zero-frequency sine bias, force and moment together.
%   "paper_force"  : Sun et al. force-only robustness case, 5 seconds.
%   "paper_moment" : Sun et al. moment-only robustness case, 5 seconds.
disturbanceCase = "combined_sine";

switch disturbanceCase
    case "combined_sine"
        % Zero-mean per-axis force/moment disturbances for general robustness
        % sweeps. Yaw moment is intentionally much smaller: Iris yaw authority
        % is far lower than roll/pitch authority.
        disturbanceType = "sin";
        disturbanceStartTime = 0.0;
        disturbanceEndTime = inf;
        forceFreq = [0.17; 0.23; 0.31];
        momentFreq = [0.19; 0.29; 0.37];
        forcePhase = [0; 1*pi/3; 2*pi/3];
        momentPhase = [pi/4; 3*pi/4; 5*pi/4];
        disturbanceLevels = struct( ...
            'name',     {'low', 'medium', 'high'}, ...
            'forceAmp', {[0.05; 0.05; 0.03], ...
                         [0.10; 0.10; 0.06], ...
                         [0.20; 0.20; 0.10]}, ...
            'momentAmp',{[0.005; 0.005; 0.0005], ...
                         [0.010; 0.010; 0.0010], ...
                         [0.020; 0.020; 0.0020]});

    case "legacy_bias"
        % The older benchmark style: zero frequency and pi/2 phase make the
        % "sinusoid" a deterministic constant bias for the whole run.
        disturbanceType = "sin";
        disturbanceStartTime = 0.0;
        disturbanceEndTime = inf;
        forceFreq = [0; 0; 0];
        momentFreq = [0; 0; 0];
        forcePhase = [pi/2; pi/2; pi/2];
        momentPhase = [pi/2; pi/2; pi/2];
        disturbanceLevels = struct( ...
            'name',     {'low', 'medium', 'high'}, ...
            'forceAmp', {[0; 0; 0], ...
                         [0; 0.031; 0], ...
                         [0; 0.061; 0]}, ...
            'momentAmp',{[0; 0; 0], ...
                         [0; 0; 0.031], ...
                         [0; 0; 0.061]});

    case "paper_force"
        % Sun et al.: constant external forces along inertial x_I.
        disturbanceType = "constant";
        disturbanceStartTime = 5.0;
        disturbanceEndTime = 10.0;
        forceFreq = [0; 0; 0];
        momentFreq = [0; 0; 0];
        forcePhase = [0; 0; 0];
        momentPhase = [0; 0; 0];
        disturbanceLevels = struct( ...
            'name',     {'low', 'medium', 'high'}, ...
            'forceAmp', {[5; 0; 0], ...
                         [10; 0; 0], ...
                         [15; 0; 0]}, ...
            'momentAmp',{[0; 0; 0], ...
                         [0; 0; 0], ...
                         [0; 0; 0]});

    case "paper_moment"
        % Sun et al.: external torques along body x_B and y_B, not yaw.
        disturbanceType = "constant";
        disturbanceStartTime = 5.0;
        disturbanceEndTime = 10.0;
        forceFreq = [0; 0; 0];
        momentFreq = [0; 0; 0];
        forcePhase = [0; 0; 0];
        momentPhase = [0; 0; 0];
        disturbanceLevels = struct( ...
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

%% ========================================================================
%% 2. Plot Setting

% Figures are always shown after the benchmark finishes.
% This single switch only controls whether the figures are also saved as PNG.
makePlots = true;
savePlots = true;

%% ========================================================================
%% 3. Simulation Settings

% "lie_rk4" is faster for sweeps. Use "ode45" to match main.m's default.
integratorName = "lie_rk4";

% Parallelizes independent trajectory/controller/disturbance simulations.
% Leave numWorkers empty to let MATLAB choose the pool size.
useParallel = true;
numWorkers = [];

% Use every time sample in the boxplot. Set to 2, 5, ... to thin samples.
errorSampleStride = 1;

% This benchmark measures continuous tracking, not terminal stopping.
% Exclude the final Sun NMPC prediction horizon from all controllers'
% statistics so the last finite-horizon boundary condition does not dominate
% the comparison.
errorEvalMode = "sun_prediction_horizon";

%% ========================================================================
%% 4. Build Config and Run

cfg.trajNames = trajNames;
cfg.controllerNames = controllerNames;

cfg.levels = disturbanceLevels;
cfg.disturbanceType = disturbanceType;
cfg.disturbanceStartTime = disturbanceStartTime;
cfg.disturbanceEndTime = disturbanceEndTime;
cfg.forceFreq = forceFreq;
cfg.momentFreq = momentFreq;
cfg.forcePhase = forcePhase;
cfg.momentPhase = momentPhase;

cfg.makePlots = makePlots;
cfg.savePlots = savePlots;

cfg.integratorName = integratorName;
cfg.useParallel = useParallel;
cfg.numWorkers = numWorkers;
cfg.errorSampleStride = errorSampleStride;
cfg.errorEvalMode = errorEvalMode;

results = run_disturbance_benchmark(cfg);

%% ========================================================================
%% 5. Output

disp("Numeric results directory:");
disp(results.Properties.UserData.outputDir);

if ~isempty(results.Properties.UserData.figureFiles)
    disp("Saved figure files:");
    disp(results.Properties.UserData.figureFiles);
end

disp("Results table:");
disp(results);
