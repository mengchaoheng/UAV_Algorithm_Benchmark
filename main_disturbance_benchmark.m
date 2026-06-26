%% main_disturbance_benchmark.m
% Run this file directly, just like main.m.
%
% This benchmark adds force/moment disturbances to the plant and compares
% controller tracking error. The boxplots use all discrete simulation-time
% tracking-error samples from each run, so the default only needs one run per
% trajectory/controller/disturbance level.

clear; clc; close all;

%% ========================================================================
%% 0. What to Run

% Available:
%   "fast_circle"
%   "figure8_horizontal"
%   "helix_flip"
trajNames = ["fast_circle", "figure8_horizontal", "helix_flip"];

% Available controllers for this comparison:
%   "geometric"
%   "on_manifold_mpc"
%   "geometric_indi"
controllerNames = ["geometric", "on_manifold_mpc", "geometric_indi"];

% Default is 1 because each boxplot is built from all time samples in a run.
% Increase this only if you want several random disturbance phases combined.
numTrials = 1;

%% ========================================================================
%% 1. Disturbance Settings

% "sin"    : sinusoidal disturbance; each trial uses a different phase.
% "random" : sample-held random disturbance.
disturbanceType = "sin";

% Three disturbance levels: force is NED-frame [N], moment is body-frame [N*m].
disturbanceLevels = struct( ...
    'name',     {'low', 'medium', 'high'}, ...
    'forceAmp', {0.05,  0.15,     0.30}, ...
    'momentAmp',{0.002, 0.006,    0.012});

%% ========================================================================
%% 2. Plot Setting

% Figures are always shown after the benchmark finishes.
% This single switch only controls whether the figures are also saved as PNG.
savePlots = false;

%% ========================================================================
%% 3. Simulation Settings

% "lie_rk4" is faster for sweeps. Use "ode45" to match main.m's default.
integratorName = "lie_rk4";

% Use every time sample in the boxplot. Set to 2, 5, ... to thin samples.
errorSampleStride = 1;

%% ========================================================================
%% 4. Build Config and Run

cfg.trajNames = trajNames;
cfg.controllerNames = controllerNames;
cfg.numTrials = numTrials;

cfg.disturbanceType = disturbanceType;
cfg.levels = disturbanceLevels;

cfg.savePlots = savePlots;

cfg.integratorName = integratorName;
cfg.errorSampleStride = errorSampleStride;

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
