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
% trajNames = ["fast_circle", "figure8_horizontal", "helix_flip"];
trajNames = ["figure8_horizontal", "figure8_vertical", "helix_flip", "flip_loop_sine",  "fast_circle"];


% Available controllers for this comparison:
%   "geometric"
%   "lu_on_manifold_lqr"
%   "geometric_indi"
controllerNames = ["geometric", "faessler", "lee", "johnson_beard", "sun_dfbc", "sun_dfbc_indi","geometric_indi", "tal_karaman"];
% controllerNames = ["lu_on_manifold_lqr", "sun_linear_mpc", "sun_nmpc_full", "sun_linear_mpc_indi"];

%% ========================================================================
%% 1. Disturbance Settings

% Use a fixed-direction disturbance instead of a sign-changing multi-axis
% sinusoid. With zero frequency and pi/2 phase, sin(...) = 1, so the
% disturbance is a deterministic bias along these directions. For helix_flip,
% the lateral loop axis avoids the disturbance periodically helping the flip.
forceDirection = [0; 1; 0];
momentDirection = [0; 0; 1];
forceFreq = [0; 0; 0];
momentFreq = [0; 0; 0];
forcePhase = [pi/2; pi/2; pi/2];
momentPhase = [pi/2; pi/2; pi/2];

% Three disturbance levels: force is NED-frame [N], moment is body-frame [N*m].
disturbanceLevels = struct( ...
    'name',     {'low', 'medium', 'high'}, ...
    'forceAmp', {0.1,  0.2,     0.3}, ...
    'momentAmp',{0.1, 0.2,    0.3});

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

cfg.levels = disturbanceLevels;
cfg.forceDirection = forceDirection;
cfg.momentDirection = momentDirection;
cfg.forceFreq = forceFreq;
cfg.momentFreq = momentFreq;
cfg.forcePhase = forcePhase;
cfg.momentPhase = momentPhase;

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
