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
% trajNames = ["fast_circle"];

% Available controllers for this comparison:
% "geometric", "faessler", "lee", "johnson_beard"
% "sun_dfbc", "sun_dfbc_indi"
% "lu_on_manifold_mpc", "sun_nmpc", "sun_nmpc_indi"
% "geometric_indi", "tal_karaman"
% controllerNames = ["geometric", "faessler", "lee", "johnson_beard", "sun_dfbc_indi","geometric_indi", "tal_karaman"];
% controllerNames = ["sun_dfbc", "sun_nmpc", "sun_dfbc_indi", "sun_nmpc_indi"];
% controllerNames = ["sun_dfbc_indi", "sun_nmpc_indi", "geometric_indi", "tal_karaman"];
% controllerNames = ["sun_dfbc_indi", "sun_nmpc", "sun_nmpc_indi", "geometric_indi"];
controllerNames = ["lu_on_manifold_mpc", "sun_nmpc", "sun_nmpc_indi", "geometric_indi"];
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
        % Zero-mean force/moment disturbances for general robustness sweeps.
        % Keep moment on body x/y rather than yaw; yaw authority is much
        % smaller and should be tested separately if desired.
        disturbanceType = "sin";
        disturbanceStartTime = 0.0;
        disturbanceEndTime = inf;
        forceDirection = [1; 0; 0];
        momentDirection = [1; 1; 0];
        forceFreq = [0.17; 0.23; 0.31];
        momentFreq = [0.19; 0.29; 0.37];
        forcePhase = [0; 2*pi/3; 4*pi/3];
        momentPhase = [pi/4; 3*pi/4; 5*pi/4];
        disturbanceLevels = struct( ...
            'name',     {'low', 'medium', 'hig7h'}, ...
            'forceAmp', {0.11,   0.32,      0.50}, ...
            'momentAmp',{0.05,   0.1,      0.2});

    case "legacy_bias"
        % The older benchmark style: zero frequency and pi/2 phase make the
        % "sinusoid" a deterministic constant bias for the whole run.
        disturbanceType = "sin";
        disturbanceStartTime = 0.0;
        disturbanceEndTime = inf;
        forceDirection = [0; 1; 0];
        momentDirection = [0; 0; 1];
        forceFreq = [0; 0; 0];
        momentFreq = [0; 0; 0];
        forcePhase = [pi/2; pi/2; pi/2];
        momentPhase = [pi/2; pi/2; pi/2];
        disturbanceLevels = struct( ...
            'name',     {'low', 'medium', 'high'}, ...
            'forceAmp', {0.0,   0.031,    0.061}, ...
            'momentAmp',{0.0,   0.031,    0.061});

    case "paper_force"
        % Sun et al.: constant external forces along inertial x_I.
        disturbanceType = "constant";
        disturbanceStartTime = 5.0;
        disturbanceEndTime = 10.0;
        forceDirection = [1; 0; 0];
        momentDirection = [0; 0; 0];
        forceFreq = [0; 0; 0];
        momentFreq = [0; 0; 0];
        forcePhase = [0; 0; 0];
        momentPhase = [0; 0; 0];
        disturbanceLevels = struct( ...
            'name',     {'low', 'medium', 'high'}, ...
            'forceAmp', {5,     10,       15}, ...
            'momentAmp',{0,     0,        0});

    case "paper_moment"
        % Sun et al.: external torques along body x_B and y_B, not yaw.
        disturbanceType = "constant";
        disturbanceStartTime = 5.0;
        disturbanceEndTime = 10.0;
        forceDirection = [0; 0; 0];
        momentDirection = [1; 1; 0];
        forceFreq = [0; 0; 0];
        momentFreq = [0; 0; 0];
        forcePhase = [0; 0; 0];
        momentPhase = [0; 0; 0];
        disturbanceLevels = struct( ...
            'name',     {'low', 'medium', 'high'}, ...
            'forceAmp', {0,     0,        0}, ...
            'momentAmp',{0.1,   0.2,      0.3});

    otherwise
        error("Unknown disturbanceCase.");
end

%% ========================================================================
%% 2. Plot Setting

% Figures are always shown after the benchmark finishes.
% This single switch only controls whether the figures are also saved as PNG.
savePlots = true;

%% ========================================================================
%% 3. Simulation Settings

% "lie_rk4" is faster for sweeps. Use "ode45" to match main.m's default.
integratorName = "lie_rk4";

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
cfg.forceDirection = forceDirection;
cfg.momentDirection = momentDirection;
cfg.forceFreq = forceFreq;
cfg.momentFreq = momentFreq;
cfg.forcePhase = forcePhase;
cfg.momentPhase = momentPhase;

cfg.savePlots = savePlots;

cfg.integratorName = integratorName;
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
