%% plot_main_disturbance_monte_carlo.m
% Replot saved Monte Carlo disturbance results without rerunning simulations.
%
% Results are produced by main_disturbance_monte_carlo.m and saved under:
%   results/disturbance_monte_carlo/<disturbanceCase>

clear; clc;

%% ========================================================================
%% Plot Settings

disturbanceCase = "random_gust";

% Use "rmse" for one sample per repeated simulation. Use "time_error" to
% plot all saved time-sample errors.
boxDataSource = "rmse";

% A repeated trial is treated as failed, and excluded from the plotted
% boxchart, if its 3-D position RMSE exceeds this threshold. Sun et al. use
% 5 m for failure counting. Set to inf to keep every finite trial.
failureRmseThreshold = 5; % [m]

% Leave empty to plot all saved trajectories/controllers/disturbance levels.
trajNames = strings(0,1);
controllerNames = strings(0,1);
controllerOrder = ["lee", "johnson", "lu", ...
    "sun_dfbc", "sun_dfbc_indi", "sun_nmpc", "sun_nmpc_indi", ...
    "tal", "geometric_indi"];
levelNames = strings(0,1);

savePlots = true;
resolution = 200;

%% ========================================================================
%% Replot

resultDir = fullfile(pwd, "results", "disturbance_monte_carlo", ...
    char(disturbanceCase));
resultFile = fullfile(resultDir, "disturbance_benchmark_results.mat");

if ~isfile(resultFile)
    error("Saved Monte Carlo result not found: %s.", resultFile);
end

figureFiles = render_disturbance_benchmark(resultDir, ...
    "SavePlots", savePlots, ...
    "Resolution", resolution, ...
    "BoxDataSource", boxDataSource, ...
    "FailureRmseThreshold", failureRmseThreshold, ...
    "TrajNames", trajNames, ...
    "ControllerNames", controllerNames, ...
    "ControllerOrder", controllerOrder, ...
    "LevelNames", levelNames);

disp("Replotted Monte Carlo disturbance results from:");
disp(resultDir);

if ~isempty(figureFiles)
    disp("Saved figure files:");
    disp(figureFiles);
end
