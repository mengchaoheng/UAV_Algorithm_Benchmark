%% plot_main_disturbance_benchmark.m
% Replot saved main_disturbance_benchmark.m results without rerunning.
%
% Results are saved under:
%   results/disturbance_benchmark

clear; clc;

%% ========================================================================
%% Plot Settings

% Use "time_error" for the one-shot benchmark default. Use "rmse" when you
% want one scalar per saved simulation row.
boxDataSource = "time_error";

% A simulation row is treated as failed, and excluded from the plotted
% boxchart, if its 3-D position RMSE exceeds this threshold. Sun et al. use
% 5 m for failure counting. Set to inf to keep every finite run.
failureRmseThreshold = 5; % [m]

% Leave empty to plot all saved trajectories/controllers/disturbance levels.
trajNames = strings(0,1);
controllerNames = strings(0,1);
controllerOrder = ["geometric", "lee", "johnson", "lu", ...
    "sun_dfbc", "sun_dfbc_indi", "sun_nmpc", "sun_nmpc_indi", ...
    "tal", "geometric_indi"];
levelNames = strings(0,1);

savePlots = true;
resolution = 200;

%% ========================================================================
%% Replot

resultDir = fullfile(pwd, "results", "disturbance_benchmark");
resultFile = fullfile(resultDir, "disturbance_benchmark_results.mat");

if ~isfile(resultFile)
    error("Saved disturbance benchmark result not found: %s.", resultFile);
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

disp("Replotted disturbance benchmark results from:");
disp(resultDir);

if ~isempty(figureFiles)
    disp("Saved figure files:");
    disp(figureFiles);
end
