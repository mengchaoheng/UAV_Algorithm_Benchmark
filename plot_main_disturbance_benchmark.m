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

% Leave empty to plot all saved trajectories/controllers/disturbance levels.
trajNames = strings(0,1);
controllerNames = strings(0,1);
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
    "TrajNames", trajNames, ...
    "ControllerNames", controllerNames, ...
    "LevelNames", levelNames);

disp("Replotted disturbance benchmark results from:");
disp(resultDir);

if ~isempty(figureFiles)
    disp("Saved figure files:");
    disp(figureFiles);
end
