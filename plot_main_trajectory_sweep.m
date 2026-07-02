%% plot_main_trajectory_sweep.m
% Replot saved main_trajectory_sweep.m results without rerunning simulations.
%
% Sweep results are saved under:
%   results/main_trajectory_sweep/<controller>
%
% Individual trajectory runs are saved under:
%   results/main_trajectory_sweep/<controller>/<trajectory>/main_run.mat

clear; clc;

%% ========================================================================
%% Plot Settings

% Use "latest" to replot the most recently modified controller sweep result.
% Or set a controller name such as "geometric_indi", "tal", "sun_nmpc_indi".
controllerName = "latest";

% Leave empty to replot the combined multi-trajectory sweep figures. Set a
% trajectory name such as "helix_flip" to replot one saved main_run.mat.
trajectoryName = "";

% Leave empty to include all saved trajectories in the combined sweep plot.
trajNames = ["figure8_horizontal", "helix_flip"];

savePlots = true;
resolution = 200;
keepFigureWindows = true;
figureSize = []; % [width height] pixels. Empty uses an automatic sweep size.

%% ========================================================================
%% Replot

if strlength(string(trajectoryName)) > 0
    if string(controllerName) == "latest"
        error("Set controllerName when plotting one trajectory.");
    end

    resultFile = fullfile(pwd, "results", "main_trajectory_sweep", ...
        matlab.lang.makeValidName(char(controllerName)), ...
        char(trajectoryName), "main_run.mat");

    if ~isfile(resultFile)
        error("Saved trajectory result not found: %s.", resultFile);
    end

    figureFiles = plot_main(resultFile, ...
        "SavePlots", savePlots, ...
        "Resolution", resolution);

    disp("Replotted saved trajectory result:");
    disp(resultFile);
else
    if string(controllerName) == "latest"
        resultFile = latestSweepResultsFile();
    else
        resultFile = fullfile(pwd, "results", "main_trajectory_sweep", ...
            matlab.lang.makeValidName(char(controllerName)), ...
            "main_trajectory_sweep_results.mat");
    end

    if ~isfile(resultFile)
        error("Saved trajectory sweep result not found: %s.", resultFile);
    end

    figureFiles = render_main_trajectory_sweep(resultFile, ...
        "SavePlots", savePlots, ...
        "Resolution", resolution, ...
        "TrajNames", trajNames, ...
        "FigureSize", figureSize, ...
        "KeepFigureWindows", keepFigureWindows);

    disp("Replotted trajectory sweep result:");
    disp(resultFile);
end

if ~isempty(figureFiles)
    disp("Saved figure files:");
    disp(figureFiles);
end

%% ========================================================================
%% Local functions

function resultFile = latestSweepResultsFile()

    rootDir = fullfile(pwd, "results", "main_trajectory_sweep");
    files = dir(fullfile(rootDir, "*", "main_trajectory_sweep_results.mat"));
    if isempty(files)
        error("No trajectory sweep result found under %s.", rootDir);
    end

    [~, idx] = max([files.datenum]);
    resultFile = fullfile(files(idx).folder, files(idx).name);
end
