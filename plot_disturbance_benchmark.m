function figureFiles = plot_disturbance_benchmark(resultsInput, cfg, varargin)
%PLOT_DISTURBANCE_BENCHMARK Plot or replot disturbance benchmark results.
%
% Usage:
%   plot_disturbance_benchmark
%   plot_disturbance_benchmark("results/disturbance_benchmark")
%   plot_disturbance_benchmark(results, cfg)

    if nargin < 1 || isempty(resultsInput)
        resultsInput = fullfile(pwd, "results", "disturbance_benchmark", ...
            "disturbance_benchmark_results.mat");
    end
    if nargin < 2
        cfg = struct();
    end

    matPath = "";
    if istable(resultsInput)
        results = resultsInput;
        if nargin < 2 || ~isstruct(cfg)
            error("cfg is required when plotting from a results table.");
        end
        args = varargin;
    else
        matPath = string(resultsInput);
        if isfolder(matPath)
            matPath = fullfile(matPath, "disturbance_benchmark_results.mat");
        end

        data = load(matPath, 'results', 'cfg');
        results = data.results;

        if nargin >= 2
            secondArg = cfg;
        end

        if nargin >= 2 && isstruct(secondArg)
            cfg = secondArg;
            args = varargin;
        else
            cfg = data.cfg;
            if nargin >= 2
                args = [{secondArg}, varargin];
            else
                args = {};
            end
        end
    end

    opts = plotDisturbanceOptions(results, cfg, matPath, args);
    if opts.savePlots
        prepareFigureDir(opts.outputDir);
    end

    figureFiles = strings(0,1);
    for iTraj = 1:numel(opts.trajectoryOrder)
        trajName = string(opts.trajectoryOrder(iTraj));
        mask = results.Trajectory == trajName & results.IsFinite;
        if ~any(mask)
            continue;
        end

        fig = figure('Color', 'w', 'Name', char(trajName + "_disturbance_boxplot"));
        [xLabel, yData, groupLabel, yLabelText] = boxchartData( ...
            results, mask, opts.boxDataSource);

        x = categorical(xLabel, opts.levelOrder, opts.levelOrder);
        group = categorical(groupLabel, opts.controllerOrder, ...
            opts.controllerOrder);

        boxchart(x, yData, 'GroupByColor', group);
        grid on;
        xlabel('disturbance amplitude');
        ylabel(yLabelText);
        title("Trajectory: " + trajName, 'Interpreter', 'none');
        legend('Location', 'northwest', 'Interpreter', 'none');
        subtitle(amplitudeSummaryText(cfg.levels), 'Interpreter', 'none');

        if opts.savePlots
            pngPath = fullfile(opts.outputDir, ...
                char(trajName + "_disturbance_boxplot.png"));
            figPath = fullfile(opts.outputDir, ...
                char(trajName + "_disturbance_boxplot.fig"));
            exportgraphics(fig, pngPath, 'Resolution', opts.resolution);
            savefig(fig, figPath);
            figureFiles(end+1,1) = string(pngPath); %#ok<AGROW>
        end
    end
end

function opts = plotDisturbanceOptions(results, cfg, matPath, args)

    opts.savePlots = logical(getCfgField(cfg, 'savePlots', false));
    opts.outputDir = defaultOutputDir(results, matPath);
    opts.resolution = 200;
    opts.boxDataSource = string(getCfgField(cfg, 'boxDataSource', "time_error"));
    opts.trajectoryOrder = string(getCfgField(cfg, 'trajNames', ...
        unique(results.Trajectory, 'stable')));
    opts.controllerOrder = string(getCfgField(cfg, 'controllerNames', ...
        unique(results.Controller, 'stable')));
    opts.levelOrder = string({cfg.levels.name});

    i = 1;
    while i <= numel(args)
        name = string(args{i});
        value = args{i+1};

        switch lower(name)
            case "saveplots"
                opts.savePlots = logical(value);
            case "outputdir"
                opts.outputDir = char(value);
            case "resolution"
                opts.resolution = double(value);
            case "boxdatasource"
                opts.boxDataSource = string(value);
            case "trajectoryorder"
                opts.trajectoryOrder = string(value);
            case "controllerorder"
                opts.controllerOrder = string(value);
            case "levelorder"
                opts.levelOrder = string(value);
            otherwise
                error("Unknown plot_disturbance_benchmark option: %s.", name);
        end

        i = i + 2;
    end

    opts.controllerOrder = appendMissing(opts.controllerOrder, ...
        unique(results.Controller, 'stable'));
    opts.levelOrder = appendMissing(opts.levelOrder, ...
        unique(results.DisturbanceLevel, 'stable'));
end

function value = getCfgField(cfg, name, defaultValue)

    if isfield(cfg, name)
        value = cfg.(name);
    else
        value = defaultValue;
    end
end

function outputDir = defaultOutputDir(results, matPath)

    outputDir = "";
    if strlength(matPath) > 0
        outputDir = fullfile(fileparts(matPath), "figures");
    elseif isfield(results.Properties.UserData, 'outputDir')
        outputDir = fullfile(results.Properties.UserData.outputDir, "figures");
    end

    if strlength(outputDir) == 0
        outputDir = fullfile(pwd, "results", "disturbance_benchmark", "figures");
    end

    outputDir = char(outputDir);
end

function prepareFigureDir(outputDir)

    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    deleteMatching(outputDir, '*.png');
    deleteMatching(outputDir, '*.fig');
end

function deleteMatching(folder, pattern)

    files = dir(fullfile(folder, pattern));
    for i = 1:numel(files)
        delete(fullfile(files(i).folder, files(i).name));
    end
end

function values = appendMissing(values, observed)

    values = string(values(:)).';
    observed = string(observed(:)).';
    missing = observed(~ismember(observed, values));
    values = [values, missing];
end

function [xLabel, yData, groupLabel, yLabelText] = boxchartData( ...
        results, mask, source)

    switch string(source)
        case "time_error"
            rowIdx = find(mask).';
            xLabel = strings(0,1);
            groupLabel = strings(0,1);
            yData = zeros(0,1);

            for r = rowIdx
                err = results.ErrorTrace{r};
                err = err(isfinite(err));
                n = numel(err);

                xLabel = [xLabel; repmat(results.DisturbanceLevel(r), n, 1)]; %#ok<AGROW>
                groupLabel = [groupLabel; repmat(results.Controller(r), n, 1)]; %#ok<AGROW>
                yData = [yData; err(:)]; %#ok<AGROW>
            end

            yLabelText = 'position tracking error samples (m)';

        case "rmse"
            xLabel = results.DisturbanceLevel(mask);
            groupLabel = results.Controller(mask);
            yData = results.RMSE(mask);
            yLabelText = 'RMS position tracking error (m)';

        otherwise
            error("Unknown box data source.");
    end
end

function txt = amplitudeSummaryText(levels)

    parts = strings(1, numel(levels));
    for i = 1:numel(levels)
        forceAmp = vector3Local(levels(i).forceAmp);
        momentAmp = vector3Local(levels(i).momentAmp);
        parts(i) = sprintf('%s: F=%s N, M=%s N*m', ...
            string(levels(i).name), amplitudeVectorText(forceAmp), ...
            amplitudeVectorText(momentAmp));
    end
    txt = strjoin(parts, ', ');
end

function txt = amplitudeVectorText(amp)

    amp = amp(:);
    txt = sprintf('[%.3g %.3g %.3g]', amp(1), amp(2), amp(3));
end

function v = vector3Local(x)

    v = x(:);

    if numel(v) ~= 3
        error("Disturbance amplitude must be a 3x1 vector.");
    end
end
