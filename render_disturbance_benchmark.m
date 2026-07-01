function figureFiles = render_disturbance_benchmark( ...
        resultsInput, cfg, varargin)
%RENDER_DISTURBANCE_BENCHMARK Render disturbance benchmark figures.
%
% Usage:
%   render_disturbance_benchmark
%   render_disturbance_benchmark("results/disturbance_benchmark")
%   render_disturbance_benchmark(results, cfg)

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

    figureFiles = strings(numel(opts.trajectoryOrder),1);
    nFigureFiles = 0;
    for iTraj = 1:numel(opts.trajectoryOrder)
        trajName = string(opts.trajectoryOrder(iTraj));
        mask = results.Trajectory == trajName ...
            & ismember(results.Controller, opts.controllerNames) ...
            & ismember(results.DisturbanceLevel, opts.levelNames) ...
            & results.IsFinite;
        if ~any(mask)
            continue;
        end

        fig = figure('Color', 'w', 'Name', char(trajName + ""));
        [xLabel, yData, groupLabel, yLabelText] = boxchartData( ...
            results, mask, opts.boxDataSource);

        x = categorical(xLabel, opts.levelOrder, opts.levelOrder);
        group = categorical(groupLabel, opts.controllerOrder, ...
            opts.controllerOrder);

        boxchart(x, yData, 'GroupByColor', group);
        grid on;
        xlabel('disturbance amplitude');
        ylabel(yLabelText);
        formatErrorAxis(gca);
        title("Trajectory: " + trajName, 'Interpreter', 'none');
        legend('Location', 'northwest', 'Interpreter', 'none');
        subtitle(amplitudeSummaryText(selectedLevels(cfg.levels, ...
            opts.levelNames)), 'Interpreter', 'none');

        if opts.savePlots
            pngPath = fullfile(opts.outputDir, ...
                char(trajName + ".png"));
            figPath = fullfile(opts.outputDir, ...
                char(trajName + ".fig"));
            exportgraphics(fig, pngPath, 'Resolution', opts.resolution);
            savefig(fig, figPath);
            nFigureFiles = nFigureFiles + 1;
            figureFiles(nFigureFiles,1) = string(pngPath);
        end
    end
    figureFiles = figureFiles(1:nFigureFiles);
end

function formatErrorAxis(ax)

    ax.YAxis.Exponent = 0;
    ytickformat(ax, '%.4g');
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
    opts.trajectoryNames = opts.trajectoryOrder;
    opts.controllerNames = opts.controllerOrder;
    opts.levelNames = opts.levelOrder;

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
            case {"trajnames", "trajectorynames"}
                opts.trajectoryNames = selectedNames(value, ...
                    unique(results.Trajectory, 'stable'));
                opts.trajectoryOrder = opts.trajectoryNames;
            case "controllernames"
                opts.controllerNames = selectedNames(value, ...
                    unique(results.Controller, 'stable'));
                opts.controllerOrder = opts.controllerNames;
            case {"levelnames", "disturbancelevels"}
                opts.levelNames = selectedNames(value, ...
                    unique(results.DisturbanceLevel, 'stable'));
                opts.levelOrder = opts.levelNames;
            case "trajectoryorder"
                opts.trajectoryOrder = string(value);
                opts.trajectoryNames = opts.trajectoryOrder;
            case "controllerorder"
                opts.controllerOrder = string(value);
            case "levelorder"
                opts.levelOrder = string(value);
            otherwise
                error("Unknown render_disturbance_benchmark option: %s.", ...
                    name);
        end

        i = i + 2;
    end

    opts.controllerOrder = appendMissing(opts.controllerOrder, ...
        unique(results.Controller(ismember(results.Controller, ...
        opts.controllerNames)), 'stable'));
    opts.levelOrder = appendMissing(opts.levelOrder, ...
        unique(results.DisturbanceLevel(ismember(results.DisturbanceLevel, ...
        opts.levelNames)), 'stable'));
end

function names = selectedNames(value, defaultNames)

    names = string(value);
    names = names(strlength(names) > 0);
    if isempty(names)
        names = string(defaultNames);
    end
    names = names(:).';
end

function levels = selectedLevels(levels, levelNames)

    mask = ismember(string({levels.name}), string(levelNames));
    levels = levels(mask);
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
            nRows = numel(rowIdx);
            nSamples = zeros(nRows,1);
            cleanErr = cell(nRows,1);

            for i = 1:nRows
                err = results.ErrorTrace{rowIdx(i)};
                err = err(isfinite(err));
                cleanErr{i} = err(:);
                nSamples(i) = numel(err);
            end

            nTotal = sum(nSamples);
            xLabel = strings(nTotal,1);
            groupLabel = strings(nTotal,1);
            yData = zeros(nTotal,1);

            idx = 0;
            for i = 1:nRows
                r = rowIdx(i);
                n = nSamples(i);
                if n == 0
                    continue;
                end

                rows = idx + (1:n);
                xLabel(rows) = repmat(results.DisturbanceLevel(r), n, 1);
                groupLabel(rows) = repmat(results.Controller(r), n, 1);
                yData(rows) = cleanErr{i};
                idx = idx + n;
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
