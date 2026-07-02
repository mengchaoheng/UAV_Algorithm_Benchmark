%% plot_paper_figures.m
% Paper-style figure entry point. This file reads saved benchmark results and
% exports editable PDF plus high-resolution PNG without rerunning simulations.
%
% Edit the settings in Sections 1-5 for paper layout, colors, line widths,
% labels, legends, and output size.

clear; clc; close all;

%% ========================================================================
%% 1. What to export

figuresToMake = ["disturbance_monte_carlo", "trajectory_sweep_3d"];

outputDir = fullfile(pwd, "results", "paper_figures");
savePdf = false;
savePng = false;

%% ========================================================================
%% 2. Global paper style

P.fontName = "Times New Roman";
P.fontSize = 8;
P.axisLineWidth = 0.5;
P.gridAlpha = 0.18;
P.titleFontSize = 9;
P.labelFontSize = 8;
P.legendFontSize = 7;

P.export.pdfResolution = 1200;
P.export.pngResolution = 600;
P.export.pdfContentType = "vector";     % set "image" if vector PDF is slow
P.export.pngContentType = "image";
P.export.padding = "tight";

%% ========================================================================
%% 3. Controller display style

P.controllerOrder = ["lee", "johnson", "lu", ...
    "sun_dfbc", "sun_dfbc_indi", "sun_nmpc", "sun_nmpc_indi", ...
    "tal", "geometric_indi"];

P.controllerLabel.lee = "GC";
P.controllerLabel.johnson = "Log-GC";
P.controllerLabel.lu = "On-Manifold MPC";
P.controllerLabel.sun_dfbc = "DFBC";
P.controllerLabel.sun_dfbc_indi = "DFBC+INDI";
P.controllerLabel.sun_nmpc = "NMPC";
P.controllerLabel.sun_nmpc_indi = "NMPC+INDI";
P.controllerLabel.tal = "DF-based INDI";
P.controllerLabel.geometric_indi = "Geometric INDI";

P.controllerColor.lee = [0.000, 0.447, 0.741];
P.controllerColor.johnson = [0.850, 0.325, 0.098];
P.controllerColor.lu = [0.929, 0.694, 0.125];
P.controllerColor.sun_dfbc = [0.494, 0.184, 0.556];
P.controllerColor.sun_dfbc_indi = [0.494, 0.184, 0.556];
P.controllerColor.sun_nmpc = [0.466, 0.674, 0.188];
P.controllerColor.sun_nmpc_indi = [0.466, 0.674, 0.188];
P.controllerColor.tal = [0.301, 0.745, 0.933];
P.controllerColor.geometric_indi = [0.635, 0.078, 0.184];

P.controllerLineStyle.lee = "-";
P.controllerLineStyle.johnson = "-";
P.controllerLineStyle.lu = "-";
P.controllerLineStyle.sun_dfbc = "--";
P.controllerLineStyle.sun_dfbc_indi = "-";
P.controllerLineStyle.sun_nmpc = "--";
P.controllerLineStyle.sun_nmpc_indi = "-";
P.controllerLineStyle.tal = "-";
P.controllerLineStyle.geometric_indi = "-";

%% ========================================================================
%% 4. Disturbance Monte Carlo boxchart figure

P.mc.resultDir = fullfile(pwd, "results", "disturbance_monte_carlo", ...
    "random_gust");
P.mc.trajNames = ["figure8_horizontal", "helix_flip"];
P.mc.controllerNames = strings(0,1);     % empty means all saved controllers
P.mc.levelNames = ["low", "medium", "high"];
P.mc.levelLabel.low = "Low disturbance";
P.mc.levelLabel.medium = "Medium disturbance";
P.mc.levelLabel.high = "High disturbance";
P.mc.failureRmseThreshold = 5;           % [m], set inf to keep all finite runs

P.mc.figureWidthCm = 16;
P.mc.figureHeightCm = 9;
P.mc.boxWidth = 0.10;
P.mc.boxLineWidth = 0.55;
P.mc.boxFaceAlpha = 0.25;
P.mc.medianColor = [0.80, 0.10, 0.10];
P.mc.outlierMarker = "o";
P.mc.outlierMarkerSize = 3.5;
P.mc.legendLocation = "northwest";
P.mc.legendNumColumns = 1;
P.mc.showLegend = true;
P.mc.yLabel = "RMS position tracking error (m)";
P.mc.xLabel = "";
P.mc.title.figure8_horizontal = "Horizontal figure-eight";
P.mc.title.helix_flip = "Helix flip";
P.mc.yLimit.figure8_horizontal = [];
P.mc.yLimit.helix_flip = [];
P.mc.disturbanceLabelFontSize = 6.5;

%% ========================================================================
%% 5. Trajectory sweep 3-D figure

P.sweep.resultFile = "latest";           % "latest" or a .mat file path
P.sweep.trajNames = ["figure8_horizontal", "helix_flip"];
P.sweep.figureWidthCm = 16;
P.sweep.figureHeightCm = 7;
P.sweep.tileSpacing = "compact";
P.sweep.padding = "compact";

P.sweep.actualColor = [0.000, 0.447, 0.741];
P.sweep.refColor = [0.850, 0.325, 0.098];
P.sweep.actualLineStyle = "-";
P.sweep.refLineStyle = "--";
P.sweep.actualLineWidth = 0.9;
P.sweep.refLineWidth = 0.8;
P.sweep.viewAzEl = [35, 25];
P.sweep.axisEqual = true;
P.sweep.legendLocation = "northeast";
P.sweep.showLegend = true;
P.sweep.xLabel = "x (m)";
P.sweep.yLabel = "y (m)";
P.sweep.zLabel = "z (m)";
P.sweep.title.figure8_horizontal = "Horizontal figure-eight";
P.sweep.title.helix_flip = "Helix flip";

%% ========================================================================
%% 6. Run selected paper plots

P.outputDir = outputDir;
P.savePdf = savePdf;
P.savePng = savePng;
applyPaperDefaults(P);

if any(figuresToMake == "disturbance_monte_carlo")
    makePaperDisturbanceBoxFigures(P);
end

if any(figuresToMake == "trajectory_sweep_3d")
    makePaperTrajectorySweepFigure(P);
end

fprintf('Paper figures saved to:\n  %s\n', outputDir);

%% ========================================================================
%% Local plotting functions

function applyPaperDefaults(P)

    set(groot, ...
        'defaultAxesFontSize', P.fontSize, ...
        'defaultAxesFontName', char(P.fontName), ...
        'defaultAxesLineWidth', P.axisLineWidth, ...
        'defaultAxesLabelFontSizeMultiplier', 1, ...
        'defaultAxesTitleFontSizeMultiplier', 1, ...
        'defaultTextFontName', char(P.fontName), ...
        'defaultLegendFontName', char(P.fontName));
end

function makePaperDisturbanceBoxFigures(P)

    resultFile = fullfile(P.mc.resultDir, "disturbance_benchmark_results.mat");
    if ~isfile(resultFile)
        error("Monte Carlo result not found: %s.", resultFile);
    end

    S = load(resultFile, 'results');
    results = S.results;

    trajNames = selectedNames(P.mc.trajNames, unique(results.Trajectory, ...
        'stable'));
    controllerNames = selectedNames(P.mc.controllerNames, ...
        unique(results.Controller, 'stable'));
    controllerOrder = appendMissing(P.controllerOrder, controllerNames);
    levelNames = selectedNames(P.mc.levelNames, ...
        unique(results.DisturbanceLevel, 'stable'));
    levelLabels = disturbanceLevelLabels(levelNames, P.mc.levelLabel);

    outDir = fullfile(P.outputDir, "disturbance_monte_carlo");
    ensureDir(outDir);

    for iTraj = 1:numel(trajNames)
        trajName = trajNames(iTraj);
        mask = string(results.Trajectory) == trajName ...
            & ismember(string(results.Controller), controllerNames) ...
            & ismember(string(results.DisturbanceLevel), levelNames) ...
            & results.IsFinite;

        if isfinite(P.mc.failureRmseThreshold)
            mask = mask & isfinite(results.RMSE) ...
                & results.RMSE <= P.mc.failureRmseThreshold;
        end

        if ~any(mask)
            warning("No Monte Carlo rows selected for trajectory %s.", trajName);
            continue;
        end

        fig = paperFigure(P.mc.figureWidthCm, P.mc.figureHeightCm);
        ax = axes(fig);
        hold(ax, 'on');
        grid(ax, 'on');
        ax.GridAlpha = P.gridAlpha;

        legendHandles = gobjects(0);
        legendLabels = strings(0,1);
        nLevels = numel(levelNames);
        nCtrl = numel(controllerOrder);
        offsets = groupedOffsets(nCtrl, 0.70);

        for iLevel = 1:nLevels
            for iCtrl = 1:nCtrl
                ctrlName = controllerOrder(iCtrl);
                rowMask = mask ...
                    & string(results.DisturbanceLevel) == levelNames(iLevel) ...
                    & string(results.Controller) == ctrlName;
                y = results.RMSE(rowMask);
                y = y(isfinite(y));
                if isempty(y)
                    continue;
                end

                x = ones(numel(y),1)*(iLevel + offsets(iCtrl));
                color = controllerColor(P, ctrlName, iCtrl);
                h = boxchart(ax, x, y, ...
                    'BoxFaceColor', color, ...
                    'BoxFaceAlpha', P.mc.boxFaceAlpha, ...
                    'BoxEdgeColor', color, ...
                    'BoxMedianLineColor', P.mc.medianColor, ...
                    'WhiskerLineColor', [0.15 0.15 0.15], ...
                    'MarkerStyle', char(P.mc.outlierMarker), ...
                    'MarkerColor', color, ...
                    'MarkerSize', P.mc.outlierMarkerSize, ...
                    'BoxWidth', P.mc.boxWidth, ...
                    'LineWidth', P.mc.boxLineWidth);

                if iLevel == 1
                    legendHandles(end+1,1) = h; %#ok<AGROW>
                    legendLabels(end+1,1) = controllerLabel(P, ctrlName); %#ok<AGROW>
                end
            end
        end

        xlim(ax, [0.5, nLevels + 0.5]);
        xticks(ax, 1:nLevels);
        ylabel(ax, P.mc.yLabel, 'FontSize', P.labelFontSize);
        xlabel(ax, P.mc.xLabel, 'FontSize', P.labelFontSize);
        title(ax, titleFor(P.mc.title, trajName), ...
            'FontSize', P.titleFontSize, 'Interpreter', 'none');
        formatAxes(ax, P);
        applyOptionalYLimit(ax, P.mc.yLimit, trajName);
        addDisturbanceLevelLabels(ax, levelLabels, P.mc.disturbanceLabelFontSize);
        xticklabels(ax, repmat({''}, 1, nLevels));

        if P.mc.showLegend && ~isempty(legendHandles)
            legend(ax, legendHandles, cellstr(legendLabels), ...
                'Location', char(P.mc.legendLocation), ...
                'NumColumns', P.mc.legendNumColumns, ...
                'FontSize', P.legendFontSize, ...
                'Interpreter', 'none');
        end

        savePaperFigure(fig, fullfile(outDir, "mc_" + trajName), ...
            P.mc.figureWidthCm, P.mc.figureHeightCm, P);
    end
end

function makePaperTrajectorySweepFigure(P)

    resultFile = string(P.sweep.resultFile);
    if resultFile == "latest"
        resultFile = latestSweepResultsFile();
    end
    if ~isfile(resultFile)
        error("Trajectory sweep result not found: %s.", resultFile);
    end

    S = load(resultFile, 'results');
    results = S.results;
    trajNames = selectedNames(P.sweep.trajNames, unique(results.Trajectory, ...
        'stable'));
    data = loadSweepData(results, trajNames);
    if isempty(data)
        error("No saved trajectory run data found for the selected sweep.");
    end

    outDir = fullfile(P.outputDir, "trajectory_sweep");
    ensureDir(outDir);

    fig = paperFigure(P.sweep.figureWidthCm, P.sweep.figureHeightCm);
    tl = tiledlayout(fig, 1, numel(data), ...
        'TileSpacing', char(P.sweep.tileSpacing), ...
        'Padding', char(P.sweep.padding));

    for i = 1:numel(data)
        ax = nexttile(tl);
        p = data(i).log.p;
        pd = data(i).log.pd;

        plot3(ax, p(1,:), p(2,:), p(3,:), ...
            'LineStyle', char(P.sweep.actualLineStyle), ...
            'Color', P.sweep.actualColor, ...
            'LineWidth', P.sweep.actualLineWidth, ...
            'DisplayName', 'actual');
        hold(ax, 'on');
        plot3(ax, pd(1,:), pd(2,:), pd(3,:), ...
            'LineStyle', char(P.sweep.refLineStyle), ...
            'Color', P.sweep.refColor, ...
            'LineWidth', P.sweep.refLineWidth, ...
            'DisplayName', 'reference');

        grid(ax, 'on');
        ax.GridAlpha = P.gridAlpha;
        if P.sweep.axisEqual
            axis(ax, 'equal');
        end
        view(ax, P.sweep.viewAzEl(1), P.sweep.viewAzEl(2));
        set(ax, 'ZDir', 'reverse');
        xlabel(ax, P.sweep.xLabel, 'FontSize', P.labelFontSize);
        ylabel(ax, P.sweep.yLabel, 'FontSize', P.labelFontSize);
        zlabel(ax, P.sweep.zLabel, 'FontSize', P.labelFontSize);
        title(ax, titleFor(P.sweep.title, data(i).name), ...
            'FontSize', P.titleFontSize, 'Interpreter', 'none');
        setTrajectoryLimits(ax, [p, pd]);
        formatAxes(ax, P);

        if P.sweep.showLegend && i == numel(data)
            legend(ax, 'Location', char(P.sweep.legendLocation), ...
                'FontSize', P.legendFontSize, 'Interpreter', 'none');
        end
    end

    savePaperFigure(fig, fullfile(outDir, "trajectory_sweep_3d"), ...
        P.sweep.figureWidthCm, P.sweep.figureHeightCm, P);
end

function fig = paperFigure(widthCm, heightCm)

    fig = figure('Color', 'w', 'Units', 'centimeters', ...
        'Position', [0, 0, widthCm, heightCm], ...
        'PaperPositionMode', 'auto', ...
        'InvertHardcopy', 'off');
end

function savePaperFigure(fig, stem, widthCm, heightCm, P)

    [folder, ~, ~] = fileparts(stem);
    ensureDir(folder);

    set(fig, 'Units', 'centimeters');
    set(fig, 'Position', [0, 0, widthCm, heightCm]);
    drawnow;

    if P.savePdf
        exportgraphics(fig, stem + ".pdf", ...
            'ContentType', P.export.pdfContentType, ...
            'BackgroundColor', 'white', ...
            'Resolution', P.export.pdfResolution, ...
            'Width', widthCm, ...
            'Height', heightCm, ...
            'Padding', P.export.padding, ...
            'Units', 'centimeters');
    end

    if P.savePng
        exportgraphics(fig, stem + ".png", ...
            'ContentType', P.export.pngContentType, ...
            'BackgroundColor', 'white', ...
            'Resolution', P.export.pngResolution, ...
            'Width', widthCm, ...
            'Height', heightCm, ...
            'Padding', P.export.padding, ...
            'Units', 'centimeters');
    end
end

function data = loadSweepData(results, trajNames)

    data = struct('name', {}, 'time', {}, 'log', {}, 'par', {});
    for i = 1:numel(trajNames)
        idx = find(string(results.Trajectory) == trajNames(i), 1);
        if isempty(idx) || strlength(string(results.ErrorMessage(idx))) > 0
            continue;
        end
        matFile = string(results.MatFile(idx));
        if ~isfile(matFile)
            continue;
        end
        S = load(matFile, 'time', 'log', 'par');
        S.name = trajNames(i);
        data(end+1) = S; %#ok<AGROW>
    end
end

function resultFile = latestSweepResultsFile()

    rootDir = fullfile(pwd, "results", "main_trajectory_sweep");
    files = dir(fullfile(rootDir, "*", "main_trajectory_sweep_results.mat"));
    if isempty(files)
        error("No trajectory sweep result found under %s.", rootDir);
    end
    [~, idx] = max([files.datenum]);
    resultFile = string(fullfile(files(idx).folder, files(idx).name));
end

function names = selectedNames(value, defaultNames)

    names = string(value);
    names = names(strlength(names) > 0);
    if isempty(names)
        names = string(defaultNames);
    end
    names = names(:).';
end

function values = appendMissing(values, observed)

    values = string(values(:)).';
    observed = string(observed(:)).';
    values = [values, observed(~ismember(observed, values))];
    values = values(ismember(values, observed));
end

function offsets = groupedOffsets(n, width)

    if n <= 1
        offsets = 0;
    else
        offsets = linspace(-width/2, width/2, n);
    end
end

function color = controllerColor(P, ctrlName, idx)

    field = matlab.lang.makeValidName(char(ctrlName));
    if isfield(P.controllerColor, field)
        color = P.controllerColor.(field);
    else
        color = lines(max(idx, 1));
        color = color(end,:);
    end
end

function label = controllerLabel(P, ctrlName)

    field = matlab.lang.makeValidName(char(ctrlName));
    if isfield(P.controllerLabel, field)
        label = string(P.controllerLabel.(field));
    else
        label = string(ctrlName);
    end
end

function txt = titleFor(titleStruct, name)

    field = matlab.lang.makeValidName(char(name));
    if isfield(titleStruct, field)
        txt = string(titleStruct.(field));
    else
        txt = string(name);
    end
end

function applyOptionalYLimit(ax, yLimits, trajName)

    field = matlab.lang.makeValidName(char(trajName));
    if isstruct(yLimits) && isfield(yLimits, field) ...
            && ~isempty(yLimits.(field))
        ylim(ax, yLimits.(field));
    end
end

function labels = disturbanceLevelLabels(levelNames, labelStruct)

    labels = string(levelNames);
    for i = 1:numel(labels)
        field = matlab.lang.makeValidName(char(labels(i)));
        if isstruct(labelStruct) && isfield(labelStruct, field)
            labels(i) = string(labelStruct.(field));
        end
    end
end

function addDisturbanceLevelLabels(ax, labels, fontSize)

    fig = ancestor(ax, 'figure');
    n = numel(labels);
    if n == 0
        return;
    end

    ax.Units = 'normalized';
    pos = ax.Position;
    bottomPad = 0.10;
    ax.Position = [pos(1), pos(2) + bottomPad, ...
        pos(3), max(0.1, pos(4) - bottomPad)];

    boxW = 0.92*pos(3)/n;
    boxH = 0.05;
    boxY = max(0.01, pos(2) + 0.025);
    for i = 1:n
        xCenter = pos(1) + pos(3)*(i - 0.5)/n;
        annotation(fig, 'textbox', ...
            [xCenter - boxW/2, boxY, boxW, boxH], ...
            'String', char(labels(i)), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'top', ...
            'Interpreter', 'none', ...
            'EdgeColor', 'none', ...
            'FitBoxToText', 'off', ...
            'FontSize', fontSize);
    end
end

function setTrajectoryLimits(ax, pAll)

    span = max(max(pAll, [], 2) - min(pAll, [], 2));
    margin = 0.1*max(span, 1);
    xlim(ax, [min(pAll(1,:))-margin, max(pAll(1,:))+margin]);
    ylim(ax, [min(pAll(2,:))-margin, max(pAll(2,:))+margin]);
    zlim(ax, [min(pAll(3,:))-margin, max(pAll(3,:))+margin]);
end

function formatAxes(ax, P)

    ax.FontName = char(P.fontName);
    ax.FontSize = P.fontSize;
    ax.LineWidth = P.axisLineWidth;
    ax.TickLabelInterpreter = 'none';
    box(ax, 'on');
end

function ensureDir(folder)

    if strlength(string(folder)) == 0
        return;
    end
    if ~exist(folder, 'dir')
        mkdir(folder);
    end
end
