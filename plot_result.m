addpath("/Users/mchmini/Proj/UAV_Algorithm_Benchmark")
close all;
% 1. 找到最近一次 disturbance benchmark 输出目录
rootDir = fullfile(pwd, "results", "disturbance_benchmark");
d = dir(rootDir);
d = d([d.isdir] & ~ismember({d.name}, {'.','..'}));
[~, idx] = max([d.datenum]);
outDir = fullfile(d(idx).folder, d(idx).name);

% 2. 读取保存的数据
S = load(fullfile(outDir, "disturbance_benchmark_results.mat"));
results = S.results;
cfg = S.cfg;

% 3. 选择箱线图数据源
cfg.boxDataSource = "time_error";  % 使用每个时刻的位置误差样本
% cfg.boxDataSource = "rmse";      % 或者只画每次仿真的 RMSE

figDir = fullfile(outDir, "figures_replot");
if ~exist(figDir, "dir")
    mkdir(figDir);
end

levelNames = string({cfg.levels.name});
controllerNames = string(cfg.controllerNames);

for iTraj = 1:numel(cfg.trajNames)
    trajName = string(cfg.trajNames(iTraj));
    mask = results.Trajectory == trajName & results.IsFinite;

    switch string(cfg.boxDataSource)
        case "time_error"
            xLabel = strings(0,1);
            groupLabel = strings(0,1);
            yData = zeros(0,1);

            for r = find(mask).'
                err = results.ErrorTrace{r};
                err = err(isfinite(err));
                n = numel(err);

                xLabel = [xLabel; repmat(results.DisturbanceLevel(r), n, 1)];
                groupLabel = [groupLabel; repmat(results.Controller(r), n, 1)];
                yData = [yData; err(:)];
            end

            yLabelText = "position tracking error samples (m)";

        case "rmse"
            xLabel = results.DisturbanceLevel(mask);
            groupLabel = results.Controller(mask);
            yData = results.RMSE(mask);
            yLabelText = "RMS position tracking error (m)";
    end

    figure("Color", "w");
    x = categorical(xLabel, levelNames, levelNames);
    group = categorical(groupLabel, controllerNames, controllerNames);

    boxchart(x, yData, "GroupByColor", group);
    grid on;
    xlabel("disturbance amplitude");
    ylabel(yLabelText);
    title("Trajectory: " + trajName, "Interpreter", "none");
    legend("Location", "northwest", "Interpreter", "none");

    ampText = strings(1, numel(cfg.levels));
    for k = 1:numel(cfg.levels)
        ampText(k) = sprintf("%s: %.3g N / %.3g N*m", ...
            string(cfg.levels(k).name), cfg.levels(k).forceAmp, cfg.levels(k).momentAmp);
    end
    subtitle(strjoin(ampText, ", "), "Interpreter", "none");

    exportgraphics(gcf, fullfile(figDir, char(trajName + "_disturbance_boxplot.png")), ...
        "Resolution", 200);
end