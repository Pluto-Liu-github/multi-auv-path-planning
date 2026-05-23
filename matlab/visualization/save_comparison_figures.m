function save_comparison_figures(robotStarts, targets, obstacles, maxX, maxY, maxZ, gbsomPaths, baselineComparison, comparisonTable, outputDir)
%SAVE_COMPARISON_FIGURES Export path and metric figures for thesis evidence.
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    save_obstacle_figure(fullfile(outputDir, 'fig5-0-obstacle-distribution.png'), ...
        robotStarts, targets, obstacles, maxX, maxY, maxZ);
    save_path_figure(fullfile(outputDir, 'fig5-1-gbsom-path.png'), 'GBSOM三维多AUV路径规划结果', ...
        robotStarts, targets, obstacles, maxX, maxY, maxZ, gbsomPaths);
    save_path_figure(fullfile(outputDir, 'fig5-2-astar-path.png'), 'A*三维路径规划结果', ...
        robotStarts, targets, obstacles, maxX, maxY, maxZ, collect_baseline_paths(baselineComparison.A_.results));
    save_path_figure(fullfile(outputDir, 'fig5-3-apf-path.png'), '人工势场法三维路径规划结果', ...
        robotStarts, targets, obstacles, maxX, maxY, maxZ, collect_baseline_paths(baselineComparison.APF.results));
    save_metric_figure(fullfile(outputDir, 'fig5-4-metric-bars.png'), comparisonTable);
end

function save_obstacle_figure(fileName, robotStarts, targets, obstacles, maxX, maxY, maxZ)
    figure('Visible', 'off');
    set(gcf, 'Position', [100, 100, 960, 720]);
    hold on;
    plot_grouped_obstacles(obstacles, 42, 0.50);
    plot3(robotStarts(1,:), robotStarts(2,:), robotStarts(3,:), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    plot3(targets(1,:), targets(2,:), targets(3,:), 'gp', 'MarkerFaceColor', 'g', 'MarkerSize', 13);
    for idx = 1:size(targets, 2)
        text(targets(1, idx), targets(2, idx), targets(3, idx) + 1, ['T', num2str(idx)], 'FontSize', 10, 'FontWeight', 'bold');
    end
    for idx = 1:size(robotStarts, 2)
        text(robotStarts(1, idx), robotStarts(2, idx), robotStarts(3, idx) - 1, ['A', num2str(idx)], 'FontSize', 9, 'Color', 'r');
    end
    axis([0 maxX 0 maxY 0 maxZ]);
    grid on;
    view(42, 24);
    xlabel('X坐标 / 栅格');
    ylabel('Y坐标 / 栅格');
    zlabel('Z坐标 / 栅格');
    title('对比实验三维障碍物分布示意图', 'FontWeight', 'bold');
    legend({'障碍物', 'AUV起点', '目标点'}, ...
        'Location', 'northeastoutside');
    saveas(gcf, fileName);
    close(gcf);
end

function save_path_figure(fileName, figureTitle, robotStarts, targets, obstacles, maxX, maxY, maxZ, paths)
    figure('Visible', 'off');
    set(gcf, 'Position', [100, 100, 960, 720]);
    hold on;

    plot_grouped_obstacles(obstacles, 30, 0.35);
    plot3(robotStarts(1,:), robotStarts(2,:), robotStarts(3,:), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
    plot3(targets(1,:), targets(2,:), targets(3,:), 'gp', 'MarkerFaceColor', 'g', 'MarkerSize', 12);

    colorMap = lines(max(1, numel(paths)));
    pathHandle = [];
    for idx = 1:numel(paths)
        path = normalize_path(paths{idx});
        if size(path, 1) < 2
            continue;
        end
        h = plot3(path(:, 1), path(:, 2), path(:, 3), '-', 'Color', colorMap(idx, :), 'LineWidth', 2.2);
        plot3(path(:, 1), path(:, 2), path(:, 3), '.', 'Color', colorMap(idx, :), 'MarkerSize', 8);
        if isempty(pathHandle)
            pathHandle = h;
        end
    end

    for idx = 1:size(targets, 2)
        text(targets(1, idx), targets(2, idx), targets(3, idx) + 1, ['T', num2str(idx)], 'FontSize', 9, 'FontWeight', 'bold');
    end
    for idx = 1:size(robotStarts, 2)
        text(robotStarts(1, idx), robotStarts(2, idx), robotStarts(3, idx) - 1, ['A', num2str(idx)], ...
            'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold');
    end

    axis([0 maxX 0 maxY 0 maxZ]);
    grid on;
    view(42, 24);
    xlabel('X坐标 / 栅格');
    ylabel('Y坐标 / 栅格');
    zlabel('Z坐标 / 栅格');
    title(figureTitle, 'FontWeight', 'bold');
    legend({'障碍物', 'AUV起点', '目标点', '规划路径'}, ...
        'Location', 'northeastoutside');
    saveas(gcf, fileName);
    close(gcf);
end

function plot_grouped_obstacles(obstacles, markerSize, alphaValue)
    obstacleColor = [0.35 0.35 0.35];
    scatter3(obstacles(1,:), obstacles(2,:), obstacles(3,:), markerSize, obstacleColor, ...
        'filled', 'MarkerFaceAlpha', alphaValue, 'MarkerEdgeAlpha', min(0.75, alphaValue + 0.25));
end

function groups = classify_obstacles(obstacles)
    groups = ones(1, size(obstacles, 2));
end

function save_metric_figure(fileName, comparisonTable)
    figure('Visible', 'off');
    set(gcf, 'Position', [100, 100, 900, 520]);

    metrics = [comparisonTable.TotalPathLength, comparisonTable.SearchSteps, comparisonTable.EquivalentEnergy];
    bar(metrics);
    set(gca, 'XTickLabel', comparisonTable.Method);
    xlabel('算法类型');
    ylabel('指标数值 / 见图例单位');
    title('三维路径规划算法量化指标对比', 'FontWeight', 'bold');
    legend({'总航程 / m', '搜索步数 / 步', '等效能耗 / 相对单位'}, 'Location', 'northwest');
    grid on;
    saveas(gcf, fileName);
    close(gcf);
end

function paths = collect_baseline_paths(results)
    paths = cell(1, numel(results));
    for idx = 1:numel(results)
        paths{idx} = results{idx}.path;
    end
end

function path = normalize_path(pathData)
    if isempty(pathData)
        path = [];
        return;
    end

    if size(pathData, 1) >= 4 && size(pathData, 2) ~= 3
        path = pathData(2:4, :)';
    elseif size(pathData, 1) == 3 && size(pathData, 2) ~= 3
        path = pathData';
    else
        path = pathData;
    end
end