% Generate A* and APF comparison figures for the fixed screenshot environment.
% This script keeps the GBSOM finalized figure unchanged and only refreshes
% the baseline path figures with sequential start points for repeated AUVs.
clear;
close all;
clc;

project_root = fileparts(mfilename('fullpath'));
addpath(fullfile(project_root, 'matlab', 'baselines'));
addpath(fullfile(project_root, 'matlab', 'utils'));
addpath(fullfile(project_root, 'matlab', 'visualization'));

MAX_X = 25; MAX_Y = 25; MAX_Z = 25;
V_AUV = 2.0;

Target_12 = [22, 20,  4, 24, 17,  3;
             22,  4, 22, 20, 15, 24;
              4, 22, 20,  2, 10,  3];

Robot_1 = [ 2,  4,  6,  8, 10;
            2,  2,  2,  2,  2;
            8,  8,  8,  8,  8];

Obstale_X = []; Obstale_Y = []; Obstale_Z = [];

for x = 12:15
    for y = 2:10
        for z = 1:25
            Obstale_X = [Obstale_X, x]; Obstale_Y = [Obstale_Y, y]; Obstale_Z = [Obstale_Z, z];
        end
    end
    for y = 18:23
        for z = 1:25
            Obstale_X = [Obstale_X, x]; Obstale_Y = [Obstale_Y, y]; Obstale_Z = [Obstale_Z, z];
        end
    end
end

for x = 12:15
    for y = 11:17
        for z = 8:15
            Obstale_X = [Obstale_X, x]; Obstale_Y = [Obstale_Y, y]; Obstale_Z = [Obstale_Z, z];
        end
    end
end

for x = 10:17
    for y = 10:18
        for z = 19:21
            Obstale_X = [Obstale_X, x]; Obstale_Y = [Obstale_Y, y]; Obstale_Z = [Obstale_Z, z];
        end
    end
end

for x = 1:5
    for y = 10:18
        for z = 2:6
            Obstale_X = [Obstale_X, x]; Obstale_Y = [Obstale_Y, y]; Obstale_Z = [Obstale_Z, z];
        end
        for z = 8:14
            Obstale_X = [Obstale_X, x]; Obstale_Y = [Obstale_Y, y]; Obstale_Z = [Obstale_Z, z];
        end
    end
end
Obstale = unique([Obstale_X; Obstale_Y; Obstale_Z]', 'rows')';

% Use the task relation from the current screenshot-version run. Repeated AUVs
% are handled sequentially inside run_baseline_comparison.
baseline_task_pairs = [4, 2; 5, 1; 3, 3; 4, 5; 2, 6; 5, 4];
baseline_comparison = run_baseline_comparison(Robot_1, Target_12, Obstale, MAX_X, MAX_Y, MAX_Z, V_AUV, baseline_task_pairs);

output_dir = fullfile(project_root, 'paper-output', 'figures');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

save_single_baseline_figure(fullfile(output_dir, 'fig5-2-astar-path.png'), ...
    'A*三维路径规划结果', Robot_1, Target_12, Obstale, MAX_X, MAX_Y, MAX_Z, ...
    collect_paths(baseline_comparison.A_.results));
save_single_baseline_figure(fullfile(output_dir, 'fig5-3-apf-path.png'), ...
    '人工势场法三维路径规划结果', Robot_1, Target_12, Obstale, MAX_X, MAX_Y, MAX_Z, ...
    collect_paths(baseline_comparison.APF.results));

task_pair_table = array2table(baseline_task_pairs, 'VariableNames', {'AUVIndex', 'TargetIndex'});
writetable(task_pair_table, fullfile(output_dir, 'fig5-task-pairs.csv'));

disp('A*与人工势场法路径图已按连续起点逻辑重新生成。');
disp(fullfile(output_dir, 'fig5-2-astar-path.png'));
disp(fullfile(output_dir, 'fig5-3-apf-path.png'));

function paths = collect_paths(results)
    paths = cell(1, numel(results));
    for idx = 1:numel(results)
        paths{idx} = results{idx}.path;
    end
end

function save_single_baseline_figure(file_name, figure_title, robot_starts, targets, obstacles, max_x, max_y, max_z, paths)
    figure('Visible', 'off');
    set(gcf, 'Position', [100, 100, 960, 720]);
    hold on;

    scatter3(obstacles(1, :), obstacles(2, :), obstacles(3, :), 30, [0.35 0.35 0.35], ...
        'filled', 'MarkerFaceAlpha', 0.35, 'MarkerEdgeAlpha', 0.60);
    plot3(robot_starts(1, :), robot_starts(2, :), robot_starts(3, :), 'ro', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 7);
    plot3(targets(1, :), targets(2, :), targets(3, :), 'gp', ...
        'MarkerFaceColor', 'g', 'MarkerSize', 12);

    color_map = lines(max(1, numel(paths)));
    path_handle = [];
    for idx = 1:numel(paths)
        path = paths{idx};
        if isempty(path) || size(path, 1) < 2
            continue;
        end
        h = plot3(path(:, 1), path(:, 2), path(:, 3), '-', ...
            'Color', color_map(idx, :), 'LineWidth', 2.2);
        plot3(path(:, 1), path(:, 2), path(:, 3), '.', ...
            'Color', color_map(idx, :), 'MarkerSize', 8);
        if isempty(path_handle)
            path_handle = h;
        end
    end

    for idx = 1:size(targets, 2)
        text(targets(1, idx), targets(2, idx), targets(3, idx) + 1, ...
            ['T', num2str(idx)], 'FontSize', 9, 'FontWeight', 'bold');
    end
    for idx = 1:size(robot_starts, 2)
        text(robot_starts(1, idx), robot_starts(2, idx), robot_starts(3, idx) - 1, ...
            ['A', num2str(idx)], 'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold');
    end

    axis([0 max_x 0 max_y 0 max_z]);
    grid on;
    view(42, 24);
    xlabel('X坐标 / 栅格');
    ylabel('Y坐标 / 栅格');
    zlabel('Z坐标 / 栅格');
    title(figure_title, 'FontWeight', 'bold');
    legend({'障碍物', 'AUV起点', '目标点', '规划路径'}, 'Location', 'northeastoutside');
    saveas(gcf, file_name);
    close(gcf);
end
