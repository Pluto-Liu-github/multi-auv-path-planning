clear;
close all;
clc;

project_root = fileparts(mfilename('fullpath'));
output_dir = fullfile(project_root, 'paper-output', 'figures');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

MAX_X = 25; MAX_Y = 25; MAX_Z = 25;

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

save_projection(fullfile(output_dir, 'fig5-0a-obstacle-top-view.png'), ...
    Obstale([1, 2], :), Robot_1([1, 2], :), Target_12([1, 2], :), ...
    [0 MAX_X 0 MAX_Y], u([20463 35270 22270 65306 88 45 89 38556 30861 29289 20998 24067]), ...
    u([88 22352 26631 32 47 32 26629 26684]), u([89 22352 26631 32 47 32 26629 26684]), 'T', 'A');

save_projection(fullfile(output_dir, 'fig5-0b-obstacle-front-view.png'), ...
    Obstale([1, 3], :), Robot_1([1, 3], :), Target_12([1, 3], :), ...
    [0 MAX_X 0 MAX_Z], u([27491 35270 22270 65306 88 45 90 38556 30861 29289 20998 24067]), ...
    u([88 22352 26631 32 47 32 26629 26684]), u([90 22352 26631 32 47 32 26629 26684]), 'T', 'A');

save_projection(fullfile(output_dir, 'fig5-0c-obstacle-side-view.png'), ...
    Obstale([2, 3], :), Robot_1([2, 3], :), Target_12([2, 3], :), ...
    [0 MAX_Y 0 MAX_Z], u([20391 35270 22270 65306 89 45 90 38556 30861 29289 20998 24067]), ...
    u([89 22352 26631 32 47 32 26629 26684]), u([90 22352 26631 32 47 32 26629 26684]), 'T', 'A');

disp(fullfile(output_dir, 'fig5-0a-obstacle-top-view.png'));
disp(fullfile(output_dir, 'fig5-0b-obstacle-front-view.png'));
disp(fullfile(output_dir, 'fig5-0c-obstacle-side-view.png'));

function text_value = u(codes)
    text_value = char(codes);
end

function save_projection(file_name, obstacles2d, robots2d, targets2d, axis_limits, figure_title, x_label, y_label, target_prefix, robot_prefix)
    projected_obstacles = unique(round(obstacles2d'), 'rows')';

    figure('Visible', 'off');
    set(gcf, 'Color', 'w', 'Position', [100, 100, 820, 720]);
    hold on;

    h_obstacle = scatter(projected_obstacles(1, :), projected_obstacles(2, :), 95, ...
        's', 'filled', 'MarkerFaceColor', [0.20 0.20 0.20], ...
        'MarkerEdgeColor', [0.05 0.05 0.05], 'LineWidth', 0.3);
    h_robot = scatter(robots2d(1, :), robots2d(2, :), 85, ...
        'o', 'filled', 'MarkerFaceColor', [0.90 0.05 0.05], ...
        'MarkerEdgeColor', [0.50 0.00 0.00], 'LineWidth', 1.1);
    h_target = scatter(targets2d(1, :), targets2d(2, :), 130, ...
        'p', 'filled', 'MarkerFaceColor', [0.00 0.75 0.15], ...
        'MarkerEdgeColor', [0.00 0.35 0.00], 'LineWidth', 1.1);

    for idx = 1:size(targets2d, 2)
        text(targets2d(1, idx) + 0.35, targets2d(2, idx) + 0.35, ...
            [target_prefix, num2str(idx)], 'FontSize', 10, 'FontWeight', 'bold');
    end
    for idx = 1:size(robots2d, 2)
        text(robots2d(1, idx) + 0.25, robots2d(2, idx) - 0.75, ...
            [robot_prefix, num2str(idx)], 'FontSize', 10, 'FontWeight', 'bold', ...
            'Color', [0.80 0.00 0.00]);
    end

    axis(axis_limits);
    axis equal;
    grid on;
    box on;
    set(gca, 'XTick', 0:5:25, 'YTick', 0:5:25, 'FontName', 'Microsoft YaHei', 'FontSize', 11);
    xlabel(x_label, 'FontName', 'Microsoft YaHei', 'FontSize', 12);
    ylabel(y_label, 'FontName', 'Microsoft YaHei', 'FontSize', 12);
    title(figure_title, 'FontName', 'Microsoft YaHei', 'FontSize', 14, 'FontWeight', 'bold');
    legend([h_obstacle, h_robot, h_target], ...
        {u([38556 30861 29289]), u([65 85 86 36215 28857]), u([30446 26631 28857])}, ...
        'Location', 'northeastoutside', 'FontName', 'Microsoft YaHei');

    try
        exportgraphics(gcf, file_name, 'Resolution', 300);
    catch
        saveas(gcf, file_name);
    end
    close(gcf);
end
