clear;
close all;
clc;

project_root = fileparts(mfilename('fullpath'));
output_dir = fullfile(project_root, 'paper-output', 'figures');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

MAX_X = 25;
MAX_Y = 25;

Target_12 = [22, 23, 14,  9, 15, 22;
              4,  6,  8, 18, 18, 20];
Robot_1 = [18,  4,  9, 18, 21;
           10, 11, 14, 15, 15];
Obstale = [16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,14,13,12,11,11,11,11,11,11,10,9,8,7,7,8,9,10,11,11,11,11,11,12,13,14,20,20,20,20,20,20,21,22,23,24,21,22,23,24;
            6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,6,6,6,6,7,8,9,10,11,11,11,11,11,16,16,16,16,16,17,18,19,20,20,20,20,6,7,8,18,19,20,8,8,8,8,18,18,18,18];;

V_ocean = 2;
aerf = zeros(MAX_X, MAX_Y);
center_x = MAX_Y / 2;
center_y = MAX_X / 2;
for y = 1:MAX_X
    for x = 1:MAX_Y
        angle_to_center = atan2d(y - center_y, x - center_x);
        aerf(y, x) = angle_to_center + 90;
    end
end

[plot_X, plot_Y] = meshgrid(1:MAX_Y, 1:MAX_X);
U_ocean_plot = zeros(size(plot_X));
V_ocean_plot = zeros(size(plot_Y));
for y = 1:MAX_X
    for x = 1:MAX_Y
        angle = aerf(y, x);
        U_ocean_plot(y, x) = V_ocean * cosd(angle);
        V_ocean_plot(y, x) = V_ocean * sind(angle);
    end
end

figure('Visible', 'off');
set(gcf, 'Color', 'w', 'Position', [100, 100, 960, 820]);
hold on;

h_current = quiver(plot_X(1:2:end, 1:2:end), plot_Y(1:2:end, 1:2:end), ...
    U_ocean_plot(1:2:end, 1:2:end), V_ocean_plot(1:2:end, 1:2:end), ...
    0.55, 'Color', [0.28 0.65 0.92], 'LineWidth', 1.0, 'MaxHeadSize', 1.4);
h_obstacle = scatter(Obstale(2, :), Obstale(1, :), 105, 's', 'filled', ...
    'MarkerFaceColor', [0.18 0.18 0.18], 'MarkerEdgeColor', [0.03 0.03 0.03], 'LineWidth', 0.5);
h_robot = scatter(Robot_1(2, :), Robot_1(1, :), 120, 'o', 'filled', ...
    'MarkerFaceColor', [0.92 0.05 0.05], 'MarkerEdgeColor', [0.45 0.00 0.00], 'LineWidth', 1.2);
h_target = scatter(Target_12(2, :), Target_12(1, :), 150, 'p', 'filled', ...
    'MarkerFaceColor', [0.00 0.75 0.18], 'MarkerEdgeColor', [0.00 0.35 0.00], 'LineWidth', 1.2);

for idx = 1:size(Robot_1, 2)
    text(Robot_1(2, idx) + 0.35, Robot_1(1, idx) - 0.55, ...
        ['A', num2str(idx)], 'Color', [0.80 0.00 0.00], ...
        'FontSize', 11, 'FontWeight', 'bold');
end
for idx = 1:size(Target_12, 2)
    text(Target_12(2, idx) + 0.35, Target_12(1, idx) + 0.45, ...
        ['T', num2str(idx)], 'Color', [0.00 0.30 0.00], ...
        'FontSize', 11, 'FontWeight', 'bold');
end

axis([0 MAX_Y + 1 0 MAX_X + 1]);
axis equal;
grid on;
box on;
set(gca, 'XTick', 0:5:25, 'YTick', 0:5:25, ...
    'FontName', 'Microsoft YaHei', 'FontSize', 12, 'Layer', 'top');
xlabel(u([88 22352 26631 32 47 32 26629 26684]), 'FontName', 'Microsoft YaHei', 'FontSize', 13, 'FontWeight', 'bold');
ylabel(u([89 22352 26631 32 47 32 26629 26684]), 'FontName', 'Microsoft YaHei', 'FontSize', 13, 'FontWeight', 'bold');
title(u([20108 32500 22810 65 85 86 20219 21153 29615 22659 22270]), ...
    'FontName', 'Microsoft YaHei', 'FontSize', 15, 'FontWeight', 'bold');
legend([h_obstacle, h_robot, h_target, h_current], ...
    {u([38556 30861 29289]), u([65 85 86 36215 28857]), u([30446 26631 28857]), u([27915 27969 26041 21521])}, ...
    'Location', 'northeastoutside', 'FontName', 'Microsoft YaHei', 'FontSize', 11);

file_name = fullfile(output_dir, 'fig2d-many-to-many-restored-screenshot-version.png');
saveas(gcf, file_name);
close(gcf);

disp(file_name);

function text_value = u(codes)
    text_value = char(codes);
end
