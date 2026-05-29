function save_chapter4_figures(Robot_1, Target_12, Obstale, MAX_X, MAX_Y, MAX_Z, Weight_act_16, energy_history, dist_history, output_dir)
%SAVE_CHAPTER4_FIGURES Export simulation figures used in Chapter 4.

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

colors = lines(size(Robot_1, 2));
target_count = size(Target_12, 2);
auv_count = size(Robot_1, 2);

fig_path = figure('Visible', 'off', 'Position', [100, 100, 960, 720]);
hold on;
path_handles = gobjects(1, 0);
path_labels = {};
for k = 1:auv_count
    path = Weight_act_16{k};
    if isempty(path)
        continue;
    end
    h_path = plot3(path(2, :), path(3, :), path(4, :), '-', ...
        'Color', colors(k, :), 'LineWidth', 2.4);
    path_handles(end + 1) = h_path;
    path_labels{end + 1} = ['AUV ', num2str(k), '规划路径'];
    plot3(path(2, 1), path(3, 1), path(4, 1), 'o', ...
        'Color', colors(k, :), 'MarkerFaceColor', colors(k, :), 'MarkerSize', 7, ...
        'HandleVisibility', 'off');
end

h_obstacle = scatter3(Obstale(1, :), Obstale(2, :), Obstale(3, :), 28, [0.06 0.06 0.06], 'filled', ...
    'MarkerFaceAlpha', 0.46, 'MarkerEdgeAlpha', 0.62);
h_start = plot3(Robot_1(1, :), Robot_1(2, :), Robot_1(3, :), 'ro', ...
    'MarkerFaceColor', [1.0 0.0 0.0], 'MarkerSize', 8, 'LineWidth', 1.2);
h_target = plot3(Target_12(1, :), Target_12(2, :), Target_12(3, :), 'p', ...
    'Color', [0.0 0.42 0.0], 'MarkerFaceColor', [0.0 0.80 0.15], 'MarkerSize', 13, 'LineWidth', 1.1);

for i = 1:target_count
    text(Target_12(1, i) + 0.4, Target_12(2, i) + 0.4, Target_12(3, i) + 0.4, ...
        ['T', num2str(i)], 'FontSize', 10, 'Color', [0 0.25 0.6], 'FontWeight', 'bold');
end
for i = 1:auv_count
    text(Robot_1(1, i) + 0.25, Robot_1(2, i) - 0.55, Robot_1(3, i) - 0.55, ...
        ['A', num2str(i)], 'FontSize', 9, 'Color', [0.72 0 0], 'FontWeight', 'bold');
end

grid on;
axis([0 MAX_X 0 MAX_Y 0 MAX_Z]);
view(42, 24);
xlabel('X坐标 / 栅格');
ylabel('Y坐标 / 栅格');
zlabel('Z坐标 / 栅格');
title('GBSOM三维路径规划结果');
legend_handles = [h_obstacle, h_start, h_target, path_handles];
legend_labels = [{'障碍物', 'AUV起点', '目标点'}, path_labels];
legend(legend_handles, legend_labels, 'Location', 'northeastoutside');
print(fig_path, fullfile(output_dir, 'fig4-1-3d-gbsom-path.png'), '-dpng', '-r300');
close(fig_path);

fig_energy = figure('Visible', 'off', 'Position', [100, 100, 780, 430]);
hold on;
x = 0:(size(energy_history, 1) - 1);
for k = 1:auv_count
    plot(x, energy_history(:, k), 'LineWidth', 2.0, 'Color', colors(k, :));
end
grid on;
xlabel('迭代步数 / 步');
ylabel('累计能耗 / 相对单位');
title('各AUV累计能耗变化曲线');
legend(compose('AUV %d号', 1:auv_count), 'Location', 'northwest');
print(fig_energy, fullfile(output_dir, 'fig4-2-energy-history.png'), '-dpng', '-r300');
close(fig_energy);

fig_dist = figure('Visible', 'off', 'Position', [100, 100, 780, 430]);
hold on;
x = 0:(size(dist_history, 1) - 1);
for k = 1:auv_count
    plot(x, dist_history(:, k), 'LineWidth', 2.0, 'Color', colors(k, :));
end
grid on;
xlabel('迭代步数 / 步');
ylabel('累计航程 / m');
title('各AUV累计航程变化曲线');
legend(compose('AUV %d号', 1:auv_count), 'Location', 'northwest');
print(fig_dist, fullfile(output_dir, 'fig4-3-distance-history.png'), '-dpng', '-r300');
close(fig_dist);
end
