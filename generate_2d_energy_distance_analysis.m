clear;
close all;
clc;

project_root = fileparts(mfilename('fullpath'));
figure_dir = fullfile(project_root, 'paper-output', 'figures');

with_data = load(fullfile(figure_dir, 'fig3-2d-current-metrics.mat'));
no_data = load(fullfile(figure_dir, 'fig3-2d-no-current-metrics.mat'));

auv_names = {'AUV1', 'AUV2', 'AUV3', 'AUV4', 'AUV5'};
with_energy_history = cumulative_abs_history(with_data.energy_history);
no_energy_history = cumulative_abs_history(no_data.energy_history);
with_energy_total = with_energy_history(end, :);
no_energy_total = no_energy_history(end, :);

summary_table = table((1:5)', ...
    no_energy_total(:), with_energy_total(:), ...
    no_data.robot_dist_total(:), with_data.robot_dist_total(:), ...
    'VariableNames', {'AUVIndex', 'EnergyNoCurrent', 'EnergyWithCurrent', ...
    'DistanceNoCurrent', 'DistanceWithCurrent'});
writetable(summary_table, fullfile(figure_dir, 'fig3-2d-current-energy-distance-summary.csv'));

% 图1：各AUV累计能耗对比
fig = new_single_figure();
bar([no_energy_total(:), with_energy_total(:)], 0.72);
set(gca, 'XTickLabel', auv_names, 'FontName', 'Microsoft YaHei', 'FontSize', 12);
ylabel('等效能耗 / 相对单位', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
xlabel('AUV编号', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
title('各AUV累计能耗对比', 'FontName', 'Microsoft YaHei', 'FontSize', 16, 'FontWeight', 'bold');
legend({'无洋流', '有洋流'}, 'Location', 'northwest', 'FontName', 'Microsoft YaHei');
grid on;
saveas(fig, fullfile(figure_dir, 'fig3-2d-energy-bar-current-vs-none.png'));
close(fig);

% 图2：各AUV累计里程对比
fig = new_single_figure();
bar([no_data.robot_dist_total(:), with_data.robot_dist_total(:)], 0.72);
set(gca, 'XTickLabel', auv_names, 'FontName', 'Microsoft YaHei', 'FontSize', 12);
ylabel('累计里程 / 栅格', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
xlabel('AUV编号', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
title('各AUV累计里程对比', 'FontName', 'Microsoft YaHei', 'FontSize', 16, 'FontWeight', 'bold');
legend({'无洋流', '有洋流'}, 'Location', 'northwest', 'FontName', 'Microsoft YaHei');
grid on;
saveas(fig, fullfile(figure_dir, 'fig3-2d-distance-bar-current-vs-none.png'));
close(fig);

% 图3：系统累计能耗变化曲线
fig = new_single_figure();
plot(0:size(no_energy_history,1)-1, sum(no_energy_history, 2), ...
    '--o', 'LineWidth', 2.0, 'MarkerSize', 4, 'Color', [0.00, 0.45, 0.74]);
hold on;
plot(0:size(with_energy_history,1)-1, sum(with_energy_history, 2), ...
    '-s', 'LineWidth', 2.0, 'MarkerSize', 4, 'Color', [0.85, 0.33, 0.10]);
xlabel('迭代步数 / 步', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
ylabel('总等效能耗 / 相对单位', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
title('系统累计能耗变化曲线', 'FontName', 'Microsoft YaHei', 'FontSize', 16, 'FontWeight', 'bold');
legend({'无洋流', '有洋流'}, 'Location', 'northwest', 'FontName', 'Microsoft YaHei');
grid on;
saveas(fig, fullfile(figure_dir, 'fig3-2d-energy-curve-current-vs-none.png'));
close(fig);

% 图4：系统累计里程变化曲线
fig = new_single_figure();
plot(0:size(no_data.dist_history,1)-1, sum(no_data.dist_history, 2), ...
    '--o', 'LineWidth', 2.0, 'MarkerSize', 4, 'Color', [0.00, 0.45, 0.74]);
hold on;
plot(0:size(with_data.dist_history,1)-1, sum(with_data.dist_history, 2), ...
    '-s', 'LineWidth', 2.0, 'MarkerSize', 4, 'Color', [0.85, 0.33, 0.10]);
xlabel('迭代步数 / 步', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
ylabel('总累计里程 / 栅格', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
title('系统累计里程变化曲线', 'FontName', 'Microsoft YaHei', 'FontSize', 16, 'FontWeight', 'bold');
legend({'无洋流', '有洋流'}, 'Location', 'northwest', 'FontName', 'Microsoft YaHei');
grid on;
saveas(fig, fullfile(figure_dir, 'fig3-2d-distance-curve-current-vs-none.png'));
close(fig);

% 保留一张汇总图，便于快速检查，不作为论文主图
fig = figure('Visible', 'off');
set(fig, 'Color', 'w', 'Position', [80, 80, 1250, 780]);
subplot(2,2,1); bar([no_energy_total(:), with_energy_total(:)]); set(gca, 'XTickLabel', auv_names, 'FontName', 'Microsoft YaHei'); ylabel('等效能耗 / 相对单位', 'FontName', 'Microsoft YaHei'); title('各AUV累计能耗对比', 'FontName', 'Microsoft YaHei', 'FontWeight', 'bold'); legend({'无洋流', '有洋流'}, 'Location', 'northwest', 'FontName', 'Microsoft YaHei'); grid on;
subplot(2,2,2); bar([no_data.robot_dist_total(:), with_data.robot_dist_total(:)]); set(gca, 'XTickLabel', auv_names, 'FontName', 'Microsoft YaHei'); ylabel('累计里程 / 栅格', 'FontName', 'Microsoft YaHei'); title('各AUV累计里程对比', 'FontName', 'Microsoft YaHei', 'FontWeight', 'bold'); legend({'无洋流', '有洋流'}, 'Location', 'northwest', 'FontName', 'Microsoft YaHei'); grid on;
subplot(2,2,3); plot(0:size(no_energy_history,1)-1, sum(no_energy_history, 2), '--o', 'LineWidth', 2.0, 'MarkerSize', 4); hold on; plot(0:size(with_energy_history,1)-1, sum(with_energy_history, 2), '-s', 'LineWidth', 2.0, 'MarkerSize', 4); xlabel('迭代步数 / 步', 'FontName', 'Microsoft YaHei'); ylabel('总等效能耗 / 相对单位', 'FontName', 'Microsoft YaHei'); title('系统累计能耗变化曲线', 'FontName', 'Microsoft YaHei', 'FontWeight', 'bold'); legend({'无洋流', '有洋流'}, 'Location', 'northwest', 'FontName', 'Microsoft YaHei'); grid on;
subplot(2,2,4); plot(0:size(no_data.dist_history,1)-1, sum(no_data.dist_history, 2), '--o', 'LineWidth', 2.0, 'MarkerSize', 4); hold on; plot(0:size(with_data.dist_history,1)-1, sum(with_data.dist_history, 2), '-s', 'LineWidth', 2.0, 'MarkerSize', 4); xlabel('迭代步数 / 步', 'FontName', 'Microsoft YaHei'); ylabel('总累计里程 / 栅格', 'FontName', 'Microsoft YaHei'); title('系统累计里程变化曲线', 'FontName', 'Microsoft YaHei', 'FontWeight', 'bold'); legend({'无洋流', '有洋流'}, 'Location', 'northwest', 'FontName', 'Microsoft YaHei'); grid on;
annotation('textbox', [0.18 0.945 0.66 0.04], 'String', '二维多AUV有无洋流条件下能耗与里程分析', 'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontName', 'Microsoft YaHei', 'FontSize', 16, 'FontWeight', 'bold');
saveas(fig, fullfile(figure_dir, 'fig3-2d-current-energy-distance-analysis.png'));
close(fig);

disp('已生成独立图片：');
disp(fullfile(figure_dir, 'fig3-2d-energy-bar-current-vs-none.png'));
disp(fullfile(figure_dir, 'fig3-2d-distance-bar-current-vs-none.png'));
disp(fullfile(figure_dir, 'fig3-2d-energy-curve-current-vs-none.png'));
disp(fullfile(figure_dir, 'fig3-2d-distance-curve-current-vs-none.png'));

function fig = new_single_figure()
    fig = figure('Visible', 'off');
    set(fig, 'Color', 'w', 'Position', [120, 120, 780, 560]);
end

function output = cumulative_abs_history(history)
    increments = [history(1, :); diff(history, 1, 1)];
    output = cumsum(abs(increments), 1);
end