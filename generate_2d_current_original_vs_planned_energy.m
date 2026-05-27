clear;
close all;
clc;

project_root = fileparts(mfilename('fullpath'));
figure_dir = fullfile(project_root, 'paper-output', 'figures');
source_data = load(fullfile(figure_dir, 'fig3-2d-current-planned-run.mat'));

MAX_X = 25;
MAX_Y = 25;
Robot_1 = source_data.Robot_1;
Target_12 = source_data.Target_12;
Obstale = source_data.Obstale;
aerf = source_data.aerf;
V_AUV = source_data.V_AUV;
V_ocean = source_data.V_ocean;
shixian_ji = source_data.shixian_ji;
auv_num = size(Robot_1, 2);

blocked = false(MAX_X, MAX_Y);
for i = 1:size(Obstale, 2)
    r = Obstale(1, i);
    c = Obstale(2, i);
    if r >= 1 && r <= MAX_X && c >= 1 && c <= MAX_Y
        blocked(r, c) = true;
    end
end

original_energy = zeros(1, auv_num);
planned_energy = zeros(1, auv_num);
original_distance = zeros(1, auv_num);
planned_distance = zeros(1, auv_num);
original_paths = cell(1, auv_num);
planned_paths = cell(1, auv_num);

valid_cols = find(shixian_ji(1, :) >= 1 & shixian_ji(1, :) <= size(Target_12, 2));
for k = 1:auv_num
    current_point = round(Robot_1(:, k));
    task_cols = valid_cols(shixian_ji(2, valid_cols) == k);
    original_paths{k} = current_point;
    planned_paths{k} = current_point;
    for idx = 1:length(task_cols)
        target_id = round(shixian_ji(1, task_cols(idx)));
        target_point = round(Target_12(:, target_id));
        distance_path = astar_grid(current_point, target_point, blocked, aerf, V_AUV, V_ocean, 'distance');
        energy_path = astar_grid(current_point, target_point, blocked, aerf, V_AUV, V_ocean, 'energy');
        [seg_energy, seg_distance] = path_metric(distance_path, aerf, V_AUV, V_ocean);
        original_energy(k) = original_energy(k) + seg_energy;
        original_distance(k) = original_distance(k) + seg_distance;
        [seg_energy, seg_distance] = path_metric(energy_path, aerf, V_AUV, V_ocean);
        planned_energy(k) = planned_energy(k) + seg_energy;
        planned_distance(k) = planned_distance(k) + seg_distance;
        original_paths{k} = append_path(original_paths{k}, distance_path);
        planned_paths{k} = append_path(planned_paths{k}, energy_path);
        current_point = target_point;
    end
end

energy_save = original_energy - planned_energy;
energy_save_rate = zeros(1, auv_num);
valid = original_energy > 0;
energy_save_rate(valid) = energy_save(valid) ./ original_energy(valid) * 100;

total_original = sum(original_energy);
total_planned = sum(planned_energy);
total_save = total_original - total_planned;
total_save_rate = total_save / total_original * 100;

summary_table = table((1:auv_num)', original_energy(:), planned_energy(:), energy_save(:), energy_save_rate(:), ...
    original_distance(:), planned_distance(:), ...
    'VariableNames', {'AUVIndex', 'OriginalPathEnergy', 'PlannedPathEnergy', 'EnergySaving', ...
    'EnergySavingPercent', 'OriginalPathDistance', 'PlannedPathDistance'});
writetable(summary_table, fullfile(figure_dir, 'fig3-2d-current-original-vs-planned-energy-summary.csv'));

auv_names = {'AUV1', 'AUV2', 'AUV3', 'AUV4', 'AUV5'};

fig = new_single_figure();
bar([original_energy(:), planned_energy(:)], 0.72);
set(gca, 'XTickLabel', auv_names, 'FontName', 'Microsoft YaHei', 'FontSize', 12);
ylabel('累计能耗 / 相对单位', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
xlabel('AUV编号', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
title('洋流条件下原路径与规划后路径能耗对比', 'FontName', 'Microsoft YaHei', 'FontSize', 16, 'FontWeight', 'bold');
legend({'原路径', '规划后路径'}, 'Location', 'northwest', 'FontName', 'Microsoft YaHei');
grid on;
saveas(fig, fullfile(figure_dir, 'fig3-2d-current-original-vs-planned-energy-bar.png'));
close(fig);

fig = new_single_figure();
b = bar([total_original, total_planned], 0.48);
b.FaceColor = 'flat';
b.CData(1,:) = [0.00, 0.45, 0.74];
b.CData(2,:) = [0.85, 0.33, 0.10];
set(gca, 'XTickLabel', {'无顺流激励', '有顺流激励'}, 'FontName', 'Microsoft YaHei', 'FontSize', 13);
ylabel('系统总能耗 / 相对单位', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
title('有无顺流激励条件下系统总能耗对比', 'FontName', 'Microsoft YaHei', 'FontSize', 16, 'FontWeight', 'bold');
grid on;
text(1, total_original, sprintf('%.2f', total_original), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontName', 'Microsoft YaHei', 'FontSize', 12);
text(2, total_planned, sprintf('%.2f', total_planned), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontName', 'Microsoft YaHei', 'FontSize', 12);
text(1.5, max([total_original,total_planned]) * 0.86, sprintf('降低 %.2f%%', total_save_rate), 'HorizontalAlignment', 'center', 'FontName', 'Microsoft YaHei', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig, fullfile(figure_dir, 'fig3-2d-current-original-vs-planned-total-energy.png'));
close(fig);

fig = new_single_figure();
bar([original_distance(:), planned_distance(:)], 0.72);
set(gca, 'XTickLabel', auv_names, 'FontName', 'Microsoft YaHei', 'FontSize', 12);
ylabel('累计里程 / 栅格', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
xlabel('AUV编号', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
title('洋流条件下原路径与规划后路径里程对比', 'FontName', 'Microsoft YaHei', 'FontSize', 16, 'FontWeight', 'bold');
legend({'原路径', '规划后路径'}, 'Location', 'northwest', 'FontName', 'Microsoft YaHei');
grid on;
saveas(fig, fullfile(figure_dir, 'fig3-2d-current-original-vs-planned-distance-bar.png'));
close(fig);

fig = figure('Visible', 'off');
set(fig, 'Color', 'w', 'Position', [100, 100, 1050, 620]);
hold on;
axis([0 MAX_Y+1 0 MAX_X+1]);
set(gca, 'YDir', 'normal', 'FontName', 'Microsoft YaHei', 'FontSize', 11);
grid on;
plot(Obstale(2,:), Obstale(1,:), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', [0.15 0.15 0.15]);
plot(Robot_1(2,:), Robot_1(1,:), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(Target_12(2,:), Target_12(1,:), 'p', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', [0 0.45 0]);
colors = lines(auv_num);
for k = 1:auv_num
    if size(original_paths{k},2) > 1
        plot(original_paths{k}(2,:), original_paths{k}(1,:), '--', 'Color', colors(k,:), 'LineWidth', 1.2);
    end
    if size(planned_paths{k},2) > 1
        plot(planned_paths{k}(2,:), planned_paths{k}(1,:), '-', 'Color', colors(k,:), 'LineWidth', 2.2);
    end
end
xlabel('X坐标 / 栅格', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
ylabel('Y坐标 / 栅格', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
title('洋流条件下原路径与规划后路径轨迹对比', 'FontName', 'Microsoft YaHei', 'FontSize', 16, 'FontWeight', 'bold');
legend({'障碍物','AUV起点','目标点','原路径','规划后路径'}, 'Location', 'northeastoutside', 'FontName', 'Microsoft YaHei');
saveas(fig, fullfile(figure_dir, 'fig3-2d-current-original-vs-planned-path-map.png'));
close(fig);

disp(fullfile(figure_dir, 'fig3-2d-current-original-vs-planned-energy-bar.png'));
disp(fullfile(figure_dir, 'fig3-2d-current-original-vs-planned-total-energy.png'));
disp(fullfile(figure_dir, 'fig3-2d-current-original-vs-planned-path-map.png'));
disp(total_save_rate);

function fig = new_single_figure()
    fig = figure('Visible', 'off');
    set(fig, 'Color', 'w', 'Position', [120, 120, 820, 580]);
end

function joined = append_path(base_path, add_path)
    if isempty(add_path)
        joined = base_path;
    elseif isempty(base_path)
        joined = add_path;
    else
        joined = [base_path, add_path(:, 2:end)];
    end
end

function path = astar_grid(start_point, goal_point, blocked, aerf, V_AUV, V_ocean, mode_name)
    max_r = size(blocked, 1);
    max_c = size(blocked, 2);
    start_point = max([1;1], min([max_r;max_c], round(start_point(:))));
    goal_point = max([1;1], min([max_r;max_c], round(goal_point(:))));
    g = inf(max_r, max_c);
    f = inf(max_r, max_c);
    open = false(max_r, max_c);
    closed = false(max_r, max_c);
    parent_r = zeros(max_r, max_c);
    parent_c = zeros(max_r, max_c);
    sr = start_point(1); sc = start_point(2);
    gr = goal_point(1); gc = goal_point(2);
    g(sr, sc) = 0;
    f(sr, sc) = heuristic_cost(start_point, goal_point, mode_name);
    open(sr, sc) = true;
    neighbors = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
    while any(open(:))
        temp_f = f;
        temp_f(~open) = inf;
        [~, linear_idx] = min(temp_f(:));
        [cr, cc] = ind2sub(size(temp_f), linear_idx);
        if cr == gr && cc == gc
            path = reconstruct_path(parent_r, parent_c, start_point, goal_point);
            return;
        end
        open(cr, cc) = false;
        closed(cr, cc) = true;
        for i = 1:size(neighbors, 1)
            nr = cr + neighbors(i, 1);
            nc = cc + neighbors(i, 2);
            if nr < 1 || nr > max_r || nc < 1 || nc > max_c || blocked(nr, nc) || closed(nr, nc)
                continue;
            end
            if strcmp(mode_name, 'energy')
                [step_cost, ~] = segment_energy([cr;cc], [nr;nc], aerf, V_AUV, V_ocean);
            else
                step_cost = hypot(nr-cr, nc-cc);
            end
            tentative_g = g(cr, cc) + step_cost;
            if ~open(nr, nc) || tentative_g < g(nr, nc)
                parent_r(nr, nc) = cr;
                parent_c(nr, nc) = cc;
                g(nr, nc) = tentative_g;
                f(nr, nc) = tentative_g + heuristic_cost([nr;nc], goal_point, mode_name);
                open(nr, nc) = true;
            end
        end
    end
    path = [start_point, goal_point];
end

function h = heuristic_cost(point, goal_point, mode_name)
    disto = hypot(goal_point(1)-point(1), goal_point(2)-point(2));
    if strcmp(mode_name, 'energy')
        h = disto * 2;
    else
        h = disto;
    end
end

function path = reconstruct_path(parent_r, parent_c, start_point, goal_point)
    path = goal_point(:);
    cr = goal_point(1);
    cc = goal_point(2);
    while ~(cr == start_point(1) && cc == start_point(2))
        pr = parent_r(cr, cc);
        pc = parent_c(cr, cc);
        if pr == 0 && pc == 0
            path = [start_point(:), goal_point(:)];
            return;
        end
        path = [[pr; pc], path];
        cr = pr;
        cc = pc;
    end
end

function [energy_total, dist_total] = path_metric(path_data, aerf, V_AUV, V_ocean)
    energy_total = 0;
    dist_total = 0;
    if isempty(path_data) || size(path_data, 2) < 2
        return;
    end
    for i = 1:size(path_data, 2)-1
        [seg_energy, seg_dist] = segment_energy(path_data(:, i), path_data(:, i+1), aerf, V_AUV, V_ocean);
        energy_total = energy_total + seg_energy;
        dist_total = dist_total + seg_dist;
    end
end

function [energy, disto] = segment_energy(p1, p2, aerf, V_AUV, V_ocean)
    dx = p2(1) - p1(1);
    dy = p2(2) - p1(2);
    disto = hypot(dx, dy);
    energy = 0;
    if disto == 0
        return;
    end
    ocean_x = max(1, min(size(aerf, 1), round(p1(1))));
    ocean_y = max(1, min(size(aerf, 2), round(p1(2))));
    ocean_dir = aerf(ocean_x, ocean_y);
    angle_path = rad2deg(atan2(dy, dx));
    angle_diff = angle_path - ocean_dir;
    V_ocean_cos = V_ocean * cosd(angle_diff);
    speed_cross = max(0, V_AUV^2 - V_ocean_cos^2);
    effective_speed = sqrt(speed_cross) + V_ocean_cos;
    effective_speed = max(effective_speed, 1e-6);
    travel_time = disto / effective_speed;
    energy = V_AUV^3 * travel_time;
end