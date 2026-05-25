clear;
close all;
clc;

% =======================================================
% 1. 三维空间与环境初始化
% =======================================================
MAX_X = 25; MAX_Y = 25; MAX_Z = 25; % 定义 3D 边界

% 目标点 (3D 坐标): [X; Y; Z]
Target_12 = [13, 21;   % 目标1 X, 目标2 X
             21,  5;   % 目标1 Y, 目标2 Y
             20,  5];  % 目标1 Z(高处), 目标2 Z(深水)

% AUV 起点: [X; Y; Z]
Robot_1 = [5; 5; 5];

% 3D 障碍物：一根从水底(Z=1)直插水面(Z=22)的巨大正方形暗礁柱子
Obstale_X = []; Obstale_Y = []; Obstale_Z = [];
for x = 11:15
    for y = 4:15
        for z = 1:22
            Obstale_X = [Obstale_X, x];
            Obstale_Y = [Obstale_Y, y];
            Obstale_Z = [Obstale_Z, z];
        end
    end
end
Obstale = [Obstale_X; Obstale_Y; Obstale_Z];

TargetNum = size(Target_12,2); 
Target_NO = 1:TargetNum;
Target = [Target_12; Target_NO]; % [X; Y; Z; 编号]

% =======================================================
% 2. 神经网络参数与能耗初始化
% =======================================================
V_AUV = 2.0; % AUV 航速

robot_energy_total = zeros(1, size(Robot_1,2)); 
robot_dist_total = zeros(1, size(Robot_1,2)); 
gamma_penalty = 1e-4; 

% 生成 3D 全局与局部神经网络活性图 (传入 MAX_Z)
[activityvalue__2_dt] = weight_wij(Target(1:3,:), Obstale, TargetNum, MAX_X, MAX_Y, MAX_Z);
[activityvalue__2_quanju_dt] = weight_quanju(Target(1:3,:), Obstale, TargetNum, MAX_X, MAX_Y, MAX_Z);

dist_act_1 = zeros(1, size(Robot_1,2));
% 机器人初始映射
[Weight_vir{1}, Weight_act{1}, Robot] = robot_no_activity(Robot_1, activityvalue__2_quanju_dt, dist_act_1);
Weight_act_total{1} = Weight_act{1};

% =======================================================
% 3. 3D 寻路与避障主循环
% =======================================================
Step = 10; 
B = 1; w = 1;
open_count = []; close_count = []; 
COO = double(1); 
shixian_ji = zeros(5,1); % [编号; ID; X; Y; Z]

% 初始化历史记录（包含初始状态 0）
energy_history = robot_energy_total;
dist_history = robot_dist_total;

for t = 1:Step
    huosheng_1 = winner_robot(Weight_vir{t}, activityvalue__2_dt, TargetNum, activityvalue__2_quanju_dt, close_count, robot_energy_total, gamma_penalty);
    [open_count, MH] = MUL_AUV(open_count, huosheng_1, close_count);
    Pn_empty = [];

    for ii = 1:TargetNum
        if ismember(ii, close_count)
            continue;
        end
        
        % 计算 3D 欧氏距离
        [distn, P_2_act, P_3_act, P_4_act] = A1_somdist(Target(1:3,ii), huosheng_1, ii, Weight_act{w});
        
        if(distn <= 0.8) % 3D 容差稍微放大至 0.8，判定为到达目标
            close_count = [close_count, ii]; 
            shixian = [ii; huosheng_1(1,ii); P_2_act; P_3_act; P_4_act];
            if(ismember(ii, shixian_ji(1,:))) == 1
                continue;
            else
                shixian_ji = [shixian_ji, shixian];
            end
        end
        
        % 任务死锁避免逻辑
        LO = ismember(ii, open_count);
        if LO == 1 
            COO = []; target_jj = 0;
            if iscell(MH) && ~isempty(MH) 
                for jj = 1:length(MH)
                    temp_COO = find(MH{jj}(2,:) == ii);
                    if ~isempty(temp_COO)
                        COO = temp_COO;
                        target_jj = jj;
                        break;
                    end
                end
            end
            if ~isempty(COO) && COO > 1
                precoo = ismember(MH{target_jj}(2, COO-1), close_count);
                if(~precoo), continue; end
            end            
        end
        
        % 调用 3D 寻路雷达 (无洋流版)
        Pn_point = Pn(huosheng_1, activityvalue__2_dt{ii}, ii, MAX_X, MAX_Y, MAX_Z); 
        
        winnerR = huosheng_1(1:5, ii); 
        wiiner_R_act = Weight_act{w}(1:5, winnerR(1,1));
        
        % 权值更新 (3D 避墙滑动)
        Neigh{w} = A1_NeighborFunction(Weight_act{w}, huosheng_1(1:5,ii), t, huosheng_1, wiiner_R_act, close_count);
        Weight_act{w+1} = A1_updateWeight(Weight_act{w}, B, Neigh{w}, distn, Target(1:3,ii), Pn_point, activityvalue__2_quanju_dt);
        
        winnernum = winnerR(1,1);
        u_before = Weight_act{w}(1:5, winnernum);
        u_after = Weight_act{w+1}(1:5, winnernum);
        temp_weight_total = {u_before, u_after}; 
        
        % 调用 3D 能耗与航程计算
        [step_dist, ~, step_energy] = A1_somdist_act(temp_weight_total, 1, 0, 0, V_AUV);
        
        robot_energy_total(winnernum) = robot_energy_total(winnernum) + step_energy;
        robot_dist_total(winnernum) = robot_dist_total(winnernum) + step_dist;

        w = w + 1;
        Pn_empty = [Pn_empty, Pn_point];
    end
    Weight_act_total{t+1} = Weight_act{w}; 
    
    % 候选者更新
    Pn_next = next_houxuan(Weight_act{w}, huosheng_1, activityvalue__2_quanju_dt, Pn_empty);
    Weight_vir{t+1} = Pn_next;
    huosheng{t} = huosheng_1;
    
    % 追加历史记录
    energy_history = [energy_history; robot_energy_total];
    dist_history = [dist_history; robot_dist_total];
    
    if ~isempty(close_count) && length(unique(close_count(1,:))) == TargetNum
        break;
    end
end

% =======================================================
% 4. Figure 1: 3D 轨迹地图可视化
% =======================================================
figure(1);
set(gcf, 'Position', [50, 100, 700, 600]); 
colors = ['r', 'b', 'g', 'm', 'c']; 

for PPB3 = 1:size(Robot_1,2)
    for PPA3 = 1:size(Weight_act,2)
        Weight_act_16{PPB3}(1:5, PPA3) = Weight_act{PPA3}(1:5, PPB3);
    end
    Weight_act_16{PPB3} = unique(Weight_act_16{PPB3}','rows','stable')';
    
    % 提取 X, Y, Z (分别对应矩阵的 2, 3, 4 行)
    path_x = Weight_act_16{PPB3}(2,:);
    path_y = Weight_act_16{PPB3}(3,:);
    path_z = Weight_act_16{PPB3}(4,:);
    
    % 使用样条插值生成平滑 3D 曲线
    if length(path_x) > 2
        t_raw = 1:length(path_x);
        t_smooth = linspace(1, length(path_x), 200);
        smooth_x = spline(t_raw, path_x, t_smooth);
        smooth_y = spline(t_raw, path_y, t_smooth);
        smooth_z = spline(t_raw, path_z, t_smooth);
        plot3(smooth_x, smooth_y, smooth_z, '-', 'Color', colors(PPB3), 'LineWidth', 2.5); hold on;
        plot3(path_x, path_y, path_z, '.', 'Color', colors(PPB3), 'MarkerSize', 8); % 打点
    else
        plot3(path_x, path_y, path_z, '.-', 'Color', colors(PPB3), 'LineWidth', 2.5, 'MarkerSize', 10); hold on;
    end
end

% 画起点、终点和障碍物
plot3(Robot_1(1,:), Robot_1(2,:), Robot_1(3,:), '.r', 'MarkerSize', 30); % AUV 起点
text(Robot_1(1,:)-1, Robot_1(2,:)+1, Robot_1(3,:), 'AUV', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');

plot3(Target_12(1,:), Target_12(2,:), Target_12(3,:), 'pg', 'MarkerFaceColor', 'g', 'MarkerSize', 15); % 目标
for i = 1:TargetNum
    text(Target_12(1,i)-1, Target_12(2,i)+1, Target_12(3,i)+1, ['T', num2str(i)], 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
end

% 绘制半透明的 3D 障碍物暗礁
scatter3(Obstale(1,:), Obstale(2,:), Obstale(3,:), 25, 'k', 'filled', 'MarkerFaceAlpha', 0.1); 

axis([0 MAX_X 0 MAX_Y 0 MAX_Z]);
grid on;
view(3); % 调整为三维默认视角
% view(-30, 30); % 你也可以解除注释这行，获得一个自定义的俯视+侧视角度
xlabel('X坐标 / 栅格', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y坐标 / 栅格', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z坐标 / 栅格', 'FontSize', 12, 'FontWeight', 'bold');
title('AUV三维空间一对一路径规划结果', 'FontSize', 14, 'FontWeight', 'bold');
legend({'AUV路径', '轨迹节点', 'AUV起点', '目标点', '障碍物'}, 'Location', 'northeastoutside');

% =======================================================
% 5. Figure 2 & 3: 能耗与航程平滑折线图
% =======================================================
AUV_num = size(Robot_1, 2);
legend_str = cell(1, AUV_num);
for k = 1:AUV_num, legend_str{k} = ['AUV ', num2str(k), '号']; end

% --- Figure 2: 能耗图 ---
figure(2); set(gcf, 'Position', [750, 400, 500, 300]); 
for k = 1:AUV_num
    x_raw = 0:(size(energy_history, 1)-1);
    y_raw = energy_history(:, k)';
    x_smooth = linspace(0, max(x_raw), 300); 
    y_smooth = spline(x_raw, y_raw, x_smooth); 
    plot(x_smooth, y_smooth, 'LineWidth', 2.5, 'Color', colors(mod(k-1, 5)+1)); hold on;
end
grid on; xlim([0, max(x_raw)]); ylim([0, inf]);
title('AUV累计能耗变化曲线', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('迭代步数 / 步'); ylabel('累计能耗 / 相对单位'); legend(legend_str, 'Location', 'northwest');

% --- Figure 3: 航程图 ---
figure(3); set(gcf, 'Position', [750, 50, 500, 300]); 
for k = 1:AUV_num
    x_raw = 0:(size(dist_history, 1)-1);
    y_raw = dist_history(:, k)';
    x_smooth = linspace(0, max(x_raw), 300); 
    y_smooth = spline(x_raw, y_raw, x_smooth); 
    plot(x_smooth, y_smooth, 'LineWidth', 2.5, 'Color', colors(mod(k-1, 5)+1)); hold on;
end
grid on; xlim([0, max(x_raw)]); ylim([0, inf]);
title('AUV累计航程变化曲线', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('迭代步数 / 步'); ylabel('累计航程 / m'); legend(legend_str, 'Location', 'northwest');