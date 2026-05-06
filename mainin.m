clear;
close all;
clc;
% =======================================================
% =======================================================
% 1. 专门展示“任务分配特征”的 3D 极端测试矩阵
% =======================================================
MAX_X = 25; MAX_Y = 25; MAX_Z = 25; % 定义 3D 边界

%{ 
================= 【6个目标点】 =================
% 极具心机的目标布局
% T1: 诱饵点(中心)  T2,T3,T4,T5: 四周散开的星芒点  T6: 深处终极目标
Target_12 = [ 8, 20, 20, 20, 20, 24;   % X 坐标
             13, 23,  3, 13, 13, 13;   % Y 坐标
             13, 13, 13, 23,  3, 13];  % Z 坐标

% ================= 【5个 AUV 起点】 =================
% AUV 密集“十字阵型”抱团出生，故意制造绝对的“抢单冲突”
Robot_1 = [ 2,  2,  2,  2,  2;   % X 坐标 (都在同一平面)
           13, 15, 11, 13, 13;   % Y 坐标
           13, 13, 13, 15, 11];  % Z 坐标 (十字排列)

% ================= 【针对性 3D 障碍物：星际之门】 =================
% 在 X=12 的地方，建一堵横跨整个地图的巨大高墙
% 但是在正中央 (Y=8~18, Z=8~18) 掏一个大洞 (窗户)
Obstale_X = []; Obstale_Y = []; Obstale_Z = [];

for y = 1:25
    for z = 1:25
        % 如果不在中央窗户的范围内，就是实心墙壁
        if ~(y >= 8 && y <= 18 && z >= 8 && z <= 18) 
            Obstale_X = [Obstale_X, 12];
            Obstale_Y = [Obstale_Y, y];
            Obstale_Z = [Obstale_Z, z];
            
            % 为了让墙厚一点，加上 X=11 和 X=13
            Obstale_X = [Obstale_X, 11, 13];
            Obstale_Y = [Obstale_Y, y, y];
            Obstale_Z = [Obstale_Z, z, z];
        end
    end
end
%}
% =======================================================
% 1. 终极测试：三维空间多机多目标 + 极限避障立体迷宫
% =======================================================
MAX_X = 25; MAX_Y = 25; MAX_Z = 25; % 定义 3D 边界

% ================= 【6个目标点 (猎物)】 =================
% 散布在迷宫的各个死角、高地和深渊
Target_12 = [22, 20,  4, 24, 17,  3;   % X 坐标
             22,  4, 22, 20, 15, 24;   % Y 坐标
              4, 22, 20,  2, 10,  3];  % Z 坐标
% T1(深海区)  T2(高空区)  T3(回头高空)
% T4(最远深海) T5(迷宫中心) T6(二次接力)

% ================= 【5个 AUV 起点 (猎犬)】 =================
% 一字排开出生在地图左边缘，面前是巨大的复杂障碍群
Robot_1 = [ 2,  4,  6,  8, 10;   % X 坐标
            2,  2,  2,  2,  2;   % Y 坐标 (全在 Y=2 的起跑线)
            8,  8,  8,  8,  8];  % Z 坐标 (全在同一深度悬停)

% ================= 【3D 立体迷宫障碍物】 =================
Obstale_X = []; Obstale_Y = []; Obstale_Z = [];

% 障碍物 1：双子塔 (彻底封锁左右两侧，从水底到水面 Z=1~25)
for x = 12:15
    % 左塔
    for y = 2:10
        for z = 1:25, Obstale_X=[Obstale_X,x]; Obstale_Y=[Obstale_Y,y]; Obstale_Z=[Obstale_Z,z]; end
    end
    % 右塔
    for y = 18:23
        for z = 1:25, Obstale_X=[Obstale_X,x]; Obstale_Y=[Obstale_Y,y]; Obstale_Z=[Obstale_Z,z]; end
    end
end

% 障碍物 2：叹息之桥 (连接双子塔，封锁中间高度)
% 故意留出海底 (Z=1~7) 的“地下水路” 和天空 (Z=16~25) 的航道
for x = 12:15
    for y = 11:17
        for z = 8:15 
            Obstale_X=[Obstale_X,x]; Obstale_Y=[Obstale_Y,y]; Obstale_Z=[Obstale_Z,z];
        end
    end
end

% 障碍物 3：悬浮天盖 (在桥的正上方加一个悬空的屋顶)
% 强迫那些想从天空飞过去的 AUV 必须玩极限“贴板滑行”
for x = 10:17
    for y = 10:18
        for z = 19:21
            Obstale_X=[Obstale_X,x]; Obstale_Y=[Obstale_Y,y]; Obstale_Z=[Obstale_Z,z];
        end
    end
end

for x = 1:5
    for y = 10:18
        for z = 2:6 
            Obstale_X=[Obstale_X,x]; Obstale_Y=[Obstale_Y,y]; Obstale_Z=[Obstale_Z,z];
        end
        for z = 8:14 
            Obstale_X=[Obstale_X,x]; Obstale_Y=[Obstale_Y,y]; Obstale_Z=[Obstale_Z,z];
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
Step = 30000; 
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
xlabel('X 轴', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y 轴', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z 轴', 'FontSize', 12, 'FontWeight', 'bold');
title('AUV 三维空间避障与路径规划', 'FontSize', 14, 'FontWeight', 'bold');

% =======================================================
% 5. Figure 2 & 3: 能耗与航程平滑折线图
% =======================================================
AUV_num = size(Robot_1, 2);
legend_str = cell(1, AUV_num);
for k = 1:AUV_num, legend_str{k} = ['AUV ', num2str(k)]; end

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
title('3D AUV 累计能耗', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('迭代步数'); ylabel('能耗 (Energy)'); legend(legend_str, 'Location', 'northwest');

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
title('3D AUV 累计航程', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('迭代步数'); ylabel('航程 (m)'); legend(legend_str, 'Location', 'northwest');