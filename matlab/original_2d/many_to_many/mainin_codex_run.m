clear;
close all;
clc;

MAX_X=25;MAX_Y=25;%地图尺寸
%{
Target_12 = [11; 11];
Robot_1 = [31; 11];
Obstale = [21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21; 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 ];
%}
Target_12=[22,23,14,9,15,22; 4,6,8,18,18,20];
Robot_1 = [18,4,9,18,21; 10,11,14,15,15];
Obstale = [16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,14,13,12,11,11,11,11,11,11,10,9,8,7,7,8,9,10,11,11,11,11,11,12,13,14,20,20,20,20,20,20,21,22,23,24,21,22,23,24;6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,6,6,6,6,7,8,9,10,11,11,11,11,11,16,16,16,16,16,17,18,19,20,20,20,20,6,7,8,18,19,20,8,8,8,8,18,18,18,18];

% ================= 1. 洋流模型与能耗参数初始化 =================
V_AUV = 2.0;   % AUV自身对水最大航速 (m/s)
V_ocean = 2; % 洋流基础流速 (m/s)
aerf = zeros(MAX_X, MAX_Y); % 洋流方向矩阵 (度)

center_x = MAX_Y / 2; % 涡旋中心 X
center_y = MAX_X / 2; % 涡旋中心 Y

for y = 1:MAX_X  
    for x = 1:MAX_Y 
        vec_x = x - center_x;
        vec_y = y - center_y;
        tangent_x = -vec_y;
        tangent_y = vec_x;
        aerf(y, x) = rad2deg(atan2(tangent_y, tangent_x));
    end
end

robot_energy_total = zeros(1, size(Robot_1,2)); % 记录5个AUV的累积能耗
robot_dist_total = zeros(1, size(Robot_1,2));
gamma_penalty = 1e-4; % 能耗衰减权重
% =========================================================
plot(Robot_1(2,:),Robot_1(1,:),'.','MarkerSize',25,'MarkerFaceColor','r','MarkerEdgeColor','r');
str_2 = {'AUV_1','AUV_2','AUV_3','AUV_4','AUV_5'};
text(Robot_1(2,:)-1,Robot_1(1,:)+1,str_2,'Color','r','FontSize',14,'Fontname','Times New Roman','FontWeight','bold');
hold on
str_2 = {'Target_1','Target_2','Target_3','Target_4','Target_5','Target_6'};
plot(Target_12(2,:),Target_12(1,:),'pb','MarkerSize',12,'MarkerFaceColor','g','MarkerEdgeColor','b');
hold on
text(Target_12(2,:)-1,Target_12(1,:)+1,str_2,'Color','b','FontSize',14,'Fontname','Times New Roman','FontWeight','bold');
hold on
plot(Obstale(2,:),Obstale(1,:),'s','MarkerSize',10,'MarkerFaceColor','k','MarkerEdgeColor','k');
hold on;
% ================= 2. 绘制洋流场可视化 =================
% 创建网格用于画图
[plot_X, plot_Y] = meshgrid(1:MAX_Y, 1:MAX_X);
U_ocean_plot = zeros(size(plot_X));
V_ocean_plot = zeros(size(plot_Y));

for y = 1:MAX_X
    for x = 1:MAX_Y
        angle = aerf(y, x);
        U_ocean_plot(y, x) = V_ocean * cosd(angle); % X 方向分量
        V_ocean_plot(y, x) = V_ocean * sind(angle); % Y 方向分量
    end
end


% 每隔 2 个网格点画一个浅蓝色的洋流箭头，避免画面太乱
quiver(plot_X(1:2:end, 1:2:end), plot_Y(1:2:end, 1:2:end), ...
       U_ocean_plot(1:2:end, 1:2:end), V_ocean_plot(1:2:end, 1:2:end), ...
       0.6, 'Color', [0.3 0.7 1.0], 'LineWidth', 1.0, 'MaxHeadSize', 1.5);
% =======================================================
hold off
axis([0 MAX_X 0 MAX_Y]);
set(gca,'xtick',1:5:MAX_X,'ytick',1:5:MAX_Y,'FontSize',14,'Fontname','Times New Roman','FontWeight','bold');
grid off
xlabel('X坐标 / 栅格','FontSize',18,'Fontname','Times New Roman','FontWeight','bold');
ylabel('Y坐标 / 栅格','FontSize',18,'Fontname','Times New Roman','FontWeight','bold');
title('二维多AUV多目标路径规划初始场景', 'FontSize', 16, 'Fontname', 'Microsoft YaHei UI', 'FontWeight', 'bold');
legend({'AUV起点', '目标点', '障碍物', '洋流方向'}, 'Location', 'northeastoutside');

TargetNum = size(Target_12,2); 
Target_NO = 1:size(Target_12,2);
Target = [Target_12;Target_NO];

Step = 80; %迭代次数
B = 1; %步长
r = 1.44; %邻域
w = 1;
open_count = []; %%%%%将多个被同一个机器人拥有的目标放入
close_count = []; %%%%%将多个被同一个机器人拥有并且已经实现的目标放入，一旦放入，该目标不再循环
COO = double(1);
shixian_ji = zeros(4,1);
Pn_empty = [];
huosheng = {};
dist_act = zeros(1,size(Robot_1,2));

[activityvalue__2_dt] = weight_wij(Target,Obstale,TargetNum,MAX_X,MAX_Y);
[activityvalue__2_quanju_dt] = weight_quanju(Target,Obstale,TargetNum,MAX_X,MAX_Y); %%%%%%%%%%全局神经元活性赋值
[Weight_vir{1},Weight_act{1},Robot] = robot_no_activity(Robot_1,activityvalue__2_quanju_dt,dist_act);
Weight_act_total{1}=Weight_act{1};
%%   画出目标点、AUV、障碍物的地图
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%主循环%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
energy_history = robot_energy_total;
dist_history = robot_dist_total;
for t = 1:Step
    huosheng_1 = winner_robot(Weight_vir{t},activityvalue__2_dt,TargetNum,activityvalue__2_quanju_dt,close_count, robot_energy_total, gamma_penalty);
    [open_count,MH] = MUL_AUV(open_count,huosheng_1,close_count);
    %open_count哪几个目标同时被一个机器人分配
    %MH拥有多个目标的机器人标号及其拥有的目标编号，及其在分配目标网络中活性
    
    for ii = 1:TargetNum
        if ismember(ii, close_count)
            continue;
        end
        [distn,P_2_act,P_3_act] = A1_somdist(Target(1:2,ii),huosheng_1,ii,Weight_act{w});
        %fprintf('[t=%d] Target%d  distn=%.4f  \n ',t,ii,distn);
        if(distn <= 0.1)%如果机器人与目标位置小于0.1，则机器人权重不更新，机器人不动
            close_count = [close_count,ii]; %已实现的目标加入，从此不再循环
            shixian = [ii; huosheng_1(1,ii); P_2_act; P_3_act];%已实现的目标的序号及其位置
            if(ismember(ii,shixian_ji(1,:))) == 1% 已实现的目标加入，从此不再循环
                continue;
            else
                shixian_ji = [shixian_ji,shixian];
            end
        end
        LO = ismember(ii,open_count);
        if LO == 1 % 如果当前目标ii属于曾经或现在被同一个机器人拥有的目标
            COO = []; % 【修复1】：每次进入前彻底清空 COO，防止残留上一轮数据
            target_jj = 0; % 记录找到的 MH 索引
            
            % 【修复2】：先确认 MH 确实是元胞数组，防止使用大括号报错
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
            
            % 【修复3】：只有安全找到了优先级，且优先级不是第一 (COO > 1) 时才做阻挡判断
            if ~isempty(COO) && COO > 1
                precoo = ismember(MH{target_jj}(2, COO-1), close_count);
                if(~precoo)
                    continue; % 优先级排在前面的目标还没完成，因此当前目标暂时挂机，不更新位置
                end
            end            
            if(COO > 1)%跳出上一个循环后，如果当前目标ii在机器人的目标优先级中不是最优，则跳过当前目标
                precoo = ismember(MH{jj}(2,COO-1),close_count);
                if(~precoo)
                    continue;
                end
            end%%跳出上一个循环后，如果当前目标ii在机器人的目标优先是最优或者目标与AUV一一对应，则跳到85行
        end
        activityvalue__3 = activityvalue__2_dt{ii};
        Pn_point = Pn(huosheng_1,activityvalue__2_dt{ii},ii,MAX_X,MAX_Y,aerf); %获胜机器人的下一时刻虚拟坐标位置
        %计算Pn与所对应目标的距离。%%%%%%%%%可能需要修改
        fprintf('t=%d, Target%d Pn=[%.2f,%.2f] score=%.4f\n',t,ii,Pn_point(2),Pn_point(3),Pn_point(4));
        winnerR = huosheng_1(1:4,ii); %获胜机器人的坐标位置
        wiiner_R_act = Weight_act{w}(1:4,winnerR(1,1));
        % 获胜神经元邻域更新
        Neigh{w} = A1_NeighborFunction(Weight_act{w},huosheng_1(1:4,ii),t,huosheng_1,wiiner_R_act,close_count); %%%%%可能需要更改
        Weight_act{w+1} = A1_updateWeight(Weight_act{w},B,Neigh{w},distn,Target(1:2,ii),Pn_point, activityvalue__2_quanju_dt);
        winnernum = winnerR(1,1);
        % 提取该获胜机器人在移动前后的坐标，组装成 A1_somdist_act 需要的格式
        u_before = Weight_act{w}(1:4, winnernum);
        u_after = Weight_act{w+1}(1:4, winnernum);
        temp_weight_total = {u_before, u_after}; 
        
        % 调用你写的能耗评估函数
        [step_dist, ~, step_energy] = A1_somdist_act(temp_weight_total, 1, 0, 0, aerf, V_AUV, V_ocean);
        
        % 累加该机器人的总能耗
        robot_energy_total(winnernum) = robot_energy_total(winnernum) + step_energy;
        robot_dist_total(winnernum) = robot_dist_total(winnernum) + step_dist;
% fprintf('--- AUV %d 累积耗电: %.2f ---\n', winnernum, robot_energy_total(winnernum));
% fprintf('t=%d, AUV%d  (%.3f,%.3f) --> (%.3f,%.3f)\n', ...
%         t, winnerR(1,1), ...
%         Weight_act{w}(2,winnerR(1,1)), Weight_act{w}(3,winnerR(1,1)), ...
%         Weight_act{w+1}(2,winnerR(1,1)), Weight_act{w+1}(3,winnerR(1,1)));
        w = w + 1;
        Pn_empty = [Pn_empty,Pn_point];
    end
    Weight_act_total{t+1}=Weight_act{w}; %所有机器人本轮循环最新位置
    %% 之前都是只获取获胜者信息，以下是获取失败者信息，并将其加入到Weight_vir
    Pn_next = next_houxuan(Weight_act{w},huosheng_1,activityvalue__2_quanju_dt,Pn_empty);
    Weight_vir{t+1} = Pn_next; %%%虚拟AUV，学习效率为1.0
    huosheng{t} = huosheng_1;
    energy_history = [energy_history; robot_energy_total];
    dist_history = [dist_history; robot_dist_total];
    if ~isempty(close_count) && length(unique(close_count(1,:))) == TargetNum
        break;
    end
end
for PPA3 = 1:size(Weight_act,2) %提取机器人走过的路径
    for PPB3 = 1:size(Robot_1,2)
        Weight_act_16{PPB3}(1:4,PPA3) = Weight_act{PPA3}(1:4,PPB3);
    end
end
for PPB3 = 1:size(Robot_1,2)
    Weight_act_16{PPB3} = unique(Weight_act_16{PPB3}','rows','stable');
    Weight_act_16{PPB3} = Weight_act_16{PPB3}';
    plot( Weight_act_16{PPB3}(3,:), Weight_act_16{PPB3}(2,:),'.-','Color','b','MarkerSize',8,'MarkerFaceColor','r','MarkerEdgeColor','r');
    hold on
    
end
plot(Robot_1(2,:),Robot_1(1,:),'.','MarkerSize',25,'MarkerFaceColor','r','MarkerEdgeColor','r');
str_2 = {'AUV_1','AUV_2','AUV_3','AUV_4','AUV_5'};
text(Robot_1(2,:)-1,Robot_1(1,:)+1,str_2,'Color','r','FontSize',14,'Fontname','Times New Roman','FontWeight','bold');
hold on
str_2 = {'Target_1','Target_2','Target_3','Target_4','Target_5','Target_6'};
plot(Target_12(2,:),Target_12(1,:),'pb','MarkerSize',12,'MarkerFaceColor','g','MarkerEdgeColor','b');
hold on
text(Target_12(2,:)-1,Target_12(1,:)+1,str_2,'Color','b','FontSize',14,'Fontname','Times New Roman','FontWeight','bold');
hold on
plot(Obstale(2,:),Obstale(1,:),'s','MarkerSize',10,'MarkerFaceColor','k','MarkerEdgeColor','k');
hold on; % 确保障碍物画完后不关闭画布

% ================= 2. 绘制洋流场可视化 (在最终图上重新绘制) =================
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

% 每隔 2 个网格点画一个浅蓝色的洋流箭头
quiver(plot_X(1:2:end, 1:2:end), plot_Y(1:2:end, 1:2:end), ...
       U_ocean_plot(1:2:end, 1:2:end), V_ocean_plot(1:2:end, 1:2:end), ...
       0.6, 'Color', [0.3 0.7 1.0], 'LineWidth', 1.0, 'MaxHeadSize', 1.5);
% =======================================================================
axis([0 MAX_X 0 MAX_Y]);
set(gca,'xtick',1:MAX_X,'ytick',1:MAX_Y,'FontSize',14,'Fontname','Times New Roman','FontWeight','bold');
grid on
xlabel('X坐标 / 栅格','FontSize',18,'Fontname','Times New Roman','FontWeight','bold');
ylabel('Y坐标 / 栅格','FontSize',18,'Fontname','Times New Roman','FontWeight','bold');
title('二维多AUV多目标路径规划结果', 'FontSize', 16, 'Fontname', 'Microsoft YaHei UI', 'FontWeight', 'bold');
legend({'AUV路径', 'AUV起点', '目标点', '障碍物', '洋流方向'}, 'Location', 'northeastoutside');
% ================= 3. 绘制能耗累计图表 =================
figure(2); % 开启一个新的图表窗口，防止覆盖原本的轨迹图
set(gcf, 'Position', [100, 100, 700, 500]); % 设置窗口大小

% 定义几种不同的颜色，以防你有多个 AUV
colors = ['r', 'b', 'g', 'm', 'c']; 
AUV_num = size(Robot_1, 2);

% 遍历所有的 AUV 画线
for k = 1:AUV_num
    % 横坐标是迭代步数，纵坐标是对应的能耗记录
    plot(0:(size(energy_history, 1)-1), energy_history(:, k), ...
         'LineWidth', 2.5, 'Color', colors(mod(k-1, 5)+1));
     s111tr1=num2str(energy_history(size(energy_history, 1), k)');text(size(energy_history, 1),energy_history(size(energy_history, 1), k),s111tr1);
    hold on;
end

grid on;
title('动态洋流环境下AUV累计能耗变化曲线', 'FontSize', 16, 'Fontname', 'Microsoft YaHei UI', 'FontWeight', 'bold');
xlabel('时间 / s', 'FontSize', 14, 'Fontname', 'Microsoft YaHei UI', 'FontWeight', 'bold');
ylabel('累计能耗 / 相对单位', 'FontSize', 14, 'Fontname', 'Microsoft YaHei UI', 'FontWeight', 'bold');

% 动态生成图例
legend_str = cell(1, AUV_num);
for k = 1:AUV_num
    legend_str{k} = ['AUV ', num2str(k), '号'];
end
legend(legend_str, 'Location', 'northwest', 'FontSize', 12);
set(gca, 'FontSize', 12, 'Fontname', 'Times New Roman', 'FontWeight', 'bold');
% =======================================================
% % ================= 4. 绘制航程累计图表 =================
% figure(3); % 开启第三个图表窗口
% % 把窗口位置稍微往右挪一点，防止挡住前两个图
% set(gcf, 'Position', [850, 100, 700, 500]); 
% 
% for k = 1:AUV_num
%     % 横坐标是迭代步数，纵坐标是对应的航程记录
%     plot(0:(size(dist_history, 1)-1), dist_history(:, k), ...
%          'LineWidth', 2.5, 'Color', colors(mod(k-1, 5)+1));
%     hold on;
% end
% 
% grid on;
% title('AUV 累计航程变化图', 'FontSize', 16, 'Fontname', 'Times New Roman', 'FontWeight', 'bold');
% xlabel('时间 / s', 'FontSize', 14, 'Fontname', 'Microsoft YaHei UI', 'FontWeight', 'bold');
% ylabel('累计航程 / m', 'FontSize', 14, 'Fontname', 'Times New Roman', 'FontWeight', 'bold');
% 
% legend(legend_str, 'Location', 'northwest', 'FontSize', 12);
% set(gca, 'FontSize', 12, 'Fontname', 'Times New Roman', 'FontWeight', 'bold');
% % =======================================================