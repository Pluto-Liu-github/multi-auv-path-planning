clear;
close all;
clc;
% =======================================================
% =======================================================
% 1. רչʾ 3D ˲Ծ
% =======================================================
MAX_X = 25; MAX_Y = 25; MAX_Z = 25; %  3D ߽

%{ 
================= 6Ŀ� =================
% ĻĿ�
% T1: ն()  T2,T3,T4,T5: ɢâ  T6: ռ�
Target_12 = [ 8, 20, 20, 20, 20, 24;   % X 
             13, 23,  3, 13, 13, 13;   % Y 
             13, 13, 13, 23,  3, 13];  % Z 

% ================= 5 AUV � =================
% AUV ܼʮ͡ųԵġͻ
Robot_1 = [ 2,  2,  2,  2,  2;   % X  (ͬһƽ)
           13, 15, 11, 13, 13;   % Y 
           13, 13, 13, 15, 11];  % Z  (ʮ)

% =================  3D ϰǼ�š =================
%  X=12 ĵطһºͼľ޴ǽ
%  (Y=8~18, Z=8~18) һ ()
Obstale_X = []; Obstale_Y = []; Obstale_Z = [];

for y = 1:25
    for z = 1:25
        % 봰ķΧڣʵ�
        if ~(y >= 8 && y <= 18 && z >= 8 && z <= 18) 
            Obstale_X = [Obstale_X, 12];
            Obstale_Y = [Obstale_Y, y];
            Obstale_Z = [Obstale_Z, z];
            
            % Ϊǽһ� X=11  X=13
            Obstale_X = [Obstale_X, 11, 13];
            Obstale_Y = [Obstale_Y, y, y];
            Obstale_Z = [Obstale_Z, z, z];
        end
    end
end
%}
% =======================================================
% 1. ռԣάռĿ + ޱԹ
% =======================================================
MAX_X = 25; MAX_Y = 25; MAX_Z = 25; %  3D ߽

% ================= 6Ŀ㣨ͼ棩 =================
% Ŀֲ˫ŽӷǺ͵ͲΧ
% չʾAUVڸάϰеЭͬ�
Target_12 = [22, 20,  4, 24, 17,  3;   % X 
             22,  4, 22, 20, 15, 24;   % Y 
              4, 22, 20,  2, 10,  3];  % Z 

% ================= 5 AUV 㣨ͼ棩 =================
% AUVһſûͼеֲ
Robot_1 = [ 2,  4,  6,  8, 10;   % X 
            2,  2,  2,  2,  2;   % Y 
            8,  8,  8,  8,  8];  % Z 

% ================= 3D Թϰͼ棩 =================
Obstale_X = []; Obstale_Y = []; Obstale_Z = [];

% ϰ1˫γ
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

% ϰ2Žӷпռ�
for x = 12:15
    for y = 11:17
        for z = 8:15
            Obstale_X = [Obstale_X, x]; Obstale_Y = [Obstale_Y, y]; Obstale_Z = [Obstale_Z, z];
        end
    end
end

% ϰ3ǣϲֱӴԽ
for x = 10:17
    for y = 10:18
        for z = 19:21
            Obstale_X = [Obstale_X, x]; Obstale_Y = [Obstale_Y, y]; Obstale_Z = [Obstale_Z, z];
        end
    end
end

% ϰ4ͲڸֲԼ
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

Obstale = unique([Obstale_X; Obstale_Y; Obstale_Z]', 'rows')';TargetNum = size(Target_12,2); 
Target_NO = 1:TargetNum;
Target = [Target_12; Target_NO]; % [X; Y; Z; ]

% =======================================================
% 2. ܺĳʼ
% =======================================================
V_AUV = 2.0; % AUV 

robot_energy_total = zeros(1, size(Robot_1,2)); 
robot_dist_total = zeros(1, size(Robot_1,2)); 
gamma_penalty = 1e-4; 
diffusion_steps = 30;

%  3D ȫֲͼ ( MAX_Z)
[activityvalue__2_dt] = weight_wij(Target(1:3,:), Obstale, TargetNum, MAX_X, MAX_Y, MAX_Z, diffusion_steps);
[activityvalue__2_quanju_dt] = weight_quanju(Target(1:3,:), Obstale, TargetNum, MAX_X, MAX_Y, MAX_Z, diffusion_steps);

dist_act_1 = zeros(1, size(Robot_1,2));
% ˳ʼӳ
[Weight_vir{1}, Weight_act{1}, Robot] = robot_no_activity(Robot_1, activityvalue__2_quanju_dt, dist_act_1);
Weight_act_total{1} = Weight_act{1};

% =======================================================
% 3. 3D Ѱ·ѭ
% =======================================================
Step = 30000; 
B = 1; w = 1;
open_count = []; close_count = []; 
COO = double(1); 
shixian_ji = zeros(5,1); % [; ID; X; Y; Z]

% ʼʷ¼ʼ״̬ 0
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
        
        %  3D ŷϾ
        [distn, P_2_act, P_3_act, P_4_act] = A1_somdist(Target(1:3,ii), huosheng_1, ii, Weight_act{w});
        
        if(distn <= 3.5) % ά²3.5դݲ�жΪĿ
            close_count = [close_count, ii]; 
            shixian = [ii; huosheng_1(1,ii); P_2_act; P_3_act; P_4_act];
            if(ismember(ii, shixian_ji(1,:))) == 1
                continue;
            else
                shixian_ji = [shixian_ji, shixian];

            end
        end
        
        % ߼
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
        
        %  3D Ѱ·״ ()
        Pn_point = Pn(huosheng_1, activityvalue__2_dt{ii}, ii, MAX_X, MAX_Y, MAX_Z); 
        
        winnerR = huosheng_1(1:5, ii); 
        wiiner_R_act = Weight_act{w}(1:5, winnerR(1,1));
        
        % Ȩֵ (3D ǽ)
        Neigh{w} = A1_NeighborFunction(Weight_act{w}, huosheng_1(1:5,ii), t, huosheng_1, wiiner_R_act, close_count);
        Weight_act{w+1} = A1_updateWeight(Weight_act{w}, B, Neigh{w}, distn, Target(1:3,ii), Pn_point, activityvalue__2_quanju_dt);
        
        winnernum = winnerR(1,1);
        u_before = Weight_act{w}(1:5, winnernum);
        u_after = Weight_act{w+1}(1:5, winnernum);
        temp_weight_total = {u_before, u_after}; 
        
        %  3D ܺ뺽�
        [step_dist, ~, step_energy] = A1_somdist_act(temp_weight_total, 1, 0, 0, V_AUV);
        
        robot_energy_total(winnernum) = robot_energy_total(winnernum) + step_energy;
        robot_dist_total(winnernum) = robot_dist_total(winnernum) + step_dist;

        w = w + 1;
        Pn_empty = [Pn_empty, Pn_point];
    end
    Weight_act_total{t+1} = Weight_act{w}; 
    
    % ѡ߸
    Pn_next = next_houxuan(Weight_act{w}, huosheng_1, activityvalue__2_quanju_dt, Pn_empty);
    Weight_vir{t+1} = Pn_next;
    huosheng{t} = huosheng_1;
    
    % ׷ʷ¼
    energy_history = [energy_history; robot_energy_total];
    dist_history = [dist_history; robot_dist_total];
    
    if ~isempty(close_count) && length(unique(close_count(1,:))) == TargetNum
        break;
    end
end

% =======================================================
% 4. Figure 1: 3D 켣ͼ�
% =======================================================
figure(1);
set(gcf, 'Position', [50, 100, 700, 600]); 
colors = ['r', 'b', 'g', 'm', 'c']; 

for PPB3 = 1:size(Robot_1,2)
    for PPA3 = 1:size(Weight_act,2)
        Weight_act_16{PPB3}(1:5, PPA3) = Weight_act{PPA3}(1:5, PPB3);
    end
    Weight_act_16{PPB3} = unique(Weight_act_16{PPB3}','rows','stable')';
    
    % ȡ X, Y, Z (ֱӦ 2, 3, 4 )
    path_x = Weight_act_16{PPB3}(2,:);
    path_y = Weight_act_16{PPB3}(3,:);
    path_z = Weight_act_16{PPB3}(4,:);
    
    % ʹֵƽ 3D 
    if length(path_x) > 2
        t_raw = 1:length(path_x);
        t_smooth = linspace(1, length(path_x), 200);
        smooth_x = spline(t_raw, path_x, t_smooth);
        smooth_y = spline(t_raw, path_y, t_smooth);
        smooth_z = spline(t_raw, path_z, t_smooth);
        plot3(smooth_x, smooth_y, smooth_z, '-', 'Color', colors(PPB3), 'LineWidth', 2.5); hold on;
        plot3(path_x, path_y, path_z, '.', 'Color', colors(PPB3), 'MarkerSize', 8); % 
    else
        plot3(path_x, path_y, path_z, '.-', 'Color', colors(PPB3), 'LineWidth', 2.5, 'MarkerSize', 10); hold on;
    end
end

% 㡢յ�
plot3(Robot_1(1,:), Robot_1(2,:), Robot_1(3,:), '.r', 'MarkerSize', 30); % AUV 
text(Robot_1(1,:)-1, Robot_1(2,:)+1, Robot_1(3,:), 'AUV', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');

plot3(Target_12(1,:), Target_12(2,:), Target_12(3,:), 'pg', 'MarkerFaceColor', 'g', 'MarkerSize', 15); % Ŀ
for i = 1:TargetNum
    text(Target_12(1,i)-1, Target_12(2,i)+1, Target_12(3,i)+1, ['T', num2str(i)], 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
end

% ư͸ 3D ϰ�
scatter3(Obstale(1,:), Obstale(2,:), Obstale(3,:), 25, 'k', 'filled', 'MarkerFaceAlpha', 0.1); 

axis([0 MAX_X 0 MAX_Y 0 MAX_Z]);
grid on;
view(3); % ΪάĬӽ
% view(-30, 30); % ҲԽעУһԶĸ+ӽǶ
xlabel('X / դ', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y / դ', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z / դ', 'FontSize', 12, 'FontWeight', 'bold');
title('AUVάռ·�', 'FontSize', 14, 'FontWeight', 'bold');
legend({'AUV·', '켣�', 'AUV', 'Ŀ', 'ϰ'}, 'Location', 'northeastoutside');

% =======================================================
% 5. Figure 2 & 3: ܺ뺽ƽ�
% =======================================================
AUV_num = size(Robot_1, 2);
legend_str = cell(1, AUV_num);
for k = 1:AUV_num, legend_str{k} = ['AUV ', num2str(k), '']; end

% --- Figure 2: ܺͼ ---
figure(2); set(gcf, 'Position', [750, 400, 500, 300]); 
for k = 1:AUV_num
    x_raw = 0:(size(energy_history, 1)-1);
    y_raw = energy_history(:, k)';
    x_smooth = linspace(0, max(x_raw), 300); 
    y_smooth = spline(x_raw, y_raw, x_smooth); 
    plot(x_smooth, y_smooth, 'LineWidth', 2.5, 'Color', colors(mod(k-1, 5)+1)); hold on;
end
grid on; xlim([0, max(x_raw)]); ylim([0, inf]);
title('άAUVۼܺı�', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(' / '); ylabel('ۼܺ / Եλ'); legend(legend_str, 'Location', 'northwest');

% --- Figure 3: ͼ ---
figure(3); set(gcf, 'Position', [750, 50, 500, 300]); 
for k = 1:AUV_num
    x_raw = 0:(size(dist_history, 1)-1);
    y_raw = dist_history(:, k)';
    x_smooth = linspace(0, max(x_raw), 300); 
    y_smooth = spline(x_raw, y_raw, x_smooth); 
    plot(x_smooth, y_smooth, 'LineWidth', 2.5, 'Color', colors(mod(k-1, 5)+1)); hold on;
end
grid on; xlim([0, max(x_raw)]); ylim([0, inf]);
title('άAUVۼƺ̱�', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(' / '); ylabel('ۼƺ / m'); legend(legend_str, 'Location', 'northwest');
% =======================================================
% 6. ͬ㷨Ա�A* ˹Ƴ
% =======================================================
baseline_task_pairs = gbsom_task_pairs_from_reached(shixian_ji, Robot_1, Target_12);
baseline_comparison = run_baseline_comparison(Robot_1, Target_12, Obstale, MAX_X, MAX_Y, MAX_Z, V_AUV, baseline_task_pairs);

gbsom_metrics = struct( ...
    'method', 'GBSOM', ...
    'successCount', length(unique(close_count)), ...
    'taskCount', TargetNum, ...
    'successRate', length(unique(close_count)) / TargetNum, ...
    'totalPathLength', sum(robot_dist_total), ...
    'averagePathLength', sum(robot_dist_total) / max(1, TargetNum), ...
    'totalSearchSteps', t, ...
    'equivalentEnergy', sum(robot_energy_total));

method_names = {gbsom_metrics.method; baseline_comparison.A_.method; baseline_comparison.APF.method};
success_rates = [gbsom_metrics.successRate; baseline_comparison.A_.successRate; baseline_comparison.APF.successRate];
total_lengths = [gbsom_metrics.totalPathLength; baseline_comparison.A_.totalPathLength; baseline_comparison.APF.totalPathLength];
average_lengths = [gbsom_metrics.averagePathLength; baseline_comparison.A_.averagePathLength; baseline_comparison.APF.averagePathLength];
search_steps = [gbsom_metrics.totalSearchSteps; baseline_comparison.A_.totalSearchSteps; baseline_comparison.APF.totalSearchSteps];
equivalent_energy = [gbsom_metrics.equivalentEnergy; baseline_comparison.A_.equivalentEnergy; baseline_comparison.APF.equivalentEnergy];

comparison_table = table(method_names, success_rates, total_lengths, average_lengths, search_steps, equivalent_energy, ...
    'VariableNames', {'Method', 'SuccessRate', 'TotalPathLength', 'AveragePathLength', 'SearchSteps', 'EquivalentEnergy'});

disp('ͬά·�㷨Ա�');
disp(comparison_table);

figure_output_dir = fullfile(pwd, 'paper-output', 'figures');
writetable(comparison_table, fullfile(figure_output_dir, 'fig5-comparison-metrics.csv'));
task_pair_table = array2table(baseline_task_pairs, 'VariableNames', {'AUVIndex', 'TargetIndex'});
writetable(task_pair_table, fullfile(figure_output_dir, 'fig5-task-pairs.csv'));
save_comparison_figures(Robot_1, Target_12, Obstale, MAX_X, MAX_Y, MAX_Z, Weight_act_16, baseline_comparison, comparison_table, figure_output_dir);
disp(['Աȷͼѱ', figure_output_dir]);
chapter4_output_dir = fullfile(pwd, 'chapter4_figures');
save_chapter4_figures(Robot_1, Target_12, Obstale, MAX_X, MAX_Y, MAX_Z, Weight_act_16, energy_history, dist_history, chapter4_output_dir);
paper_chapter4_output_dir = fullfile(pwd, 'paper-output', 'figures');
save_chapter4_figures(Robot_1, Target_12, Obstale, MAX_X, MAX_Y, MAX_Z, Weight_act_16, energy_history, dist_history, paper_chapter4_output_dir);
disp(['Chapter 4 no-current figures saved to: ', chapter4_output_dir]);

function taskPairs = gbsom_task_pairs_from_reached(reachedTargets, robotStarts, targets)
    targetCount = size(targets, 2);
    taskPairs = zeros(0, 2);
    usedTargets = [];

    if size(reachedTargets, 2) > 1
        reachedTargets = reachedTargets(:, 2:end);
        for idx = 1:size(reachedTargets, 2)
            targetIdx = round(reachedTargets(1, idx));
            robotIdx = round(reachedTargets(2, idx));
            if targetIdx < 1 || targetIdx > targetCount || ...
                    robotIdx < 1 || robotIdx > size(robotStarts, 2) || ...
                    ismember(targetIdx, usedTargets)
                continue;
            end
            taskPairs = [taskPairs; robotIdx, targetIdx];
            usedTargets = [usedTargets, targetIdx];
        end
    end

    for targetIdx = 1:targetCount
        if ismember(targetIdx, usedTargets)
            continue;
        end
        distances = sqrt(sum((robotStarts - targets(:, targetIdx)).^2, 1));
        [~, robotIdx] = min(distances);
        taskPairs = [taskPairs; robotIdx, targetIdx];
    end
end
