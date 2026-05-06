function huosheng_1 = winner_robot(Weight_vir, activity_dt, TargetNum, activity_quanju, close_count, robot_energy_total, gamma_penalty)
    huosheng_1 = zeros(6, TargetNum); % 3D 需要 6 行来存储信息
    num_nodes = size(Weight_vir, 2);

    for ii = 1:TargetNum
        if ismember(ii, close_count)
            continue;
        end
        best_score = -inf;
        best_node = zeros(6, 1);
        for k = 1:num_nodes
            robotID = Weight_vir(1, k);
            x = round(Weight_vir(2, k));
            y = round(Weight_vir(3, k));
            z = round(Weight_vir(4, k)); % 提取 Z 坐标

            x = max(1, min(size(activity_quanju, 1), x));
            y = max(1, min(size(activity_quanju, 2), y));
            z = max(1, min(size(activity_quanju, 3), z));

            base_act = activity_dt{ii}(x, y, z);
            energy = 0;
            if robotID > 0 && robotID <= length(robot_energy_total), energy = robot_energy_total(robotID); end

            if base_act > 0
                final_score = base_act / (1 + gamma_penalty * energy);
            else
                final_score = base_act - gamma_penalty * energy;
            end

            if final_score > best_score
                best_score = final_score;
                global_act = activity_quanju(x, y, z);
                best_node = [robotID; x; y; z; global_act; final_score]; 
            end
        end
        huosheng_1(:, ii) = best_node;
    end
end