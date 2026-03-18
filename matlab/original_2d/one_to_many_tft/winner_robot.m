function huosheng_1 = winner_robot(Weight_vir, activity_dt, TargetNum, activity_quanju, close_count, robot_energy_total, gamma_penalty)
    %{
        > Weight_vir         (5×N)或(4×N) // 当前所有虚拟节点和实际机器人的状态
        > activity_dt        cell array   // 每个目标对应的 BNN 活性图
        > TargetNum          scalar       // 目标总数
        > activity_quanju    (X×Y)        // 全局活性图
        > close_count        array        // 已完成的目标序号
        > robot_energy_total (1×M)        // 每个 AUV 的累积能耗
        > gamma_penalty      scalar       // 能耗惩罚系数
        < huosheng_1         (5×T)        // 每个目标的获胜节点信息 [ID; x; y; 全局活性; 目标子活性/最终得分]
    %}

    % 初始化获胜者矩阵
    huosheng_1 = zeros(5, TargetNum);
    num_nodes = size(Weight_vir, 2);

    for ii = 1:TargetNum
        % 1. 过滤已完成的目标
        if ismember(ii, close_count)
            % 如果目标已被吃掉，用 0 占位，防止后续分配逻辑越界
            huosheng_1(:, ii) = [0; 0; 0; 0; 0];
            continue;
        end

        best_score = -inf;
        best_node = zeros(5, 1);

        % 2. 遍历所有可用的神经元（AUV 或其候选点）
        for k = 1:num_nodes
            robotID = Weight_vir(1, k);
            % 提取坐标并四舍五入到网格索引
            x = round(Weight_vir(2, k));
            y = round(Weight_vir(3, k));

            % 边界保护，防止数组索引越界报错
            x = max(1, min(size(activity_quanju, 1), x));
            y = max(1, min(size(activity_quanju, 2), y));

            % 提取该点在当前目标 ii 的 BNN 网络中的基础活性值
            base_act = activity_dt{ii}(x, y);

            % 获取当前节点的归属 AUV 累积能耗
            if robotID > 0 && robotID <= length(robot_energy_total)
                energy = robot_energy_total(robotID);
            else
                energy = 0; % 防止虚拟节点没有合法的 ID
            end

            % =========================================================
            % 3. 核心修复：引入能耗软惩罚 (Soft Penalty)
            % =========================================================
            if base_act > 0
                % 【软惩罚公式】：用除法代替减法
                % 即使 AUV 极度缺电，得分最多无限趋近于 0，而绝不会变成负数（变成障碍物）
                final_score = base_act / (1 + gamma_penalty * energy);
            else
                % 如果 base_act <= 0，说明这个点本身就是障碍物排斥区或死水区
                % 此时用减法让它变得更“不讨喜”是合理的
                final_score = base_act - gamma_penalty * energy;
            end

            % 4. 选出得分最高（最适合去抓取该目标）的节点
            if final_score > best_score
                best_score = final_score;
                global_act = activity_quanju(x, y);
                % 记录获胜节点：[归属AUV_ID; X坐标; Y坐标; 全局活性; 最终带惩罚的得分]
                best_node = [robotID; x; y; global_act; final_score]; 
            end
        end
        
        % 保存当前目标 ii 的获胜者
        huosheng_1(:, ii) = best_node;
    end
end