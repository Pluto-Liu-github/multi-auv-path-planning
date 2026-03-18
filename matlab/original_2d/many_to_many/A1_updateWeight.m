function weight_new = A1_updateWeight(weight,B,Neigh,dist,winnerT,winnerR_1_1, activity_quanju)
    %{
        > weight       (4×N)  // 旧权值
        > B            scalar // 学习率
        > Neigh        (1×N)  // 邻域函数 h(j)
        > dist         scalar // 目标与获胜节点距离 (m)
        > winnerT      (2×1)  // 目标坐标 [x; y]
        > winnerR_1_1  (4×1)  // 获胜节点信息 [nodeID, x, y, act]
        > activity_quanju (X×Y) // 【新增参数】全局活性图，用于碰撞检测
        < weight_new  (4×N)  // 更新后权值    
    %}
    weight_new = weight;
    i = size(weight,2);
    eta = 0.8;
    
    [MAX_X, MAX_Y] = size(activity_quanju);
    
    for j = 1:i
        % 计算方向向量
        angle_x = winnerR_1_1(2,1)-weight(2, j);
        angle_y = winnerR_1_1(3,1)-weight(3, j);
        
        % 1. 计算理论上的预期位移
        delta_x = B * Neigh(j) * angle_x;
        delta_y = B * Neigh(j) * angle_y;
        
        target_x = weight(2, j) + delta_x;
        target_y = weight(3, j) + delta_y;
        
        % 将连续坐标四舍五入，用于在网格地图中查阅是否安全
        grid_x = max(1, min(MAX_X, round(target_x)));
        grid_y = max(1, min(MAX_Y, round(target_y)));
        

        if activity_quanju(grid_x, grid_y) ~= -1
            % 如果预期目标点不是障碍物，直接移动（斜着走）
            weight_new(2, j) = target_x;
            weight_new(3, j) = target_y;
        else
            
            % 尝试仅在 X 方向移动（Y 保持原地）
            test_x_grid = max(1, min(MAX_X, round(target_x)));
            test_y_grid = max(1, min(MAX_Y, round(weight(3, j))));
            can_slide_x = (activity_quanju(test_x_grid, test_y_grid) ~= -1);
            
            % 尝试仅在 Y 方向移动（X 保持原地）
            test_x_grid2 = max(1, min(MAX_X, round(weight(2, j))));
            test_y_grid2 = max(1, min(MAX_Y, round(target_y)));
            can_slide_y = (activity_quanju(test_x_grid2, test_y_grid2) ~= -1);
            
            if can_slide_x && ~can_slide_y
                weight_new(2, j) = target_x;
                weight_new(3, j) = weight(3, j);
            elseif can_slide_y && ~can_slide_x
                weight_new(2, j) = weight(2, j);
                weight_new(3, j) = target_y;
            elseif can_slide_x && can_slide_y
                % 如果两边都能滑，挑一个位移分量较大的方向滑过去
                if abs(delta_x) > abs(delta_y)
                    weight_new(2, j) = target_x;
                    weight_new(3, j) = weight(3, j);
                else
                    weight_new(2, j) = weight(2, j);
                    weight_new(3, j) = target_y;
                end
            else
                % 如果 X 和 Y 方向都被堵死了（进入死角），就老老实实呆在原地
                weight_new(2, j) = weight(2, j);
                weight_new(3, j) = weight(3, j);
            end
        end
        % =========================================================
        
        weight_new(1,j)=weight(1,j);
        weight_new(4,j)=weight(4,j);
    end
    
    % 获胜节点的绝对靠拢（当距离足够近时直接跳到目标）
    if(dist <= eta)
        winnernum = winnerR_1_1(1, 1);
        weight_new(2:3,winnernum) = winnerT(1:2,1);
        weight_new(1,winnernum) = weight(1,winnernum);
        weight_new(4,winnernum) = weight(4,winnernum);
    end
end