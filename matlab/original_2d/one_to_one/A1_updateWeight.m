function weight_new = A1_updateWeight(weight,B,Neigh,dist,winnerT,winnerR_1_1, activity_quanju)
    weight_new = weight;
    i = size(weight,2);
    eta = 0.8;
    [MAX_X, MAX_Y, MAX_Z] = size(activity_quanju);
    
    for j = 1:i
        % 计算方向向量
        angle_x = winnerR_1_1(2,1)-weight(2, j);
        angle_y = winnerR_1_1(3,1)-weight(3, j);
        angle_z = winnerR_1_1(4,1)-weight(4, j);
        
        delta_x = B * Neigh(j) * angle_x;
        delta_y = B * Neigh(j) * angle_y;
        delta_z = B * Neigh(j) * angle_z;
        
        % 限制最大步长
        max_step = 1.0; 
        step_len = sqrt(delta_x^2 + delta_y^2 + delta_z^2);
        if step_len > max_step
            delta_x = (delta_x / step_len) * max_step;
            delta_y = (delta_y / step_len) * max_step;
            delta_z = (delta_z / step_len) * max_step;
        end
        
        target_x = weight(2, j) + delta_x;
        target_y = weight(3, j) + delta_y;
        target_z = weight(4, j) + delta_z;
        
        grid_x = max(1, min(MAX_X, round(target_x)));
        grid_y = max(1, min(MAX_Y, round(target_y)));
        grid_z = max(1, min(MAX_Z, round(target_z)));
        
        % 获取当前所在的安全网格坐标
        cx = max(1, min(MAX_X, round(weight(2, j))));
        cy = max(1, min(MAX_Y, round(weight(3, j))));
        cz = max(1, min(MAX_Z, round(weight(4, j))));

        if activity_quanju(grid_x, grid_y, grid_z) ~= -1
            % 如果斜向目标点完全安全，直接移动
            weight_new(2, j) = target_x;
            weight_new(3, j) = target_y;
            weight_new(4, j) = target_z;
        else
            % ==== 核心修复：3D 独立轴贴墙滑行逻辑 ====
            % 先假定全被堵死，保持原地
            weight_new(2, j) = weight(2, j);
            weight_new(3, j) = weight(3, j);
            weight_new(4, j) = weight(4, j);
            
            % 使用临时变量记录每一步滑动后的位置
            cx_temp = cx; 
            cy_temp = cy; 
            cz_temp = cz;
            
            % 1. 尝试仅在 X 轴滑动
            if activity_quanju(grid_x, cy_temp, cz_temp) ~= -1
                weight_new(2, j) = target_x;
                cx_temp = grid_x; % 更新参考态
            end
            
            % 2. 基于新的 X，尝试在 Y 轴滑动
            if activity_quanju(cx_temp, grid_y, cz_temp) ~= -1
                weight_new(3, j) = target_y;
                cy_temp = grid_y; % 更新参考态
            end
            
            % 3. 基于新的 X 和 Y，尝试在 Z 轴滑动
            if activity_quanju(cx_temp, cy_temp, grid_z) ~= -1
                weight_new(4, j) = target_z;
            end
            % ======================================
        end
        
        weight_new(1,j)=weight(1,j);
        weight_new(5,j)=weight(5,j); % 活性值现在在第 5 行
    end
    
    % 获胜节点的绝对靠拢
    if(dist <= eta)
        winnernum = winnerR_1_1(1, 1);
        weight_new(2:4,winnernum) = winnerT(1:3,1);
        weight_new(1,winnernum) = weight(1,winnernum);
        weight_new(5,winnernum) = weight(5,winnernum);
    end
end