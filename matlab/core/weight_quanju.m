function [weight_2_quanju_dt] = weight_quanju(Target_quanju, Obstale_quanju, TarNum, MAX_X, MAX_Y, MAX_Z)
    % 传入参数新增了 MAX_Z
    
    Et = 20;
    Eo = -20;
    gamma = 2;  % 保持较低的衰减率，防止大空间内梯度消失
    beta = 0.8; % 防止活性值爆炸
    
    % ==== 核心升维 1：生成 3D 空间的 26 个邻域方向 ====
    neighborhood = [];
    for dx = -1:1
        for dy = -1:1
            for dz = -1:1
                if dx==0 && dy==0 && dz==0, continue; end
                neighborhood = [neighborhood; dx, dy, dz];
            end
        end
    end
    
    % ==== 核心升维 2：初始化三维张量 ====
    activity_quanju = zeros(MAX_X, MAX_Y, MAX_Z);
    weight_1_quanju = zeros(MAX_X, MAX_Y, MAX_Z);

    % 障碍物赋值 (现在传入的 Obstale_quanju 必须是 3 行: [x; y; z])
    for ijk = 1:size(Obstale_quanju, 2)
        obs_x = Obstale_quanju(1, ijk);
        obs_y = Obstale_quanju(2, ijk);
        obs_z = Obstale_quanju(3, ijk);
        weight_1_quanju(obs_x, obs_y, obs_z) = -1;
        activity_quanju(obs_x, obs_y, obs_z) = -1;
    end

    % 目标点赋值
    for i = 1:TarNum
        tx = Target_quanju(1,i);
        ty = Target_quanju(2,i);
        tz = Target_quanju(3,i);
        activity_quanju(tx, ty, tz) = 1;
        weight_1_quanju(tx, ty, tz) = 1;
    end
    
    % 神经网络活性扩散主循环
    for j = 1:100 
        for x = 1:MAX_X
            for y = 1:MAX_Y
                for z = 1:MAX_Z  % 新增 Z 轴遍历
                    weight_cur = weight_1_quanju(x,y,z);
                    
                    for n = 1:26 % 遍历 26 个空间邻居
                        nei_x = x + neighborhood(n,1);
                        nei_y = y + neighborhood(n,2);
                        nei_z = z + neighborhood(n,3);
                        
                        % 3D 边界防吞噬与障碍物检查
                        if(nei_x<1 || nei_x>MAX_X || nei_y<1 || nei_y>MAX_Y || nei_z<1 || nei_z>MAX_Z)
                            weight_nei = weight_1_quanju(x,y,z); 
                        elseif(activity_quanju(nei_x,nei_y,nei_z) == -1)
                            continue; 
                        else
                            weight_nei = weight_1_quanju(nei_x,nei_y,nei_z);
                        end
                        
                        if(activity_quanju(x,y,z) == -1) 
                            weight_cur = 0;
                        end
                        
                        % 3D 距离平方 = dx^2 + dy^2 + dz^2
                        dist = neighborhood(n,1)^2 + neighborhood(n,2)^2 + neighborhood(n,3)^2;
                        Wij = exp(-gamma*dist);
                        weight_cur = weight_cur + Wij * weight_nei;
                    end
                    
                    tmp = weight_cur + (activity_quanju(x,y,z)==1)*Et + (activity_quanju(x,y,z)==-1)*Eo;  
                    
                    % 保持平原封印解除状态
                    if tmp > 0 
                        tmp = beta * tmp; 
                    else 
                        tmp = 0;
                    end
                    weight_1_quanju(x,y,z) = tmp;
                end
            end
        end
    end
    
    % ==== 核心修复：强制还原 3D 障碍物的 -1 标签 ====
    for ijk = 1:size(Obstale_quanju, 2)
        obs_x = Obstale_quanju(1, ijk);
        obs_y = Obstale_quanju(2, ijk);
        obs_z = Obstale_quanju(3, ijk);
        weight_1_quanju(obs_x, obs_y, obs_z) = -1;
    end

    weight_2_quanju_dt = weight_1_quanju;
end