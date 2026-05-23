function [weight_2_dt] = weight_wij(Target_ij, Obstale_ij, TarNum, MAX_X, MAX_Y, MAX_Z, diffusionSteps)
    if nargin < 7
        diffusionSteps = 100;
    end
    Et = 20; 
    Eo = -20;
    gamma = 5;  
    beta = 0.8; 
    
    % ==== 核心升维 1：自动生成 3D 空间的 26 个邻域方向 ====
    neighborhood = [];
    for dx = -1:1
        for dy = -1:1
            for dz = -1:1
                if dx==0 && dy==0 && dz==0, continue; end
                neighborhood = [neighborhood; dx, dy, dz];
            end
        end
    end
    
    % ==== 核心升维 2：三维张量初始化 ====
    activity_ij = zeros(MAX_X, MAX_Y, MAX_Z);
    weight_1_ij = zeros(MAX_X, MAX_Y, MAX_Z);

    % 障碍物赋值 (假设 Obstale_ij 现在是 3行: [x; y; z])
    for ijk = 1:size(Obstale_ij, 2)
        obs_x = Obstale_ij(1, ijk);
        obs_y = Obstale_ij(2, ijk);
        obs_z = Obstale_ij(3, ijk);
        weight_1_ij(obs_x, obs_y, obs_z) = -1;
        activity_ij(obs_x, obs_y, obs_z) = -1;
    end

    for i = 1:TarNum
        tx = Target_ij(1,i); ty = Target_ij(2,i); tz = Target_ij(3,i);
        activity_ij(tx, ty, tz) = 1;
        weight_1_ij(tx, ty, tz) = 1;
        
        for j = 1:diffusionSteps % 迭代扩散
            for x = 1:MAX_X
                for y = 1:MAX_Y
                    for z = 1:MAX_Z  % <== 新增 Z 轴循环
                        weight_cur = weight_1_ij(x,y,z);
                        
                        for n = 1:26 % 遍历 26 个邻居
                            nei_x = x + neighborhood(n,1);
                            nei_y = y + neighborhood(n,2);
                            nei_z = z + neighborhood(n,3);
                            
                            % 越界及障碍物判断
                            if(nei_x<1 || nei_x>MAX_X || nei_y<1 || nei_y>MAX_Y || nei_z<1 || nei_z>MAX_Z)
                                weight_nei = weight_1_ij(x,y,z); % 边界反射
                            elseif(activity_ij(nei_x,nei_y,nei_z) == -1)
                                continue; 
                            else
                                weight_nei = weight_1_ij(nei_x,nei_y,nei_z);
                            end
                            
                            if(activity_ij(x,y,z) == -1), weight_cur = 0; end
                            
                            % 3D 距离平方 = dx^2 + dy^2 + dz^2
                            dist = neighborhood(n,1)^2 + neighborhood(n,2)^2 + neighborhood(n,3)^2;
                            Wij = exp(-gamma*dist);
                            weight_cur = weight_cur + Wij * weight_nei;
                        end
                        
                        tmp = weight_cur + (activity_ij(x,y,z)==1)*Et + (activity_ij(x,y,z)==-1)*Eo;
                        if tmp > 0, tmp = beta * tmp; else, tmp = 0; end
                        weight_1_ij(x,y,z) = tmp;
                    end
                end
            end
        end
        
        % 还原障碍物
        for ijk = 1:size(Obstale_ij,2)
            weight_1_ij(Obstale_ij(1,ijk), Obstale_ij(2,ijk), Obstale_ij(3,ijk)) = -1;
        end
        
        weight_2_dt{i} = weight_1_ij;
        weight_1_ij = zeros(MAX_X, MAX_Y, MAX_Z);
        activity_ij(tx, ty, tz) = 0;
    end
end