function [dist_act, dist_total, energy_total] = A1_somdist_act(Weight_act_total,w,dist_act_1,energy_1,V_AUV)
    u_1 = Weight_act_total{w}(2:4,:); % 提取 3D 坐标
    u_2 = Weight_act_total{w+1}(2:4,:);
    N = size(u_2, 2);
    energy = zeros(1, N);

    for t=1:N
        dx = u_2(1,t) - u_1(1,t);
        dy = u_2(2,t) - u_1(2,t);
        dz = u_2(3,t) - u_1(3,t);
        disto = sqrt(dx^2 + dy^2 + dz^2); % 3D 欧氏距离
        
        if(disto == 0)
            energy(t) = 0;
            continue
        end
        
        % 纯 3D 几何推进能耗估算
        T = disto / V_AUV;
        energy(t) = (V_AUV^3) * T;
    end
    
    dist_act = sum(sqrt(sum((u_2 - u_1).^2, 1)));
    dist_total = dist_act + dist_act_1;
    energy_total = sum(energy) + energy_1;
end