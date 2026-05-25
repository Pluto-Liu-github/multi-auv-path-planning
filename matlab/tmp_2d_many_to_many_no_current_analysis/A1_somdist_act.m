function [dist_act,dist_total,energy_total] = A1_somdist_act(Weight_act_total,w,dist_act_1,energy_1,aerf,V_AUV,V_ocean)
    %{
        > Weight_act_total cell array // {w}(4×N) 每步节点坐标
        > w            scalar // 当前步序号
        > dist_act_1   scalar // 之前累计航程 
        > energy_1     scalar // 之前累计能耗 
        > aerf         (X×Y)  // 海流方向场 
        > V_AUV        scalar // AUV 对水最大航速 
        > V_ocean      scalar // 海流大小 
        < dist_act    scalar // 本段航程 
        < dist_total  scalar // 累计航程 
        < energy_total scalar // 累计能耗 
        能量中AUV速度是AUV推进器以额定转速输出后AUV速度，不是AUV绝对速度
    %}
    u_1 = Weight_act_total{w}(2:3,:);
    u_2 = Weight_act_total{w+1}(2:3,:);
    N = size(u_2, 2);
    energy = zeros(1, N);

    for t=1:N
        ocean_dir = aerf(round(u_1(1,t)),round(u_1(2,t)));
        dx = u_2(1,t) - u_1(1,t);
        dy = u_2(2,t) - u_1(2,t);
        disto = hypot(dx,dy);
        if(disto == 0)
            energy(t) = 0;
            continue
        end
        angle_ij_x = rad2deg(atan2(dy,dx));
        angle_ij_ocean = angle_ij_x-ocean_dir;
        V_ocean_sin = V_ocean * sind(angle_ij_ocean);
        V_ocean_cos = V_ocean * cosd(angle_ij_ocean);
        V_AUV_ij_do = V_AUV^2 - V_ocean_cos^2;
        V_AUV_ij_do = max(0,V_AUV_ij_do);
        V_AUV_ij = sqrt(V_AUV_ij_do);

        T = disto/(V_AUV_ij+V_ocean_cos);
        energy(t) = V_AUV^3*T;
    end
    dist_act = sum(hypot(u_2(1,:)-u_1(1,:), u_2(2,:)-u_1(2,:)));
    dist_total = dist_act+dist_act_1;
    energy_total = sum(energy)+energy_1;
end