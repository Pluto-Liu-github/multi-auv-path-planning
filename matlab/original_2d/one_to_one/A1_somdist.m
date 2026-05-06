function [distn, P_2_act, P_3_act, P_4_act] = A1_somdist(x,huosheng_1,ii,Weight_act)
    P_1 = huosheng_1(1,ii);
    P_2_act = Weight_act(2,P_1);
    P_3_act = Weight_act(3,P_1);
    P_4_act = Weight_act(4,P_1); % 新增 Z 轴
    PN = [P_2_act; P_3_act; P_4_act];
    dist = x - PN; % x 此时是 3×1 的目标坐标 [x;y;z]
    distn = sqrt(sum(dist.^2));
end