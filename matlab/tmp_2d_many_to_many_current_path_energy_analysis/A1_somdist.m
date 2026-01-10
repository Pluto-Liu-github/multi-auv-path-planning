function [distn,P_2_act,P_3_act] = A1_somdist(x,huosheng_1,ii,Weight_act)
	%{
        > x            (2×1)  // 目标坐标 [x; y]
        > huosheng_1   (5×T)  // 各目标获胜机器人 [robotID, x, y, globalAct, subAct]
        > ii           scalar // 当前目标索引
        > Weight_act   (4×N)  // 实际节点坐标 [nodeID, x, y, act]
        < distn       scalar // 欧氏距离 (m)
        < P_2_act     scalar // 获胜节点 x 坐标
        < P_3_act     scalar // 获胜节点 y 坐标
    %}
	P_1 = huosheng_1(1,ii);
	P_2_act = Weight_act(2,P_1);
	P_3_act = Weight_act(3,P_1);
	PN = [P_2_act;P_3_act];
	dist = x - PN;
	dist = sum(dist.^2);
	distn = dist.^0.5;
end