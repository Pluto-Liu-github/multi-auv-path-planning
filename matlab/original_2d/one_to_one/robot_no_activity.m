function [Weight_vir, Weight_act, Robot] = robot_no_activity(Robot_1, activityvalue__2_quanju, dist_act_1)
    Robot = [1:size(Robot_1,2); Robot_1]; % [ID; X; Y; Z]
    L_8 = [];
    for L_4 = 1:size(Robot_1,2)
        L_5 = Robot(2,L_4);
        L_6 = Robot(3,L_4);
        L_7 = Robot(4,L_4); % 提取 Z 坐标
        L_8 = [L_8, activityvalue__2_quanju(L_5, L_6, L_7)];
    end
    Robot = [Robot; L_8; dist_act_1]; 
    Weight_vir = Robot;
    Weight_act = Robot;
end