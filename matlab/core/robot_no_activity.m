function [Weight_vir, Weight_act, Robot] = robot_no_activity(Robot_1, activityvalue__2_quanju, dist_act_1)
    Robot = [1:size(Robot_1, 2); Robot_1]; % [ID; X; Y; Z]
    initialActivity = zeros(1, size(Robot_1, 2));

    for idx = 1:size(Robot_1, 2)
        x = Robot(2, idx);
        y = Robot(3, idx);
        z = Robot(4, idx);
        initialActivity(idx) = activityvalue__2_quanju(x, y, z);
    end

    Robot = [Robot; initialActivity; dist_act_1];
    Weight_vir = Robot;
    Weight_act = Robot;
end