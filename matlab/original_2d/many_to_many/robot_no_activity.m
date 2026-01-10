function [Weight_vir,Weight_act,Robot] = robot_no_activity(Robot_1,activityvalue__2_quanju,dist_act_1)
    %{
        > Robot_1      (2×M)  // 机器人初始坐标 [x; y]
        > activityvalue__2_quanju (X×Y) // 全局活性
        > dist_act_1   (1×M)  // 初始零航程
        < Weight_vir  (4×M)  // 虚拟节点（初始=robot）
        < Weight_act  (4×M)  // 实际节点（初始=robot）
        < Robot       (4×M)  // 带编号与活性 [robotID, x, y, globalAct]    
    %}
%UNTITLED2 获取机器人初始时刻在总的神经网络中的活性值及其序号，后边寻找weight_act与weight_vir对应关系
    Robot = [1:size(Robot_1,2);Robot_1];%给机器人编号
    L_8 = [];%%%%%%%%%%%%获取机器人初始时刻在总的神经网络中的活性值

    for L_4 = 1:size(Robot_1,2)
        L_5 = Robot(2,L_4);
        L_6 = Robot(3,L_4);
        L_7 = activityvalue__2_quanju(L_5,L_6);
        L_8 = [L_8,L_7];
    end
    Robot = [Robot;L_8;dist_act_1];%将机器人在全局中神经元活性强度及从开始到现在走过的路程加入到机器人信息
    Weight_vir = Robot;
    Weight_act = Robot;
end

