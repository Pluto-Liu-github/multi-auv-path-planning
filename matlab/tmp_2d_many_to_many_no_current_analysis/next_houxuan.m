function Pn_next= next_houxuan(Weight_act,huosheng_1,activityvalue__2_quanju,Pn_empty)
    %{
        > Weight_act   (4×N)  // 本轮实际节点
        > huosheng_1   (5×T)  // 各目标获胜机器人 [robotID, x, y, globalAct, subAct]
        > activityvalue__2_quanju (X×Y) // 全局活性图
        > Pn_empty     (4×q)  // 本轮已生成的候选点
        < Pn_next     (4×r)  // 加入非获胜节点后的完整候选池
        将本次非获胜者加入到下一次的候选者，本次非获胜者坐标是实际坐标，需要四舍五入到虚拟坐标，并将其虚拟坐标在总神经网络中的活性值求出
    %}
    CC=setdiff(Weight_act(1,:),huosheng_1(1,:)); %非获胜者
    if ~isempty(CC)  %如果刚好机器人与分配目标点一一对应，则该语句可避免出错
        BS=[];
        CS_4=[];
        for BBD=1:size(CC,2)
            AS=Weight_act(2:3,(CC(1,BBD)));
            BS=[BS,AS];
        end
        BS=[CC;BS];%不是该次循环中获胜者机器人的实际位置
        CS=round(BS(1:3,:));%不是该次循环中获胜者机器人的整数位置
        for ES=1:size(CS,2)
            idx_x = max(1, min(size(activityvalue__2_quanju, 1), CS(2,ES)));
            idx_y = max(1, min(size(activityvalue__2_quanju, 2), CS(3,ES)));
            CS_3 = activityvalue__2_quanju(idx_x, idx_y);
            CS_4=[CS_4,CS_3];
        end
            DS=[CS;CS_4];
            Pn_next=[Pn_empty,DS];
    else
        Pn_next=[Pn_empty];
    end
    QW = round(Weight_act(2:3,:));
    Weight_act(2:3,:) = QW(1:2,:);
end