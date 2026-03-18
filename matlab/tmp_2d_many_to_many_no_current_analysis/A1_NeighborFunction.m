function neigh = A1_NeighborFunction(weight,winner,t,huosheng_1,wiiner_R_act,close_count)
    %{
        > weight       (4×N)  // 当前权值矩阵 [nodeID, x, y, act]
        > winner       (4×1)  // 本次获胜记录 [nodeID, x, y, act]
        > t             // 当前迭代序号
        > huosheng_1   (5×T)  // 各目标获胜机器人 [robotID, x, y, globalAct, subAct]
        > wiiner_R_act (4×1)  // 获胜机器人实际坐标 [robotID, x, y, act]
        > close_count  (1×k)  // 已完成目标编号列表
        < neigh       (1×N)  // 邻域函数值 h(j)∈[0,1]，0 表示不更新
    %}
    G = 1; % 可调整参数
    a = 0.01;
    R = 6; %邻域半径，可调整参数
    [~,n] = size(weight);
	winner_no = wiiner_R_act(2:3,1);
	Weight_no = weight(2:3,:);
	dist = sqrt(sum((winner_no*ones(1,n) - Weight_no).^2));
    neigh = 0 * ones(1,size(weight,2));
    for i = 1:size(weight,2)
        if(dist(i) >= R)
            neigh(i) = 0;
        end
        if(dist(i) < R)
            neigh(i) = exp((-dist(i))/((((1-a)^t) * G)^2));
        end
        ASD = weight(1,i);%如果i属于获胜者集合又不是当前的获胜者，则i不动。
        BSD = huosheng_1(1,:);
        CSD = (ASD==BSD);
        DSD = double(CSD);
        ESD = ismember(1,DSD);
        occupied_by_closed = false(1,n);      % 预分配逻辑数组
        T_num = size(huosheng_1, 2);
for k = 1:n
    nodeID = weight(1,k);               % 节点编号
    % 如果该节点出现在 close_count 对应的目标里，就标记
   if nodeID == winner(1,1)
                occupied_by_closed(k) = false;
            else
                % 否则，如果它在别的已完成目标上当获胜者，就把它锁死
                occupied_by_closed(k) = any(huosheng_1(1,:)==nodeID & ismember(1:T_num, close_count));
            end
end
neigh(occupied_by_closed) = 0;
    end
    fprintf('Neigh max=%g, min=%g\n',max(neigh),min(neigh));
end