function neigh = A1_NeighborFunction(weight, winner, t, huosheng_1, winner_R_act, close_count)
    G = 1; a = 0.01; R = 6; 
    [~, robotCount] = size(weight);
    winner_position = winner_R_act(2:4, 1);
    robot_positions = weight(2:4, :);
    dist = sqrt(sum((winner_position * ones(1, robotCount) - robot_positions).^2));

    neigh = zeros(1, robotCount);
    active_mask = dist < R;
    neigh(active_mask) = exp(-dist(active_mask) / (((1 - a)^t) * G)^2);

    occupied_by_closed = false(1, robotCount);
    targetCount = size(huosheng_1, 2);
    closedTargetMask = ismember(1:targetCount, close_count);

    for k = 1:robotCount
        robotId = weight(1, k);
        if robotId ~= winner(1, 1)
            occupied_by_closed(k) = any(huosheng_1(1, :) == robotId & closedTargetMask);
        end
    end

    neigh(occupied_by_closed) = 0;
end
