function neigh = A1_NeighborFunction(weight,winner,t,huosheng_1,wiiner_R_act,close_count)
    G = 1; a = 0.01; R = 6; 
    [~,n] = size(weight);
	winner_no = wiiner_R_act(2:4,1); % [x; y; z]
	Weight_no = weight(2:4,:);
	dist = sqrt(sum((winner_no*ones(1,n) - Weight_no).^2));
    neigh = 0 * ones(1,size(weight,2));
    for i = 1:size(weight,2)
        if(dist(i) >= R), neigh(i) = 0; end
        if(dist(i) < R), neigh(i) = exp((-dist(i))/((((1-a)^t) * G)^2)); end
        
        ASD = weight(1,i);
        BSD = huosheng_1(1,:);
        CSD = (ASD==BSD);
        DSD = double(CSD);
        occupied_by_closed = false(1,n);      
        T_num = size(huosheng_1, 2);
        for k = 1:n
            nodeID = weight(1,k);               
            if nodeID == winner(1,1)
                occupied_by_closed(k) = false;
            else
                occupied_by_closed(k) = any(huosheng_1(1,:)==nodeID & ismember(1:T_num, close_count));
            end
        end
        neigh(occupied_by_closed) = 0;
    end
end