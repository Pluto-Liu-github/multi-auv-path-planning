function neighbors = get_neighbors_3d(node, obstacleGrid)
%GET_NEIGHBORS_3D Return all valid 26-neighborhood free nodes.
    candidates = zeros(26, 3);
    count = 0;

    for dx = -1:1
        for dy = -1:1
            for dz = -1:1
                if dx == 0 && dy == 0 && dz == 0
                    continue;
                end

                nextNode = node + [dx, dy, dz];
                if is_free_node(nextNode, obstacleGrid)
                    count = count + 1;
                    candidates(count, :) = nextNode;
                end
            end
        end
    end

    neighbors = candidates(1:count, :);
end
