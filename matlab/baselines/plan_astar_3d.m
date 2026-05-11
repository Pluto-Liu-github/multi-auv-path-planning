function result = plan_astar_3d(startNode, goalNode, obstacleGrid)
%PLAN_ASTAR_3D A* path planning on a 26-neighborhood 3D grid.
    startNode = round(startNode(:)');
    goalNode = round(goalNode(:)');
    mapSize = size(obstacleGrid);

    result = init_planner_result('A*', startNode, goalNode);
    if ~is_free_node(startNode, obstacleGrid) || ~is_free_node(goalNode, obstacleGrid)
        return;
    end

    gScore = inf(mapSize);
    fScore = inf(mapSize);
    closedSet = false(mapSize);
    cameFromX = zeros(mapSize);
    cameFromY = zeros(mapSize);
    cameFromZ = zeros(mapSize);

    gScore(startNode(1), startNode(2), startNode(3)) = 0;
    fScore(startNode(1), startNode(2), startNode(3)) = norm(startNode - goalNode);
    openSet = startNode;

    while ~isempty(openSet)
        openScores = zeros(size(openSet, 1), 1);
        for idx = 1:size(openSet, 1)
            node = openSet(idx, :);
            openScores(idx) = fScore(node(1), node(2), node(3));
        end

        [~, bestIdx] = min(openScores);
        current = openSet(bestIdx, :);
        openSet(bestIdx, :) = [];
        result.searchSteps = result.searchSteps + 1;

        if isequal(current, goalNode)
            result.path = reconstruct_astar_path(current, cameFromX, cameFromY, cameFromZ);
            result.pathLength = path_length_3d(result.path);
            result.success = true;
            result.collision = false;
            return;
        end

        closedSet(current(1), current(2), current(3)) = true;
        neighbors = get_neighbors_3d(current, obstacleGrid);

        for idx = 1:size(neighbors, 1)
            neighbor = neighbors(idx, :);
            if closedSet(neighbor(1), neighbor(2), neighbor(3))
                continue;
            end

            stepCost = norm(neighbor - current);
            tentativeG = gScore(current(1), current(2), current(3)) + stepCost;
            if tentativeG >= gScore(neighbor(1), neighbor(2), neighbor(3))
                continue;
            end

            cameFromX(neighbor(1), neighbor(2), neighbor(3)) = current(1);
            cameFromY(neighbor(1), neighbor(2), neighbor(3)) = current(2);
            cameFromZ(neighbor(1), neighbor(2), neighbor(3)) = current(3);
            gScore(neighbor(1), neighbor(2), neighbor(3)) = tentativeG;
            fScore(neighbor(1), neighbor(2), neighbor(3)) = tentativeG + norm(neighbor - goalNode);

            if ~ismember(neighbor, openSet, 'rows')
                openSet = [openSet; neighbor];
            end
        end
    end
end

function path = reconstruct_astar_path(current, cameFromX, cameFromY, cameFromZ)
    path = current;
    while true
        parent = [
            cameFromX(current(1), current(2), current(3)), ...
            cameFromY(current(1), current(2), current(3)), ...
            cameFromZ(current(1), current(2), current(3))];

        if all(parent == 0)
            break;
        end

        path = [parent; path];
        current = parent;
    end
end
