function result = plan_apf_3d(startNode, goalNode, obstacleGrid, options)
%PLAN_APF_3D Artificial potential field planner on a 3D grid.
    if nargin < 4
        options = struct();
    end

    maxIterations = get_option(options, 'maxIterations', 500);
    attractiveGain = get_option(options, 'attractiveGain', 1.0);
    repulsiveGain = get_option(options, 'repulsiveGain', 3.0);
    influenceRadius = get_option(options, 'influenceRadius', 3.0);

    startNode = round(startNode(:)');
    goalNode = round(goalNode(:)');
    result = init_planner_result('APF', startNode, goalNode);

    if ~is_free_node(startNode, obstacleGrid) || ~is_free_node(goalNode, obstacleGrid)
        return;
    end

    current = startNode;
    path = current;
    visited = false(size(obstacleGrid));
    visited(current(1), current(2), current(3)) = true;

    for iter = 1:maxIterations
        result.searchSteps = iter;
        if norm(current - goalNode) <= 1
            path = [path; goalNode];
            result.path = path;
            result.pathLength = path_length_3d(path);
            result.success = true;
            result.collision = false;
            return;
        end

        neighbors = get_neighbors_3d(current, obstacleGrid);
        if isempty(neighbors)
            break;
        end

        potentials = zeros(size(neighbors, 1), 1);
        for idx = 1:size(neighbors, 1)
            node = neighbors(idx, :);
            potentials(idx) = apf_potential(node, goalNode, obstacleGrid, ...
                attractiveGain, repulsiveGain, influenceRadius);

            if visited(node(1), node(2), node(3))
                potentials(idx) = potentials(idx) + 1000;
            end
        end

        [~, bestIdx] = min(potentials);
        nextNode = neighbors(bestIdx, :);
        if isequal(nextNode, current)
            break;
        end

        current = nextNode;
        visited(current(1), current(2), current(3)) = true;
        path = [path; current];
    end

    result.path = path;
    result.pathLength = path_length_3d(path);
end

function potential = apf_potential(node, goalNode, obstacleGrid, attractiveGain, repulsiveGain, influenceRadius)
    attractive = attractiveGain * norm(node - goalNode);
    nearestObstacle = nearest_obstacle_distance(node, obstacleGrid, influenceRadius);

    repulsive = 0;
    if nearestObstacle < influenceRadius
        nearestObstacle = max(nearestObstacle, 0.1);
        repulsive = repulsiveGain * (1 / nearestObstacle - 1 / influenceRadius)^2;
    end

    potential = attractive + repulsive;
end

function distance = nearest_obstacle_distance(node, obstacleGrid, radius)
    mapSize = size(obstacleGrid);
    minX = max(1, node(1) - ceil(radius));
    maxX = min(mapSize(1), node(1) + ceil(radius));
    minY = max(1, node(2) - ceil(radius));
    maxY = min(mapSize(2), node(2) + ceil(radius));
    minZ = max(1, node(3) - ceil(radius));
    maxZ = min(mapSize(3), node(3) + ceil(radius));

    distance = inf;
    for x = minX:maxX
        for y = minY:maxY
            for z = minZ:maxZ
                if obstacleGrid(x, y, z)
                    distance = min(distance, norm(node - [x, y, z]));
                end
            end
        end
    end
end

function value = get_option(options, name, defaultValue)
    if isfield(options, name)
        value = options.(name);
    else
        value = defaultValue;
    end
end
