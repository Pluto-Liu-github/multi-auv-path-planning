function isFree = is_free_node(node, obstacleGrid)
%IS_FREE_NODE Check whether a 3D grid node is inside bounds and not occupied.
    mapSize = size(obstacleGrid);
    x = node(1);
    y = node(2);
    z = node(3);

    isInside = x >= 1 && x <= mapSize(1) && ...
        y >= 1 && y <= mapSize(2) && ...
        z >= 1 && z <= mapSize(3);

    isFree = isInside && ~obstacleGrid(x, y, z);
end
