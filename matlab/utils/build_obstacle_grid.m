function obstacleGrid = build_obstacle_grid(obstacles, maxX, maxY, maxZ)
%BUILD_OBSTACLE_GRID Convert obstacle coordinates to a logical 3D grid.
    obstacleGrid = false(maxX, maxY, maxZ);

    for idx = 1:size(obstacles, 2)
        x = obstacles(1, idx);
        y = obstacles(2, idx);
        z = obstacles(3, idx);

        if x >= 1 && x <= maxX && y >= 1 && y <= maxY && z >= 1 && z <= maxZ
            obstacleGrid(x, y, z) = true;
        end
    end
end
