function Pn_point = Pn(huosheng_1_1, activity_3_1, ii, MAX_X, MAX_Y, MAX_Z)
    % huosheng 中第 2-4 行分别为当前 AUV 的 X/Y/Z 坐标。
    nodeId = huosheng_1_1(1, ii);
    x0 = huosheng_1_1(2, ii);
    y0 = huosheng_1_1(3, ii);
    z0 = huosheng_1_1(4, ii);

    bestScore = -inf;
    bestNode = [x0; y0; z0];

    for dx = -1:1
        for dy = -1:1
            for dz = -1:1
                if dx == 0 && dy == 0 && dz == 0
                    continue;
                end

                x = x0 + dx;
                y = y0 + dy;
                z = z0 + dz;
                if x < 1 || x > MAX_X || y < 1 || y > MAX_Y || z < 1 || z > MAX_Z
                    continue;
                end

                score = activity_3_1(x, y, z);
                if score > bestScore
                    bestScore = score;
                    bestNode = [x; y; z];
                end
            end
        end
    end

    Pn_point = [nodeId; bestNode; bestScore];
end