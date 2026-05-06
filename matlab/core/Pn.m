function Pn_point = Pn(huosheng_1_1, activity_3_1, ii, MAX_X, MAX_Y, MAX_Z, aerf)
    % 假设 huosheng 现在多了一行 Z 坐标，比如 z 是第 4 行，原本的 act 往下顺延
    N0 = huosheng_1_1(1,ii);
    x0 = huosheng_1_1(2,ii);
    y0 = huosheng_1_1(3,ii);
    z0 = huosheng_1_1(4,ii); % <== 新增 Z 坐标提取
    
    bestScore = -inf;
    bx = x0; by = y0; bz = z0;
    
    % 在 3D 空间内扫描下一步
    for dx = -1:1
        for dy = -1:1
            for dz = -1:1  % <== 新增 Z 轴搜索
                if(dx==0 && dy==0 && dz==0) continue; end 
                
                x = x0 + dx; y = y0 + dy; z = z0 + dz;
                if(x<1 || x>MAX_X || y<1 || y>MAX_Y || z<1 || z>MAX_Z) continue; end 
                
                % 查询 3D 张量中的分数
                score = activity_3_1(x,y,z);   
                
                % (注：3D 洋流计算会更复杂，需要三维向量点乘。若暂无 3D 洋流数据，先关闭洋流奖励)
                % ocean_bonus = ... (需重新定义三维风向)
                
                if(score > bestScore)
                    bestScore = score;
                    bx = x; by = y; bz = z;
                end
            end
        end
    end
    
    % 返回带 3D 坐标的点
    Pn_point = [N0; bx; by; bz; bestScore];
end