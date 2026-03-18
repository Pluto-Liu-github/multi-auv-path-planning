function [open_count,MH] = MUL_AUV(open_count,huosheng_1,close_count) % 不需要传 Target 了
    MA = huosheng_1(1,:);
    MB = tabulate(MA);
    
    % 剔除 ID 为 0 的“幽灵机器人”（已完成的目标）
    valid_rows = MB(:,1) > 0 & MB(:,2) > 1; 
    MC = MB(valid_rows, 1);
    
    if (~isempty(MC))
        for MD = 1:size(MC,1)
            ME = find(MA == MC(MD,1));
            open_count = [open_count,ME];
            open_count = unique(open_count);
            
            % --- 核心：恢复使用 BNN 活性值降序排列！这是绕开障碍物的灵魂 ---
            MF = huosheng_1(5,ME); % 获取机器人在各个目标 BNN 中的活性值
            MG = [ME; MF];
            [~, idx] = sort(MG(2,:), 'descend'); % 活性越大，避障路径越短，越优先！
            MG = MG(:,idx);
            % ---------------------------------------------------------
            
            MG = [MC(MD,1)*ones(1,size(MG,2)); MG];
            MQ = intersect(MG(2,:),close_count);
            
            if(~isempty(MQ))
                for MW = 1:size(MQ,2)
                    MU = MQ(1,MW);
                    [~,MS] = find(MG(2,:) == MU);
                    MG(:,MS) = [];
                end
            end
            MH{MD} = MG;
        end
    else
        MH = [];
    end
end