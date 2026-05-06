function [open_count,MH] = MUL_AUV(open_count,huosheng_1,close_count)
    MA = huosheng_1(1,:);
    MB = tabulate(MA);
    valid_rows = MB(:,1) > 0 & MB(:,2) > 1; 
    MC = MB(valid_rows, 1);
    
    if (~isempty(MC))
        for MD = 1:size(MC,1)
            ME = find(MA == MC(MD,1));
            open_count = [open_count,ME];
            open_count = unique(open_count);
            
            % 提取在目标 BNN 中的活性值（现在在第 6 行）
            MF = huosheng_1(6,ME); 
            MG = [ME; MF];
            [~, idx] = sort(MG(2,:), 'descend'); 
            MG = MG(:,idx);
            
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