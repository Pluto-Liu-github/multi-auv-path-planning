function Pn_next= next_houxuan(Weight_act,huosheng_1,activityvalue__2_quanju,Pn_empty)
    CC=setdiff(Weight_act(1,:),huosheng_1(1,:)); 
    if ~isempty(CC)
        BS=[]; CS_5=[];
        for BBD=1:size(CC,2)
            AS=Weight_act(2:4,(CC(1,BBD))); % Ã·»° XYZ
            BS=[BS,AS];
        end
        BS=[CC;BS];
        CS=round(BS(1:4,:));
        for ES=1:size(CS,2)
            idx_x = max(1, min(size(activityvalue__2_quanju, 1), CS(2,ES)));
            idx_y = max(1, min(size(activityvalue__2_quanju, 2), CS(3,ES)));
            idx_z = max(1, min(size(activityvalue__2_quanju, 3), CS(4,ES)));
            CS_4 = activityvalue__2_quanju(idx_x, idx_y, idx_z);
            CS_5=[CS_5,CS_4];
        end
        DS=[CS;CS_5];
        Pn_next=[Pn_empty,DS];
    else
        Pn_next=[Pn_empty];
    end
    QW = round(Weight_act(2:4,:));
    Weight_act(2:4,:) = QW(1:3,:);
end