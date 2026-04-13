function [weight_2_dt]=weight_wij(Target_ij,Obstale_ij,TarNum,MAX_X,MAX_Y)
    %{
        > Target_ij    (2×T)  // 目标坐标
        > Obstale_ij   (2×O)  // 障碍坐标
        > TarNum       scalar // 障碍数量
        > MAX_X,MAX_Y  scalar // 边界
        < weight_2_dt  cell   // {i}(X×Y) 每个目标独立的局部活性图
    %}
    Et = 20; %目标点的激活因子
    Eo = -20;
    gamma = 4;
    beta = 0.8; %beta
    neighborhood = [1,1; 1,0; 1,-1; -1,0; -1,-1; -1,1; 0,1; 0,-1];
    activity_ij = zeros(MAX_X,MAX_Y);%将所有神经元活性赋0
    weight_1_ij = zeros(MAX_X,MAX_Y);%将所有神经元活性初始值赋0

    for ijk = 1:size(Obstale_ij,2)%障碍物=-1
        Obstale_x = Obstale_ij(1,ijk);
        Obstale_y = Obstale_ij(2,ijk);
        weight_1_ij(Obstale_x,Obstale_y) = -1;
        activity_ij(Obstale_x,Obstale_y) = -1;
    end

    for i = 1:TarNum
        activity_ij(Target_ij(1,i),Target_ij(2,i)) = 1;
        weight_1_ij(Target_ij(1,i),Target_ij(2,i)) = 1;
        for j = 1:100
            for x = 1:MAX_X
                for y = 1:MAX_Y
                    weight_cur = weight_1_ij(x,y);
                    %fprintf('x=%d,y=%d\n',x,y)
                    
                    for n = 1:8
                        nei_x = x + neighborhood(n,1);
                        nei_y = y + neighborhood(n,2);
                        if(nei_x<1 || nei_x>MAX_X || nei_y<1 || nei_y>MAX_Y)
                            continue; 
                        end
                        if(activity_ij(nei_x,nei_y) == -1)
                            continue; 
                        end
                        if(activity_ij(x,y) == -1) 
                            weight_cur = 0;
                        end
                        weight_nei = weight_1_ij(nei_x,nei_y);
                        dist = (nei_x-x)^2 + (nei_y-y)^2;
                        Wij = exp(-gamma*dist);
                        weight_cur = weight_cur + Wij * weight_nei;
                    end
                    tmp = weight_cur + (activity_ij(x,y)==1)*Et + (activity_ij(x,y)==-1)*Eo;  % 目标点加 Et
                    if(tmp>0 && tmp<1) 
                        tmp = beta*tmp; 
                    elseif(tmp >= 1)
                        tmp = 1;
                    else 
                        tmp = 0;
                    end
                    weight_1_ij(x,y) = tmp;
                end
            end
        end
        weight_2_dt{i} = weight_1_ij;
        weight_1_ij=zeros(MAX_X,MAX_Y);
        activity_ij(Target_ij(1,i),Target_ij(2,i)) = 0;
    end
end
