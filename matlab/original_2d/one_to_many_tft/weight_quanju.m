function [weight_2_quanju_dt]=weight_quanju(Target_quanju,Obstale_quanju,TarNum,MAX_X,MAX_Y)
    %{
        > Target_ij    (2×T)  // 目标坐标
        > Obstale_ij   (2×O)  // 障碍坐标
        > TarNum       scalar // 障碍数量
        > MAX_X,MAX_Y  scalar // 边界
        < weight_2_quanju_dt (X×Y) // 全局活性图
    %}
    Et = 20;%目标点的激活因子
    Eo = -20;
    gamma = 3;
    beta = 0.5; %beta
    neighborhood = [1,1; 1,0; 1,-1; -1,0; -1,-1; -1,1; 0,1; 0,-1];
    activity_quanju = zeros(MAX_X,MAX_Y);%将所有神经元活性赋0
    weight_1_quanju = zeros(MAX_X,MAX_Y);%将所有神经元活性初始值赋0
    new_w = zeros(MAX_X, MAX_Y);

    for ijk = 1:size(Obstale_quanju,2)%障碍物=-1
        Obstale_x = Obstale_quanju(1,ijk);
        Obstale_y = Obstale_quanju(2,ijk);
        weight_1_quanju(Obstale_x,Obstale_y) = -1;
        activity_quanju(Obstale_x,Obstale_y) = -1;
    end

    for i = 1:TarNum
        activity_quanju(Target_quanju(1,i),Target_quanju(2,i)) = 1;
        weight_1_quanju(Target_quanju(1,i),Target_quanju(2,i)) = 1;
    end
    for j = 1:100
        for x = 1:MAX_X
            for y = 1:MAX_Y
                weight_cur = weight_1_quanju(x,y);
                for n = 1:8
                    nei_x = x + neighborhood(n,1);
                    nei_y = y + neighborhood(n,2);
                    if(nei_x<1 || nei_x>MAX_X || nei_y<1 || nei_y>MAX_Y), continue; end
                    if(activity_quanju(nei_x,nei_y) == -1), continue; end
                    if(activity_quanju(x,y) == -1) 
                        weight_cur = 0;
                    end
                    weight_nei = weight_1_quanju(nei_x,nei_y);
                    dist = (nei_x-x)^2 + (nei_y-y)^2;
                    Wij = exp(-gamma*dist);
                    weight_cur = weight_cur + Wij * weight_nei;
                end
                tmp = weight_cur + (activity_quanju(x,y)==1)*Et + (activity_quanju(x,y)==-1)*Eo;  % 目标点加 Et
                if(tmp>0 && tmp<1) 
                    tmp = beta*tmp; 
                elseif(tmp >= 1)
                    tmp = 1;
                else 
                    tmp = 0;
                end
                weight_1_quanju(x,y) = tmp;
            end
        end
    end
    for ijk = 1:size(Obstale_quanju,2)
        obs_x = Obstale_quanju(1,ijk);
        obs_y = Obstale_quanju(2,ijk);
        weight_1_quanju(obs_x, obs_y) = -1;
    end
    weight_2_quanju_dt = weight_1_quanju;
    [X,Y] = meshgrid(1:MAX_X,1:MAX_Y);
    Z = log(weight_1_quanju);
    for lll = 1:size(Obstale_quanju,2)
        A = min(Z(:));
        b_1_1 = Obstale_quanju(1,lll);
        b_2_2 = Obstale_quanju(2,lll); %障碍物行列互换为的是与定义的行列匹配
        Z(b_1_1,b_2_2) = -120;
    end
    contourf(X,Y,Z,30,'linestyle','none');
    hold on
end