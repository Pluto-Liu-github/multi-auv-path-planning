function Pn_point = Pn(huosheng_1_1, activity_3_1, ii, MAX_X, MAX_Y, aerf)
    N0 = huosheng_1_1(1,ii);
    x0 = huosheng_1_1(2,ii);
    y0 = huosheng_1_1(3,ii);
    z0 = huosheng_1_1(5,ii);      
    if(z0 == 1)
        Pn_point = huosheng_1_1(1:4,ii);
        return
    end
    
    bestScore = -inf;
    bx = x0;  
    by = y0;
    
    idx_x = max(1, min(MAX_X, round(x0)));
    idx_y = max(1, min(MAX_Y, round(y0)));
    current_ocean_angle = aerf(idx_x, idx_y);

    for dx = -1:1
        for dy = -1:1
            if(dx==0 && dy==0)  continue; end 
            x = x0 + dx;
            y = y0 + dy;
            if(x<1 || x>MAX_X || y<1 || y>MAX_Y)  continue; end 
            
            score = activity_3_1(x,y);   
            
            if score ~= 0 && score > -9999 
                move_angle = rad2deg(atan2(dx, dy));
                angle_diff = move_angle - current_ocean_angle;
                
                ocean_bonus = abs(score) * 0.2 * cosd(angle_diff);
                score = score + ocean_bonus; 
            end

            if(score > bestScore)
                bestScore = score;
                bx = x;
                by = y;
            end
        end
    end
    Pn_point = [N0; bx; by; bestScore];
end