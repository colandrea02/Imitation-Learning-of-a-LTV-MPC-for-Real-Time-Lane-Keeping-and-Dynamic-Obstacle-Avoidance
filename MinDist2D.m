function MinDist2d = GetMinDistance2D(car_x, car_y, Walls)
    MinDist2d = inf;
    closing_distance = 40; 
    
    for w = 1:length(Walls)
        
        dist_to_wall_front = Walls(w).x_pos - car_x;
        
        if dist_to_wall_front > closing_distance
             curr_y = Walls(w).y_open;
        elseif dist_to_wall_front > 0
             ratio = (closing_distance - dist_to_wall_front) / closing_distance;
             curr_y = Walls(w).y_open + ratio*(Walls(w).y_close - Walls(w).y_open);
        else
             curr_y = Walls(w).y_close; 
        end
        
        % 2. Geometria del blocco ostacolo 
        x_min = Walls(w).x_pos;
        x_max = Walls(w).x_pos + Walls(w).width;
        y_min = curr_y - Walls(w).h_block / 2;
        y_max = curr_y + Walls(w).h_block / 2;
        
        % 3. Calcolo distanza Euclidea 2D Punto-Rettangolo
        dx = max([x_min - car_x, 0, car_x - x_max]);
        dy = max([y_min - car_y, 0, car_y - y_max]);
        
        dist = sqrt(dx^2 + dy^2);
        
        % Aggiorna la distanza minima trovata finora
        if dist < MinDist2d
            MinDist2d = dist;
        end
    end
end


