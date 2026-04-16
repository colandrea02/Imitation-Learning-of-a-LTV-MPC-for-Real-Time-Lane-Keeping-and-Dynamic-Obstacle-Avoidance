function Run3DAnimation(fig_handle, time_vec, x_disp_arr, Y_traj, X_states, Walls, car_dims, color_traj, video3D, T_sampling)
    video3D.FrameRate = 1 / T_sampling; % Imposta i frame al secondo in base al tuo campionamento
    open(video3D); % Apre il file per scriverci dentro
    
    ax = axes('Parent', fig_handle);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
    view(ax, 3); 
    
    L_road = x_disp_arr(end) + 100;
    patch(ax, [-20, L_road, L_road, -20], [-8, -8, 8, 8], [0, 0, 0, 0], [0.5 0.5 0.5], 'EdgeColor', 'none'); 
    plot3(ax, [-20, L_road], [8, 8], [0.05, 0.05], 'k-', 'LineWidth', 3);
    plot3(ax, [-20, L_road], [-8, -8], [0.05, 0.05], 'k-', 'LineWidth', 3);
    plot3(ax, [-20, L_road], [0, 0], [0.05, 0.05], 'w--', 'LineWidth', 3);
    DrawFinishLine(ax, x_disp_arr(end), 8);
    hTraj3D = plot3(ax, 0, 0, 0.5, [color_traj '-'], 'LineWidth', 3);
    
    for k = 1:length(time_vec)
        if k > size(Y_traj, 2), break; end
        delete(findobj(ax, 'Tag', 'moving_obj'));
        
        t = time_vec(k); x = x_disp_arr(k); y = Y_traj(1, k); psi = X_states(3, k); 
        set(hTraj3D, 'XData', x_disp_arr(1:k), 'YData', Y_traj(1, 1:k), 'ZData', zeros(1, k)+0.1);
        DrawBox3D(ax, [x, y, car_dims(3)/2], car_dims, psi, color_traj, 1);
        
        for w = 1:length(Walls)
            dist_to_wall = Walls(w).x_pos - x;
            closing_distance = 40; 
            
            if dist_to_wall > closing_distance
                 curr_y = Walls(w).y_open;
            elseif dist_to_wall > 0
                 ratio = (closing_distance - dist_to_wall) / closing_distance;
                 curr_y = Walls(w).y_open + ratio*(Walls(w).y_close - Walls(w).y_open);
            else
                 curr_y = Walls(w).y_close;
            end
            
            wall_h = 3.0; 
            wall_center = [Walls(w).x_pos + Walls(w).width/2, curr_y, wall_h/2];
            wall_size   = [Walls(w).width, Walls(w).h_block, wall_h]; 
            DrawBox3D(ax, wall_center, wall_size, 0, 'r', 0.8);
        end
        
        cam_x = x - 20; cam_z = 12;
        campos(ax, [cam_x, y, cam_z]); camtarget(ax, [x+15, y, 0]); camva(ax, 30);                      
        drawnow;

        frame = getframe(fig_handle);
        writeVideo(video3D, frame);
    end
    close(video3D);
    disp('Fine.');
end