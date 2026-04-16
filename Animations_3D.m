function Animations_3D(T_sampling, T_duration, x_dot_ref, Walls, X, Y, video3D, f_anim)

%% ANIMAZIONE 3D
disp('Avvio Animazione 3D...');

video3D.FrameRate = 1 / T_sampling; % Imposta i frame al secondo in base al tuo campionamento
open(video3D); % Apre il file per scriverci dentro

time_vec = 0:T_sampling:T_duration;
x_disp = time_vec * x_dot_ref; 


ax = axes('Parent', f_anim);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
view(ax, 3); 

% Strada e Traguardo
L_road = x_disp(end) + 100;
patch(ax, [-20, L_road, L_road, -20], [-8, -8, 8, 8], [0, 0, 0, 0], [0.5 0.5 0.5], 'EdgeColor', 'none'); 
plot3(ax, [-20, L_road], [8, 8], [0.05, 0.05], 'k-', 'LineWidth', 3);
plot3(ax, [-20, L_road], [-8, -8], [0.05, 0.05], 'k-', 'LineWidth', 3);
plot3(ax, [-20, L_road], [0, 0], [0.05, 0.05], 'w--', 'LineWidth', 3);
DrawFinishLine(ax, x_disp(end), 8);

hTraj3D = plot3(ax, 0, 0, 0.5, 'b-', 'LineWidth', 3);

% --- Dimensioni Auto Ottimizzate ---
% Larghezza 1.8m invece di 2.0m per coerenza col margine di sicurezza usato
car_dims = [4.2, 1.7, 1.4]; 

for k = 1:length(time_vec)
    if k > size(Y, 2), break; end
    
    delete(findobj(ax, 'Tag', 'moving_obj'));
    
    t = time_vec(k);
    x = x_disp(k); 
    y = Y(1, k); 
    psi = X(3, k); 
    
    set(hTraj3D, 'XData', x_disp(1:k), 'YData', Y(1, 1:k), 'ZData', zeros(1, k)+0.1);
    
    DrawBox3D(ax, [x, y, car_dims(3)/2], car_dims, psi, 'b', 1);
    
    for w = 1:length(Walls)
        t_hit_front = Walls(w).x_pos / x_dot_ref;
        t_start_closing = t_hit_front - 2.0; 
        
        if t < t_start_closing
             curr_y = Walls(w).y_open;
        elseif t < t_hit_front
             ratio = (t - t_start_closing)/(t_hit_front - t_start_closing);
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
    campos(ax, [cam_x, y, cam_z]);      
    camtarget(ax, [x+15, y, 0]);        
    camva(ax, 30);                      
    drawnow;

    frame = getframe(f_anim);
    writeVideo(video3D, frame);
end
close(video3D);
disp('Fine.');

end

%% FUNZIONI HELPER
function DrawBox3D(ax, center, size, yaw, color, alpha)
    cx = center(1); cy = center(2); cz = center(3);
    L = size(1); W = size(2); H = size(3);
    v = [ -L/2, -W/2, -H/2; L/2, -W/2, -H/2; L/2, W/2, -H/2; -L/2, W/2, -H/2;
          -L/2, -W/2,  H/2; L/2, -W/2,  H/2; L/2, W/2,  H/2; -L/2, W/2,  H/2 ];
    R = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    v_rot = (R * v')'; v_final = v_rot + [cx, cy, cz];
    faces = [ 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    patch(ax, 'Vertices', v_final, 'Faces', faces, 'FaceColor', color, ...
          'FaceAlpha', alpha, 'EdgeColor', 'k', 'Tag', 'moving_obj');
end

function DrawFinishLine(ax, x_start, road_width)
    sq_size = 1.0; 
    for row = 0:1 
        for y = -road_width : sq_size : (road_width - sq_size)
            if mod((y+road_width)/sq_size + row, 2) == 0, col=[0.1 0.1 0.1]; else, col=[0.9 0.9 0.9]; end
            Xf = [x_start + row*sq_size, x_start + (row+1)*sq_size, x_start + (row+1)*sq_size, x_start + row*sq_size];
            Yf = [y, y, y+sq_size, y+sq_size]; Zf = 0.02*ones(1,4);
            patch(ax, Xf, Yf, Zf, col, 'EdgeColor', 'none');
        end
    end
    %plot3(ax, [x_start, x_start], [road_width+1, road_width+1], [0, 5], 'k-', 'LineWidth', 4);
    
end