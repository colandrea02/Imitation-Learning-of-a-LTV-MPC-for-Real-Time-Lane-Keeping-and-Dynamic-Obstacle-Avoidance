function DrawBox3D(ax, center, size, yaw, color, alpha)
    % DRAWBOX3D Disegna un parallelepipedo semplice
    % center: [x, y, z]
    % size:   [L, W, H]
    % yaw:    rotazione Z (rad)
    % color:  'r', 'b', etc.
    % alpha:  0-1 trasparenza
    
    cx = center(1); cy = center(2); cz = center(3);
    L = size(1); W = size(2); H = size(3);

    % Vertici (Cubo centrato in 0,0,0)
    v = [ -L/2, -W/2, -H/2;
           L/2, -W/2, -H/2;
           L/2,  W/2, -H/2;
          -L/2,  W/2, -H/2;
          -L/2, -W/2,  H/2;
           L/2, -W/2,  H/2;
           L/2,  W/2,  H/2;
          -L/2,  W/2,  H/2 ];

    % Rotazione Yaw
    R = [cos(yaw) -sin(yaw) 0;
         sin(yaw)  cos(yaw) 0;
         0         0        1];
    
    v_rot = (R * v')';
    v_final = v_rot + [cx, cy, cz];

    % Facce
    faces = [ 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];

    % Disegna con Tag 'moving_obj' per cancellazione rapida
    patch(ax, 'Vertices', v_final, 'Faces', faces, ...
          'FaceColor', color, 'FaceAlpha', alpha, ...
          'EdgeColor', 'k', 'LineWidth', 1, ...
          'Tag', 'moving_obj');
end