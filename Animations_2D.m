function Animations_2D(T_sampling, T_duration, x_dot_ref, Walls, Y, video2D)

%% ANIMAZIONE 2D
disp('Avvio Animazione...');

video2D.FrameRate = 1 / T_sampling; % Imposta i frame al secondo in base al tuo campionamento
open(video2D); % Apre il file per scriverci dentro

time_vec = 0:T_sampling:T_duration;
x_disp = time_vec * x_dot_ref; 
f_anim = figure('Name', 'Corrected Ghost Wall', 'Color', 'w', 'Position', [50 100 1400 600]);
ax = axes('Parent', f_anim);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');

plot(ax, [0, x_disp(end)], [8, 8], 'k-', 'LineWidth', 2);  
plot(ax, [0, x_disp(end)], [-8, -8], 'k-', 'LineWidth', 2); 
plot(ax, [0, x_disp(end)], [0,0], 'k--', 'LineWidth', 1); 

car_len = 3; 
car_wid = 2;
hCar = rectangle(ax, 'Position', [0,0,0,0], 'FaceColor', 'b', 'EdgeColor', 'k');
hTraj = plot(ax, 0,0, 'b-', 'LineWidth', 2);

hWalls = [];
hArrows = []; %Array per i grafici delle scie degli ostacoli

for w = 1:length(Walls)
    x_p = Walls(w).x_pos;
    w_b = Walls(w).width;
    h_b = Walls(w).h_block;
    y_start = Walls(w).y_open;
    cx = x_p + w_b / 2; % Centro X dell'ostacolo
    
    % 1. FANTASMA (Statico): Disegnato una volta sola, non lo aggiorneremo mai
    rectangle(ax, 'Position', [x_p, y_start - h_b/2, w_b, h_b], ...
              'FaceColor', 'none', 'EdgeColor', 'r', ...
              'LineStyle', '--', 'LineWidth', 1.5);
          
    % 2. FRECCIA (Dinamica): Inizializzata con lunghezza Y (VData) pari a 0
    % La sintassi è quiver(X, Y, U, V, autoscale). Mettiamo 0 per usare le misure reali.
    hq = quiver(ax, cx, y_start, 0, 0, 0, ...
           'Color', [1 0 0 0.6], 'LineWidth', 1.5, ...
           'MaxHeadSize', 0.5, 'LineStyle', ':');
    hArrows = [hArrows, hq];
    
    % 3. OSTACOLO SOLIDO (Dinamico)
    h = rectangle(ax, 'Position', [x_p, y_start - h_b/2, w_b, h_b], ...
                  'FaceColor', [1 0 0 0.2], 'EdgeColor', 'r', 'LineWidth', 1.5);
    hWalls = [hWalls, h];
end

for k = 1:length(time_vec)
    if k > size(Y, 2), break; end
    t = time_vec(k);
    x = x_disp(k); 
    y = Y(1, k); 
    
    set(hCar, 'Position', [x-car_len/2, y-1, car_len, car_wid]);
    set(hTraj, 'XData', x_disp(1:k), 'YData', Y(1, 1:k));
    xlim(ax, [x - 20, x + 80]); ylim(ax, [-12, 12]);
    
    % Grafica Muri (interpolazione solo visiva)
    for w = 1:length(Walls)
        t_hit_front = Walls(w).x_pos / x_dot_ref;
        t_start_closing = t_hit_front - 3.0; 
        
        if t < t_start_closing
             curr_y = Walls(w).y_open;
        elseif t < t_hit_front
             ratio = (t - t_start_closing)/(t_hit_front - t_start_closing);
             curr_y = Walls(w).y_open + ratio*(Walls(w).y_close - Walls(w).y_open);
        else
             curr_y = Walls(w).y_close;
        end
        
        % 1. Aggiorna l'Ostacolo Solido
        h_b = Walls(w).h_block;
        set(hWalls(w), 'Position', [Walls(w).x_pos, curr_y - h_b/2, Walls(w).width, h_b]);
        
        % 2. Aggiorna la Freccia
        % Negli oggetti "quiver", l'estensione lungo Y si chiama 'VData'
        set(hArrows(w), 'VData', curr_y - Walls(w).y_open);
    end
    drawnow;

    frame = getframe(f_anim); % Cattura l'immagine corrente della figura
    writeVideo(video2D, frame); % Scrive l'immagine nel file MP4
end
close(video2D);

disp('Fine.');
end