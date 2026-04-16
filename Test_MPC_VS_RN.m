%% TEST: LTV-MPC (Accelerazione + Attrito) con ostacoli sincronizzati spazialmente
clc; clear; close all;
warning('off','all'); 

% --- RETE NEURALE ---
if exist('TrainedNet.mat', 'file')
    load('TrainedNet.mat', 'net');
    disp('Rete Neurale caricata.');
else
    error('File TrainedNet.mat non trovato! Esegui prima il training.');
end

%% 2. COSTANTI E IMPOSTAZIONI
g = 9.81; m_car = 2050; I = 3344;
a = 1.2; b = 1.6; C_alpha_f = 120000; C_alpha_r = 150000; 
T_sampling = 0.1;
road_len = 300;
T_duration = 100; 
x_0 = [0 0 0 0 0]'; 
umin = -0.6; umax = 0.6; 
N_horizon = 30; 
R_rho = 10;   
xmax_bound = [inf; inf; inf;  8; inf]; 
xmin_bound = [-inf; -inf; -inf; -8; -inf];

mu_friction = 0.5; 
v_x_start = 0.1;  
v_x_target = 15;
C = [0 0 0 1 0]; D = 0;

%% 3. GENERAZIONE PISTA SICURA
Walls = GenerateRandomWalls(5, 40, road_len - 60);

N_step = floor(T_duration / T_sampling);

%% 4. SIMULAZIONE REAL-TIME CON LA RETE NEURALE
disp('Simulazione in corso (Guida RN)...');

% Pre-allocazione Array per l'AI
X_AI = zeros(5, N_step+1); X_AI(:,1) = x_0;
U_AI = zeros(2, N_step); 
Y_AI = zeros(1, N_step+1); Y_AI(1) = C*x_0;
Time_AI_log = zeros(1, N_step);

% Tracciamento Spaziale e Longitudinale per l'AI
V_x_log_AI = zeros(1, N_step+1); V_x_log_AI(1) = v_x_start;
Car_X_log_AI = zeros(1, N_step+1); Car_X_log_AI(1) = 0;

% Limiti di accelerazione fisica in base all'attrito
a_min_AI = -0.8 * mu_friction * 9.81; 
a_max_AI =  0.4 * mu_friction * 9.81; 

tic; 
for i = 1:N_step
    
    car_x = Car_X_log_AI(i);
    v_curr = V_x_log_AI(i);
    
    % --- A. COSTRUZIONE SENSORI ---
    dist_next_obs = 100; 
    y_upper = 8.0;       
    y_lower = -8.0;      
    
    for w = 1:length(Walls)
        if Walls(w).x_pos + Walls(w).width > car_x
            dist_next_obs = Walls(w).x_pos - car_x;
            half_h = Walls(w).h_block / 2;
            y_upper = (Walls(w).y_close + half_h);
            y_lower = (Walls(w).y_close - half_h);
            break; 
        end
    end
    
    features = [X_AI(:, i); v_curr; dist_next_obs; y_upper; y_lower; mu_friction; v_x_target];
    
    t_start_ai = tic;

    % --- B. PREDIZIONE DELL'INTELLIGENZA ARTIFICIALE ---
    u_pred = net(features); 
    steer_AI = max(min(u_pred(1), umax), umin);
    accel_AI = max(min(u_pred(2), a_max_AI), a_min_AI);

    Time_AI_log(i) = toc(t_start_ai);

    U_AI(:, i) = [steer_AI; accel_AI];
    
    % --- C. FISICA LPV E AGGIORNAMENTO STATO ---
    
    % 1. Fisica Longitudinale
    v_next = v_curr + accel_AI * T_sampling;
    V_x_log_AI(i+1) = max(v_next, 0.1); % Evitiamo velocità negative o zero
    Car_X_log_AI(i+1) = car_x + v_curr * T_sampling;
    
    % 2. Fisica Laterale 
    Caf_eff = C_alpha_f * mu_friction; 
    Car_eff = C_alpha_r * mu_friction;
    
    v_matrix = max(v_curr, 0.1); 
        
    A_lat_cont = [ -(Caf_eff+Car_eff)/(m_car*v_matrix),  -v_matrix + (Car_eff*b - Caf_eff*a)/(m_car*v_matrix), 0, 0, Caf_eff/m_car;
                       (Car_eff*b - Caf_eff*a)/(I*v_matrix), -(Caf_eff*a^2 + Car_eff*b^2)/(I*v_matrix),         0, 0, Caf_eff*a/I;
                       0, 1, 0, 0, 0; 1, 0, v_matrix, 0, 0; 0, 0, 0, 0, 0];
    B_lat_cont  = [0; 0; 0; 0; 1]; 
    
    sys_d = c2d(ss(A_lat_cont, B_lat_cont, C, D), T_sampling);
    A_dyn = sys_d.A; 
    B_dyn = sys_d.B;
    
    % 3. Avanzamento dello stato laterale
    X_AI(:, i+1) = A_dyn * X_AI(:, i) + B_dyn * steer_AI;
    Y_AI(i+1) = C * X_AI(:, i+1);
    
    % Check traguardo
    if Car_X_log_AI(i+1) >= road_len + 10
        X_AI = X_AI(:, 1:i+1);
        U_AI = U_AI(:, 1:i);
        Y_AI = Y_AI(:, 1:i+1);
        V_x_log_AI = V_x_log_AI(1:i+1);
        Car_X_log_AI = Car_X_log_AI(1:i+1);
        Time_AI_log = Time_AI_log(1:i);
        break;
    end
end
tempo_ai = toc;
fprintf('Tempo calcolo RN: %.4f s\n', tempo_ai);
time_vec_AI = (0:(length(V_x_log_AI)-1)) * T_sampling;

%% 4. SIMULAZIONE LTV-MPC (Disaccoppiato con Frenata e Attrito)
disp('Calcolo Traiettoria MPC LTV (Frenata Attiva)...');
tic;
[X_MPC, U_MPC, Y_MPC, V_x_log, Car_X_log, Time_MPC_log] = MPC_Select_LTV(m_car, I, a, b, C_alpha_f, C_alpha_r, mu_friction, C, D, N_horizon, T_sampling, T_duration, R_rho, x_0, v_x_start, v_x_target, umin, umax, xmin_bound, xmax_bound, Walls);
tempo_mpc = toc;
time_vec = (0:(length(Car_X_log)-1)) * T_sampling;
fprintf('Tempo calcolo MPC: %.4f s\n', tempo_mpc);

%% 5. ANIMAZIONE 3D
car_dims_vis = [3, 2, 1.4]; 

disp('Avvio Animazione 3D (LTV-MPC Control)...');
video3D_MPC = VideoWriter('Simulazione_3D_MPC.mp4', 'MPEG-4');
f_anim_mpc = figure('Name', 'Test MPC - 3D Simulation', 'Color', 'w', 'Position', [50 50 1200 450]);
Run3DAnimation(f_anim_mpc, time_vec, Car_X_log, Y_MPC, X_MPC, Walls, car_dims_vis, 'g',video3D_MPC, T_sampling);

disp('Avvio Animazione 3D (RN Control)...');
video3D_RN = VideoWriter('Simulazione_3D_RN.mp4', 'MPEG-4');
f_anim_ia = figure('Name', 'Test RN - 3D Simulation', 'Color', 'w', 'Position', [50 50 1200 450]);
Run3DAnimation(f_anim_ia, time_vec_AI, Car_X_log_AI, Y_AI, X_AI, Walls, car_dims_vis, 'b',video3D_RN, T_sampling);



%% 6. GRAFICO CONFRONTO 2D E VELOCITÀ
f = figure('Name', 'Analisi MPC', 'Color', 'w', 'Position', [100 100 1200 800]);

% --- SUBPLOT 1: TRAIETTORIA ---
subplot(2, 1, 1);
hold on; grid on;
plot([0 road_len], [8 8], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
plot([0 road_len], [-8 -8], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
plot([0 road_len], [0 0], 'k--', 'HandleVisibility', 'off');

for w = 1:length(Walls)
    x_p = Walls(w).x_pos; w_b = Walls(w).width; h_b = Walls(w).h_block;
    y_start = Walls(w).y_open; y_end = Walls(w).y_close; cx = x_p + w_b/2; 
    X_rect = [x_p, x_p+w_b, x_p+w_b, x_p];
    
    dark_red = [0.6 0 0];
    
    Y_start_rect = [y_start - h_b/2, y_start - h_b/2, y_start + h_b/2, y_start + h_b/2];
    patch(X_rect, Y_start_rect, dark_red, 'FaceColor', 'none', 'EdgeColor', dark_red, ...
          'EdgeAlpha', 0.5, 'LineStyle', '--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
          
    quiver(cx, y_start, 0, y_end - y_start, 0, 'Color', [dark_red 0.6], 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'LineStyle', ':', 'HandleVisibility', 'off');
    
    Y_end_rect = [y_end - h_b/2, y_end - h_b/2, y_end + h_b/2, y_end + h_b/2];
    if w == 1
        patch(X_rect, Y_end_rect, dark_red, 'FaceColor', dark_red, 'FaceAlpha', 0.2, 'EdgeColor', dark_red, 'LineWidth', 1.5, 'DisplayName', 'Ostacolo Finale');
    else
        patch(X_rect, Y_end_rect, dark_red, 'FaceColor', dark_red, 'FaceAlpha', 0.2, 'EdgeColor', dark_red, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
end
% Plot Traiettoria
plot(Car_X_log, Y_MPC(1, 1:length(Car_X_log)), 'k--', 'LineWidth', 2, 'DisplayName', 'MPC');
plot(Car_X_log_AI, Y_AI, 'r-', 'LineWidth', 2, 'DisplayName', 'Rete Neurale');
xlabel('Distanza X [m]'); ylabel('Posizione Laterale Y [m]'); 
title('Traiettoria (Vista Superiore)');
legend('Location', 'northeast'); ylim([-10 10]); xlim([0 road_len]);

% --- SUBPLOT 2: PROFILO DI VELOCITÀ ---
subplot(2, 1, 2);
hold on; grid on;
plot(Car_X_log, ones(size(time_vec)) * v_x_target * 3.6, 'k--', 'LineWidth', 1, 'DisplayName', 'Target / Cruise Control');
plot(Car_X_log, V_x_log * 3.6, 'k--', 'LineWidth', 2, 'DisplayName', 'Velocità Reale MPC');
plot(Car_X_log_AI, V_x_log_AI * 3.6, 'r-', 'LineWidth', 2, 'DisplayName', 'Velocità Reale RN');
xlabel('Tempo [s]'); ylabel('Velocità [km/h]');
title(sprintf('Profilo di Velocità (Coefficiente di Attrito \\mu = %.1f)', mu_friction));
legend('Location', 'southwest');
ylim([0, v_x_target*3.6 + 20]);
xlim([0 Car_X_log(end)]);

exportgraphics(f, 'Confronto_Simulazioni_E_Profili_Velocità.pdf');
disp('Simulazione completata.');

%% 7 CALCOLO DEGLI INDICI DI COSTO NORMALIZZATI

Q_lat_eval = 1.0;         % Mantenere il centro pista
R_lat_eval = R_rho;       % Uso dello sterzo
Q_lon_eval = 5;           % Errore velocità
R_lon_eval = 5;           % Sforzo frenata
margin_cost = 1.9;        % Margine di sicurezza
road_limit = 8.0;         % Limite della pista

tolleranza_num = 0.05;    % 5 centimetri di tolleranza

% --- PESO PENALITÀ SOFT ---
W_margin = 100;          

% 1. CALCOLO COSTO MPC 
J_lat_MPC = 0; J_lon_MPC = 0; J_pen_MPC = 0;
N_mpc = length(U_MPC);
Cum_Cost_MPC_arr = zeros(1, N_mpc); Inst_Cost_MPC_arr = zeros(1, N_mpc);
Inst_Lat_MPC_arr  = zeros(1, N_mpc); Inst_Lon_MPC_arr  = zeros(1, N_mpc);

for i = 1:N_mpc
    car_x = Car_X_log(i); car_y = X_MPC(4, i);
    
    penalty_margin = 0;
    
    % A. Bordi Pista (con tolleranza)
    if car_y > (road_limit - margin_cost + tolleranza_num)
        invasione_bordo = car_y - (road_limit - margin_cost);
        penalty_margin = penalty_margin + W_margin * (invasione_bordo^4);
    elseif car_y < (-road_limit + margin_cost - tolleranza_num)
        invasione_bordo = (-road_limit + margin_cost) - car_y;
        penalty_margin = penalty_margin + W_margin * (invasione_bordo^4);
    end
    
    % B. Ostacoli (con tolleranza)
    dist_obs = 100; wall_idx = -1;
    for w = 1:length(Walls)
        x_s = Walls(w).x_pos; x_e = x_s + Walls(w).width; y_c = Walls(w).y_close; h_b = Walls(w).h_block;
        if x_e > car_x && (x_s - car_x) < dist_obs
            dist_obs = x_s - car_x; wall_idx = w; 
        end
        
        if car_x >= x_s && car_x <= x_e
            distanza_laterale = abs(car_y - y_c) - (h_b / 2);
            if distanza_laterale < (margin_cost - tolleranza_num)
                invasione = margin_cost - distanza_laterale;
                penalty_margin = penalty_margin + W_margin * (invasione^4); 
            end
        end
    end
    
    step_cost_lat = (Q_lat_eval * Y_MPC(i)^2) + (R_lat_eval * U_MPC(i)^2) + penalty_margin;
    J_pen_MPC = J_pen_MPC + penalty_margin; 
    
    if dist_obs < 80 && dist_obs > 0 && wall_idx > 0
        w_y = Walls(wall_idx).y_close; w_h = Walls(wall_idx).h_block;
        dy_sopra = abs(car_y - (w_y + w_h/2 + margin_cost));
        dy_sotto = abs(car_y - (w_y - w_h/2 - margin_cost));
        dy_min = max(min(dy_sopra, dy_sotto), 0.5);
        v_safe = dist_obs * sqrt((mu_friction * 9.81 * 0.3) / dy_min);
        v_target_mpc = max(min(v_safe, v_x_target), 5.0);
    else
        v_target_mpc = v_x_target;
    end
    a_x_MPC = (V_x_log(i+1) - V_x_log(i)) / T_sampling;
    step_cost_lon = (Q_lon_eval * (V_x_log(i) - v_target_mpc)^2) + (R_lon_eval * a_x_MPC^2);
    
    Inst_Lat_MPC_arr(i) = step_cost_lat; Inst_Lon_MPC_arr(i) = step_cost_lon; Inst_Cost_MPC_arr(i) = step_cost_lat + step_cost_lon;
    J_lat_MPC = J_lat_MPC + step_cost_lat; J_lon_MPC = J_lon_MPC + step_cost_lon; Cum_Cost_MPC_arr(i) = J_lat_MPC + J_lon_MPC;
end

% 2. CALCOLO COSTO RETE NEURALE (L'Allievo)
J_lat_AI = 0; J_lon_AI = 0; J_pen_AI = 0;
N_ai = length(U_AI(1,:));
Cum_Cost_AI_arr = zeros(1, N_ai); Inst_Cost_AI_arr = zeros(1, N_ai);
Inst_Lat_AI_arr  = zeros(1, N_ai); Inst_Lon_AI_arr  = zeros(1, N_ai);

for i = 1:N_ai
    car_x = Car_X_log_AI(i); car_y = X_AI(4, i);
    
    penalty_margin = 0;
    
    % A. Bordi Pista (con tolleranza)
    if car_y > (road_limit - margin_cost + tolleranza_num)
        invasione_bordo = car_y - (road_limit - margin_cost);
        penalty_margin = penalty_margin + W_margin * (invasione_bordo^4);
    elseif car_y < (-road_limit + margin_cost - tolleranza_num)
        invasione_bordo = (-road_limit + margin_cost) - car_y;
        penalty_margin = penalty_margin + W_margin * (invasione_bordo^4);
    end
    
    dist_obs = 100; wall_idx = -1;
    for w = 1:length(Walls)
        x_s = Walls(w).x_pos; x_e = x_s + Walls(w).width; y_c = Walls(w).y_close; h_b = Walls(w).h_block;
        if x_e > car_x && (x_s - car_x) < dist_obs
            dist_obs = x_s - car_x; wall_idx = w; 
        end
        
        if car_x >= x_s && car_x <= x_e
            distanza_laterale = abs(car_y - y_c) - (h_b / 2);
            if distanza_laterale < (margin_cost - tolleranza_num)
                invasione = margin_cost - distanza_laterale;
                penalty_margin = penalty_margin + W_margin * (invasione^4);
            end
        end
    end
    
    step_cost_lat = (Q_lat_eval * Y_AI(i)^2) + (R_lat_eval * U_AI(1, i)^2) + penalty_margin;
    J_pen_AI = J_pen_AI + penalty_margin; 
    
    if dist_obs < 80 && dist_obs > 0 && wall_idx > 0
        w_y = Walls(wall_idx).y_close; w_h = Walls(wall_idx).h_block;
        dy_sopra = abs(car_y - (w_y + w_h/2 + margin_cost));
        dy_sotto = abs(car_y - (w_y - w_h/2 - margin_cost));
        dy_min = max(min(dy_sopra, dy_sotto), 0.5);
        v_safe = dist_obs * sqrt((mu_friction * 9.81 * 0.3) / dy_min);
        v_target_ai = max(min(v_safe, v_x_target), 5.0);
    else
        v_target_ai = v_x_target;
    end
    a_x_AI = U_AI(2, i);
    step_cost_lon = (Q_lon_eval * (V_x_log_AI(i) - v_target_ai)^2) + (R_lon_eval * a_x_AI^2);
    
    Inst_Lat_AI_arr(i) = step_cost_lat; Inst_Lon_AI_arr(i) = step_cost_lon; Inst_Cost_AI_arr(i) = step_cost_lat + step_cost_lon;
    J_lat_AI = J_lat_AI + step_cost_lat; J_lon_AI = J_lon_AI + step_cost_lon; Cum_Cost_AI_arr(i) = J_lat_AI + J_lon_AI;
end

% 3. RISULTATI E STAMPA
J_lat_MPC_medio = J_lat_MPC / N_mpc; J_lon_MPC_medio = J_lon_MPC / N_mpc; J_tot_MPC_medio = J_lat_MPC_medio + J_lon_MPC_medio;
J_lat_AI_medio = J_lat_AI / N_ai; J_lon_AI_medio = J_lon_AI / N_ai; J_tot_AI_medio = J_lat_AI_medio + J_lon_AI_medio;
RMSE_Y_MPC = sqrt(mean(Y_MPC.^2)); RMSE_Y_AI  = sqrt(mean(Y_AI.^2));

fprintf('--- COSTO MEDIO MPC (Il Maestro) ---\n');
fprintf('TOTALE MPC   : %.4f\n', J_tot_MPC_medio);
fprintf('Di cui Multe : %.4f\n\n', J_pen_MPC / N_mpc);

fprintf('--- COSTO MEDIO RETE NEURALE ---\n');
fprintf('TOTALE RN    : %.4f\n', J_tot_AI_medio);
fprintf('Di cui Multe : %.4f\n\n', J_pen_AI / N_ai);

diff_totale = ((J_tot_AI_medio - J_tot_MPC_medio) / J_tot_MPC_medio) * 100;
fprintf('-> DIFFERENZA: La RN si scosta dall''ottimo del %+.2f%%\n', diff_totale);
if diff_totale > 0
    disp('VERDETTO: L''MPC vince dimostrando maggiore sicurezza e precisione!');
else
    disp('VERDETTO: La RN ottiene un punteggio migliore!');
end
disp('---------------------------------------------------');
%% 8 CONFRONTO TEMPI COMPUTAZIONALI (Scala Logaritmica)
disp('Generazione Grafico Tempi Computazionali...');

f_time = figure('Name', 'Analisi Temporale Computazionale', 'Color', 'w', 'Position', [200 200 1000 450]);
hold on; grid on;

set(gca, 'YScale', 'log');

N_eval_time = min(length(Time_MPC_log), length(Time_AI_log));
time_eval_array = (0:N_eval_time-1) * T_sampling;

% Grafico in scala logaritmica
semilogy(time_eval_array, Time_MPC_log(1:N_eval_time)*1000, 'k-s', 'MarkerIndices', 1:5:N_eval_time, 'MarkerFaceColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Tempo Esecuzione MPC');
semilogy(time_eval_array, Time_AI_log(1:N_eval_time)*1000, 'r-o', 'MarkerIndices', 1:5:N_eval_time, 'MarkerFaceColor', 'r', 'LineWidth', 1.5, 'DisplayName', 'Tempo Esecuzione RN');
xlim([0 time_eval_array(end)]);

% for w = 1:length(Walls)
%     t_muro = Walls(w).x_pos / v_x_target; 
%     if t_muro < time_eval_array(end)
%         xline(t_muro, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
%     end
% end

xlabel('Tempo [s]', 'FontSize', 11);
ylabel('Tempo Calcolo (ms)', 'FontSize', 11);
title('Costo Computazionale Istantaneo: MPC vs RN', 'FontSize', 13);
legend('Location', 'northeast');
ax = gca; 
ax.YMinorGrid = 'on'; 
ax.YMinorTick = 'on';
fprintf('Tempo calcolo MPC: %.4f s\n', sum(Time_MPC_log));
fprintf('Tempo calcolo RN: %.4f s\n', sum(Time_AI_log));
exportgraphics(f_time, 'Confronto_Tempi_Computazionali.pdf');

%% 9. DASHBOARD VISIVA DELLE PRESTAZIONI (KPI)
f_kpi = figure('Name', 'Dashboard Indici di Costo', 'Color', 'w', 'Position', [150 150 1200 700]);

% Sincronizziamo la lunghezza degli array per il grafico
N_eval = min(N_mpc, N_ai); 
time_eval = (0:N_eval-1) * T_sampling;

% --- SUBPLOT 1 ---
subplot(2, 2, 1); hold on; grid on;
plot(time_eval, Cum_Cost_MPC_arr(1:N_eval), 'k--', 'LineWidth', 2.5, 'DisplayName', 'Costo Cumulato MPC');
plot(time_eval, Cum_Cost_AI_arr(1:N_eval), 'r-', 'LineWidth', 2.5, 'DisplayName', 'Costo Cumulato RN');
xlim([0 time_eval(end)]);
for w = 1:length(Walls)
    t_muro = Walls(w).x_pos / v_x_target; 
    if t_muro < time_eval(end)
        xline(t_muro, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'HandleVisibility', 'off');
        text(t_muro, max(Cum_Cost_MPC_arr)*0.1, sprintf(' Muro %d', w), 'Color', [0.4 0.4 0.4], 'FontSize', 8);
    end
end
xlabel('Tempo [s]'); ylabel('Costo Cumulato J_{tot}');
title('Evoluzione del Costo durante la simulazione');
legend('Location', 'northwest');

% --- SUBPLOT 2 ---
subplot(2, 2, 2); 
hold on; axis off;

xlim([0 1]);
ylim([-0.1 1.1]); 

plot([0.0, 0.95], [0.88, 0.88], 'k-', 'LineWidth', 1.5);

text(0.05, 0.75, '\bfMetrica', 'FontSize', 14);
text(0.45, 0.75, '\bfMPC', 'FontSize', 14, 'Color', 'k');
text(0.75, 0.75, '\bfRN ', 'FontSize', 14, 'Color', 'k');

plot([0.0, 0.95], [0.62, 0.62], 'k-', 'LineWidth', 1);

text(0.05, 0.49, 'Punteggio Medio', 'FontSize', 14);
text(0.45, 0.49, sprintf('%.2f', J_tot_MPC_medio), 'FontSize', 14, 'Color', 'k');
text(0.75, 0.49, sprintf('%.2f', J_tot_AI_medio), 'FontSize', 14, 'Color', 'k');

plot([0.0, 0.95], [0.36, 0.36], 'k-', 'LineWidth', 1);

text(0.05, 0.23, 'RMSE Laterale', 'FontSize', 14);
text(0.45, 0.23, sprintf('%.2f m', RMSE_Y_MPC), 'FontSize', 14, 'Color', 'k');
text(0.75, 0.23, sprintf('%.2f m', RMSE_Y_AI), 'FontSize', 14, 'Color', 'k');

plot([0.0, 0.95], [0.10, 0.10], 'k-', 'LineWidth', 1.5);

diff_perc = ((J_tot_AI_medio - J_tot_MPC_medio) / J_tot_MPC_medio) * 100;

if diff_perc > 0
    testo_dev = sprintf('\\bfDeviazione RN da Ottimo: \\color{red}+%.2f %%', diff_perc);
else
    testo_dev = sprintf('\\bfDeviazione RN da Ottimo: \\color[rgb]{0 0.5 0}%.2f %%', diff_perc);
end

text(0.05, -0.05, testo_dev, 'FontSize', 14, 'Color', 'k');

% --- SUBPLOT 3 ---

subplot(2, 2, 3); hold on; grid on;
b_lon = bar([1, 2], [J_lon_MPC_medio, J_lon_AI_medio]);

dark_red = [0.6 0 0]; 

b_lon.FaceColor = 'flat';
b_lon.EdgeColor = 'flat'; 
b_lon.LineWidth = 1.5;    
b_lon.FaceAlpha = 0.75;    

b_lon.CData(1,:) = [0 0 0];  
b_lon.CData(2,:) = dark_red; 

set(gca, 'XTick', [1, 2], 'XTickLabel', {'MPC', 'RN'});
ylabel('Costo Medio'); 
title('Costo LONGITUDINALE');

% --- SUBPLOT 4 ---

subplot(2, 2, 4); hold on; grid on;
b_lat = bar([1, 2], [J_lat_MPC_medio, J_lat_AI_medio]);

b_lat.FaceColor = 'flat';
b_lat.EdgeColor = 'flat'; 
b_lat.LineWidth = 1.5;
b_lat.FaceAlpha = 0.75;

b_lat.CData(1,:) = [0 0 0];  
b_lat.CData(2,:) = dark_red; 

set(gca, 'XTick', [1, 2], 'XTickLabel', {'MPC', 'RN'});
ylabel('Costo Medio per Step'); 
title('Costo LATERALE');

exportgraphics(f_kpi, 'Analisi_Indici_Costo.pdf');
disp('Dashboard Costi aggiornata e salvata!');
%% 11. ANALISI DELLA DISTANZA DI SICUREZZA (CLEARANCE)
disp('Generazione Grafico Distanza di Sicurezza (Clearance)...');

% 1. Calcolo Vettori Distanza Istante per Istante
N_mpc = length(U_MPC);
Dist_Log_MPC = zeros(1, N_mpc);
for i = 1:N_mpc
    Dist_Log_MPC(i) = GetMinDistance2D(Car_X_log(i), Y_MPC(i), Walls);
end

N_ai = length(U_AI(1,:));
Dist_Log_AI = zeros(1, N_ai);
for i = 1:N_ai
    Dist_Log_AI(i) = GetMinDistance2D(Car_X_log_AI(i), Y_AI(i), Walls);
end

f_clearance = figure('Name', 'Analisi Clearance (Safety)', 'Color', 'w', 'Position', [200 200 1200 400]);
hold on; grid on;

plot(Car_X_log(1:N_mpc), Dist_Log_MPC, 'k--', 'LineWidth', 2, 'DisplayName', 'Clearance MPC');
plot(Car_X_log_AI(1:N_ai), Dist_Log_AI, 'r-', 'LineWidth', 2, 'DisplayName', 'Clearance Rete Neurale');

% 3. Linee di Sicurezza
yline(1.9, '--', 'Margine Ottimizzatore (1.9m)', 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'LabelHorizontalAlignment', 'left', 'FontSize', 11, 'HandleVisibility', 'off');
yline(1.0, '-', '\bfCOLLISIONE FISICA (1.0m)', 'Color', [0.8 0 0], 'LineWidth', 2, 'LabelHorizontalAlignment', 'left', 'FontSize', 11, 'HandleVisibility', 'off');

for w = 1:length(Walls)
    x_start = Walls(w).x_pos;
    x_end = Walls(w).x_pos + Walls(w).width;
    patch([x_start x_end x_end x_start], [0 0 1 1], [0.4 0.4 0.4], 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    text(x_start + Walls(w).width/2, 0.5, sprintf('Muro %d', w), 'HorizontalAlignment', 'center', 'FontSize', 9);
end

xlabel('Posizione Longitudinale X [m]', 'FontSize', 12);
ylabel('Distanza dal Muro [m]', 'FontSize', 12);
title('Profilo di Avvicinamento agli Ostacoli (Safety Check)', 'FontSize', 14);
legend('Location', 'northeast', 'FontSize', 11);

ylim([0 6]); 
xlim([0 road_len]);
ax = gca; ax.YMinorGrid = 'on';

exportgraphics(f_clearance, 'Analisi_Distanza_Sicurezza.pdf');
disp('Grafico Clearance generato!');


%% 12. CONFRONTO DEGLI INGRESSI DI CONTROLLO (STERZO E ACCELERAZIONE)
disp('Generazione Grafico Ingressi di Controllo (Attuatori)...');

f_inputs = figure('Name', 'Analisi Attuatori: Sterzo e Accelerazione', 'Color', 'w', 'Position', [250 250 1200 650]);

% 1. Ricostruzione dell'accelerazione MPC dai log di velocità
A_MPC = (V_x_log(2:N_mpc+1) - V_x_log(1:N_mpc)) / T_sampling;
lim_acc_max =  0.4 * mu_friction * 9.81;
lim_acc_min = -0.8 * mu_friction * 9.81;
A_MPC = max(min(A_MPC, lim_acc_max), lim_acc_min);

time_mpc = (0:N_mpc-1) * T_sampling;
time_ai = (0:N_ai-1) * T_sampling;
time_lim = min(time_mpc(end), time_ai(end));

% --- SUBPLOT 1: Comandi di Sterzo ---
subplot(2, 1, 1); hold on; grid on;
plot(time_mpc, U_MPC(1:N_mpc), 'k--', 'LineWidth', 2, 'DisplayName', 'Volante MPC');
plot(time_ai, U_AI(1, 1:N_ai), 'r-', 'LineWidth', 2, 'DisplayName', 'Volante Rete Neurale');
yline(umax, 'k--', 'Max Sterzo', 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
yline(umin, 'k--', 'Max Sterzo', 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
for w = 1:length(Walls)
    t_muro = Walls(w).x_pos / v_x_target; if t_muro < time_lim, xline(t_muro, 'k:', 'HandleVisibility', 'off'); end
end
xlabel('Tempo [s]', 'FontSize', 11); ylabel('Angolo Sterzo \delta [rad]', 'FontSize', 11);
title('Input 1: Dinamica del Volante (Angolo di Sterzo)', 'FontSize', 13);
legend('Location', 'southeast'); xlim([0 time_lim]); ylim([umin-0.1 umax+0.1]);

% --- SUBPLOT 2: Comandi Longitudinali  ---
subplot(2, 1, 2); hold on; grid on;
plot(time_mpc, A_MPC, 'k--', 'LineWidth', 2, 'DisplayName', 'Pedali MPC');
plot(time_ai, U_AI(2, 1:N_ai), 'r-', 'LineWidth', 2, 'DisplayName', 'Pedali Rete Neurale');
yline(lim_acc_max, 'k--', 'Max Acceleratore (Fisico)', 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
yline(lim_acc_min, 'k--', 'Max Freno (Fisico)', 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
for w = 1:length(Walls)
    t_muro = Walls(w).x_pos / v_x_target; if t_muro < time_lim, xline(t_muro, 'k:', 'HandleVisibility', 'off'); end
end
xlabel('Tempo [s]', 'FontSize', 11); ylabel('Accelerazione a_x [m/s^2]', 'FontSize', 11);
title('Input 2: Dinamica dei Pedali (Accelerazione / Frenata)', 'FontSize', 13);
legend('Location', 'southeast'); xlim([0 time_lim]); ylim([lim_acc_min-1 lim_acc_max+1]);

exportgraphics(f_inputs, 'Confronto_Ingressi_Controllo.pdf');
disp('Grafico degli Attuatori generato!');


