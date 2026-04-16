%% SCRIPT PER LA RACCOLTA DATI (IMITATION LEARNING - END-TO-END)
clc; clear; close all;
warning('off','all');
disp('Inizio generazione dataset per il Machine Learning ...');

%% 1. COSTANTI E IMPOSTAZIONI 
g = 9.81; m_car = 2050; I = 3344;
a = 1.2; b = 1.6; C_alpha_f = 120000; C_alpha_r = 150000; 
T_sampling = 0.1;
road_len = 300;
T_duration = ceil(road_len / 8.0); 

N_horizon = 30; 
R_rho = 10;   
x_0 = [0 0 0 0 0]'; 
v_x_start = 0.1; % Velocità di partenza
umin = -0.6; umax = 0.6; 
xmax_bound = [inf; inf; inf;  8; inf]; 
xmin_bound = [-inf; -inf; -inf; -8; -inf];
C = [0 0 0 1 0]; D = 0;

%% 2. PREPARAZIONE MATRICI DATASET
N_simulations = 2000; % Numero di piste
N_step = floor(T_duration / T_sampling);

total_samples = N_simulations * N_step;

% INPUT CORRETTO: 11 features (5 Stati + V_x + 3 Info Ostacolo + Aderenza MU + V_x Target)
InputData = zeros(11, total_samples);  
% OUTPUT: 2 Azioni (Sterzo, Accelerazione)
OutputData = zeros(2, total_samples); 

sample_idx = 1; 

%% 3. LOOP DI SIMULAZIONE MASSIVA
tic; 
for sim = 1:N_simulations
    
    if mod(sim, 50) == 0
        fprintf('Esecuzione Simulazione %d di %d...\n', sim, N_simulations);
    end
    
    % Generazione Attrito (mu) e Muri casuali
    mu_rand = 0.4 + (1.0 - 0.4) * rand();
    % Genera una velocità target randomica tra 10 e 20
    v_x_target = randi([10, 20]);

    N_obstacles_desired = 5;
    Walls = GenerateRandomWalls(N_obstacles_desired, 40, road_len - 60);
    
    % Chiamata all'MPC Perfetto
    [X_sim, U_sim, Y_sim, V_x_log, Car_X_log] = MPC_Select_LTV(m_car, I, a, b, C_alpha_f, C_alpha_r, mu_rand, C, D, N_horizon, T_sampling, T_duration, R_rho, x_0, v_x_start, v_x_target, umin, umax, xmin_bound, xmax_bound, Walls);
    
    N_actual = size(U_sim, 2); 
    
    for i = 1:N_actual
        car_x = Car_X_log(i);
        
        dist_next_obs = 100; 
        y_upper = 8.0;       
        y_lower = -8.0;      
        
        for w = 1:length(Walls)
            if Walls(w).x_pos + Walls(w).width > car_x
                dist_next_obs = Walls(w).x_pos - car_x;
                half_h = Walls(w).h_block / 2;
                y_upper = Walls(w).y_close + half_h;
                y_lower = Walls(w).y_close - half_h;
                break; 
            end
        end
        
        % --- ESTRAZIONE DATI ---
        current_state = X_sim(:, i);
        current_vx = V_x_log(i); 
        
        % Calcolo dell'accelerazione usata in questo step dall'MPC Longitudinale
        % a = (V_futura - V_attuale) / tempo
        current_ax = (V_x_log(i+1) - V_x_log(i)) / T_sampling;
        
        % Salvataggio riga Input (10 valori, mu incluso!)
        InputData(:, sample_idx) = [current_state; current_vx; dist_next_obs; y_upper; y_lower; mu_rand; v_x_target];
        
        % Salvataggio riga Output (2 valori: Sterzo e Accelerazione)
        OutputData(:, sample_idx) = [U_sim(1, i); current_ax];
        
        sample_idx = sample_idx + 1;
    end
end

% Pulizia finale
InputData = InputData(:, 1:sample_idx-1);
OutputData = OutputData(:, 1:sample_idx-1);

time_elapsed = toc;
fprintf('\nDataset completato in %.1f secondi!\n', time_elapsed);
fprintf('Campioni finali salvati: %d\n', length(OutputData));

%% 4. SALVATAGGIO FILE .MAT
save('TrainingData.mat', 'InputData', 'OutputData');
disp('Dati salvati con successo in "TrainingData.mat" (11 Input, 2 Output)!');

