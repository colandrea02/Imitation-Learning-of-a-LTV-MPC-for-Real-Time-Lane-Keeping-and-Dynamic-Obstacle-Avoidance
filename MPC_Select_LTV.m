% === MPC LTV LATERALE (Sincronizzato Spazialmente) ===
function [X, U, Y, V_x_log, Car_X_log, Time_MPC_log] = MPC_Select_LTV(m_car, I, a, b, C_alpha_f, C_alpha_r, mu, C, D, N_horizon, T_sampling, T_duration, rho, x_0, v_x_start, v_x_target, umin, umax, xmin, xmax, Walls)
    N_step = floor(T_duration / T_sampling);
    
    X = zeros(5, N_step+1); X(:,1) = x_0;
    U = zeros(1, N_step);
    Y = zeros(1, N_step+1); Y(:,1) = C*x_0;
    V_x_log = zeros(1, N_step+1); V_x_log(1) = v_x_start;
    Car_X_log = zeros(1, N_step+1); Car_X_log(1) = 0;
    Time_MPC_log = zeros(1, N_step);
    
    options = optimoptions('quadprog','Display','off');
    prev_decision = 0; 
    margin = 1.9;
    road_len = 300;
    
    a_min = -0.8 * mu * 9.81; % Frena in base all'attrito
    a_max =  0.4 * mu * 9.81; 
    
    for i = 1:N_step 
        t_start_mpc = tic;

        % 1. CERVELLO LONGITUDINALE (Frenata Dinamica Intelligente)
        car_x = Car_X_log(i);
        car_y = X(4, i); % Posizione laterale attuale
        
        dist_obs = 100;
        wall_idx = -1;
        
        % Troviamo il prossimo ostacolo
        for w = 1:length(Walls)
            if Walls(w).x_pos + Walls(w).width > car_x
                dist_obs = Walls(w).x_pos - car_x; 
                wall_idx = w;
                break;
            end
        end
        
        % Se c'è un ostacolo vicino e rilevante, calcoliamo la velocità ottimale
        if dist_obs < 80 && dist_obs > 0 && wall_idx > 0
            
            % Dati dell'ostacolo
            w_y = Walls(wall_idx).y_close;
            w_h = Walls(wall_idx).h_block;
            
            % Ingombro del muro con margine
            y_sup = w_y + w_h/2 + margin;
            y_inf = w_y - w_h/2 - margin;
            
            % Calcoliamo lo spostamento laterale richiesto (dy)
            dy_sopra = abs(car_y - y_sup);
            dy_sotto = abs(car_y - y_inf);
            dy_min = min(dy_sopra, dy_sotto);
            
            % Evitiamo divisioni per zero
            dy_min = max(dy_min, 0.5); 
            
            % FORMULA MAGICA: V_safe = dist * sqrt( (mu * g * k) / dy )
            % k = 0.3 è un fattore di sicurezza per non slittare
            v_safe = dist_obs * sqrt((mu * 9.81 * 0.3) / dy_min);
            
            % Saturiamo la velocità tra un minimo di 5 m/s e il massimo
            v_target = max(min(v_safe, v_x_target), 5.0); 
        else
            % Nessun ostacolo: torna alla velocità di crociera
            v_target = v_x_target; 
        end
        
        % L'MPC Longitudinale calcola l'accelerazione per raggiungere v_target
        [a_x, v_next] = MPC_Longitudinal(V_x_log(i), v_target, T_sampling, N_horizon, a_min, a_max);
        
        V_x_log(i+1) = max(v_next, 0.1); 
        Car_X_log(i+1) = Car_X_log(i) + V_x_log(i) * T_sampling; 


        % 2. FISICA ADATTIVA LTV
        v_curr = V_x_log(i);
        Caf_eff = C_alpha_f * mu; Car_eff = C_alpha_r * mu;
        
        v_matrix = max(v_curr, 0.1); % <-- AGGIUNGI QUESTA RIGA (Protezione Matrice)
        
        % SOSTITUISCI 'v_curr' con 'v_matrix' SOLO NELLA MATRICE A
        A_lat_cont = [ -(Caf_eff+Car_eff)/(m_car*v_matrix),  -v_matrix + (Car_eff*b - Caf_eff*a)/(m_car*v_matrix), 0, 0, Caf_eff/m_car;
                       (Car_eff*b - Caf_eff*a)/(I*v_matrix), -(Caf_eff*a^2 + Car_eff*b^2)/(I*v_matrix),         0, 0, Caf_eff*a/I;
                       0, 1, 0, 0, 0; 1, 0, v_matrix, 0, 0; 0, 0, 0, 0, 0];
        B_lat_cont  = [0; 0; 0; 0; 1]; 
        sys_d = c2d(ss(A_lat_cont, B_lat_cont, C, D), T_sampling);
        A_dyn = sys_d.A; B_dyn = sys_d.B;
        [n, m] = size(B_dyn);
        
        % 3. RICALCOLO PREDIZIONI
        Q = C'*C;
        R = rho;
        G = zeros(n*N_horizon, m*N_horizon);
        for k_hor = 1:N_horizon
            for j = 1:k_hor, G((k_hor-1)*n+1:k_hor*n, (j-1)*m+1:j*m) = A_dyn^(k_hor-j)*B_dyn; end
        end
        H = zeros(n*N_horizon, n);
        for k_hor = 1:N_horizon, H((k_hor-1)*n+1:k_hor*n, :) = A_dyn^k_hor; end
        Q_bar = kron(eye(N_horizon), Q);
        R_bar = R * eye(N_horizon);
        M = G' * Q_bar * G + R_bar; 
        M = blkdiag(M, 1e6); 
        M = (M + M') / 2;
        LB = [repmat(umin, N_horizon, 1); 0]; 
        UB = [repmat(umax, N_horizon, 1); inf];
        
        % 4. CERVELLO LATERALE
        F_base = []; V_base = [];
        idx = find(~isinf(xmax));
        for k_hor=1:N_horizon
            r_st=(k_hor-1)*n+1; r_en=k_hor*n;
            G_k = G(r_st:r_en, :); H_k = H(r_st:r_en, :);
            F_base = [F_base; G_k(idx,:), -1; -G_k(idx,:), -1]; 
            V_base = [V_base; (xmax(idx)-margin)-H_k(idx,:)*X(:,i); -(xmin(idx)+margin)+H_k(idx,:)*X(:,i)];
        end
        
        Gradient = [G' * Q_bar * H * X(:,i); 0];
        
        % --- CHECK SPAZIALE OSTACOLI ---
        obstacle_detected = false;
        for k_hor = 1:N_horizon
            x_pred = Car_X_log(i) + v_curr * (k_hor * T_sampling); 
            for j = 1:length(Walls)
                if x_pred >= Walls(j).x_pos && x_pred <= Walls(j).x_pos + Walls(j).width
                    obstacle_detected = true; break;
                end
            end
            if obstacle_detected, break; end
        end
        
        if ~obstacle_detected
            [Z, ~, flag] = quadprog(M, 2*Gradient, F_base, V_base, [], [], LB, UB, [], options);
            best_Z = Z; best_flag = flag;
        else
            F_A = F_base; V_A = V_base;
            F_B = F_base; V_B = V_base;
            for k_hor = 1:N_horizon
                x_pred = Car_X_log(i) + v_curr * (k_hor * T_sampling); 
                for j = 1:length(Walls)
                    if x_pred >= (Walls(j).x_pos - margin) && x_pred <= (Walls(j).x_pos + Walls(j).width + margin)                         % Quando l'auto è DENTRO l'ostacolo, calcoliamo la posizione finale chiusa
                         curr_y = Walls(j).y_close;
                         y_max_dyn = curr_y + Walls(j).h_block / 2;
                         y_min_dyn = curr_y - Walls(j).h_block / 2;
                         
                         r_idx = (k_hor-1)*n + 4;
                         G_k = G(r_idx,:); H_k = H(r_idx,:);
                         
                         F_A = [F_A; -G_k, -1]; V_A = [V_A; -(y_max_dyn + margin) + H_k*X(:,i)];
                         F_B = [F_B; G_k, -1];  V_B = [V_B; (y_min_dyn - margin) - H_k*X(:,i)];
                    end
                end
            end
            
            [Z_A, cost_A, flag_A] = quadprog(M, 2*Gradient, F_A, V_A, [], [], LB, UB, [], options);
            [Z_B, cost_B, flag_B] = quadprog(M, 2*Gradient, F_B, V_B, [], [], LB, UB, [], options);
            
            % if prev_decision == 1, cost_A = cost_A * 0.9; end
            % if prev_decision == -1, cost_B = cost_B * 0.9; end
            
            if (flag_A > 0) && (flag_B > 0)
                if cost_A < cost_B, best_Z = Z_A; best_flag = flag_A; prev_decision = 1; 
                else, best_Z = Z_B; best_flag = flag_B; prev_decision = -1; end
            elseif (flag_A > 0), best_Z = Z_A; best_flag = flag_A; prev_decision = 1; 
            elseif (flag_B > 0), best_Z = Z_B; best_flag = flag_B; prev_decision = -1; 
            else
                if cost_A < cost_B, best_Z = Z_A; best_flag = flag_A;
                else, best_Z = Z_B; best_flag = flag_B; end
            end
        end
        
        if best_flag > 0 || ~isempty(best_Z), U(:,i) = best_Z(1);
        else, U(:,i) = 0; end
        
        Time_MPC_log(i) = toc(t_start_mpc);

        X(:,i+1) = A_dyn*X(:,i) + B_dyn*U(:,i);
        Y(:,i+1) = C*X(:,i+1);

        if Car_X_log(i+1) >= road_len + 10 % Se supera il traguardo di 10 metri
            % Taglia gli array eliminando gli zeri in eccesso
            X = X(:, 1:i+1);
            U = U(:, 1:i);
            Y = Y(:, 1:i+1);
            V_x_log = V_x_log(1:i+1);
            Car_X_log = Car_X_log(1:i+1);
            Time_MPC_log = Time_MPC_log(1:i);
            break; 
        end
    end
end