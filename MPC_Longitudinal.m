% === MPC LONGITUDINALE ===
function [a_x, v_next] = MPC_Longitudinal(v_curr, v_target, T_s, N_hor, a_min, a_max)
    G_lon = tril(ones(N_hor)) * T_s;
    H_lon = ones(N_hor, 1);
    Q = 5 * eye(N_hor);
    R = 5 * eye(N_hor);  
    M = G_lon' * Q * G_lon + R; 
    M = (M + M') / 2; 
    V_ref = v_target * ones(N_hor, 1);
    Gradient = G_lon' * Q * (H_lon * v_curr - V_ref);
    LB = a_min * ones(N_hor, 1); 
    UB = a_max * ones(N_hor, 1);
    options = optimoptions('quadprog','Display','off');
    [U, ~, flag] = quadprog(M, Gradient, [], [], [], [], LB, UB, [], options);
    if flag > 0 && ~isempty(U), a_x = U(1); else, a_x = 0; end
    v_next = v_curr + a_x * T_s;
end