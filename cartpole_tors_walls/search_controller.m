clear
clc

baseDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(baseDir, 'data');

addpath(genpath(dataDir));

dt = 0.01;
mP = 0.1;
mC = 1;
l = 0.5;
g = 9.81;
k = 10;

k_tors = 0;
b_tors = 0.05;
tau = 2;
left_loc = -0.2; % -100 turns off wall
right_loc = 0.2; % 100 turns of wall

if (left_loc==0 && right_loc==0) || (left_loc==-100 && right_loc ==100)
    nox_bool = 1; % no walls in use, no need for x data
else
    nox_bool = 0; % walls in use, x data is used
end

p   = [mP mC l g k tau left_loc right_loc k_tors b_tors]';          % parameters


% X_t = load('k50_tors_X_t_500001_cartpole_2_passive_-0.1wall0.1_2_0.05.csv');
% X_tp1 = load('k50_tors_X_tp1_500001_cartpole_2_passive_-0.1wall0.1_2_0.05.csv'); % assuming passive data

%X_t = load('tors_X_t_500001_cartpole_2_actuated_-0.2wall0.2_2_0.05.csv');
%U_t = load('tors_U_t_500001_cartpole_2_actuated_-0.2wall0.2_2_0.05.csv');
%X_tp1 = load('tors_X_tp1_500001_cartpole_2_actuated_-0.2wall0.2_2_0.05.csv'); 

X_t   = load(fullfile(dataDir, 'tors_X_t_500001_cartpole_2_actuated_-0.2wall0.2_2_0.05.csv'));
U_t   = load(fullfile(dataDir, 'tors_U_t_500001_cartpole_2_actuated_-0.2wall0.2_2_0.05.csv'));
X_tp1 = load(fullfile(dataDir, 'tors_X_tp1_500001_cartpole_2_actuated_-0.2wall0.2_2_0.05.csv')); 

x_weight = 100;
th_weight = 100;
xdot_weight = 100;
thdot_weight = 100;

R = 1; 

%%  TEST A RANGE OF EPSILON VALUES AND NUMBER OF OBSERVABLES FOR CONTROLLER PERFORMANCE

n_values = 2:1:30;        % range of n
eps_values = 0.1:1:20.1;     % range of epsilons

loss_fn_results = zeros(length(n_values), length(eps_values)); 

num_best = 5;
best_losses = inf(1,num_best);
best_models = cell(1,num_best);

for i = 1:length(n_values)
    n = n_values(i);

    for j = 1:length(eps_values)
        eps = eps_values(j);

        fprintf('Currently on n = %d, epsilon = %.2f\n', n, eps)

        Q_diag =  zeros(n+5, 1);
        Q_diag(1) = x_weight; 
        Q_diag(2) = th_weight; 
        Q_diag(3) = xdot_weight; 
        Q_diag(4) = thdot_weight; 
        Q = diag(Q_diag);
        
        %koop_model = generate_koopman(X_t, X_tp1, n, eps, p, nox_bool);
        koop_model = generate_koopman_DMDc(X_t, U_t, X_tp1, n, eps, p, nox_bool);
        [koop_model, flag] = generate_lqr(koop_model, Q, R);

        if flag == 1
            z_out_0 = simulate_cartpole_controller(koop_model, p, 0);
            z_out_1 = simulate_cartpole_controller(koop_model, p, 1);
            z_out_2 = simulate_cartpole_controller(koop_model, p, 2);

            z_total = [z_out_0, z_out_1, z_out_2];
            mean_vals = mean(abs(z_total), 2); % MEAN ABSOLUTE VALUES!!
            mean_vals(isnan(mean_vals)) = 1000; % replace NaNs with 1000

        else
            mean_vals = [1000, 1000, 1000, 1000];
        end
                          
        weighted_mean = [50*mean_vals(1), 30*mean_vals(2), 1*mean_vals(3), 1*mean_vals(4)];
        loss_fn = norm(weighted_mean);
        loss_fn_results(i,j) = loss_fn;

        % check for best
        [worst_loss, worst_idx] = max(best_losses);
        if loss_fn < worst_loss
            koop_model.m_x  = mean_vals(1);
            koop_model.m_th = mean_vals(2);
            koop_model.m_xd = mean_vals(3);
            koop_model.m_thd = mean_vals(4);

            best_losses(worst_idx) = loss_fn;
            best_models{worst_idx} = koop_model;
        end

    end
end

[best_losses, sort_idx] = sort(best_losses);
best_models = best_models(sort_idx);

koop_model = best_models{1,1};
save(fullfile('koopman_models', ...
    ['bestDMDc_tau_' num2str(tau) '_obs_' num2str(length(koop_model.A)-5) '_eps_' num2str(koop_model.epsilon) '_' num2str(left_loc) '_wall_' num2str(right_loc) '_' num2str(k_tors) '_' num2str(b_tors) '.mat']), 'koop_model');