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

tau = 2; % make sure this always reflects the dataset

k_tors = 2;
b_tors = 0.05;
left_loc = -0.2;
right_loc = 0.2;

if (left_loc==0 && right_loc==0) || (left_loc==-100 && right_loc ==100)
    nox_bool = 1;
else
    nox_bool = 0;
end

p   = [mP mC l g k tau left_loc right_loc k_tors b_tors]';        % parameters


X_t   = load(fullfile(dataDir, 'tors_X_t_500001_cartpole_2_actuated_-0.2wall0.2_2_0.05.csv'));
U_t   = load(fullfile(dataDir, 'tors_U_t_500001_cartpole_2_actuated_-0.2wall0.2_2_0.05.csv'));
X_tp1 = load(fullfile(dataDir, 'tors_X_tp1_500001_cartpole_2_actuated_-0.2wall0.2_2_0.05.csv')); 

%%  BASELINE, ASSUMING NO OBSERVABLES

%A = X_tp1' * pinv(X_t');
% error from training data set
%error_vec = vecnorm((comparison_tp1' - A*comparison_t'), 2, 1); 
%MSE_baseline_error = mean(error_vec)

%%  CREATE/EVALUATE A KOOPMAN MODEL WITH FIXED PARAMETERS

n = 9;
eps = 1.1;

%koop_model = generate_koopman(X_t, X_tp1, n, eps, p, nox_bool);
koop_model = generate_koopman_DMDc(X_t, U_t, X_tp1, n, eps, p, nox_bool);
%MSE = evaluate_koopman_prediction(comparison_t, comparison_tp1, koop_model)

%error_reduction = (MSE_baseline_error-MSE)



%%  CREATE LQR GAINS BASED ON PREVIOUS KOOPMAN MODEL;

A = koop_model.A;
B = koop_model.B;

Q_diag = zeros(n+5, 1);
Q_diag(1) = 100; % we want x = 0
Q_diag(2) = 100; % we want theta = 0
Q_diag(3) = 100; % we want xdot = 0
Q_diag(4) = 100; % we want thetadot = 0
Q = diag(Q_diag);

R = 1;

[K,S,P] = dlqr(A, B, Q, R);

koop_model.K = K;
koop_model.Q = Q;
koop_model.R = R;
koop_model.S = S;
koop_model.P = P;

mean_K  = mean(K)
max_K = max(K)

save(fullfile('koopman_models', ...
    ['feb_DMDc_tau_' num2str(tau) '_obs_' num2str(n) '_eps_' num2str(eps) '_' num2str(left_loc) '_wall_' num2str(right_loc) '_' num2str(k_tors) '_' num2str(b_tors) '.mat']), 'koop_model');

%save(['obs_' num2str(n) '_eps_' num2str(eps) '.mat'], 'koop_model');


%%  TEST A RANGE OF EPSILON VALUES AND NUMBER OF OBSERVABLES FOR PREDICTION ACCURACY 

%{
n_values = 2:1:30;        % range of n
eps_values = 0.1:1:20.1;     % range of eps

MSE_best = 1000;
MSE_results = zeros(length(n_values), length(eps_values));  % storage

for i = 1:length(n_values)
    n = n_values(i);

    for j = 1:length(eps_values)
        eps = eps_values(j);

        fprintf('Currently on n = %d, epsilon = %.2f\n', n, eps)

        koop_model = generate_koopman(X_t, X_tp1, n, eps);
        MSE = evaluate_koopman_prediction(comparison_t, comparison_tp1, koop_model);

        % store result
        MSE_results(i,j) = MSE;

        % check for best
        if (i == 1 && j == 1) || (MSE < MSE_best)
            best_model = koop_model;
            MSE_best = MSE;
            best_epsilon = eps;
            best_n = n;
        end
    end
end

%}



