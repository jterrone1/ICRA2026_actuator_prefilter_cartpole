clear
clc

baseDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(baseDir, 'cartpole_functions');

addpath(genpath(dataDir));
% set up data generation parameters
N = 500001; % number of trajectories (around 2000)
dt = 0.01; %IMPORTANT PARAMETER

mP = 0.1;
mC = 1;
l = 0.5;
g = 9.81;
k = 10;
k_tors = 0;
b_tors = 0.05;

tau = 2;

left_loc = -0.2;
right_loc = 0.2;
p   = [mP mC l g k tau left_loc right_loc k_tors b_tors]';        % parameters

if (left_loc==0 && right_loc==0) || (left_loc==-100 && right_loc ==100)
    nox_bool = 1; % no walls in use, no need for x data
    x0_upper = 0;
    x0_lower = 0;
else
    nox_bool = 0; % walls in use
    if left_loc ~= -100 
        x0_lower = left_loc;% try to get away from walls
    else
        x0_lower = -1;
    end

    if right_loc ~= 100
        x0_upper = right_loc;
    else
        x0_upper = 1;
    end

end

X_t = zeros(N, 5);
X_tp1 = zeros(N, 5);
u = zeros(N, 1);

X_t_act = zeros(N, 5);
X_tp1_act = zeros(N, 5);
u_act = zeros(N, 1);

for n = 1:1:N
    %z0 = [0; rand_num(-pi/3, pi/3, 1); rand_num(-1, 1, 1); rand_num(-1, 1, 1); rand_num(-10, 10, 1)]
    z0 = [rand_num(x0_lower, x0_upper, 1); rand_num(-pi/3, pi/3, 1); rand_num(-1, 1, 1); rand_num(-1, 1, 1); rand_num(-50, 50, 1)];
    [z_out, z_out_act, u_out] = simulate_cartpole_data_single(p, z0, dt);

    %z0_save = [z0_save, z0];

    X_t(n, :) = z_out(:, 1)';
    X_tp1(n, :) = z_out(:, 2)';
    u(n,:) = 0;

    X_t_act(n, :) = z_out_act(:, 1)';
    X_tp1_act(n, :) = z_out_act(:, 2)';
    u_act(n,:) = u_out;

end

X_t = [X_t; X_t_act];
X_tp1 = [X_tp1; X_tp1_act];
u = [u; u_act];

scatter(X_tp1(:,1), X_tp1(:,2))
xlabel("x-pos")
ylabel("theta")


writematrix(X_t,"X_t_" + N + "_cartpole_" + dt+"dt.csv") 
writematrix(X_tp1_act,"X_tp1_" + N + "_cartpole_" + dt+"dt.csv") 
writematrix(u_t, "inputs_" + N + "_cartpole_" + dt+"dt.csv") 

%% FUNctions

function [z_out, z_out_act, u]  = simulate_cartpole_data_single(p, z0, dt)


    %% Perform Dynamic simulation    
    z_out = zeros(5,2);
    z_out_act = zeros(5,2);
    z_out(:,1) = z0;
    z_out_act(:,1) = z0;

    [dz_pass, ~, ~] = dynamics(z_out(:,1), p, 0); %passive
    [dz, u, ~] = dynamics(z_out(:,1), p, 3); %actuated

    z_out(:,2) = z_out(:,1) + dz_pass*dt;
    z_out_act(:,2) = z_out(:,1) + dz*dt;
    %z_out(1:6,i+1) = z_out(1:6,i) + z_out(7:end,i+1)*dt;

    
end


function r = rand_num(a, b, n)
    r = a + (b-a)*rand(n,1);
end

