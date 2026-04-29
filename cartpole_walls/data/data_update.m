clear;
clc;

% Define fixed paramters

mP = 0.1;
mC = 1;
l = 0.5;
g = 9.81;
k = 10;

tau = 2; % make sure this always reflects the Koopman model
rng_val =  5;

left_loc = -0.2; % -100 turns off wall
right_loc = 0.2; % 100 turns of wall
d = abs(left_loc);

p   = [mP mC l g k tau left_loc right_loc]';        % parameters

X_t = load('testing2_X_t_4000_pass_cartpole0_0.01dt_02.csv');
u_t = load('testing2_inputs_4000_act_cartpole1_0.01dt_02.csv');

Fmax = 10;

full = height(X_t);
%new_Xt_data = [X_t, u_t];

half = height(X_t)/2;
u_random = -Fmax + 2*Fmax * rand(half, 1); % between [-10, 10]
new_Xt_data = [X_t, [u_t(1:half, :); u_random]];
new_Xtp1_data = [X_t, [u_t(1:half, :); u_random]];

 
dt = 0.01;

for i=1:full
    [dz, u, ~] = dynamics(new_Xt_data(i,:)', p, 0);
    new_Xtp1_data(i,:) = new_Xt_data(i,:) + dz'*dt;
end

writematrix(new_Xt_data,"2_X_t_" + 4000 + "_cartpole_walls_" + tau+"tau_half_F" + Fmax + ".csv") 
writematrix(new_Xtp1_data,"2_X_tp1_" + 4000 + "_cartpole_walls_" + tau+"tau_half_F" + Fmax + ".csv") 


%X_tp1 = load('testing_X_tp1_4000_act_cartpole1_0.01dt_02.csv'); % sanity check

