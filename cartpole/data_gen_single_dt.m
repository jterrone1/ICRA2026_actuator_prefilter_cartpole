clear
clc

addpath('C:\Users\jasmi\OneDrive\Desktop\rimless_matlab\cartpole_ICRA\cartpole_functions\');


% set up data generation parameters
N = 500001; % number of trajectories (around 2000)
dt = 0.01; %IMPORTANT PARAMETER

mP = 0.1;
mC = 1;
l = 0.5;
g = 9.81;
k = 10;

tau = 5;
left_loc = -0.1;
right_loc = 0.1;
p   = [mP mC l g k tau left_loc right_loc]';        % parameters

if (left_loc==0 && right_loc==0) || (left_loc==-100 && right_loc ==100)
    nox_bool = 1; % no walls in use, no need for x data
    x0_upper = 0;
    x0_lower = 0;
else
    nox_bool = 0; % walls in use
    if left_loc ~= -100 
        x0_lower = left_loc;
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


for n = 1:1:N
    %z0 = [0; rand_num(-pi/3, pi/3, 1); rand_num(-1, 1, 1); rand_num(-1, 1, 1); rand_num(-10, 10, 1)]
    z0 = [rand_num(x0_lower, x0_upper, 1); rand_num(-pi/3, pi/3, 1); rand_num(-1, 1, 1); rand_num(-1, 1, 1); rand_num(-10, 10, 1)];
    z_out = simulate_cartpole_data_single(p, z0, dt);

    %z0_save = [z0_save, z0];

    X_t(n, :) = z_out(:, 1)';

    X_tp1(n, :) = z_out(:, 2)';

    if mod(n, floor(N/100)) == 0
        fprintf('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        fprintf('%d%% completed \n', 100*n/N)
    end

end

scatter(X_tp1(:,1), X_tp1(:,2))
xlabel("x-pos")
ylabel("theta")


%{
writematrix(X_t,"X_t_" + N + "_pass_tb_LA36_" + dt+"dt.csv") 
writematrix(X_tp1_pass,"X_tp1_" + N + "_pass_tb_LA36_" + dt+"dt.csv") 

writematrix(X_t,"X_t_" + N + "_act_tb_LA36_" + dt+"dt.csv") 
writematrix(X_tp1_act,"X_tp1_" + N + "_act_tb_LA36_" + dt+"dt.csv") 
writematrix(u_t, "inputs_" + N + "_act_tb_LA36_" + dt+"dt.csv") 
%}


writematrix(X_t,"X_t_" + N + "_cartpole_" + tau+"_passive_"+left_loc + 'wall' + right_loc+".csv") 
writematrix(X_tp1,"X_tp1_" + N + "_cartpole_" + tau+"_passive_"+left_loc + 'wall' + right_loc+".csv") 

%writematrix(X_t,"X_t_" + N + "_cartpole_" + dt+"dt.csv") 
%writematrix(X_tp1_act,"X_tp1_" + N + "_cartpole_" + dt+"dt.csv") 
%writematrix(u_t, "inputs_" + N + "_cartpole_" + dt+"dt.csv") 

%% FUNctions

function z_out = simulate_cartpole_data_single(p, z0, dt)


    %% Perform Dynamic simulation    
    z_out = zeros(5,2);
    z_out(:,1) = z0;

    %[dz, u, flag] = dynamics(z_out(:,1), p, 1); %actuated
    [dz_pass, ~, ~] = dynamics(z_out(:,1), p, 0); %passive

    z_out(:,2) = z_out(:,1) + dz_pass*dt;
    %z_out(1:6,i+1) = z_out(1:6,i) + z_out(7:end,i+1)*dt;

    
end


function r = rand_num(a, b, n)
    r = a + (b-a)*rand(n,1);
end

