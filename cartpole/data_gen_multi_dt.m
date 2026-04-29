clear
clc

addpath('C:\Users\jasmi\OneDrive\Desktop\rimless_matlab\cartpole_ICRA\cartpole_functions\');


% set up data generation parameters
N = 2000; % number of trajectories (around 2000)
dt = 0.01; %IMPORTANT PARAMETER
tf = 60*dt; % [MAX] duration of each trajectory (previously 100 * dt)

mP = 0.1;
mC = 1;
l = 0.5;
g = 9.81;
k = 10;
tau = 2;
left_loc = 0;
right_loc = 0;
p   = [mP mC l g k tau left_loc right_loc]';        % parameters

X_t = [];
X_tp1_act = [];
X_tp1_pass = [];
u_t = [];
z0_save = [];


for n = 1:1:N

    z0 = [0.2*(rand(1)-0.5); 0; 2*(rand(1)-0.5); 2*(rand(1)-0.5); rand(1)];
    [z_out, z_out_pass, u_out, flag] = simulate_cartpole_data_traj(p, z0, dt, tf);

    %z0_save = [z0_save, z0];

    X_t = [X_t; z_out(:, 1:end-1)'];
    u_t = [u_t; u_out'];
    
    X_tp1_act = [X_tp1_act; z_out(:, 2:end)'];
    X_tp1_pass = [X_tp1_pass; z_out_pass(:, 2:end)'];

    if mod(n, floor(N/100)) == 0
        fprintf('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        fprintf('%d%% completed \n', 100*n/N)
    end

end

%scatter(X_t(:,1), X_t(:,2))
%xlabel("x-pos")
%ylabel("theta")


%{
writematrix(X_t,"X_t_" + N + "_pass_tb_LA36_" + dt+"dt.csv") 
writematrix(X_tp1_pass,"X_tp1_" + N + "_pass_tb_LA36_" + dt+"dt.csv") 

writematrix(X_t,"X_t_" + N + "_act_tb_LA36_" + dt+"dt.csv") 
writematrix(X_tp1_act,"X_tp1_" + N + "_act_tb_LA36_" + dt+"dt.csv") 
writematrix(u_t, "inputs_" + N + "_act_tb_LA36_" + dt+"dt.csv") 
%}


writematrix(X_t,"X_t_" + N + "_cartpole_" + dt+"dt_comparison.csv") 
writematrix(X_tp1_pass,"X_tp1_" + N + "_cartpole_" + dt+"dt_comparison.csv") 

%writematrix(X_t,"X_t_" + N + "_cartpole_" + dt+"dt.csv") 
%writematrix(X_tp1_act,"X_tp1_" + N + "_cartpole_" + dt+"dt.csv") 
%writematrix(u_t, "inputs_" + N + "_cartpole_" + dt+"dt.csv") 

%% FUNctions

function [z_out, z_out_pass, u_out, flag] = simulate_cartpole_data_traj(p, z0, dt, tf)


    %% Perform Dynamic simulation    
    num_steps = floor(tf/dt);
    tspan = linspace(0, tf, num_steps); 
    z_out = zeros(5,num_steps);
    z_out(:,1) = z0;
    z_out_pass = zeros(5,num_steps);
    z_out_pass(:,1) = z0;
    u_out = zeros(1,num_steps-1);


    for i=1:num_steps-1
        [dz, u, flag] = dynamics(z_out(:,i), p, 1); %actuated
        [dz_pass, u_pass, flag_pass] = dynamics(z_out(:,i), p, 0); %passive

        if flag == 1 %check if forces went crazy 
            i;
            break
        end

        z_out(:,i+1) = z_out(:,i) + dz*dt;
        z_out_pass(:,i+1) = z_out(:,i) + dz_pass*dt;
        %z_out(1:6,i+1) = z_out(1:6,i) + z_out(7:end,i+1)*dt;

        u_out(:, i) = u;

        if max(abs(z_out(1,i+1))) > 1 % stop if flying away
            flag = 1;
            break
        end

    end
    
end


function r = rand_num(a, b, n)
    r = a + (b-a)*rand(n,1);
end

