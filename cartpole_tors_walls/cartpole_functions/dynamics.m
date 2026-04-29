function [dz, u, flag] = dynamics(z, p, controller_type, varargin)

    if nargin > 3
        koop_model = varargin{1};
        % Use Koopman_model if provided
    else
        koop_model = [];
    end

    % Get mass matrix
    A = A_cartpole(z,p);
    
    % Get forces
    %u = [ 0 -5*z(4)]';
    b = b_cartpole(z, p);
    lambda = contact_force(z, p);

    forces = [-lambda(1); 0] + [lambda(2); 0];
    QF_P = jacobian_P(z,p)' * forces;

    flag = 0;

    if  max(max(abs(forces))) > 10e4 %check here
        flag = 1;
    end

    if controller_type == 1
        % STABILIZING CONTROLLER

        fprintf("THIS IS NOT COMPATIBLE FOR THE ACTUATOR PREFILTER MODEL")
        K = [3.6931  -46.7030    3.3885   -5.7147   0];
        L = [-13.9797   13.9767];
    
        u = K*z + L*lambda;

    
    elseif controller_type == 2
        % KOOPMAN LQR CONTROLLER
        
        if isempty(koop_model)
            fprintf('ERROR: Missing koopman model \n')
        end

        K = koop_model.K;
        centers = koop_model.centers;
        epsilon = koop_model.epsilon;
        nox_bool = koop_model.nox_bool;

        if nox_bool == 1
            z_lifted = lift(centers, epsilon, z(2:end)')';        
            u = -K*[z(1); z_lifted];
        else
            z_lifted = lift(centers, epsilon, z')';
            u = -K*z_lifted;
        end
    
    elseif controller_type == 3
        % random control input
        
        u = (10)*(rand(1)-0.5);
    

    elseif controller_type == 0
        % NO CONTROL INPUT
        u = zeros(1,1);
    end

    qdd = A\(b + QF_P + [z(5); 0] + [0; -p(10)*z(4)]);
    %qdd = qdd + [1/mC; 1/(l*mC)]*u;
    tau = p(6);
    dF = u/tau - z(5)/tau; %prefilter time constant
    dz = 0*z;
    
    % Form dz
    dz(1:2) = z(3:4);
    dz(3:4) = qdd;
    dz(5) = dF;

    
end