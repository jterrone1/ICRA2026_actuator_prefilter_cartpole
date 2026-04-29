clear
name = 'cartpole';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
% syms t th dth ddth phi1 dphi1 ddphi1 phi2 dphi2 ddphi2 x y dx dy ddx ddy real
syms t th dth ddth x dx ddx F dF ddF real
syms mP mC l g k tau left_loc right_loc real
syms u Fx Fy real

% Group them
q   = [x  ; th  ];%; phi1   ; phi2];      % generalized coordinates
dq  = [dx ; dth ];%; dphi1  ; dphi2];    % first time derivatives
ddq = [ddx; ddth ];%; ddphi1 ; ddphi2];  % second time derivatives
u   = [u];     % controls
F   = [Fx ; Fy];

p   = [mP mC l g k tau left_loc right_loc]';        % parameters

% Generate Vectors and Derivativess
xhat = [1; 0; 0];
yhat = [0; 1; 0];

zhat = cross(xhat,yhat);
ehat =  -sin(th)*xhat + cos(th)*yhat;

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives

rC = x*xhat;
drC = ddt(rC);

rP = rC + l * ehat;
drP = ddt(rP);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces


omega = dth;

T = (1/2)*mC*dot(drC, drC) + (1/2)* mP *dot(drP, drP);

Vg = mC*g*dot(rC, yhat) + mP*g*dot(rP, yhat);

%Q_tau1 = M2Q(tau1*khat,omega1*khat);
%Q_tau2 = M2Q(tau2*khat,omega2*khat); 
%Q_tau2R= M2Q(-tau2*khat,omega1*khat);


%Q_tau = Q_tau1+Q_tau2 + Q_tau2R;

%Q = Q_tau;
control = F2Q(u*xhat, rC);

% Assemble the array of cartesian coordinates of the key points
keypoints = [rC(1:2) rP(1:2)];
keypoints_vel = [drC(1:2) drP(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T+Vg;
L = T-Vg;
eom = ddt(jacobian(L,dq).') - jacobian(L,q).'; %- Q;

% Rearrange Equations of Motion
A = simplify(jacobian(eom,ddq));
b = simplify(A*ddq - eom);

% Equations of motion are
% eom = A *ddq + (coriolis term) + (gravitational term) - Q = 0
%Mass_Joint_Sp = A;
Grav = simplify(jacobian(Vg, q)');
Corr = simplify( eom  - Grav - A*ddq);

% Compute jacobian
JC = jacobian(rC, q);
JP = jacobian(rP,q);

% Compute ddt( J )
dJ= reshape( ddt(JP(:)) , size(JP) );

% Write Energy Function and Equations of Motion
z  = [q ; dq; F];


matlabFunction(A,'file',['A_' name],'vars',{z p});
matlabFunction(b,'file',['b_' name],'vars',{z p});
matlabFunction(keypoints,'file',['keypoints_' name],'vars',{z p});
matlabFunction(keypoints_vel,'file',['keypoints_vel_' name],'vars',{z p});

matlabFunction(JC(1:2,:) ,'file',['jacobian_C'],'vars',{z p});
matlabFunction(JP(1:2,:) ,'file',['jacobian_P'],'vars',{z p});


%matlabFunction(rE,'file',['position_foot'],'vars',{z p});
%matlabFunction(drE,'file',['velocity_foot'],'vars',{z p});
%matlabFunction(ddrE,'file',['acceleration_foot'],'vars',{z p});
%matlabFunction(J ,'file',['jacobian_foot'],'vars',{z p});

%matlabFunction(dJ ,'file',['jacobian_dot_foot'],'vars',{z p});
%{
matlabFunction(Grav_Joint_Sp ,'file', ['Grav_leg'] ,'vars',{z p});
matlabFunction(Corr_Joint_Sp ,'file', ['Corr_leg']     ,'vars',{z p});
matlabFunction(Lambda ,'file', ['Lambda'] ,'vars',{z p});
matlabFunction(Mu ,'file', ['Mu']     ,'vars',{z p});
matlabFunction(Rho ,'file', ['Rho'] ,'vars',{z p});
matlabFunction(keypoints,'file',['keypoints_' name],'vars',{z p});
%}

