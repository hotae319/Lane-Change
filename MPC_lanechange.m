function [feas, zOpt, uOpt, zpred] = MPC_lanechange(P, x0, M, N)
%% Kinematic Bicycle Model & Lane Setting
% Parameter setting
lr = 1; lf = 1;
dt = 0.2; % sampling time


% State and Input
% z = (x, y, v, phi), u = [acc, beta]
nz = 4; nu = 2; 
z0 = [1,0,1,0];
fdyn = @(z,u) [z(3)*cos(z(4)+u(2)); z(3)*sin(z(4)+u(2)); u(1); z(3)/lr*sin(u(2))];
znext = @(z,u) z + fdyn(z,u)*dt; %Discrete time

% Lane width 
x1 = -2; x2 = 0; x3 = 2;
x_goal = (x1+x2)/2;

%% MPC Problem Set up
% Constraints

d_safe = 0.3;       % distance to avoid collision
vbar = 2; vlim = 5; % Limits
alim = 1;           % acc limit
betalim = 0.7;      % wheel angle limit

% Terminal constraint
eps = 1;            % termianl safe distance

% Objective Function
% stage cost : (x-xgoal)'*Q*(X-xgoal)+u'*R*u+phi'*P*phi
% terminal cost : xN'*PN*xN 
R = eye(nu); Q = 1; P = 1; PN = 1;

% MPC Horizon
N = 4;
% MPC Solve setup
M = 25;
zsim = zeros(nz, M+1);
zsim(:,1) = z0;
usim = zeros(nu, M);
feas = false([1,M]);

% MPC solve with measurement of target vehicle's state
for t = 1:M
    fprintf('Solving simstep: %i\n',t)
    [feas(t), xOpt, uOpt, Jopt] = solve_cftoc(P, PN, Q, R, N, z0, xL, xU, uL, uU, bf, Af);
    if ~feas(t)
        warning('Infeasible');
        return;
    end      
    usim(:,t) = uOpt(1);
    zsim(:,t+1) = znext(zsim(:,t),usim(:,t));
end


% only for test to pull github

end
