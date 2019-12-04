function [feas, zOpt, uOpt, zpred] = MPC_lanechange(P, x0, M, N)
%Kinematic Bicycle Model
lr = 1; lf = 1;
dt =0.2;
% z = (x, y, v, phi), u = [acc, beta]
fdyn = @(z,u) [z(3)*cos(z(4)+u(2)); z(3)*sin(z(4)+u(2)); u(1); z(3)/lr*sin(u(2))];

% Lane width 
x1 = -2; x2 = 0; x3 = 2;
x_goal = (x1+x2)/2;

% Parameters
vbar = 2; vlim = 5;

%Optimization problem set up
z = sdpvar(4, N+1);
u = sdpvar(2, N);

R = eye(2); Q = 1; PN = 1;
cost = 0; 
constraints = [z(1,N+1) <= x2, z(1,N+1)>=1];
for k = 1:N
    cost = cost + (z(1,k)-x_goal)^2 + z (3,k)*Q*z(3,k) + u(:,k).'*R.u(:,k); 
    % state
    constraints = constraints + [x1<= z(1,k), z(1,k)<=x3, abs(z(3,k)-vbar)<=vlim]; 
    
end 
cost = cost + z(1,N+1)*PN*z(1,N+1);




end
