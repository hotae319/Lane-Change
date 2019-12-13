function [feas, zOpt, uOpt, ztar_pred] = solve_cftoc(P, PN, Q, R, N, z0, zL, zU, uL, uU, bf, Af, safe_param, zgoal, ztar)

yalmip('clear')
% mpc paramter
nz = 4; nu = 2;
dt = 0.1;

z = sdpvar(nz,N+1);
u = sdpvar(nu,N);
ztar_pred = zeros(2*nz,N+1); %target 2 
ztar_pred(:,1) = ztar; % match initial state with measurement state 
feas = false;

constr = [z(:,1) == z0];

% Terminal Constraint
if isempty(Af)
    constr = [constr z(:,N+1)==bf];
else
    constr = [constr Af*z(:,N+1)<=bf];
end

% 
cost = (z(1,N+1)-zgoal)*PN*(z(1,N+1)-zgoal); 

for k = 1:N
    % potential function
    gap = z(1:2,k)-ztar_pred(1:2,k);
    gap2 = z(1:2,k)-ztar_pred(1+nz:2+nz,k);
    
    %potential = gap(1,1)^2+gap(2,1)^2 + gap2(1,1)^2+gap2(2,1)^2;
    % cost at each time step
    cost = cost + (z(1,k)-zgoal)*Q*(z(1,k)-zgoal)  + z(4,k)*P*z(4,k) + u(:,k).'*R*u(:,k); 
    % constraint at each time step
    constr = constr + [uL <= u(:,k), u(:,k) <= uU, zL <= z(:,k), z(:,k)<=zU];
    % state equation (transition)
    constr = constr + [z(:,k+1) == ego_vehicle(z(:,k),u(:,k))];
    % time-varying constraint for safety --> using safe_param (d_safe;eps)    
    constr = constr + [gap(1,1)^2+gap(2,1)^2>= safe_param(1)];
    % prediction of ztar in the loop of mpc
    ztar_pred(2,k+1) = ztar_pred(2,k) + ztar(3)*dt; 
    ztar_pred(1,k+1) = ztar_pred(1,k);
    ztar_pred(3:4,k+1) = ztar_pred(3:4,k);   
    % target 2
    constr = constr + [gap2(1,1)^2+gap2(2,1)^2>= safe_param(1)];
    ztar_pred(2+nz,k+1) = ztar_pred(2+nz,k) + ztar(3+nz)*dt; 
    ztar_pred(1+nz,k+1) = ztar_pred(1+nz,k);
    ztar_pred(3+nz:4+nz,k+1) = ztar_pred(3+nz:4+nz,k);
end
% constr = constr + [ztar_pred(2,end)<=z(2,end)-safe_param(2)]; 

options = sdpsettings('verbose',0,'solver','fmincon');

sol = optimize(constr,cost,options);
%sol = optimize(constr);
if sol.problem == 0
    feas = true;
elseif sol.problem == 3
    disp(sol.problem);    
    check(constr)
else
    zOpt = [];
    uOpt = [];
    disp(sol.problem)    
    check(constr)

    disp(double(z))
    disp(double(u))
    return;
end

zOpt = double(z);
uOpt = double(u);
end
