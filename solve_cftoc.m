function [feas, xOpt, uOpt, Jopt] = solve_cftoc(P, PN, Q, R, N, x0, xL, xU, uL, uU, bf, Af)

yalmip('clear')
nx = size(A,2);
nu = size(B,2);
x = sdpvar(nx,N+1);
u = sdpvar(nu,N);

feas = false;

constr = [x(:,1) == x0];

if isempty(Af)
    constr = [constr x(:,N+1)==bf];
else
    constr = [constr Af*x(:,N+1)<=bf];
end


cost = 0; 
constraints = [z(1,N+1) <= x2, z(1,N+1)>=1];
for k = 1:N
    cost = cost + (z(1,k)-x_goal)^2 + z (3,k)*Q*z(3,k) + u(:,k).'*R.u(:,k); 
    % state
    constraints = constraints + [x1<= z(1,k), z(1,k)<=x3, abs(z(3,k)-vbar)<=vlim]; 
    
end 
cost = cost + z(1,N+1)*PN*z(1,N+1);

cost = x(:,N+1)'*P*x(:,N+1);

for k = 1:N
    constr = [constr, x(:,k+1) == A*x(:,k) + B*u(:,k),...
        uL <= u(:,k),u(:,k) <= uU,...
        xL <= x(:,k+1),x(:,k+1)<=xU];
    cost = cost + x(:,k)'*Q*x(:,k) + u(:,k)'*R*u(:,k);
end

%     options = sdpsettings('verbose',0);
options = sdpsettings('verbose',0,'solver','quadprog');
sol = optimize(constr,cost,options);

if sol.problem == 0
    feas = true;
else
    xOpt = [];
    uOpt = [];
    return;
end
Jopt = cost;
xOpt = double(x);
uOpt = double(u);
end
