function [feas, xOpt, uOpt, Jopt] = solve_cftoc(A, B, P, Q, R, N, x0, xL, xU, uL, uU, bf, Af)

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