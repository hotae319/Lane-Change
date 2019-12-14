function [feas, zego, uego, ztar, zpred, upred, ztar_pred] = MPC_lanechange(M, N)
%% Lane Setting

% Lane width 
x1 = -3; x2 = 0; x3 = 3;
x_goal = (x1+x2)/2;

%z0 = [(x2+x3)/2;0;10;0];
z0 = [x_goal;3;10;0];

%% MPC Problem Set up
nz = 4; nu = 2;
% Constraints

d_safe = 4;       % distance to avoid collision
vbar = 0; vlim = 50; % Limits
alim = 10;           % acc limit
betalim = 1;      % steering angle limit

zL = [x1;-inf;vbar-vlim;-0.3];
zU = [x3;inf;vbar+vlim;0.3];
uL = [-alim;-betalim];
uU = [alim;betalim];

% Terminal constraint
eps = 0.3;            % termianl safe distance
phiN = 0.2;
Af = [1 0 0 0;-1 0 0 0;0 0 0 1;0 0 0 -1];
bf = [x2-0.5;-x1+0.5;phiN;phiN];

% Objective Function
% stage cost : (x-xgoal)'*Q*(X-xgoal)+u'*R*u+phi'*P*phi
% terminal cost : xN'*PN*xN 
%R = 0.1*eye(nu); 
Q = 1; P = 1; PN = 6;
R = [0.1 0;0 3];
% MPC Solve setup

zgoal = x_goal;
safe_param = [d_safe;eps];

% ego vehicle state history
zego = zeros(nz, M+1); 
zego(:,1) = z0;
uego = zeros(nu, M);
zpred = zeros(nz,N+1);
feas = false([1,M]);

% Target vehicle history
% ztar = zeros(nz, M+1);
ztar = zeros(nz*2, M+1); % target 2
ztar(1:nz,1) = [x_goal;0;10;0];
ztar(nz+1:end,1) = [x_goal;10;10;0];

% MPC solve with measurement of target vehicle's state
for t = 1:M
    fprintf('Solving simstep: %i\n',t)
    [feas(t), zOpt, uOpt, ztar_pred] = solve_cftoc(P, PN, Q, R, N, zego(:,t), zL, zU, uL, uU, bf, Af, safe_param, zgoal, ztar(:,t));
    if ~feas(t)
        disp('Infeasible');
        return;
    end      
    zpred = zOpt;
    upred = uOpt;
    uego(:,t) = uOpt(:,1);
    zego(:,t+1) = ego_vehicle(zego(:,t),uego(:,t));
    % Target vehicle update
    ztar(1:nz,t+1) = target_vehicle(ztar(:,t), zego(3,t));
    ztar(nz+1:end,t+1) = target_vehicle(ztar(nz+1:end, t), zego(3,t)); % target 2
end


end
