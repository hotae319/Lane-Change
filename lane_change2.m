%% Scenario 1

% MPC Horizon
N = 20;
% MPC Solve setup
M = 50;

[feas, zego, uego, ztar] = MPC_lanechange(M, N);

% Simulate MPC solution
figure()
plot(zego(1,:),zego(2,:),'-');
hold on
plot(ztar(1,:), ztar(2,:), '--');
legend('ego', 'target')
title('ego vehicle trajectory')


