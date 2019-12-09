%% Scenario 1

% Lane width 
x1 = -3; x2 = 0; x3 = 3;
x_goal = (x1+x2)/2;
x_init = (x2+x3)/2;
% MPC Horizon
N = 7;
% MPC Solve setup
M = 40;

[feas, zego, uego, ztar, zpred, upred] = MPC_lanechange(M, N);

% Simulate MPC solution
figure()
plot(zego(1,:),zego(2,:),'o');
hold on
plot(ztar(1,:), ztar(2,:), 'x');
hold on

legend('ego', 'target')
title('ego vehicle trajectory')
xlim([-3 3])


car_size = 0.2;
ego_x = @(xego) [xego-car_size, xego-car_size, xego+car_size, xego+car_size];
ego_y = @(yego) [yego, yego + car_size, yego+car_size, yego];
tar_x = @(xtar) [xtar-car_size, xtar-car_size, xtar+car_size, xtar+car_size];
tar_y = @(ytar) [ytar, ytar+car_size, ytar+car_size, ytar];

hold on
ego = fill(ego_x(x_init), ego_y(0), 'g');
tar = fill(tar_x(x_goal), tar_y(0), 'b');


for t = 1:M+1
    xego = zego(1,t); yego = zego(2,t);
    xtar = ztar(1,t); ytar = ztar(2,t);
    set(ego, 'Xdata', ego_x(xego), 'Ydata', ego_y(yego));
    set(tar, 'Xdata', tar_x(xtar), 'Ydata', tar_y(ytar));
    axis([-3 3 0 5])
    axis 'auto y'
    axis square
    pause(0.2)
    legend('ego', 'target')
    drawnow 
end

title('ego vehicle trajectory')
