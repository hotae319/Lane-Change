%% Scenario 1
nz = 4; nu = 2;
% Lane width 
x1 = -3; x2 = 0; x3 = 3;
x_goal = (x1+x2)/2;
x_init = (x2+x3)/2;
% MPC Horizon
N = 10;
% MPC Solve setup
M = 40;

[feas, zego, uego, ztar, zpred, upred, ztar_pred] = MPC_lanechange(M, N);

% Simulate MPC solution
figure()
plot(zego(1,:),zego(2,:),'o');
hold on
plot(ztar(1,:), ztar(2,:), 'xb');
hold on
plot(ztar(1+nz,:), ztar(2+nz,:), 'xb'); % target2
hold on

legend('ego', 'target', 'target2')
title('ego vehicle trajectory')
xlim([-3 3])


car_size_h = 2;
car_size_w = 1;
ego_x = @(xego) [xego-car_size_w, xego-car_size_w, xego+car_size_w, xego+car_size_w];
ego_y = @(yego) [yego, yego + car_size_h, yego+car_size_h, yego];
tar_x = @(xtar) [xtar-car_size_w, xtar-car_size_w, xtar+car_size_w, xtar+car_size_w];
tar_y = @(ytar) [ytar, ytar+car_size_h, ytar+car_size_h, ytar];

lane = fill([-3 -3 3 3], [0 60 60 0], 'w');
hold on
ego = fill(ego_x(x_init), ego_y(0), 'g');
tar = fill(tar_x(x_goal), tar_y(0), 'b');
tar2 = fill(tar_x(x_goal), tar_y(7), 'b'); % add target 2

for t = 1:M+1
    xego = zego(1,t); yego = zego(2,t);
    xtar = ztar(1,t); ytar = ztar(2,t);
    xtar2 = ztar(1+nz,t); ytar2 = ztar(2+nz,t);
    set(ego, 'Xdata', ego_x(xego), 'Ydata', ego_y(yego));
    set(tar, 'Xdata', tar_x(xtar), 'Ydata', tar_y(ytar));
    set(tar2, 'Xdata', tar_x(xtar2), 'Ydata', tar_y(ytar2)); % target2
    axis([-3 3 -5 60])
%     axis 'auto y'
%     axis square   
    axis image
    pause(0.1)
    legend([ego, tar, tar2], 'ego', 'target', 'target2') % target2
    drawnow 
end

title('ego vehicle trajectory')
