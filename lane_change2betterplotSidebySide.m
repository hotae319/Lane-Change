%% Scenario 1
nz = 4; nu = 2;
% Lane width 
x1 = -3; x2 = 0; x3 = 3;
x_goal = (x1+x2)/2;
x_init = (x2+x3)/2;
% MPC Horizon
N = 7;
% MPC Solve setup
M = 30;

%%
% Simulate MPC solution
% define
car_size_h = 1;
car_size_w = 0.5;
car_size = 0.4;
ego_x = @(xego) [xego-car_size_w, xego-car_size_w, xego+car_size_w, xego+car_size_w];
ego_y = @(yego) [yego-car_size_h, yego + car_size_h, yego+car_size_h, yego-car_size_h];
tar_x = @(xtar) [xtar-car_size_w, xtar-car_size_w, xtar+car_size_w, xtar+car_size_w];
tar_y = @(ytar) [ytar-car_size_h, ytar+car_size_h, ytar+car_size_h, ytar-car_size_h];


f1 = figure();
subplot(1,2,1)
plot(zego(1,:),zego(2,:),'ro');
hold on
plot(ztar(1,:), ztar(2,:), 'bx');
hold on
plot(ztar(1+nz,:), ztar(2+nz,:), 'g*'); % target2
hold on
xline(0,':');
xline(-3,':');
xline(3,':');
axis image
title('Ego Vehicle Trajectory')
hold on
ego = fill(ego_x(zego(1,end)), ego_y(zego(2,end)), 'r');
tar = fill(tar_x(ztar(1,end)), tar_y(ztar(2,end)), 'b');
tar2 = fill(tar_x(ztar(1+nz,end)), tar_y(ztar(2+nz,end)), 'g');
xlim([-3 3])

subplot(1,2,2)
plot(zego2(1,:),zego2(2,:),'ro');
hold on
plot(ztar2(1,:), ztar2(2,:), 'bx');
hold on
plot(ztar2(1+nz,:), ztar2(2+nz,:), 'g*'); % target2
hold on
xline(0,':');
% xline(-3,':');
% xline(3,':');
axis image
title('Ego Vehicle Trajectory')
xlim([-3 3])
hold on
ego = fill(ego_x(zego2(1,end)), ego_y(zego2(2,end)), 'r');
tar = fill(tar_x(ztar2(1,end)), tar_y(ztar2(2,end)), 'b');
tar2 = fill(tar_x(ztar2(1+nz,end)), tar_y(ztar2(2+nz,end)), 'g');
legend('Ego', 'Target', 'Target2', 'Location','northwestoutside')
