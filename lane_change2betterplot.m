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

[feas, zego, uego, ztar, zpred, upred, ztar_pred] = MPC_lanechange(M, N);

% Simulate MPC solution
f1 = figure();
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

legend('Ego', 'Target', 'Target2', 'Location','best')
title('Ego Vehicle Trajectory')
xlim([-3 3])

f = figure('Position', [100, 100, 1500, 800]);
plot(zego(1,:),zego(2,:),'wo');
hold on
plot(ztar(1,:), ztar(2,:), 'wx');
hold on
plot(ztar(1+nz,:), ztar(2+nz,:), 'w*'); % target2
hold on
title('Ego Vehicle Trajectory')
xline(0,':');
xline(-3,':');
xline(3,':');
axis image
% car_size_h = 2;
% car_size_w = 0.6;
car_size = 0.4;
ego_x = @(xego) [xego-car_size, xego-car_size, xego+car_size, xego+car_size];
ego_y = @(yego) [yego-car_size, yego + car_size, yego+car_size, yego-car_size];
tar_x = @(xtar) [xtar-car_size, xtar-car_size, xtar+car_size, xtar+car_size];
tar_y = @(ytar) [ytar-car_size, ytar+car_size, ytar+car_size, ytar-car_size];
%lane = fill([-3 -3 3 3], [0 60 60 0], 'w');
hold on
ego = fill(ego_x(x_init), ego_y(0), 'r');
tar = fill(tar_x(x_goal), tar_y(0), 'b');
tar2 = fill(tar_x(x_goal), tar_y(7), 'b'); % add target 2
% Annotation help
steerVec = zego(4,:);
speedVec = zego(3,:);
dim = [0.732 0.28 .3 .3];
dim2 = [0.732 0.13 0.3 0.3];
Egoheader = "Ego Vehicle States";
EgoangleAnnotate = 'Angle = ';
EgospeedAnnotate = 'Speed = ';
for t = 1:M+1
    xego = zego(1,t); yego = zego(2,t);
    xtar = ztar(1,t); ytar = ztar(2,t);
    xtar2 = ztar(1+nz,t); ytar2 = ztar(2+nz,t);
    % Create box at origin
    xvals = ego_x(0);
    x1 = xvals(1);
    x2 = xvals(2);
    x3 = xvals(3);
    x4 = xvals(4);
    yvals = ego_y(0);
    y1 = yvals(1);
    y2 = yvals(2);
    y3 = yvals(3);
    y4 = yvals(4);
    % Rotate about z axis and translate
    p1 = [xego;yego;0] + rotz(steerVec(t)*(180/pi))*[x1;y1;0];
    p2 = [xego;yego;0] + rotz(steerVec(t)*(180/pi))*[x2;y2;0];
    p3 = [xego;yego;0] + rotz(steerVec(t)*(180/pi))*[x3;y3;0];
    p4 = [xego;yego;0] + rotz(steerVec(t)*(180/pi))*[x4;y4;0];
    
    set(ego, 'Xdata', [p1(1),p2(1),p3(1),p4(1)], 'Ydata', [p1(2),p2(2),p3(2),p4(2)]);
    set(tar, 'Xdata', tar_x(xtar), 'Ydata', tar_y(ytar));
    set(tar2, 'Xdata', tar_x(xtar2), 'Ydata', tar_y(ytar2));
    axis image
    steerVal = num2str(steerVec(t));
    speedVal = num2str(speedVec(t));
    k = annotation('textbox',dim,'Color','r','String',{Egoheader, strcat(EgoangleAnnotate, steerVal), strcat(EgospeedAnnotate,speedVal)},'FitBoxToText','on');
    pause(0.5)
    legend([ego, tar, tar2], 'Ego', 'Target', 'Target2', 'Location', 'Best') % target2
    drawnow 
    F(t) = getframe(f);
    delete(k)
end
k = annotation('textbox',dim,'Color','r','String',{Egoheader, strcat(EgoangleAnnotate, steerVal), strcat(EgospeedAnnotate,speedVal)},'FitBoxToText','on');
%k2 = annotation('textbox',dim2,'Color','b','String',{Targetheader, strcat(EgoangleAnnotate, targetSteerVal), strcat(EgospeedAnnotate,targetSpeedVal)},'FitBoxToText','on');
title('Ego Vehicle Trajectory')

%% movie
fig = figure;movie(fig,F,1,5)