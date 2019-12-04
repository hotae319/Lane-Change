function [xtar, ytar, vtar] = target_vehicle(xtar_pre, ytar_pre, vtar_pre, vego)
%% Target Car Reaction Model

Ts = 0.1; % target vehicle update timestep

% 1 - constant velocity
% vtar = 1;
% ytar = ytar_pre + Ts*vtar;

% 2 - safety velocity
dcc = 0.5;
vtar = vtar_pre - dcc(vtar-vego);
ytar = ytar_pre + Ts*vtar;

end