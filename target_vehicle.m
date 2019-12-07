function ztar = target_vehicle(ztar_pre, vego)
%% Target Car Reaction Model

dt = 0.1; % target vehicle update timestep

% 1 - constant velocity
ztar(3) = ztar_pre(3);
ztar(2) = ztar_pre(2) + dt*ztar_pre(3);
ztar(1) = ztar_pre(1);
ztar(4) = ztar_pre(4);

% 2 - safety velocity
% dcc = 0.5;
% ztar(3) = ztar_pre(3) - dcc(ztar_pre(3)-vego);
% ztar(2) = ztar_pre(2) + dt*ztar(3);
% ztar(1) = ztar_pre(1);
% ztar(4) = ztar_pre(4);

end