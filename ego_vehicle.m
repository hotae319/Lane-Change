function znext = ego_vehicle(z, u)
%% Kinematic Bicycle Model 

lr = 1; lf = 1;
dt = 0.5; % sampling time

% State and Input
% z = (x, y, v, phi), u = [acc, beta]
nz = 4; nu = 2; 


%     xp = x + TS*v*cos(psi+beta);
%     yp = y + TS*v*sin(psi+beta);
%     vp = v + TS*a;
%     psip = psi + TS*v*sin(beta)/lr;
fdyn = @(z,u) [ -z(3)*sin(z(4)+u(2));z(3)*cos(z(4)+u(2)); u(1); z(3)/lr*sin(u(2))];
znext = z + fdyn(z,u)*dt;%Discrete time
   

end