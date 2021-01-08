function xdot = xdot(t,x,u,ut)
% xdot evluates the state time derivatives for the bicycle model
%
%   Paolo Falcone
%   Copyright 2020 Unimore

%%%%% Vehicle params %%%%%
m = 1573;       %[Kg]
Iz = 2873;      %[Kg*m2]
lf = 1.1;       %[m]
lr = 1.58;      %[m]
Cf = 8e4;       %[N/deg]
Cr = 8e4;       %[N/deg]
Vx = x(5);        %[Kph]

% air drag force
F_drag = .5*1.225*(1.6+0.00056*(1573-765))*Vx^2;
F_roll = .015*m*9.81; 

% system matrices in continuous time
A = [0          1                   0                            0; 
     0  -2*(Cf+Cr)/m/Vx             0     -Vx-2*(Cf*lf-Cr*lr)/m/Vx;
     0          0                   0                            1;
     0  -2*(Cf*lf-Cr*lr)/Iz/Vx      0   -2*(Cf*lf^2+Cr*lr^2)/Iz/Vx];
B = [     0; 
     2*Cf/m;
          0;
     2*lf*Cf/Iz];
 
%  C = eye(size(A,1)); D=zeros(size(A,1),size(B,2));
% sys = ss(A,B,C,D);

delta = interp1(ut,u(1,:),t);
ax = interp1(ut,u(2,:),t);

if Vx>=0.05*3.6
    xddot = 0; %ax-F_drag/m-F_roll/m;
else
    xddot = 0;
end

xdot = [A*x(1:4)+B*delta; xddot; Vx*cos(x(3))-x(2)*sin(x(3)); Vx*sin(x(3))+x(2)*cos(x(3))];