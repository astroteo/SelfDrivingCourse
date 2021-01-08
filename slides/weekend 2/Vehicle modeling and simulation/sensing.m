function out = sensing(t,x,x_c,Tstep)
%                         ^ ^ ^     ^      ^
%                         | |              |---------- time step
%                         | |------------- vehicle state
%                         |--------------- current desired heading
% sensing extracts the state of the bicycle model formulated wrt a path
%
%   Paolo Falcone
%   Copyright 2020 Unimore

% the reference path is straight for 20 s then is circular with a curvature
% radius of 700m
t_straight = 20;        %[s]
radius = 700;           %[m]

V_x = x(5);

% reference path orientation rate of change
if t<= t_straight
    psi_dot_des = 0;
else
    psi_dot_des = V_x/radius;
end

X_des = x_c(5);
Y_des = x_c(6);
psi_des = x_c(7);

X_des_next = X_des + Tstep*V_x*cos(psi_des);
Y_des_next = Y_des + Tstep*V_x*sin(psi_des);
psi_des_next = psi_des + Tstep*psi_dot_des;

e1 = x_c(1) + Tstep*x_c(2);
e2 = x(2) + V_x*(x(3) - psi_des);
e3 = x(3) - psi_des;
e4 = x(4) - psi_dot_des;


out = [e1; e2; e3; e4; X_des_next; Y_des_next; psi_des_next];
