% bicycle_sim simulates a vehicle bicycle model
%
%   Paolo Falcone
%   Copyright 2020 Unimore

%%% Vehicle simulation script
clear all, close all, clc

Tstep = 0.1;    % simulation step
V0 = 50;        % intial speed [Kph]

%%%% simulation parameters %%%%
tsim = 120;                       % simulation time
t = 0;

% initial condition
x0 = [0; 
    0;      % lateral velocity
    0;      % heading angle
    0;      % yaw rate
    V0/3.6; % longitudinal velocity
    0;      % longitudinal position in the global frame
    0];     % lateral position in the global frame
x = [x0];

% state to be feedback to the controller + reference path
xc = sensing(t,x(:,end),zeros(7,1),Tstep); 

%%%%%%% Comment below if a steering and longitudinal controller are used 
% define the steering and the longitudinal acceleration input here
u = [1*pi/180*sin(0.1*(0:Tstep:tsim)); zeros(1,length(0:Tstep:tsim))];    
ut = 0:Tstep:tsim;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


while t<=tsim
    % Extract here the current lateral deviation and orientation error from
    % the measured states
    xc = [xc sensing(t,x(:,end),xc(:,end),Tstep)];
   
    % Calculate the steering and acceleration inputs
    
    % simulating the system
    [tsol,xsol] = ode45(@(t,x) xdot(t,x,u,ut), [t t+Tstep], x(:,end));
    x = [x xsol(end,:)'];
    t = t+Tstep;
end

figure
subplot(2,2,1),yyaxis left
plot(0:Tstep:tsim,x(5,1:end-1)*3.6),xlabel('Time [s]'),ylabel('V_x [Kph]'),grid on
title('Velocities in the body frame')
yyaxis right
plot(0:Tstep:tsim,x(2,1:end-1)),xlabel('Time [s]'),ylabel('V_y [m/s]')
subplot(2,2,2),yyaxis left
plot(x(6,1:end-1),x(7,1:end-1)),xlabel('X [m]'),ylabel('Y [m]'), grid on
title('Coordinates in the global frame')
yyaxis right
plot(xc(5,:),xc(6,:),'r')
subplot(2,2,3),plot(0:Tstep:tsim,x(4,1:end-1)),xlabel('Time [s]'),ylabel('d \psi/dt [rad/s]'),grid on
title('Yaw rate')
subplot(2,2,4), yyaxis left
plot(0:Tstep:tsim,u(1,:)),xlabel('Time [s]'),ylabel('\delta [rad]'),grid on
title('Control inputs')
yyaxis right
plot(0:Tstep:tsim,u(2,:)),xlabel('Time [s]'),ylabel('a_x [m/s^2]'),grid on