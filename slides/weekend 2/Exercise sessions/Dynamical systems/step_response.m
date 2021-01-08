clear all, close all, clc

%%%%%%%%%% Exercise %%%%%%%%
% Choose damping and natural frequency such that
%   - The overshoot is not larger than 30%
%   - the settling time to 5% is not larger than 3s

%%%%% second order system %%%%%
delta = 0.05;
omega_n = 1;

A = [0 1; -omega_n^2 -2*delta*omega_n^2];
B = [0; omega_n^2];
C = [1 0]; D=0;
sys = ss(A,B,C,D);

Tstep = 0.1;    % discretization and simulation step

%%%%% discrete-time model %%%%%%
sysD = c2d(sys,Tstep,'zoh');    % you can perturb your model here before discretizing it

%%%% simulation in Matlab of the step response %%%%
tsim = 120;             % simulation time
t = 0;
x0 = [0 0]';           % initial condition
x = [x0];
u = 1;

while t<=tsim
    % simulating the system
    [tsol,xsol] = ode45(@(t,x) xdot(t,x,u,A,B), [t t+Tstep], x(:,end));
    x = [x xsol(end,:)'];
    t = t+Tstep;
end
plot(0:Tstep:tsim,C*x(:,1:end-1)), xlabel('Time [s]'), ylabel('x_1'), grid on
% figure
% step(sysD,'r--'), grid on
% lsim(sysD, u*ones(length(0:Tstep:tsim),1), 0:Tstep:tsim,'c-.'), hold on




