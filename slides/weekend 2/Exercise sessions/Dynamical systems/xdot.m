function xdot = xdot(t,x,u,A,B)
%VDP1  Evaluate the van der Pol ODEs for mu = 1
%
%   See also ODE113, ODE23, ODE45.

%   Jacek Kierzenka and Lawrence F. Shampine
%   Copyright 1984-2014 The MathWorks, Inc.

% delta = 0.05;
% omega_n = 5;
% A = [0 1; -omega_n^2 -2*delta*omega_n^2];
% B = [0; omega_n^2];
% C = [1 0]; D=0;

% x = in(1:2);
% u = in(3);

xdot = A*x+B*u;