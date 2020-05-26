function [xbar, Pbar] = vanderpol_ekf_tu(xhat, Phat, Qd, dt)
%
% van der Pol EKF Time Update
% one-step propagation 
%

% xhat(k|k)
x1 = xhat(1);
x2 = xhat(2);
x3 = xhat(3);
x4 = xhat(4);
x5 = xhat(5);

% Jacobian
f12 = dt;
f21 = dt * (-2*x4*x1*x2/x3 - x5);
f22 = 1 + dt * x4*(1-x1^2/x3);
f23 = dt * (x4*x1^2/(x3^2))*x2;
f24 = dt * (1-x1^2/x3)*x2;
f25 = dt * (-x1);

F = [1   f12  0    0    0;
     f21 f22  f23  f24  f25;
     0   0    1    0    0;
     0   0    0    1    0;
     0   0    0    0    1 ];

  
% one-step time update 
xbar = vanderpol_dyn(xhat, Qd, dt, 'kf');
Pbar = F * Phat * F' + Qd;
