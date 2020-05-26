function [xbar, Pbar] = motor_ekf_tu(xhat, u, Phat, Qd, dt, params)
%
% motor EKF Time Update
% one-step propagation 
%

J = params.J;

% Jacobian
F = [1-dt/J*xhat(2)  -dt/J*xhat(1);
     0               1 ];

% one-step time update 
xbar = motor_dyn(xhat, u, Qd, dt, params, 'kf');
Pbar = F * Phat * F' + Qd;
