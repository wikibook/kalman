function [xbar, Pbar] = sonar_ekf_tu(xhat, U, Phat, Qd, dt)
%
% sonar EKF Time Update
% one-step propagation 
%

% system model
F = [eye(2) dt*eye(2); zeros(2,2) eye(2)];
Gw = [(dt)^2/2*eye(2); dt*eye(2)];

% time update 
xbar = F * xhat - U;
Pbar = F * Phat * F' + Gw*Qd*Gw';
