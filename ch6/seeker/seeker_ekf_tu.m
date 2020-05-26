function [xbar, Pbar] = seeker_ekf_tu(xhat, Phat, a_cmd, Qd, dt)
%
% seeker EKF Time Update
% one-step propagation 
%

% system model
F = [eye(3) dt*eye(3); zeros(3,3) eye(3)];
G = [zeros(3,3); -dt*eye(3)];

Gw = [zeros(3,3);
     eye(3)];

% time update 
xbar = F * xhat + G * a_cmd;
Pbar = F * Phat * F' + Gw*Qd*Gw';
