function [xhat, Phat, zhat, S] = atc_kf(xhat0, Phat0, z, Qd, Rd, dt) 
%
% KF
%

% sysem matrix
F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
H = [1 0 0 0; 0 0 1 0];

% TU
xbar = F * xhat0;
Pbar = F * Phat0 * F' + Qd;

% MU
zhat = H * xbar; 
S = H * Pbar *H' + Rd;
Phat = Pbar - Pbar * H' * inv(S) * H * Pbar;
K = Pbar * H' * inv(S);
xhat = xbar + K * (z - zhat);