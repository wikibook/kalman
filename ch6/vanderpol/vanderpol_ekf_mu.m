function [xhat, Phat, zhat, S] = vanderpol_ekf_mu(z, xbar, Pbar, Rd)
%
% van der Pol EKF Measurement Update
% one-step propagation 
%

% Jacobian
H = [1 0 0 0 0];

% measurement update
zhat = vanderpol_meas(xbar, Rd, 'kf');
S = H * Pbar *H' + Rd;
Phat = Pbar - Pbar * H' * inv(S) * H * Pbar;
K = Pbar * H' * inv(S);
xhat = xbar + K * (z - zhat);
