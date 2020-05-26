function [xhat, Phat, zhat, S] = motor_ekf_mu(z, xbar,u, Pbar, Rd, params)
%
% motor EKF Measurement Update
% one-step propagation 
%

% constant
J = params.J;

% Jacobian
H = [ 1            0;
     -xbar(2)/J  -xbar(1)/J];

% measurement update
zhat = motor_meas(xbar, u, Rd, params, 'kf');
S = H * Pbar *H' + Rd;
Phat = Pbar - Pbar * H' * inv(S) * H * Pbar;
K = Pbar * H' * inv(S);
xhat = xbar + K * (z - zhat);
