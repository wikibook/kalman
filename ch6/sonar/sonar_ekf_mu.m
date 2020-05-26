function [xhat, Phat, zhat, S] = sonar_ekf_mu(z, xbar, Pbar, Rd)
%
% sonar EKF Measurement Update
% one-step propagation 
%


% Jacobian
x1 = xbar(1);
x2 = xbar(2);

H = [x2/(x1^2+x2^2) -x1/(x1^2+x2^2) 0 0];

% measurement estimate
zhat = sonar_meas(xbar(1:2,1), Rd, 'kf');

% measurement update 
S = H * Pbar *H' + Rd;
Phat = Pbar - Pbar * H' * inv(S) * H * Pbar;
K = Pbar * H' * inv(S);
xhat = xbar + K * (z - zhat);
