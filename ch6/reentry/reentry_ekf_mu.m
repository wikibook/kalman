function [xhat, Phat, zhat, S] = reentry_ekf_mu(z, xbar, Pbar, Rd, params)
%
% Reentry EKF Measurement Update
% one-step propagation 
%

% constants
xR = params.xR;
yR = params.yR;

% xbar(k|k-1)
x1 = xbar(1);
x2 = xbar(2);

% Jacobian
r_mag = sqrt((x1-xR)^2 + (x2-yR)^2);
h11 = (x1-xR) / r_mag;
h12 = (x2-yR) / r_mag;
h21 = -(x2-yR) / (r_mag^2);
h22 = (x1-xR) / (r_mag^2);

H = [h11 h12 0 0 0;
     h21 h22 0 0 0];

% measurement update
zhat = reentry_meas(xbar, Rd, params, 'kf');
S = H * Pbar *H' + Rd;
Phat = Pbar - Pbar * H' * inv(S) * H * Pbar;
K = Pbar * H' * inv(S);
xhat = xbar + K * (z - zhat);
