function [xbar, Pbar] = motor_ukf_tu(xhat, u, Phat, Qd, dt, params)
%
% motor UKF Time Update
% one-step propagation 
%

% constants
J = params.J;
kappa = params.kappa;
n=length(xhat);

% sigma points
[Xi,W]=sigma_point(xhat,Phat,kappa);

% unscented transform
[n,mm] = size(Xi); 
Xibar = zeros(n,mm);
for jj=1:mm
    Xibar(:, jj) =motor_dyn(Xi(:,jj), u, Qd, dt, params, 'kf');
end

% one-step time update
xbar = zeros(n,1);
for jj=1:mm
    xbar = xbar + W(jj).*Xibar(:,jj);
end

Pbar = zeros(n,n);
for jj=1:mm
    Pbar = Pbar + W(jj)*(Xibar(:,jj)-xbar)*(Xibar(:,jj)-xbar)';
end

Pbar = Pbar + Qd;

