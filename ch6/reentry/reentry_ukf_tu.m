function [xbar, Pbar] = reentry_ukf_tu(xhat, Phat, Qd, dt, params)
%
% Reentry UKF Time Update
% one-step propagation 
%

% constants
kappa = params.kappa;
n=length(xhat);

% sigma points
[Xi,W]=sigma_point(xhat,Phat,kappa);
 
G = [zeros(2,3);
     eye(3)];

% unscented transform
[n,mm] = size(Xi); 
Xibar = zeros(n,mm);
for jj=1:mm
    Xibar(:, jj) = reentry_dyn(Xi(:,jj), Qd, dt, params, 'kf');
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

Pbar = Pbar + G*Qd*G';
