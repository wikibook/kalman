function [xbar, Pbar] = sonar_ukf_tu(xhat, U, Phat, Qd, dt, params)
%
% sonar UKF Time Update
% one-step propagation 
%

% constants
kappa = params.kappa;
n=length(xhat);


% system model
F = [eye(2) dt*eye(2); zeros(2,2) eye(2)];
Gw = [(dt)^2/2*eye(2); dt*eye(2)];


% sigma points
[Xi,W]=sigma_point(xhat,Phat,kappa);
 
% unscented transform
[n,mm] = size(Xi); 
Xibar = zeros(n,mm);
for jj=1:mm
    Xibar(:, jj) = F * Xi(:,jj) - U;
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

Pbar = Pbar + Gw*Qd*Gw';
