function [xhat, Phat, zhat, Pzz] = vanderpol_ukf_mu(z, xbar, Pbar, Rd, params)
%
% van der Pol UKF Measurement Update
% one-step propagation 
%

% constants
kappa = params.kappa;
n = length(xbar);
p = length(z);

% sigma points
[Xi,W] = sigma_point(xbar,Pbar,kappa);

% unscented transform
[n,mm] = size(Xi);
Zi = zeros(p,mm);
for jj=1:mm
    Zi(:, jj) = vanderpol_meas(Xi(:,jj), Rd, 'kf');
end

% measurement update
zhat = zeros(p,1);
for jj=1:mm
    zhat = zhat + W(jj).*Zi(:,jj);
end

Pxz = zeros(n,p);
Pzz = zeros(p,p);
for jj=1:mm
    Pxz = Pxz + W(jj)*(Xi(:,jj)-xbar)*(Zi(:,jj)-zhat)';
    Pzz = Pzz + W(jj)*(Zi(:,jj)-zhat)*(Zi(:,jj)-zhat)';
end

Pzz = Pzz + Rd;

K = Pxz * inv(Pzz);
Phat = Pbar - K * Pzz *K';

xhat = xbar + K * (z - zhat);
