function [Xi W] = sigma_point(xbar, P, kappa)
%
n = length(xbar);
Xi = zeros(n,2*n+1);
W = zeros(n,1);

Xi(:,1) = xbar;
W(1) = kappa/(n+kappa);

A = sqrt(n+kappa)*(chol(P))';

for k=1:n
    Xi(:,k+1) = xbar + A(:,k);
    W(k+1) = 1/(2*(n+kappa));
end

for k=1:n
    Xi(:,n+k+1) = xbar - A(:,k);
    W(n+k+1) = 1/(2*(n+kappa));
end