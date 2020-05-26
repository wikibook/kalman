function px = gauss_pdf(x, xbar, Pxx)
% 
% Gaussian pdf
%

[nn,nn] = size(Pxx);

tmp = (x - xbar)' * inv(Pxx) * (x - xbar);
exptmp = exp(-0.5 * tmp);
px = exptmp / (sqrt((2*pi)^nn * det(Pxx)));