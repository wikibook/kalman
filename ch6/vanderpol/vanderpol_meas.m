function z = vanderpol_meas(x, Rd, status)
%
% van der Pol measurement
% 

% measurement noise
if status == 'sy'
    v = sqrt(Rd)*randn(1,1);
else
    v = 0;
end

z = x(1) + v;
