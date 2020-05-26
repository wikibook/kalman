function z = atc_meas(x, Rd, status)
%
% sonar measurement
% 


% measurement noise
if status == 'sy'
    v = sqrt(Rd)*randn(2,1);
else
    v = [0; 0];
end

z = [1 0 0 0; 0 0 1 0] * x + v;