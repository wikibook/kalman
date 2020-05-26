function z =sonar_meas(r_rel, Rd, status)
%
% sonar measurement
% 


% measurement noise
if status == 'sy'
    v = sqrt(Rd)*randn();
else
    v = 0;
end

z = atan2(r_rel(1), r_rel(2)) + v;