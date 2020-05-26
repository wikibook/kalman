function z = motor_meas(x, u, Rd, params, status)
%
% motor measurement
% 

% constant
J = params.J;

% measurement noise
if status == 'sy'
    v = sqrt(Rd)*randn(2,1);
else
    v = zeros(2,1);
end

z1 = x(1);
z2 = (u - x(1)*x(2)) / J;
z = [z1; z2] + v;
