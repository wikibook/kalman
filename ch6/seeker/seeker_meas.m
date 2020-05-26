function z = seeker_meas(r_rel, Cbn, Rd, status)
%
% seeker measurement
% 

rb = Cbn * r_rel;

% measurement noise
if status == 'sy'
    v = sqrt(Rd)*randn(3,1);
else
    v = zeros(3,1);
end

R = sqrt(rb'*rb) + v(1);
the_g = atan2(-rb(3), sqrt((rb(1))^2+(rb(2))^2)) + v(2);
psi_g = atan2(rb(2), rb(1)) + v(3);

z = [R; the_g; psi_g];