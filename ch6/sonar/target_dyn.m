function [r_next, v_next] = target_dyn(r, v, Qd, dt, status)
%
% target constant velocity model
% one-step propagation of position and velocity of target
% r(k+1) = r(k) + dt*v(k) 
% v(k+1) = v(k) + w(k)

% process noise
if status == 'sy'
    w = sqrt(Qd)*randn(2,1);
else
    w = zeros(2,1);
end

r_next = r + dt*v;
v_next = v + w;
