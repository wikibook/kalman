function [r_next, v_next, Cbn] = observer_dyn(r, v, U, dt)
%
% observer model
% one-step propagation of position and velocity of target
% r(k+1) = r(k) + dt*v(k) 
% v(k+1) = v(k) + dt*a(k)

r_next = r + dt*v;
v_next = v + U(3:4,1);
