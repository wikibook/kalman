function [r_next, v_next, Cbn] = missile_dyn(r, v, a_cmd, dt)
%
% missile model with constant speed
% one-step propagation of position and velocity of target
% r(k+1) = r(k) + dt*v(k) 
% v(k+1) = v(k) + dt*a(k)

r_next = r + dt*v;
v_next = v + dt*a_cmd;

% DCM b->n
the = atan2(-v(3), sqrt((v(1))^2+(v(2))^2));
psi = atan2(v(2), v(1));
cp = cos(psi); ct = cos(the);
sp = sin(psi); st = sin(the);
Cbn = [cp*ct sp*ct -st; -sp cp 0; cp*st sp*st ct];