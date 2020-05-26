function x_next = aircraft_dyn(x, Omega, Qr, dt, status)
%
% ATC aircraft model
% one-step propagation of position and velocity of target
% r(k+1) = r(k) + dt*v(k) 
% v(k+1) = v(k) + dt*a(k)

Omega = Omega + 1e-8;

F = [1 sin(Omega*dt)/Omega     0 -(1-cos(Omega*dt))/Omega;
     0 cos(Omega*dt)           0 -sin(Omega*dt);
     0 (1-cos(Omega*dt))/Omega 1 sin(Omega*dt)/Omega;
     0 sin(Omega*dt)           0 cos(Omega*dt) ];


% system noise
if status == 'sy'
    w = sqrt(Qr)*randn(4,1);
else
    w = zeros(4,1);
end

x_next = F * x + w;
