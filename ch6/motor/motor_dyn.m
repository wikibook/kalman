function x_next = motor_dyn(x, u, Qd, dt, params, status)
%
% motor dynamics
% one-step propagation 
%

% params
J = params.J;

% x(k)
x1 = x(1);
x2 = x(2);

% process noise
if status == 'sy'
    w = sqrt(Qd)*randn(2,1);
else
    w = zeros(2,1);
end

% x(k+1)
x1_next = x1 + dt/J * (u - x1*x2) + w(1);
x2_next = x2 + w(2);

% next state
x_next = [x1_next; x2_next];