function x_next = reentry_dyn(x, Qd, dt, params, status)
%
% Reentry dynamics
% one-step propagation 
%

% constants
R0 = params.R0;
H0 = params.H0;
beta0 = params.beta0;
mu = params.mu;

% x(k)
x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);
x5 = x(5);

% some parameters
R = sqrt(x1^2+x2^2);
V = sqrt(x3^2+x4^2);
beta = beta0 * exp(x5);
D = -beta * exp((R0 - R)/H0) * V;
G = -mu/(R^3);

% process noise
if status == 'sy'
    w = sqrt(Qd)*randn(3,1);
else
    w = zeros(3,1);
end

% x(k+1)
x1_next = x1 + dt * x3;
x2_next = x2 + dt * x4;
x3_next = x3 + dt * (D*x3 + G*x1) + w(1);
x4_next = x4 + dt * (D*x4 + G*x2) + w(2);
x5_next = x5 + w(3);

% next state
x_next = [x1_next; x2_next; x3_next; x4_next; x5_next];