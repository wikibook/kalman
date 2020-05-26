function x_next = vanderpol_dyn(x, Q, dt, status)
%
% van der Pol oscillator
% one-step propagation 
%

% x(k)
x1 = x(1);
x2 = x(2);
x3 = x(3); % x0^2
x4 = x(4); % friction coef.
x5 = x(5); % omega^2

% process noise
if status == 'sy'
    w = sqrt(Q)*randn(5,1);
else
    w = zeros(5,1);
end

% x(k+1)
x1_next = x1 + dt * x2 + w(1);
x2_next = x2 + dt * (x4*(1-x1^2/x3)*x2- x5*x1) + w(2);
x3_next = x3 + w(3);
x4_next = x4 + w(4);
x5_next = x5 + w(5);

% next state
x_next = [x1_next; x2_next; x3_next; x4_next; x5_next];