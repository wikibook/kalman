function [xbar, Pbar] = reentry_ekf_tu(xhat, Phat, Qd, dt, params)
%
% Reentry EKF Time Update
% one-step propagation 
%

% constants
R0 = params.R0;
H0 = params.H0;
beta0 = params.beta0;
mu = params.mu;

% xhat(k|k)
x1 = xhat(1);
x2 = xhat(2);
x3 = xhat(3);
x4 = xhat(4);
x5 = xhat(5);

% some parameters
R = sqrt(x1^2+x2^2);
V = sqrt(x3^2+x4^2);
beta = beta0 * exp(x5);
D = -beta * exp((R0 - R)/H0) * V;
G = -mu/(R^3);

% Jacobian
dDdx1 = -x1/(R*H0) * D;
dDdx2 = -x2/(R*H0) * D;
dDdx3 = x3 * D / (V^2);
dDdx4 = x4 * D / (V^2);
dDdx5 = D;
dGdx1 = -3*x1 * G /(R^2);
dGdx2 = -3*x2 * G /(R^2);

f31 = dt * (dDdx1*x3 + dGdx1*x1 + G);
f32 = dt * (dDdx2*x3 + dGdx2*x1);
f33 = 1 + dt * (dDdx3*x3 + D);
f34 = dt * (dDdx4*x3);
f35 = dt * (dDdx5*x3);
f41 = dt * (dDdx1*x4 + dGdx1*x2);
f42 = dt * (dDdx2*x4 + dGdx2*x2 + G);
f43 = dt * (dDdx3*x4);
f44 = 1 + dt * (dDdx4*x4 + D);
f45 = dt * (dDdx5*x4);

F = [1   0   dt   0   0;
     0   1   0   dt   0;
     f31 f32 f33 f34 f35;
     f41 f42 f43 f44 f45;
     0   0   0   0   1 ];
 
G = [zeros(2,3);
     eye(3)];

% one-step time update 
xbar = reentry_dyn(xhat, Qd, dt, params, 'kf');
Pbar = F * Phat * F' + G*Qd*G';
