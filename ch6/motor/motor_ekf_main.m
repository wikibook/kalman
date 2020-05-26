%
% motor EKF main
%


clear all

% constants
params.J = 10;

% sampling time
dt = 0.01;

% process noise
Qd = diag([1e-6 1e-2]); % for EKF 
Qr = diag([1e-6 1e-4]); % for real system

% measurement noise
Rd = diag([1e-4 1e-4]);

% system initial state
x0 = [0; 1];
P0 = diag([1 1000]);

% EKF initialization
xbar = x0 + sqrt(P0)*randn(2,1);
Pbar = P0;

% collections
X = [];
U = [];
XHAT = [];
PHAT = [];
Z = [];
ZHAT = [];
SBAR = [];
TIME = [];

% EKF propagation -----------------------------------
for k = 0:4000
    
    t = dt*k
    
    % input
    u = double(mod(t,1)<0.5) - 0.5;
    
    % measurement
    z = motor_meas(x0, u, Rd, params, 'sy');
    
    % MU
    [xhat, Phat, zhat, S] = motor_ekf_mu(z, xbar,u, Pbar, Rd, params);
    
    % TU
    [xbar, Pbar] = motor_ekf_tu(xhat, u, Phat, Qd, dt, params);
    
    % motor dynamics
    x = motor_dyn(x0, u, Qr, dt, params, 'sy');
    
    if t == 20
        x(2) = 10; % step change in friction coef.
    end
    
    % collections
    X = [X; x0'];
    U = [U; u'];
    XHAT = [XHAT; xhat'];
    PHAT = [PHAT; (diag(Phat))'];
    Z = [Z; z'];
    ZHAT = [ZHAT; zhat'];
    SBAR = [SBAR; (diag(S))'];
    TIME = [TIME; t];
        
    % for next step
    x0 = x;
   
end
    