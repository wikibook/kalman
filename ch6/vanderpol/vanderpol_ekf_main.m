%
% van der Pol EKF main
%

clear all

% constants
xref = 4;
xref2 = xref^2;
gamma = 2;
omega = 3;
om2 = omega^2;

gamma_f = 1;
omega_f = 1;
om2_f = omega_f^2;

% sampling time
dt = 0.01;

% process noise
Qd = diag([0 0 1e-2 1e-3 1e-3]); % for EKF 
Qr = diag([0 0 0 1e-6 1e-6]); % for real system

% measurement noise
Rd = 4e-2;

% system initial state
x0 = [0; 0.1; xref2; gamma; om2];
P0 = diag([1 1 100 1 100]);

% EKF initialization
xbar = x0 + sqrt(P0)*randn(5,1);
Pbar = P0;

% collections
X = [];
XHAT = [];
PHAT = [];
Z = [];
ZHAT = [];
SBAR = [];
TIME = [];

% EKF propagation -----------------------------------
for k = 0:15000
    
    t = dt*k
    
    % measurement
    z = vanderpol_meas(x0, Rd, 'sy');
    
    % MU
    [xhat, Phat, zhat, S] = vanderpol_ekf_mu(z, xbar, Pbar, Rd);
    
    % TU
    [xbar, Pbar] = vanderpol_ekf_tu(xhat, Phat, Qd, dt);
    
    % van der Pol oscillator
    x = vanderpol_dyn(x0, Qr, dt, 'sy');
    
    if t == 50
        x(4) = gamma_f; % step change in friction coef.
    end    
    if t == 100
        x(5) = om2_f; % step change in natural freq.
    end
    
    % collections
    X = [X; x0'];
    XHAT = [XHAT; xhat'];
    PHAT = [PHAT; (diag(Phat))'];
    Z = [Z; z'];
    ZHAT = [ZHAT; zhat'];
    SBAR = [SBAR; (diag(S))'];
    TIME = [TIME; t];
        
    % for next step
    x0 = x;
   
end
    