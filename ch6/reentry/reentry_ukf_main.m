%
% Reentry UKF main
%

%clear all

% constants
params.R0 = 6374;
params.H0 = 9.3; 
params.beta0 = 0.59783;
params.mu = 3.986e5;
params.xR = 6374;
params.yR = 0;
params.kappa = -2;

% sampling time
dt = 0.05;

% process noise
Qcf = diag([2.4065e-4 2.4065e-4 1e-5]); % 2.4064e-5 * diag([1 1 0]);
Qd = Qcf * dt;

Qcr = diag([2.4065e-4 2.4065e-4 0]); % real % 2.4064e-5 * diag([1 1 0]);
Qr = Qcr * dt;

% measurement noise
Rd = diag([1e-6  (17e-3)^2]);

% system initial state
x_mean0 = [6500.4; 349.14; -1.8093; -6.7967; 0.6932];
P0 = 1e-6 * diag([1 1 1 1 0]);
x0 = x_mean0 + sqrt(P0) * randn(5,1);

% UKF initialization
xhat = [6500.4; 349.14; -1.8093; -6.7967; 0];
Phat = diag([1e-6 1e-6 1e-6 1e-6 1]);

% collections
X = [];
XHAT = [];
PHAT = [];
Z = [];
ZHAT = [];
SBAR = [];
TIME = [];

% UKF propagation -----------------------------------
for k = 1:4000
    
    t = dt*k;
    
    % TU
    [xbar, Pbar] = reentry_ukf_tu(xhat, Phat, Qd, dt, params);
    
    % Reentry dynamics
    x = reentry_dyn(x0, Qr, dt, params, 'sy');
    
    % for next step
    x0 = x;
    xhat = xbar;
    Phat = Pbar;
    
    if mod(k,2) == 0 % (measurement rate 10 Hz)
        
        % measurement
        z = reentry_meas(x, Rd, params, 'sy');
    
        % MU
        [xhat, Phat, zhat, S] = reentry_ukf_mu(z, xbar, Pbar, Rd, params);
        
        % collections
        X = [X; x'];
        XHAT = [XHAT; xhat'];
        PHAT = [PHAT; (diag(Phat))'];
        Z = [Z; z'];
        ZHAT = [ZHAT; zhat'];
        SBAR = [SBAR; (diag(S))'];
        TIME = [TIME; t];
        
    end
    
    
end
    