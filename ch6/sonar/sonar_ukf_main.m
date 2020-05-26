%
% Sonar UKF main
%

%clear all

% constants
params.kappa = -1;

% sampling time
dt = 60; % 60 sec = 1 min
t1 = 13*60; % 13 min
t2 = 17*60; % 17 min

% process noise (m/s^2)
Qd = diag([(1.6e-3)^2 (1.6e-3)^2]); % for UKF
Qr = diag([(1e-3)^2 (1e-3)^2]); % real

% measurement noise
sig_psi = 1.5 * pi/180;
Rd =sig_psi^2;

% observer initial state
rOb = [0; 0];
VOb = 2.57; % m/s
psiOb_1 = 140 * pi/180;
psiOb_2 = 20 * pi/180;
psiOb_dot = -0.5 * pi/180;

rOb0 = [0; 0];

vOb0 =[VOb * sin(psiOb_1);
      VOb * cos(psiOb_1) ];

% target initial state
rbar_ini =  5000; % m
sig_range = 2000;
psi_ini = 80 *pi/180; % initial bearing
VT = 2.056; % target speed (m/s)
sig_speed = 1.028; % m/s
psiT = -140 * pi/180;
sig_azimuth = 51.96 * pi/180;

rT0 = [rbar_ini * sin(psi_ini) ;
      rbar_ini * cos(psi_ini)] + rOb;

vT0 =[ VT * sin(psiT);
      VT * cos(psiT)];

% system initialization
x0 = [rT0 - rOb0;
      vT0 - vOb0];

Ppos11 = sig_range^2 * (sin(psi_ini))^2 + rbar_ini^2 * sig_psi^2 * (cos(psi_ini))^2;
Ppos22 = sig_range^2 * (cos(psi_ini))^2 + rbar_ini^2 * sig_psi^2 * (sin(psi_ini))^2;
Ppos12 = (sig_range^2 -rbar_ini^2 * sig_psi^2) * sin(psi_ini) * cos(psi_ini);
Ppos = [Ppos11 Ppos12; Ppos12 Ppos22];

Pvel11 = sig_speed^2 * (sin(psiT))^2 + VT^2 * sig_azimuth^2 * (cos(psiT))^2;
Pvel22 = sig_speed^2 * (cos(psiT))^2 + VT^2 * sig_azimuth^2 * (sin(psiT))^2;
Pvel12 = (sig_speed^2 -VT^2 * sig_azimuth^2) * sin(psiT) * cos(psiT);
Pvel = [Pvel11 Pvel12; Pvel12 Pvel22];

P0 = [Ppos zeros(2,2); zeros(2,2) Pvel];

% UKF initialization
xbar = x0 + sqrtm(P0) * randn(4,1);
Pbar = P0;

% collections
POST = [];
POSOb = [];
X = [];
XHAT = [];
PHAT = [];
Z = [];
ZHAT = [];
SBAR = [];
TIME = [];


% UKF propagation -----------------------------------
for k = 0:30
    
    t = dt*k;
    
    % sonar measurement
    r_rel = rT0-rOb0; 
    z = sonar_meas(r_rel, Rd, 'sy');
    
    % MU
    [xhat, Phat, zhat, S] = sonar_ukf_mu(z, xbar, Pbar, Rd, params);
    
     % observer input
    if t < t1
        vxOb = VOb * sin(psiOb_1);
        vyOb = VOb * cos(psiOb_1); 
        vxOb_af = vxOb;
        vyOb_af = vyOb;
    elseif t < t2
        vxOb = VOb * sin(psiOb_1 + psiOb_dot * (t-t1));
        vyOb = VOb * cos(psiOb_1 + psiOb_dot * (t-t1));
        vxOb_af = VOb * sin(psiOb_1 + psiOb_dot * (t-t1+dt));
        vyOb_af = VOb * cos(psiOb_1 + psiOb_dot * (t-t1+dt));
    else
        vxOb = VOb * sin(psiOb_2);
        vyOb = VOb * cos(psiOb_2); 
        vxOb_af = vxOb;
        vyOb_af = vyOb;
    end
    
    U = [0; 0;
         vxOb_af - vxOb;
         vyOb_af - vyOb ];
    
    % TU
    [xbar, Pbar] = sonar_ukf_tu(xhat, U, Phat, Qd, dt, params);
    
     % observer dynamics
    [rOb, vOb] = observer_dyn(rOb0, vOb0, U, dt);
    
    % target dynamics
    [rT, vT] = target_dyn(rT0, vT0, Qr, dt, 'sy');
    
    % collections
    X = [X; rT0'-rOb0' vT0'-vOb0'];
    XHAT = [XHAT; xhat'];
    PHAT = [PHAT; (diag(Phat))'];
    Z = [Z; z'];
    ZHAT = [ZHAT; zhat'];
    SBAR = [SBAR; (diag(S))'];
    TIME = [TIME; t];
    
    POST = [POST; rT0'];
    POSOb = [POSOb; rOb0'];
    
    
    % for next step
    rOb0 = rOb;
    vOb0 = vOb;
    rT0 = rT;
    vT0 = vT;
    
end
    