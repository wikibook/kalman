%
% Missile seeker UKF main
%

clear all

% constants
params.kappa = -3;

% sampling time
dt = 0.01;

% process noise
Qc = 5^2 * diag([1 1 0]);
Qd = Qc * dt;

% measurement noise
Rd = diag([5^2  (pi/180)^2 (pi/180)^2]);

% target initial state
rT0 =  [20000; 30000; 0];
VT = 20; % target speed
thetaT = 0; psiT = 120 * pi/180;
cp = cos(psiT); ct = cos(thetaT);
sp = sin(psiT); st = sin(thetaT);
vT0 = VT * [cp*ct; sp*ct; -st];

P0 = diag([200^2 200^2 200^2 10^2 10^2 0]);
rvT0 =[rT0; vT0];

% missile initial state
rM0 = [0; 0; -500];
VM = 270; % missile speed
thetaM = 0; psiM = 0;
cp = cos(psiM); ct = cos(thetaM);
sp = sin(psiM); st = sin(thetaM);
vM0 = VM * [cp*ct; sp*ct; -st];
rvM0 = [rM0; vM0];

% UKF initialization
xhat = (rvT0 - rvM0) + sqrt(P0) * randn(6,1);
Phat = diag([200^2 200^2 200^2 10^2 10^2 10^2]);

% initial guidance law
rhat = xhat(1:3,1);
vhat = xhat(4:6,1);
a_cmd0 = missile_guidance(rhat, vhat, vM0);

% collections
POST = [];
POSM = [];
X = [];
XHAT = [];
PHAT = [];
Z = [];
ZHAT = [];
SBAR = [];
ACMD = [];
TIME = [];


% UKF propagation -----------------------------------
for k = 1:20000
    
    t = dt*k
    
    % TU
   
    [xbar, Pbar] = seeker_ukf_tu(xhat, Phat, a_cmd0, Qd, dt, params);
    
    % missile dynamics
    [rM, vM, Cbn] = missile_dyn(rM0, vM0, a_cmd0, dt);
    
    % target dynamics
    [rT, vT] = target_dyn(rT0, vT0, Qd, dt, 'sy');
    
    % seeker measurement
    r_rel = rT-rM; 
    z = seeker_meas(r_rel, Cbn, Rd, 'sy');
    
    % MU
    [xhat, Phat, zhat, S] = seeker_ukf_mu(z, xbar, Pbar, Rd, Cbn, params);
    
    % guidance law
    rhat = xhat(1:3,1);
    vhat = xhat(4:6,1);
    a_cmd = missile_guidance(rhat, vhat, vM);
    
    a_cmd_b = Cbn * a_cmd; % acceleration command in body frame 
    % collections
    X = [X; rT'-rM' vT'-vM'];
    XHAT = [XHAT; xhat'];
    PHAT = [PHAT; (diag(Phat))'];
    Z = [Z; z'];
    ZHAT = [ZHAT; zhat'];
    SBAR = [SBAR; (diag(S))'];
    ACMD = [ACMD; a_cmd_b'];
    TIME = [TIME; t];
    
    POST = [POST; rT'];
    POSM = [POSM; rM'];
    
    % for next step
    rM0 = rM;
    vM0 = vM;
    rT0 = rT;
    vT0 = vT;
    a_cmd0 = a_cmd;
    
    % terminal condition
    if norm(rT-rM) < 5
        break;
    end
    
end
    