%
% Missile check!!
%

clear all

% sampling time
dt = 0.01;

% process noise
Qc = 1 * diag([1 1 0]);
Qd = Qc * dt;


% target initial state
rT0 =  [20000; 30000; 0];
vT0 = [20; 10; 0];
P0 = diag([200^2 200^2 200^2 1^2 1^2 0^2]);
rvT0 =[rT0; vT0] + sqrt(P0) * randn(6,1);


% missile initial state
rM0 = [0; 0; -500];
vM_val = 270; % missile speed
thetaM = 0; psiM = 0;
cp = cos(psiM); ct = cos(thetaM);
sp = sin(psiM); st = sin(thetaM);
vM0 = vM_val * [cp*ct; sp*ct; -st];
rvM0 = [rM0; vM0];

%--------------------------------------------------
if 0  % aircraft target data

VT=200;
% Initial Pose (wrt inertial frame)
thetaT0=-10*pi/180;
psiT0=90*pi/180;

cp = cos(psiT0); ct = cos(thetaT0);
sp = sin(psiT0); st = sin(thetaT0);
vT0 = VT * [cp*ct; sp*ct; -st];
rT0=[10000 0 0]';
rvT0 = [rT0; vT0];

VM0=300;
% Initial Pose (wrt inertial frame)
thetaM0=0;
psiM0=0;
cp = cos(psiM0); ct = cos(thetaM0);
sp = sin(psiM0); st = sin(thetaM0);
vM0 = VM0 * [cp*ct; sp*ct; -st];
% Initial Position (wrt inertial frame)
rM0=[0  0  0]';
rvM0 = [rM0; vM0];

end
%----------------------------------------------------

% collections
POST = [];
POSM = [];
X = [];
Z = [];
ACMD = [];
TIME = [];


% EKF propagation -----------------------------------
for k = 1:18000
    
    t = dt*k
   
    % guidance law
    r_rel0 = rT0 - rM0;
    v_rel0 = vT0 - vM0;
    a_cmd = missile_guidance(r_rel0, v_rel0, vM0);
    
    % missile dynamics
    [rM, vM, Cbn] = missile_dyn(rM0, vM0, a_cmd, dt);
    
    % target dynamics
    [rT, vT] = target_dyn(rT0, vT0, Qd, dt, 'sy');
    
    % seeker measurement
    r_rel = rT-rM; 
    z = seeker_meas(r_rel, Cbn, zeros(3,3), 'kf');
    
    % collections
    X = [X; rT'-rM' vT'-vM'];
    Z = [Z; z'];
    ACMD = [ACMD; a_cmd'];
    TIME = [TIME; t];
    
    
    % for next step
    POST = [POST; rT'];
    POSM = [POSM; rM'];
    rM0 = rM;
    vM0 = vM;
    rT0 = rT;
    vT0 = vT;
    
    
    if norm(rT-rM)<5
        break;
    end
    
end
    