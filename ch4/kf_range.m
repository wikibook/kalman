%
% 4.9 ¿¹Á¦
%
clear all
close all

% time step
dt = 1;

% process and measrement noise
Qc = 150;
Q = Qc * [dt^3/3 dt^2/2; dt^2/2 dt];
R = 30;

% state initialization
x0 = [2000; 10]; % system
P0 = 30 * eye(2);

% KF initialization 
xbar = x0 + sqrt(P0) * randn(2,1);
Pbar = P0;
Qf = Q;

% collections
X = [];
XHAT = [];
PHAT = [];
KK = [];
Z = [];
ZHAT = [];
SBAR = [];
TIME = [];

% system matrix
F = [1 dt; 0 1];
H = [1 0];

for time = 0:100
    
    % measurement
    z = H * x0 +  sqrt(R) * randn();
    
    % MU
    zhat = H * xbar; 
    S = H * Pbar * H' + R;
    Phat = Pbar - Pbar * H' * inv(S) * H * Pbar;
    K = Pbar * H' * inv(S);
    xhat = xbar + K * (z - zhat);
    
    % TU
    xbar = F * xhat;
    Pbar = F * Phat * F' + Qf;
    
    % system dynamics
    x = F * x0 + sqrt(Q) * randn(2,1);
       
    % collections
    X = [X; x0'];
    XHAT = [XHAT; xhat'];
    PHAT = [PHAT; diag(Phat)'];
    Z = [Z; z'];
    ZHAT = [ZHAT; zhat'];
    SBAR = [SBAR; diag(S)'];
    TIME = [TIME; time];
    KK = [KK; K'];
    
    % for next step
    x0 = x;
    
end

% plotting
figure, plot(TIME, X(:,1),'r', TIME, XHAT(:,1),'b'),title('x1')
figure, plot(TIME, X(:,2),'r', TIME, XHAT(:,2),'b'),title('x2')
figure, plot(TIME, X(:,1)-XHAT(:,1),'r',TIME,sqrt(PHAT(:,1)),'b',TIME,-sqrt(PHAT(:,1)),'b'),title('x1')
figure, plot(TIME, X(:,2)-XHAT(:,2),'r',TIME,sqrt(PHAT(:,2)),'b',TIME,-sqrt(PHAT(:,2)),'b'),title('x2')
figure, plot(TIME, Z(:,1)-ZHAT(:,1),'r',TIME,sqrt(SBAR(:,1)),'b',TIME,-sqrt(SBAR(:,1)),'b'),title('r')
figure, plot(TIME, KK(:,1),'r',TIME,KK(:,2),'b'),title('Gain')
