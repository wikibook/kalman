%
% 4.9 ¿¹Á¦ steady-state KF
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
Z = [];
ZHAT = [];
TIME = [];

% system matrix
F = [1 dt; 0 1];
H = [1 0];

% steady-state solution
[Pinf X_, G_] = dare(F', H', Qf, R);
Sinf = H * Pinf * H' + R;
Kinf = Pinf * H' * inv(Sinf);
Phat = Pinf - Pinf * H' * inv(Sinf) * H * Pinf;

for time = 0:100
    
    % measurement
    z = H * x0 +  sqrt(R) * randn();
    
    % MU
    zhat = H * xbar; 
    xhat = xbar + Kinf * (z - zhat);
    
    % TU
    xbar = F * xhat;
    
    % system dynamics
    x = F * x0 + sqrt(Q) * randn(2,1);
       
    % collections
    X = [X; x0'];
    XHAT = [XHAT; xhat'];
    Z = [Z; z'];
    ZHAT = [ZHAT; zhat'];
    TIME = [TIME; time];
    
    % for next step
    x0 = x;
    
end

% plotting
figure, plot(TIME, X(:,1),'r', TIME, XHAT(:,1),'b'),title('x1')
figure, plot(TIME, X(:,2),'r', TIME, XHAT(:,2),'b'),title('x2')
figure, plot(TIME, X(:,1)-XHAT(:,1),'r',TIME,sqrt(Phat(1,1))*ones(size(TIME)),'b',TIME,-sqrt(Phat(1,1))*ones(size(TIME)),'b'),title('x1')
figure, plot(TIME, X(:,2)-XHAT(:,2),'r',TIME,sqrt(Phat(2,2))*ones(size(TIME)),'b',TIME,-sqrt(Phat(2,2))*ones(size(TIME)),'b'),title('x2')
