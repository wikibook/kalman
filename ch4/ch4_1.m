%
% example 4.1
%

clear all;

% intialization
x0=0;
P0=1;
Q=1;
R=1;

% initial collection
X = x0+sqrt(P0)*randn();
XHAT=x0;
PHAT = P0; 
XBAR = []; % start from k=1
PBAR = []; % start from k=1
Z = []; % start from k=1
TIME = 0;


% EKF propagation -----------------------------------
for t = 0:59
    
    % dynamics
    x = x0 + sqrt(Q)*randn();
    z = x + randn(R)*randn();
    
    % TU
    xbar = x0;
    Pbar = P0 + Q;
    
    % MU
    K = Pbar/(Pbar+R);
    xhat = xbar + K*(z - xbar);
    Phat = Pbar - Pbar^2/(Pbar+R);
    
    % collections
    X = [X; x];
    XHAT = [XHAT; xhat];
    XBAR = [XBAR; xbar];
    PHAT = [PHAT; Phat];
    PBAR = [PBAR; Pbar];
    Z = [Z; z];
    TIME = [TIME; t+1];
    
    % for next step
    x0 = x;
    P0 = Phat;
    xbar = xhat;
    
end


% plotting
figure, plot(TIME, X-XHAT,'b', TIME, sqrt(PHAT),'r', TIME,-sqrt(PHAT),'r')
figure, plot(TIME, X,'r',TIME, XHAT,'b', TIME(2:end,1),Z,'k')
figure, plot(TIME(2:end,1), PBAR,'r',TIME, PHAT,'b')