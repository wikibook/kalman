%
% example 4.2
%
clear all
close all


% process and measrement noise
Q = 1;
R = 1;

% state initialization
xt0 = [1; 10]; % system
xm0 = 0;      % model
P0 = 1;

% KF initialization 
xhat = xm0 + sqrt(P0) * randn();
Phat = P0;

% collections
XT = xt0';
XHAT = xhat;
PHAT = Phat;
KK = [];
Z = [];
ZHAT = [];
SBAR = [];
TIME = 0;


for time=1:50
    
    % true system
    xt = [1 1;0 1] * xt0;
    z = [1 0] * xt +  sqrt(R) * randn();
       
    % TU
    xbar = xhat; 
    Pbar = Phat + Q;
    
    % MU
    zhat = xbar; 
    S = Pbar + R;
    Phat = Pbar - Pbar * Pbar/ S;
    K = Pbar / S;
    xhat = xbar + K * (z - zhat);
    
    % collections
    XT = [XT; xt'];
    XHAT = [XHAT; xhat];
    PHAT = [PHAT; Phat];
    Z = [Z; z'];
    ZHAT = [ZHAT; zhat'];
    SBAR = [SBAR; S];
    TIME = [TIME; time];
    KK = [KK; K];
    
    % for next step
    xt0 = xt;
    
end

% plotting
if 1
figure, plot(TIME, XT(:,1),'r', TIME, XHAT,'b')
figure, plot(TIME, XT(:,1)-XHAT,'r',TIME,sqrt(PHAT),'b',TIME,-sqrt(PHAT),'b'),title('x1')
end
%plot(TIME, XT(:,1),'r'), hold on
%plot(TIME(2:end),KK,'y'),hold on