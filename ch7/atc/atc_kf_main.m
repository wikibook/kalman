%
% ATC KF main
%

%clear all

% load simulation parameters
atc_sim_para; 

% collections
X = [];
XHAT = [];
PHAT = [];
Z = [];
ZHAT = [];
SBAR = [];
TIME = [];


% KF propagation -----------------------------------
for t = dt:dt:t5
    
    % target dynamics
    if t < t1
        Omega = 0;
    elseif t < t2
        Omega = Om1;
    elseif t < t3
        Omega = 0;
    elseif t < t4
        Omega = Om2;
    else
        Omega = 0;
    end
    
    x = aircraft_dyn(x0, Omega, Qr, dt, 'sy');
    
    % atc measurement
    z = atc_meas(x, Rd, 'sy');
    
    % one-step KF
    [xhat, Phat, zhat, S] = atc_kf(xhat, Phat, z, Qd2, Rd, dt);
    
    % collections
    X = [X; x'];
    XHAT = [XHAT; xhat'];
    PHAT = [PHAT; (diag(Phat))'];
    Z = [Z; z'];
    ZHAT = [ZHAT; zhat'];
    SBAR = [SBAR; (diag(S))'];
    TIME = [TIME; t];
    
    % for next step
    x0 = x;
    
end
    