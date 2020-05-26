%
% ATC GPB1-2CV model main
%

clear all

% load simulation parameters
atc_sim_para; 

% two KF initialization
for jj = 1:kf_num
    xhat_gpb(:,jj) = xhat;
    Phat_gpb(:,:,jj) = Phat;
end

[nn, mm]= size(xhat);

% collections
X = [];
XHAT = [];
PHAT = [];
Z = [];
ZHAT = [];
SBAR = [];
TIME = [];

WW = [];


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
    

    % one-step KF 1
    [xhat_gpb(:,1), Phat_gpb(:,:,1), zhat1, S1] = atc_kf(xhat, Phat, z, Qd1, Rd, dt);
    % one-step KF 2
    [xhat_gpb(:,2), Phat_gpb(:,:,2), zhat2, S2] = atc_kf(xhat, Phat, z, Qd3, Rd, dt);
    
    % Likelihood function
    LL = [gauss_pdf(z, zhat1, S1); 
          gauss_pdf(z, zhat2, S2)];
    
    % model probability update
    piw = PPII'*ww0;
    piw_sum = LL' * piw;
    ww = LL.*piw / piw_sum;
    
    
    % fusion
    xhat = xhat_gpb * ww;
    
    Phat = zeros(nn,nn);
    for jj = 1:kf_num
        Phat = Phat + ...
        ww(jj) * (Phat_gpb(:,:,jj) + ...
        (xhat_gpb(:,jj)-xhat) * (xhat_gpb(:,jj)-xhat)');
    end
       
    % collections
    X = [X; x'];
    XHAT = [XHAT; xhat'];
    PHAT = [PHAT; (diag(Phat))'];
    Z = [Z; z'];
    ZHAT = [ZHAT; zhat1'];
    SBAR = [SBAR; (diag(S1))'];
    TIME = [TIME; t];
    
    WW = [WW; ww'];
    
    % for next step
    x0 = x;
    ww0 = ww;
    
end
    