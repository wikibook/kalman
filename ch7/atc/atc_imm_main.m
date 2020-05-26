%
% ATC IMM-2CV model main
%

clear all


% load simulation parameters
atc_sim_para; 

% two KF initialization
for jj = 1:kf_num
    xhat_imm(:,jj) = xhat;
    Phat_imm(:,:,jj) = Phat;
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
    
    % mixing probability
    piw = PPII'*ww0;
    mix_prob = PPII./(piw'.*ones(kf_num, kf_num)).*(ww0.*ones(kf_num,kf_num));
    
    % mixing estimates and covariance
    xhat0_imm = xhat_imm * mix_prob;
    for jj = 1:kf_num
        Phat0_imm(:,:,jj) = zeros(nn,nn);
        for ii = 1:kf_num
            Phat0_imm(:,:,jj) = Phat0_imm(:,:,jj) + ...
                mix_prob(ii,jj) * (Phat_imm(:,:,ii) + ...
                (xhat_imm(:,ii)-xhat0_imm(:,jj)) * (xhat_imm(:,ii)-xhat0_imm(:,jj))');
        end
    end

    % one-step KF 1
    [xhat_imm(:,1), Phat_imm(:,:,1), zhat1, S1] = atc_kf(xhat0_imm(:,1), Phat0_imm(:,:,1), z, Qd1, Rd, dt);
    % one-step KF 2
    [xhat_imm(:,2), Phat_imm(:,:,2), zhat2, S2] = atc_kf(xhat0_imm(:,2), Phat0_imm(:,:,2), z, Qd3, Rd, dt);
    
    % Likelihood function
    LL = [gauss_pdf(z, zhat1, S1); 
          gauss_pdf(z, zhat2, S2)];
    
    % model probability update
    piw_sum = LL' * piw;
    ww = LL.*piw / piw_sum;
    
    % fusion
    xhat = xhat_imm * ww;
    
    Phat = zeros(nn,nn);
    for jj = 1:kf_num
        Phat = Phat + ...
        ww(jj) * (Phat_imm(:,:,jj) + ...
        (xhat_imm(:,jj)-xhat) * (xhat_imm(:,jj)-xhat)');
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
    