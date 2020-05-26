%
% ATC simulation parameters
%

% sampling time
dt = 5;
t1 = 50; % first turn
t2 = 185; 
t3 = 400; % second turn
t4 = 445; 
t5 = 600;



Om1 = 1 * pi/180; % for 135 sec
Om2 = 3 * pi/180; % for 45 sec

% process noise of real system (m/s^2)
qc_real = 0.0 * sqrt(dt);
Qr = qc_real^2 * [dt^3/3  dt^2/2  0       0;
                 dt^2/2  dt      0       0;
                 0       0       dt^3/3  dt^2/2;
                 0       0       dt^2/2   dt ];

% target initial state
x0 = [10000; 0; 0;  120];
P0 = diag([100^2, 10^2, 100^2, 10^2]);

% KF initialization ----------------------
xhat = x0 + sqrt(P0) * randn(4,1);
Phat = P0;

% measurement noise
Rd = 50^2 * eye(2);

% filter process noise (three sets)
qc_f1 = 0.1 * sqrt(dt);
Qd1 = qc_f1^2 * [dt^3/3  dt^2/2  0       0;
                 dt^2/2  dt      0       0;
                 0       0       dt^3/3  dt^2/2;
                 0       0       dt^2/2   dt ];


qc_f2 = 1.5 * sqrt(dt);
Qd2 = qc_f2^2 * [dt^3/3  dt^2/2  0       0;
                 dt^2/2  dt      0       0;
                 0       0       dt^3/3  dt^2/2;
                 0       0       dt^2/2   dt ];
             

qc_f3 = 3 * sqrt(dt);
Qd3 = qc_f3^2 * [dt^3/3  dt^2/2  0       0;
                 dt^2/2  dt      0       0;
                 0       0       dt^3/3  dt^2/2;
                 0       0       dt^2/2   dt ];
             
             
if 0 %----------------------------------------------------      
% filter discretime process noise (three sets)
sigma_f1 = 0.1;
Qd1 = sigma_f1^2 * [dt^4/4  dt^3/2  0       0;
                 dt^2/3  dt^2      0       0;
                 0       0       dt^4/4  dt^3/2;
                 0       0       dt^3/2   dt^2 ];


sigma_f2 = 1.5;
Qd2 = sigma_f2^2 * [dt^4/4  dt^3/2  0       0;
                 dt^2/3  dt^2      0       0;
                 0       0       dt^4/4  dt^3/2;
                 0       0       dt^3/2   dt^2 ];
             

sigma_f3 = 2;
Qd3 = sigma_f3^2 * [dt^4/4  dt^3/2  0       0;
                 dt^2/3  dt^2      0       0;
                 0       0       dt^4/4  dt^3/2;
                 0       0       dt^3/2   dt^2 ];
             
end %-------------------------------------------------------------
  
% model probability initialization
kf_num = 2; % number of KF
ww0 = ones(kf_num,1)/kf_num;

% model transition probability for GPB1 and IMM
PPII = [0.85 0.15; 0.15 0.85];

