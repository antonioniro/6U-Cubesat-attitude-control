clear all; close all; clc;

addpath('aerodynamics')
addpath('magnetic_field') 
load Sat_Aeromodel_coarse25step.mat
load_system("quest_initial_state.slx")
addpath('C:\Users\pietr\OneDrive - Università di Napoli Federico II\MAGISTRALE\II ANNO\SPACECRAFT DYNAMICS AND CONTROL\Project\SPICE\mice\src\mice')
addpath('C:\Users\pietr\OneDrive - Università di Napoli Federico II\MAGISTRALE\II ANNO\SPACECRAFT DYNAMICS AND CONTROL\Project\SPICE\mice\lib')
which mice
cspice_furnsh('C:\Users\pietr\OneDrive - Università di Napoli Federico II\MAGISTRALE\II ANNO\SPACECRAFT DYNAMICS AND CONTROL\Project\SPICE\Kernels_SPICE\naif0012.tls')
cspice_furnsh('C:\Users\pietr\OneDrive - Università di Napoli Federico II\MAGISTRALE\II ANNO\SPACECRAFT DYNAMICS AND CONTROL\Project\SPICE\Kernels_SPICE\de430.bsp')
cspice_furnsh('C:\Users\pietr\OneDrive - Università di Napoli Federico II\MAGISTRALE\II ANNO\SPACECRAFT DYNAMICS AND CONTROL\Project\SPICE\Kernels_SPICE\pck00010.tpc');
cspice_furnsh('C:\Users\pietr\OneDrive - Università di Napoli Federico II\MAGISTRALE\II ANNO\SPACECRAFT DYNAMICS AND CONTROL\Project\SPICE\Kernels_SPICE\earth_assoc_itrf93.tf');
cspice_furnsh('C:\Users\pietr\OneDrive - Università di Napoli Federico II\MAGISTRALE\II ANNO\SPACECRAFT DYNAMICS AND CONTROL\Project\SPICE\Kernels_SPICE\earth_latest_high_prec.bpc');
cspice_furnsh('C:\Users\pietr\OneDrive - Università di Napoli Federico II\MAGISTRALE\II ANNO\SPACECRAFT DYNAMICS AND CONTROL\Project\SPICE\Kernels_SPICE\earth_latest_high_prec.bpc');
%% Satellite Parameters
Sat_mass=12.172; %satellite mass in kg;  Could be: Sat_mass    = 8.3; if 6U
Sat_Inertia=diag([6.295, 1.644, 5.462]*10^-1); %Inertia matrix (kg*m^2); Could be: Sat_Inertia = diag([0.0266, 0.0594, 0.0860]) if 6U
Magnetic_dipole=[0.1; 0.1;  0.1]; % The residual one of the spacecraft 6U, should add 0.01 of residual 
%% Initial Position and Attitude
%The position and velocity propagator requires position in km and velocity in km/s
% initial_position=[3766.120830239372,  -5888.669959510132, 0.000103180560200602];  %Initial position in ECI when in nighttime, SMA = 619 km;s
% initial_velocity=[-0.8699759584524811, -0.5563962371165492, 7.480504573790035]; %Initial velocity in ECI when in nighttime;
initial_position=[-805.2933204511233 , -515.0283481792998, 6924.330181946009 ];  %Initial position in ECI when in daytime, TA = 90 deg, SMA = 619 km
initial_velocity=[-4.068622299276111, 6.361658290515891, 1.390150232142416e-15]; %Initial velocity in ECI when in daytime;

ang_vel_init=convangvel([0 0 0],'deg/s','rad/s');
eul_0=convang([0 0 0],'deg','rad'); %Initial Euler angles: in order, yaw pitch roll
init = [eul_0,ang_vel_init];
[quat_0,ang_vel_init2]=initial_attitude(initial_position,initial_velocity,init);
init_state = [initial_position,initial_velocity,ang_vel_init2,quat_0]';

%% Parameters of the Earth
m_earth = 5.9736e24;  %mass;
G_earth = 6.6742e-11; %G;
r_earth = 6371e3;     %radius [m];
mu0 = 4*pi*1e-7;      %mu_0;


%% Earth Magnetic Field Model Model
nmax = 3;                               % maximum degree of harmonics 
mmax = 3;                               % maximum order of harmonics
[G, H] = IGRF95;                        % Gauss coefficients for IGRF
K = schmidt(nmax, mmax);                % Schmidt coefficient
%SNAP_magnets=[0.3;0.3;0.3];
DateVector=[2025 7 23 8 30 00];
Date_UTC = datestr(DateVector, 'yyyy-mm-ddTHH:MM:SS'); % oppure 'dd mmm yyyy HR:MN:SC'
ET0 = cspice_str2et(Date_UTC);
SNAP_starting_ECI2ECEF_dcm =dcmeci2ecef('IAU-2000/2006',DateVector);

%% Aerodynamic model parameters (in case of simplified Aerodynamic Model)
C_d=2.2;

%% Controller Feedback
quat_feed_reg.mi = 1;
quat_feed_reg.kd = 0.5*Sat_Inertia; 
quat_feed_reg.kp = 1.5*Sat_Inertia;
quat_feed_reg.ki = 0.001*Sat_Inertia;
quat_feed_reg.mi_forw = 1;
quat_feed_reg.I = Sat_Inertia;
%gains = [quat_feed_reg.kd;quat_feed_reg.kp;quat_feed_reg.ki];
info = Simulink.Bus.createObject(quat_feed_reg);
Quat_feed_reg = eval(info.busName);
clear(info.busName);
clear('info')

%% PD gains
approach = 2;
[K_pr, K_pp, K_py, K_dr, K_dp, K_dy] = evaluate_gains_nogyro(approach); % proportional gains are in N*m, 
% K_pp = K_pp/10;
% K_pr = K_pr * 4;
% K_dr = K_dr * 4;

% K_py = K_py*10;
K_pp = K_pp/2;
% K_dp = K_dp*2;
% derivate gains are in N*m*s
% K_pr = 1.5; % N*m
% K_dr = 1.4; % N*m*s
% K_pp = 1.5; % N*m
% K_dp = 1.4; % N*m*s
% K_py = 1.5; % N*m
% K_dy = 1.4; % N*m*s

% K_pr = 1.5; % N*m
% K_dr = 0.5; % N*m*s
% K_pp = 1.5; % N*m
% K_dp = 0.5; % N*m*s
% K_py = 1.5; % N*m
% K_dy = 0.5; % N*m*s

% K_pr = K_pr; % N*m
% K_dr = 0.5; % N*m*s
% K_pp = 1.5; % N*m
% K_dp = 0.5; % N*m*s
% K_py = 1.5; % N*m
% K_dy = 0.5; % N*m*s
%Satellite
%% Simplified Orbital Omega in ORF
mu = 398600;   %[km/s^2]
P = 2*pi*sqrt(6990^3/(mu));
omegaOrb = [0;-2*pi/P;0];

%% RWs configuration Building
beta1 = 45;
alfa1 = 45;
beta2 = 45;
alfa2 = 45;
beta3 = 45; 
alfa3 = 45;
beta4 = 45;
alfa4 = 45;

A_w = [cosd(beta1)*sind(alfa1)  -cosd(beta2)*sind(alfa2) -cosd(beta3)*sind(alfa3) cosd(beta4)*sind(alfa4)
       cosd(beta1)*cosd(alfa1)  +cosd(beta2)*cosd(alfa2) -cosd(beta3)*cosd(alfa3) -cosd(beta4)*cosd(alfa4)
       sind(beta1) sind(beta2) sind(beta3) sind(beta4)];

A_wP=pinv(A_w);
RWs_Inertia=diag([9.0718e-06, 9.0718e-06, 9.0718e-06, 9.0718e-06]); % Reaction wheels' inertia [kg*m^2];
RWs_Inertia_Inv=inv(RWs_Inertia);
omega_max = 1047.197551 ; %247.3717 rad/s 
h_max = omega_max * RWs_Inertia(1); %N*m*s
threshold_omega = omega_max * 0.8; % maximum angular velocity of the wheel

%% Magnetotorquers
d_min = 0; % A*m^2
d_max = 0.29; % A*m^2

%% Sim Parameters
tsim = ceil(4*P); %4 orbits, in seconds

%% Simplified disturbance model;
% 
% PSAstart = 3000;
% PSAend = 16500;
% PropulsiveMoment = zeros(tsim,3);
% PropulsiveMoment(PSAstart+1:PSAend,:) = repmat([0,-0.000021685,-0.000014425],PSAend-PSAstart,1);
% PropulsiveMoment = timeseries(PropulsiveMoment);



%% Process model covariance
Qattitudefactor=1e-3; % tuning if necessary, era 10^-6
Qangvelfactor=1e-7; % tuning if necessary, era 10^-6

Q=[eye(3,3).*Qattitudefactor zeros(3,3);
   zeros(3,3) eye(3,3).*Qangvelfactor];

%% Sun sensors 
SS_number = 6;
FOV = 166; %deg
%SigmaangularSun = 0.2; %deg, sigma angular error SS
SigmaangularSun = 0.1;
MeanangularSun = 0; %deg, mean angular error SS

Sun_sensors = zeros(3, SS_number); %matrix of the SS normals
% Sun sensor 1
Sun_sensors(:,1) = [0, 0, 1];

% Sun sensor 2
Sun_sensors(:,2) = [0, 1, 0];

% Sun sensor 3
Sun_sensors(:,3) = [1, 0, 0];

% Sun sensor 4
Sun_sensors(:,4)= [0, 0, -1];

% Sun sensor 5
Sun_sensors(:,5) = [0, -1, 0];

% Sun sensor 6
Sun_sensors(:,6) = [-1, 0, 0];


%% Magnetometers surface normals
Magnetometers_number = 2;
%SigmaangularMag = 0.4; %deg, sigma angular error magnetometers
%SigmaangularMag = 0;
%MeanangularMag = 0; %deg, mean angular error magnetometers
Sigma_cart_Mag = (50*10^-9)/3; % noise per channel in Tesla
%Sigma_cart_Mag = (5*10^-9)/3; % noise per channel in Tesla

Mean_cart_Mag = 0; %Tesla
%% Measurement model settings
% Observation model covariance values

R_SS = (SigmaangularSun*pi/180)^2;
R_B = (Sigma_cart_Mag)^2;

% R_SS = (10^-12);
% R_B = (10^-12);

%% Data for SRP
P_sun = 4.56e-6; % N/m^2 a 1 AU

%% Initialization e tuning parameters setting

% Sigmaanginit = 1; % Degree
% Sigma_omegainit = 2e-3; % rad/s
% 
% % Attitude
% axiserrinit=rand(1,3);
% axiserrinit=axiserrinit./norm(axiserrinit);
% phidqinit=randn(1)*Sigmaanginit;
% dqerrinit=[axiserrinit'.*sind(phidqinit/2);cosd(phidqinit/2)];
% qinit = q_product(dqerrinit,quat_0');
% qinit = qinit./norm(qinit);% Normalizzazione per sicurezza
% % Angular velocity
% omegaIBinit =ang_vel_init2+Sigma_omegainit.*randn(1,3);
% 
% % Initial state and covariance
% x0 = [qinit; omegaIBinit'];
% x_prev = x0;
% 
% P0=[(Sigmaanginit*pi/180)^2 0 0 0 0 0;
%    0 (Sigmaanginit*pi/180)^2 0 0 0 0;
%    0 0 (Sigmaanginit*pi/180)^2 0 0 0;
%    0 0 0 (Sigma_omegainit)^2 0 0;
%    0 0 0 0 (Sigma_omegainit)^2 0;
%    0 0 0 0 0 (Sigma_omegainit)^2];
% % P0=[(10)^-12 0 0 0 0 0;
% %    0 (10)^-12  0 0 0 0;
% %    0 0 (10)^-12  0 0 0;
% %    0 0 0 (10)^-12 0 0;
% %    0 0 0 0 (10)^-12 0;
% %    0 0 0 0 0 (10)^-12];
% P_prev = P0;


out = sim('quest_initial_state.slx','StopTime','0');
P_dtheta=out.Pdtheta.signals.values;
P_dg = P_dtheta / 4;
q0 =out.q0.signals.values;
P_omega = eye(3)*10^-7;
omegaIBinit = omegaOrb;

x0 = [q0; omegaIBinit];
x_prev = x0;
P_prev = [P_dg, zeros(3,3);
          zeros(3,3), P_omega];
