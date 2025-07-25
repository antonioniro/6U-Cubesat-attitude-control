function [K_pr, K_pp, K_py, K_dr, K_dp, K_dy] = evaluate_gains_nogyro(approccio)

load('dist_nominal_attitude.mat');

I=diag([6.295, 1.644, 5.462]*10^-1); % satellite inertia

T_tot_max1 = max(abs(T_ext1 - GGtorque1));
T_tot_max2 = max(abs(T_ext2 - GGtorque2));
T_tot_max3 = max(abs(T_ext3 - GGtorque3));

srp_max1 = max(abs(srp_torque1));
aerotorque_max1 = max(abs(aerotorque1));
T_mag_max1 = max(abs(T_mag1));

srp_max2 = max(abs(srp_torque2));
aerotorque_max2 = max(abs(aerotorque2));
T_mag_max2 = max(abs(T_mag2));

srp_max3 = max(abs(srp_torque3));
aerotorque_max3 = max(abs(aerotorque3));
T_mag_max3 = max(abs(T_mag3));

T_max1 = max([T_tot_max1,srp_max1, T_mag_max1, aerotorque_max1]);
T_max2 = max([T_tot_max2,srp_max2, T_mag_max2, aerotorque_max2]);
T_max3 = max([T_tot_max3,srp_max3, T_mag_max3, aerotorque_max3]);


% alpha_ss = deg2rad(0.1); % rad
% beta_ss = deg2rad(0.1); % rad
% gamma_ss = deg2rad(0.1); % rad, could be larger , questi sono i guadagni
% del pomeriggio del 16 luglio
% alpha_ss = deg2rad(0.25); % rad
% beta_ss = deg2rad(0.25); % rad, was 0.25
% gamma_ss = deg2rad(0.15); % rad

% gains per caso con gyros
alpha_ss = deg2rad(0.4); % rad
beta_ss = deg2rad(0.6); % rad, was 0.25
gamma_ss = deg2rad(0.5); % rad


if approccio == 2
    %% roll gains - 2nd approach
    K_p_roll_2 = T_max1/alpha_ss; % N*m
    tau_p_roll_2 = (4*I(1,1)/K_p_roll_2)^(0.5) % s, K_p = K_p_BI
    K_d_roll_2 = K_p_roll_2*tau_p_roll_2; % N*m*s 
    K_pr = K_p_roll_2; K_dr = K_d_roll_2; 

    %% pitch gains - 2nd approach
    K_p_pitch_2 = T_max2/beta_ss; % N*m
    tau_p_pitch_2 = (4*I(2,2)/K_p_pitch_2)^(0.5) % s, K_p = K_p_BI
    K_d_pitch_2 = K_p_pitch_2*tau_p_pitch_2; % N*m*s
    K_pp = K_p_pitch_2; K_dp = K_d_pitch_2; 

    %% yaw gains - 2nd approach
    K_p_yaw_2 = T_max3/gamma_ss; % N*m
    tau_p_yaw_2 = (4*I(3,3)/K_p_yaw_2)^(0.5) % s, K_p = K_p_BI
    K_d_yaw_2 = K_p_yaw_2*tau_p_yaw_2; % N*m*s
    K_py = K_p_yaw_2; K_dy = K_d_yaw_2;

else 
    tau_1 =45;
    %% roll gains - 1nd approach
    K_d_roll_1 = 4*I(1,1)/tau_1; % N*m*s
    K_p_roll_1 = K_d_roll_1/tau_1; % N*m
    roll_sse_1 = T_max1 / K_p_roll_1; 
    roll_sse_1_deg = rad2deg(roll_sse_1)
    K_dr = K_d_roll_1; K_pr = K_p_roll_1;

    %% pitch gains - 1nd approach
    K_d_pitch_1 = 4*I(2,2)/tau_1; % N*m*s
    K_p_pitch_1 = K_d_pitch_1/tau_1; % N*m
    pitch_sse_1 = T_max2 / K_p_pitch_1; 
    pitch_sse_1_deg = rad2deg(pitch_sse_1)
    K_dp = K_d_pitch_1; K_pp = K_p_pitch_1;

    %% yaw gains - 1nd approach
    K_d_yaw_1 = 4*I(3,3)/tau_1; % N*m*s
    K_p_yaw_1 = K_d_yaw_1/tau_1; % N*m
    yaw_sse_1 = T_max3 / K_p_yaw_1;
    yaw_sse_1_deg = rad2deg(yaw_sse_1)
    K_dy = K_d_yaw_1; K_py = K_p_yaw_1;

end
end
