% Definizione colori personalizzati
if ~exist('plots', 'dir')
    mkdir('plots');
end

yellow = [0.93, 0.69, 0.13];
orange = [0.85, 0.33, 0.10];
lightblue = [0.30, 0.75, 0.93];
green = [ 0 1 0];

time = out.T_ext.Time;
%% RWs velocities
omega_wheels = out.omega_wheels.Data;
omega_wheels_1 = omega_wheels(1,:);
omega_wheels_2 = omega_wheels(2,:);
omega_wheels_3 = omega_wheels(3,:);
omega_wheels_4 = omega_wheels(4,:);
figure;
plot(time, omega_wheels_1, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', '\omega_w_1');
hold on;
plot(time, omega_wheels_2, 'Color', orange, 'LineWidth', 1.5, 'DisplayName', '\omega_w_2');
plot(time, omega_wheels_3, 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', '\omega_w_3');
plot(time, omega_wheels_4, 'Color', green, 'LineWidth', 1.5, 'DisplayName', '\omega_w_4');

hold off;

xlabel('Time (s)','FontSize',20);
xlim([0 23265 ]);
ylabel('Angular velocities (rad/s)','FontSize',20);
title('Reaction wheels angular velocities','FontSize',16);
legend('Location','southwest','FontSize',12);
set(gca, 'FontSize', 18)
grid on;
saveas(gcf, fullfile('plots', 'RWs_angular_velocities.png'));
%% Magnetorquer d required
ac_req = out.actuators_req.Data;
ac_req_x = ac_req(1,:);
ac_req_y = ac_req(2,:);
ac_req_z = ac_req(3,:);

figure;
plot(time, ac_req_x, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Required d along x_B_R_F');
hold on;
plot(time, ac_req_y, 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Required d along y_B_R_F');
plot(time, ac_req_z, 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Required d along z_B_R_F');
hold off;

xlabel('Time (s)','FontSize',20);
xlim([0 23265 ]);
ylabel('Required d (Am^2)','FontSize',20);
title('Required d along the BRF axes ','FontSize',16);
legend('Location','southwest','FontSize',12);
set(gca, 'FontSize', 18)
grid on;
saveas(gcf, fullfile('plots', 'req_ac.png'));
%% Magnetorquers d effective
ac_true = out.actuators_true.Data;
ac_true_x = ac_true(1,:);
ac_true_y = ac_true(2,:);
ac_true_z = ac_true(3,:);

figure;
plot(time, ac_true_x, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Effective d along x_B_R_F');
hold on;
plot(time, ac_true_y, 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Effective d along y_B_R_F');
plot(time, ac_true_z, 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Effective d along z_B_R_F');
hold off;

xlabel('Time (s)','FontSize',20);
xlim([0 23265 ]);
ylabel('Effective d (Am^2)','FontSize',20);
title('Effective d along the BRF axes','FontSize',16);
legend('Location','southwest','FontSize',12);
set(gca, 'FontSize', 18)
grid on;
saveas(gcf, fullfile('plots', 'effective_d_.png'));
%% Angular velocities error
angular_omega_errors = out.omega_errors_.Data;
omega_errors_x = angular_omega_errors(:,1);
omega_errors_y = angular_omega_errors(:,2);
omega_errors_z = angular_omega_errors(:,3);

figure;
plot(time, omega_errors_x, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Errors on \omega_x');
hold on;
plot(time, omega_errors_y, 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Errors on \omega_y');
plot(time, omega_errors_z, 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Errors on \omega_z');
hold off;

xlabel('Time (s)','FontSize',20);
xlim([0 23265 ]);
ylabel('Angular velocities errors (rad/s)','FontSize',20);
title('Angular velocities errors ','FontSize',16);
legend('Location','southwest','FontSize',12);
set(gca, 'FontSize', 18)
grid on;
saveas(gcf, fullfile('plots', 'angular_velocities_errors.png'));
%% Magnetic field
mag_field = out.mag_field.Data;
mag_field_x = mag_field(1,:);
mag_field_y = mag_field(2,:);
mag_field_z = mag_field(3,:);

figure;
plot(time, mag_field_x, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'B_x');
hold on;
plot(time, mag_field_y, 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'B_y');
plot(time, mag_field_z, 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'B_z');
hold off;

xlabel('Time (s)','FontSize',20);
xlim([0 23265 ]);
ylabel('B_0 (T)','FontSize',20);
title('Magnetic field components in ORF','FontSize',16);
legend('Location','southwest','FontSize',12);
set(gca, 'FontSize', 18)
grid on;
saveas(gcf, fullfile('plots', 'mag_field_plot.png'));

%% Eclipse
eclipse = out.eclipse.Data;

figure;
plot(time, eclipse, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Eclipse data');
xlabel('Time (s)', 'FontSize', 20);
ylabel('Eclipse phase', 'FontSize', 20);
xlim([0 23265 ])
title('Eclipse phases vs time', 'FontSize', 16);
legend('Location', 'northeast', 'FontSize', 12);
set(gca, 'FontSize', 18)
grid on;

% Impostazione assi Y
ylim([-0.1, 1.2]); 
yticks([0 1]); 
yticklabels({'Eclipse', 'No eclipse'});

saveas(gcf, fullfile('plots', 'eclipse_data_plot.png'));
%% Angles
angle_sun_mag = out.angle_sun_mag.Data;
for i= 2:23265
    if angle_sun_mag(i+1)==angle_sun_mag(i)
        angle_sun_mag(i)=NaN;
    end
    diff = angle_sun_mag(i+1)- angle_sun_mag(i);
    if abs(diff )> 5
        angle_sun_mag(i)=NaN;
    end

end

figure;
plot(time, angle_sun_mag, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', ' data');
xlabel('Time (s)', 'FontSize', 20);
ylabel('S - B angle (°)', 'FontSize', 20);
xlim([450 2100 ])
ylim([0 180])
title('S - B angle', 'FontSize', 16);
set(gca, 'FontSize', 18)
grid on;

saveas(gcf, fullfile('plots', 'angle_sun_mag_plot.png'));
%% Torque esterno
T_ext = out.T_ext.Data; % 3 colonne
T_ext_x = T_ext(1,:);
T_ext_y = T_ext(2,:);
T_ext_z = T_ext(3,:);

figure;
plot(time, T_ext_x, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'x_BRF torque');
hold on;
plot(time, T_ext_y, 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'y_BRF torque');
plot(time, T_ext_z, 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'z_BRF torque');
hold off;

xlabel('Time (s)','FontSize',20);
ylabel('Torque (N·m)','FontSize',20);
title('External disturbance torque','FontSize',16);
legend('Location','best','FontSize',12);
grid on;
saveas(gcf, fullfile('plots', 'T_ext.png'));

%% Roll, pitch, yaw
roll = out.roll.Data;
pitch = out.pitch.Data;
yaw = out.yaw.Data;

figure;
plot(time, roll, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Roll');
hold on;
plot(time, pitch, 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Pitch');
plot(time, yaw, 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Yaw');
hold off;

xlabel('Time (s)','FontSize',20);
ylabel('Angle (°)','FontSize',20);
title('Roll, pitch, and yaw vs time','FontSize',16);
xlim([0 23266]);
legend('Location','northeast','FontSize',12);
grid on;
saveas(gcf, fullfile('plots', 'roll_pitch_yaw_plot.png'));

%% Roll, pitch, yaw error
roll_error = out.roll_error.Data;
pitch_error = out.pitch_error.Data;
yaw_error = out.yaw_error.Data;

figure;
plot(time, roll_error, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', '\alpha error');
hold on;
plot(time, pitch_error, 'Color', orange, 'LineWidth', 1.5, 'DisplayName', '\beta error');
plot(time, yaw_error, 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', '\gamma error');
hold off;
xlim([0 23266]);
xlabel('Time (s)','FontSize',20);
ylabel('Error (°)','FontSize',20);
title('\alpha, \beta and \gamma errors vs time', 'FontSize', 16);
legend('Location','best','FontSize',12);
grid on;
saveas(gcf, fullfile('plots', 'roll_pitch_yaw_error_plot.png'));

%% Sigma omega
%% Sigma omega (3 componenti)
sigma_omega = out.sigma_omega.Data; % 3 colonne
sigma_omega_x = sigma_omega(:,1);
sigma_omega_y = sigma_omega(:,2);
sigma_omega_z = sigma_omega(:,3);

figure;
plot(time, sigma_omega_x, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', '\sigma_{x}');
hold on;
plot(time, sigma_omega_y, 'Color', orange, 'LineWidth', 1.5, 'DisplayName', '\sigma_{y}');
plot(time, sigma_omega_z, 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', '\sigma_{z}');
hold off;

xlabel('Time (s)','FontSize',20);
ylabel('\sigma (rad/s)','FontSize',20);
title('\sigma of \omega vs time','FontSize',16);
xlim([0 23266]);
legend('Location','best','FontSize',12);
set(gca, 'FontSize', 18);
grid on;
saveas(gcf, fullfile('plots', 'sigma_omega_plot.png'));

%% Sigma theta attitude (3 componenti)
sigma_theta_attitude = out.sigma_theta_attitude.Data; % 3 colonne
figure;
plot(time, sigma_theta_attitude(:,1), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', '\sigma_{r}');
hold on;
plot(time, sigma_theta_attitude(:,2), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', '\sigma_{p}');
plot(time, sigma_theta_attitude(:,3), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', '\sigma_{y}');
hold off;

xlabel('Time (s)','FontSize',20);
xlim([0 23265 ]);
ylabel('\sigma (°)','FontSize',20);
title('\sigma vs time','FontSize',16);
legend('Location','northeast','FontSize',18);
set(gca, 'FontSize', 18);
grid on;
saveas(gcf, fullfile('plots', 'sigma_theta_attitude_plot.png'));

%% omega ECI
omega = out.omega_ECI.Data;
figure;
plot(time, omega(:,1), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', '\omega_x');
hold on;
plot(time, omega(:,2), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', '\omega_y');
plot(time, omega(:,3), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', '\omega_z');
hold off;

xlabel('Time (s)','FontSize',20);
ylabel('\omega (°/sec)','FontSize',20);
title('\omega  vs time','FontSize',16);
legend('Location','best','FontSize',12);
grid on;
xlim([0 23266]);
saveas(gcf, fullfile('plots', 'omega_ECI_plot.png'));
%% T control
T_control = out.T_Control.Data; % 3 colonne
T_control_x = T_control(:,1);
T_control_y = T_control(:,2);
T_control_z = T_control(:,3);

figure;
plot(time, T_control_x, 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Control Torque X');
hold on;
plot(time, T_control_y, 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Control Torque Y');
plot(time, T_control_z, 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Control Torque Z');
hold off;

xlabel('Time (s)','FontSize',20);
ylabel('Control Torque (N·m)','FontSize',20);
title('Control Torque vs Time','FontSize',16);
legend('Location','best','FontSize',12);
grid on;
saveas(gcf, fullfile('plots', 'T_control_plot.png'));


%% --- PLOT SOLO PER I PRIMI 3000 SECONDI ---
if ~exist(fullfile('plots', '3000_secs'), 'dir')
    mkdir(fullfile('plots', '3000_secs'));
end

idx_3000 = time <= 3000;
time_3000 = time(idx_3000);

% Torque Esterno
figure;
plot(time_3000, T_ext_x(idx_3000), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'x_BRF torque');
hold on;
plot(time_3000, T_ext_y(idx_3000), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'y_BRF torque');
plot(time_3000, T_ext_z(idx_3000), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'z_BRF torque');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Torque (N·m)', 'FontSize', 20); title('External Disturbance Torque (0–3000 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', '3000_secs', 'T_ext_0_3000.png'));

% Roll, Pitch, Yaw
figure;
plot(time_3000, roll(idx_3000), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Roll');
hold on;
plot(time_3000, pitch(idx_3000), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Pitch');
plot(time_3000, yaw(idx_3000), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Yaw');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Angle (degrees)', 'FontSize', 20); title('Roll, Pitch, Yaw (0–3000 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', '3000_secs', 'rpy_0_3000.png'));

% Roll, Pitch, Yaw Error
figure;
plot(time_3000, roll_error(idx_3000), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Roll Error');
hold on;
plot(time_3000, pitch_error(idx_3000), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Pitch Error');
plot(time_3000, yaw_error(idx_3000), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Yaw Error');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Error (degrees)', 'FontSize', 20); title('Roll, Pitch, Yaw Error (0–3000 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', '3000_secs', 'rpy_error_0_3000.png'));

% Sigma Omega
figure;
plot(time_3000, sigma_omega_x(idx_3000), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Sigma Omega X');
hold on;
plot(time_3000, sigma_omega_y(idx_3000), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Sigma Omega Y');
plot(time_3000, sigma_omega_z(idx_3000), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Sigma Omega Z');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Sigma Omega', 'FontSize', 20); title('Sigma Omega (0–3000 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', '3000_secs', 'sigma_omega_0_3000.png'));

% Sigma Theta Attitude
figure;
plot(time_3000, sigma_theta_attitude(idx_3000,1), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Sigma Theta 1');
hold on;
plot(time_3000, sigma_theta_attitude(idx_3000,2), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Sigma Theta 2');
plot(time_3000, sigma_theta_attitude(idx_3000,3), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Sigma Theta 3');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Sigma Theta', 'FontSize', 20); title('Sigma Theta Attitude (0–3000 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', '3000_secs', 'sigma_theta_0_3000.png'));

% Omega ECI
figure;
plot(time_3000, omega(idx_3000,1), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'omega x');
hold on;
plot(time_3000, omega(idx_3000,2), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'omega y');
plot(time_3000, omega(idx_3000,3), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'omega z');
hold off;

xlabel('Time (s)','FontSize',20);
ylabel('Omega ECI (degrees/sec)','FontSize',20);
title('Omega ECI vs Time (0-3000 s)','FontSize',16);
legend('Location','best','FontSize',12);
grid on;
saveas(gcf, fullfile('plots', '3000_secs', 'oemga_0_3000.png'));

% T Control
figure;
plot(time_3000, T_control_x(idx_3000), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Control Torque X');
hold on;
plot(time_3000, T_control_y(idx_3000), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Control Torque Y');
plot(time_3000, T_control_z(idx_3000), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Control Torque Z');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Control Torque (N·m)', 'FontSize', 20); title('Control Torque (0–3000 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', '3000_secs', 'T_control_0_3000.png'));

%% --- PLOT PER 3000–5140 SECONDI ---
if ~exist(fullfile('plots', 'eclipse'), 'dir')
    mkdir(fullfile('plots', 'eclipse'));
end

idx_eclipse = (time >= 2980) & (time <= 5140);
time_eclipse = time(idx_eclipse);

% Torque Esterno
figure;
plot(time_eclipse, T_ext_x(idx_eclipse), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Torque X');
hold on;
plot(time_eclipse, T_ext_y(idx_eclipse), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Torque Y');
plot(time_eclipse, T_ext_z(idx_eclipse), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Torque Z');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Torque (N·m)', 'FontSize', 20); title('External Disturbance Torque (2980–5140 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', 'eclipse', 'T_ext_2980_5140.png'));

% Roll, Pitch, Yaw
figure;
plot(time_eclipse, roll(idx_eclipse), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Roll');
hold on;
plot(time_eclipse, pitch(idx_eclipse), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Pitch');
plot(time_eclipse, yaw(idx_eclipse), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Yaw');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Angle (degrees)', 'FontSize', 20); title('Roll, Pitch, Yaw (2980–5140 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', 'eclipse', 'rpy_2980_5140.png'));

% Roll, Pitch, Yaw Error
figure;
plot(time_eclipse, roll_error(idx_eclipse), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Roll Error');
hold on;
plot(time_eclipse, pitch_error(idx_eclipse), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Pitch Error');
plot(time_eclipse, yaw_error(idx_eclipse), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Yaw Error');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Error (degrees)', 'FontSize', 20); title('Roll, Pitch, Yaw Error (2980–5140 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', 'eclipse', 'rpy_error_2980_5140.png'));

% Sigma Omega
figure;
plot(time_eclipse, sigma_omega_x(idx_eclipse), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Sigma Omega X');
hold on;
plot(time_eclipse, sigma_omega_y(idx_eclipse), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Sigma Omega Y');
plot(time_eclipse, sigma_omega_z(idx_eclipse), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Sigma Omega Z');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Sigma Omega', 'FontSize', 20); title('Sigma Omega (2980–5140 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', 'eclipse', 'sigma_omega_2980_5140.png'));

% Sigma Theta Attitude
figure;
plot(time_eclipse, sigma_theta_attitude(idx_eclipse,1), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Sigma Theta 1');
hold on;
plot(time_eclipse, sigma_theta_attitude(idx_eclipse,2), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Sigma Theta 2');
plot(time_eclipse, sigma_theta_attitude(idx_eclipse,3), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Sigma Theta 3');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Sigma Theta', 'FontSize', 20); title('Sigma Theta Attitude (2980–5140 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', 'eclipse', 'sigma_theta_2980_5140.png'));

% T Control
figure;
plot(time_eclipse, T_control_x(idx_eclipse), 'Color', yellow, 'LineWidth', 1.5, 'DisplayName', 'Control Torque X');
hold on;
plot(time_eclipse, T_control_y(idx_eclipse), 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Control Torque Y');
plot(time_eclipse, T_control_z(idx_eclipse), 'Color', lightblue, 'LineWidth', 1.5, 'DisplayName', 'Control Torque Z');
hold off;
xlabel('Time (s)', 'FontSize', 20); ylabel('Control Torque (N·m)', 'FontSize', 20); title('Control Torque (2980–5140 s)');
legend('Location','best'); grid on;
saveas(gcf, fullfile('plots', 'eclipse', 'T_control_2980_5140.png'));