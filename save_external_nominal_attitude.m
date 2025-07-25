%% save external dist (run after simulation)
T_mag1 = out.T_mag.Data(1,:);
T_mag2 = out.T_mag.Data(2,:);
T_mag3 = out.T_mag.Data(3,:);

GGtorque1 = out.GGtorque.Data(:,1)';
GGtorque2 = out.GGtorque.Data(:,2)';
GGtorque3 = out.GGtorque.Data(:,3)';

srp_torque1 = out.srp_torque.Data(1,:);
srp_torque2 = out.srp_torque.Data(2,:);
srp_torque3 = out.srp_torque.Data(3,:);

aerotorque1 = out.aerotorque.Data(:,1)';
aerotorque2 = out.aerotorque.Data(:,2)';
aerotorque3 = out.aerotorque.Data(:,3)';

T_ext1 = out.T_ext1.Data(1,:);
T_ext2 = out.T_ext2.Data(1,:);
T_ext3 = out.T_ext3.Data(1,:);

save('dist_nominal_attitude.mat', 'T_mag1', 'T_mag2', 'T_mag3', ...
    'GGtorque1', 'GGtorque2', 'GGtorque3', 'srp_torque1', 'srp_torque2', ...
    'srp_torque3','aerotorque1', 'aerotorque2', 'aerotorque3', 'T_ext1', ...
    'T_ext2', 'T_ext3');
