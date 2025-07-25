addpath('libaero')
addpath('libastro')
addpath('mod')
load Sat_Aero_def.mat

SNAP_aeromodel_comp = Sat_Aeromodel;
%% Importa i momenti per una configurazione
VelMax=7730;
phi = SNAP_aeromodel.pitch;         %pitch angle 
theta = SNAP_aeromodel.roll;       %roll angle
altitude = 302;         %quota in km
rho = 2.5e-11;

YAW=[1,6,10,11,12,16,21];
yaw_m10=reshape(SNAP_aeromodel.comp(:,1),3,21,21); %ORF torque factor components, psi=-10°;
yaw_m5=reshape(SNAP_aeromodel.comp(:,6),3,21,21);  %ORF torque factor components, psi=-5°;
yaw_m1=reshape(SNAP_aeromodel.comp(:,10),3,21,21); %ORF torque factor components, psi=-1°;
yaw_0=reshape(SNAP_aeromodel.comp(:,11),3,21,21);  %ORF torque factor components, psi=0°;
yaw_1=reshape(SNAP_aeromodel.comp(:,12),3,21,21);  %ORF torque factor components, psi=1°;
yaw_5=reshape(SNAP_aeromodel.comp(:,16),3,21,21);  %ORF torque factor components, psi=5°;
yaw_10=reshape(SNAP_aeromodel.comp(:,21),3,21,21); %ORF torque factor components, psi=10°;





%% COMPONENTS YAW=-10°

%Definizione delle componenti in body
for i=1:length(phi)
for j=1:length(theta)
MM(:,:,j,i)=angle2dcm(-0.1745,phi(i),theta(j),'ZYX'); %from ORF to BRF
yaw_m10_body(:,j,i)=MM(:,:,j,i)*yaw_m10(:,j,i); %matrice 3D componenti body;

 
end
end

%%
% Aerotorque components YAW=-10°

TorqueParam = SNAP_aeromodel.T(:,:,1);            %righe => theta;
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam; %Aerodynamic Torque Magnitude;
Aero_Torque_ORF_ym10=yaw_m10*rho*VelMax^2;        %Aerodynamic Torque ORF;
Aero_Torque_BRF_ym10=yaw_m10_body*rho*VelMax^2;   %Aerodynamic Torque BRF;

M_Aero_Torque_Magnitude=max(max(Aero_Torque_Magnitude));

[max_row_idx,max_col_idx]=ind2sub(size(Aero_Torque_Magnitude),find(Aero_Torque_Magnitude==M_Aero_Torque_Magnitude));
theta_T_max=convang(theta(max_row_idx),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max=convang(phi(max_col_idx),'rad','deg');   %pitch angle corresponding to max Aero_Torque;

M_Aero_Torque_Magnitude_ORF_x_y_z=Aero_Torque_ORF_ym10(:,max_row_idx,max_col_idx);
M_Aero_Torque_Magnitude_BRF_x_y_z=Aero_Torque_BRF_ym10(:,max_row_idx,max_col_idx);


M_Aero_Torque_Magnitude_BRF_x=max(max(Aero_Torque_BRF_ym10(1,:,:)));
M_Aero_Torque_Magnitude_BRF_y=max(max(Aero_Torque_BRF_ym10(2,:,:)));
M_Aero_Torque_Magnitude_BRF_z=max(max(Aero_Torque_BRF_ym10(3,:,:)));
[max_row_idx_x,max_col_idx_x,u]=ind2sub(size(Aero_Torque_BRF_ym10(1,:,:)),find(Aero_Torque_BRF_ym10(1,:,:)==M_Aero_Torque_Magnitude_BRF_x));
[max_row_idx_y,max_col_idx_y,u_]=ind2sub(size(Aero_Torque_BRF_ym10(2,:,:)),find(Aero_Torque_BRF_ym10(2,:,:)==M_Aero_Torque_Magnitude_BRF_y));
[max_row_idx_z,max_col_idx_z,k_]=ind2sub(size(Aero_Torque_BRF_ym10(3,:,:)),find(Aero_Torque_BRF_ym10(3,:,:)==M_Aero_Torque_Magnitude_BRF_z));
theta_T_max_x=convang(theta(max_col_idx_x),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max_x=convang(phi(u),'rad','deg');   %pitch angle corresponding to max Aero_Torque;
theta_T_max_y=convang(theta(max_col_idx_y),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max_y=convang(phi(u_),'rad','deg');   %pitch angle corresponding to max Aero_Torque;
theta_T_max_z=convang(theta(max_col_idx_z),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max_z=convang(phi(k_),'rad','deg'); 


%%

%ORF

%y components ORF
% figure(1)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_ORF_ym10(2,:,i));
% hold on
% end
% title('Aerodynamic Torque y_O_R_F components as a function of attitude; \psi=-10°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque y_O_R_F [Nm]');
% saveas(figure(1),'y_ORF.png')
% 
% 
% %z components ORF
% figure(2)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_ORF_ym10(3,:,i));
% hold on
% end
% title('Aerodynamic Torque z_O_R_F components as a function of attitude; \gamma = -10°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque z_O_R_F [Nm]');
% saveas(figure(2),'z_ORF.png')

% BODY

% %x components BRF
% figure(3)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_ym10(1,:,i),'LineWidth',1.5);
% hold on
% end
% title('Aerodynamic Torque x_B_R_F components as a function of attitude; \gamma = -10°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque x_B_R_F [Nm]');
% saveas(figure(3),'x_BRF.png')
% 
% %y components BRF
% figure(4)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_ym10(2,:,i),'LineWidth',1.5);
% hold on
% end
% title('Aerodynamic Torque y_B_R_F components as a function of attitude; \gamma = -10°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque y_B_R_F [Nm]');
% saveas(figure(4),'y_BRF.png')

% %z components BRF
% figure(5)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_BRF_ym10(3,:,i),'LineWidth',1.5);
% hold on
% end
% title('Aerodynamic Torque z_B_R_F components as a function of attitude; \gamma = -10°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque z_B_R_F [Nm]');
% saveas(figure(5),'z_BRF.png')

% %Aerodynamic Torque Magnitude 
% figure(6)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_Magnitude(:,i),'LineWidth',1.5);
% hold on
% end
% title('Aerodynamic Torque as function of attitude; \gamma = -10°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque Magnitude [Nm]');
% saveas(figure(6),'magnitude.png')

figure(7)
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,21),'k--','LineWidth',1.2); hold on
legend('\beta = -10°', '\beta = -5°', '\beta = 0°', '\beta = 5°', '\beta = 10°')
title('Aerodynamic Torque Magnitude, \gamma = -10°')
xlabel('Roll \alpha [deg]');
ylabel('Aerodynamic Torque Magnitude [Nm]');
saveas(figure(7),'aero_torque_3beta.png')

figure(8)
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(1,:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(1,:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(1,:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(1,:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(1,:,21),'k--','LineWidth',1.2);
legend('\beta = -10°','\beta = -5°','\beta = 0°','\beta = 5°','\beta = 10°')
title('Aerodynamic Torque x component in BRF, \gamma = -10°')
xlabel('Roll \alpha [deg]')
ylabel('x_B_R_F [Nm]')
saveas(figure(8),'aero_torque_x_BRF_3beta.png')

figure(9)
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(2,:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(2,:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(2,:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(2,:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(2,:,21),'k--','LineWidth',1.2);
legend('\beta = -10°','\beta = -5°','\beta = 0°','\beta = 5°','\beta = 10°')
title('Aerodynamic Torque y component in BRF, \gamma = -10°')
xlabel('Roll \alpha [deg]')
ylabel('y_B_R_F [Nm]')
saveas(figure(9),'aero_torque_y_BRF_3beta.png')

figure(10)
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(3,:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(3,:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(3,:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(3,:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_ym10(3,:,21),'k--','LineWidth',1.2);
legend('\beta = -10°','\beta = -5°','\beta = 0°','\beta = 5°','\beta = 10°')
title('Aerodynamic Torque z component in BRF, \gamma = -10°')
xlabel('Roll \alpha [deg]')
ylabel('z_B_R_F [Nm]')
saveas(figure(10),'aero_torque_z_BRF_3beta.png')





%% COMPONENTS YAW=-5°

%BODY
for i=1:length(phi)
for j=1:length(theta)
MM2(:,:,j,i)=angle2dcm(convang(-5,'deg','rad'),phi(i),theta(j),'ZYX');
yaw_m5_body(:,j,i)=MM2(:,:,j,i)*yaw_m5(:,j,i);
end
end

% Aerodynamic Torque Components YAW=-5°

TorqueParam = SNAP_aeromodel.T(:,:,6);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;
Aero_Torque_ORF_ym5=yaw_m5*rho*VelMax^2;        %Aerodynamic Torque ORF;
Aero_Torque_BRF_ym5=yaw_m5_body*rho*VelMax^2;   %Aerodynamic Torque BRF;



% %ORF
% 
% %y components ORF
% figure(1)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_ORF_ym5(2,:,i));
% hold on
% end
% title('Aerodynamic Torque y_O_R_F components as a function of attitude; \psi=-5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque y_O_R_F [Nm]');
% saveas(figure(1),'y_ORF.png')
% 
% %z components ORF
% figure(2)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_ORF_ym5(3,:,i));
% hold on
% end
% title('Aerodynamic Torque z_O_R_F components as a function of attitude; \psi=-5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque z_O_R_F [Nm]');
% saveas(figure(2),'z_ORF.png')

%BODY

%x components BRF
% figure(3)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_ym5(1,:,i));
% hold on
% end
% title('Aerodynamic Torque x_B_R_F components as a function of attitude; \psi=-5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque x_B_R_F [Nm]');
% saveas(figure(3),'x_BRF.png')
% 
% %y components BRF
% figure(4)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_ym5(2,:,i));
% hold on
% end
% title('Aerodynamic Torque y_B_R_F components as a function of attitude; \psi=-5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque y_B_R_F [Nm]');
% saveas(figure(4),'y_BRF.png')
% 
% %z components BRF
% figure(5)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_BRF_ym5(3,:,i));
% hold on
% end
% title('Aerodynamic Torque z_B_R_F components as a function of attitude; \psi=-5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque z_B_R_F [Nm]');
% saveas(figure(5),'z_BRF.png')
% 
% %Aerodynamic Torque Magnitude 
% figure(6)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_Magnitude(:,i));
% hold on
% end
% title('Aerodynamic Torque as function of attitude; \psi=-5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque Magnitude [Nm]');
% saveas(figure(6),'magnitude.png')


%% COMPONENTS YAW=-1°

% %ORF
% 
% %y components ORF
% figure(1)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_m1(2,:,i));
% hold on
% end
% title('Torque factor y_O_R_F components as a function of attitude; \psi=-1°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor y_O_R_F [m^3]');
% 
% %z components ORF
% figure(2)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_m1(3,:,i));
% hold on
% end
% title('Torque factor z_O_R_F components as a function of attitude; \psi=-1°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor z_O_R_F [m^3]');

%BODY

for i=1:length(phi)
for j=1:length(theta)
MM3(:,:,j,i)=angle2dcm(convang(-1,'deg','rad'),phi(i),theta(j),'ZYX');
yaw_m1_body_o(:,j,i)=yaw_m1(:,j,i)'*MM3(:,:,j,i);
yaw_m1_body(:,:,i)=yaw_m1_body_o(:,:,i)';
end
end

% %x components body
% figure(3)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_m1_body(1,:,i));
% hold on
% end
% title('Torque factor x_b_o_d_y components as a function of attitude; \psi=-1°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor x_b_o_d_y [m^3]');
% 
% %y components body
% figure(4)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_m1_body(2,:,i));
% hold on
% end
% title('Torque factor y_b_o_d_y components as a function of attitude; \psi=-1°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor y_b_o_d_y [m^3]');
% 
% %z components body
% figure(5)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_m1_body(3,:,i));
% hold on
% end
% title('Torque factor z_b_o_d_y components as a function of attitude; \psi=-1°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor z_b_o_d_y [m^3]');


% Aerodynamic Torque Components YAW=-1°

TorqueParam = SNAP_aeromodel.T(:,:,10);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;
Aero_Torque_ORF_ym1=yaw_m1*rho*VelMax^2;        %Aerodynamic Torque ORF;
Aero_Torque_BRF_ym1=yaw_m1_body*rho*VelMax^2;   %Aerodynamic Torque BRF;

%ORF

%y components ORF
figure(1)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_ORF_ym1(2,:,i));
hold on
end
title('Aerodynamic Torque y_O_R_F components as a function of attitude; \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_O_R_F [Nm]');
saveas(figure(1),'y_ORF.png')

%z components ORF
figure(2)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_ORF_ym1(3,:,i));
hold on
end
title('Aerodynamic Torque z_O_R_F components as a function of attitude; \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_O_R_F [Nm]');
saveas(figure(2),'z_ORF.png')

%BODY

%x components BRF
figure(3)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_ym1(1,:,i));
hold on
end
title('Aerodynamic Torque x_B_R_F components as a function of attitude; \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque x_B_R_F [Nm]');
saveas(figure(3),'x_BRF.png')

%y components BRF
figure(4)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_ym1(2,:,i));
hold on
end
title('Aerodynamic Torque y_B_R_F components as a function of attitude; \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_B_R_F [Nm]');
saveas(figure(4),'y_BRF.png')

%z components BRF
figure(5)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_BRF_ym1(3,:,i));
hold on
end
title('Aerodynamic Torque z_B_R_F components as a function of attitude; \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_B_R_F [Nm]');
saveas(figure(5),'z_BRF.png')

%Aerodynamic Torque Magnitude 
figure(6)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_Magnitude(:,i));
hold on
end
title('Aerodynamic Torque as function of attitude; \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque Magnitude [Nm]');
saveas(figure(6),'magnitude.png')


%% COMPONENTS YAW=0°

%BODY

for i=1:length(phi)
for j=1:length(theta)
MM4(:,:,j,i)=angle2dcm(convang(0,'deg','rad'),phi(i),theta(j),'ZYX');
yaw_0_body(:,j,i)=MM4(:,:,j,i)*yaw_0(:,j,i);

end
end

% Aerodynamic Torque Components YAW=0°

TorqueParam = SNAP_aeromodel.T(:,:,11);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;
Aero_Torque_ORF_y0=yaw_0*rho*VelMax^2;        %Aerodynamic Torque ORF;
Aero_Torque_BRF_y0=yaw_0_body*rho*VelMax^2;   %Aerodynamic Torque BRF;

M_Aero_Torque_Magnitude=max(max(Aero_Torque_Magnitude));
disp(M_Aero_Torque_Magnitude)

[max_row_idx,max_col_idx]=ind2sub(size(Aero_Torque_Magnitude),find(Aero_Torque_Magnitude==M_Aero_Torque_Magnitude));
theta_T_max=convang(theta(max_row_idx),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max=convang(phi(max_col_idx),'rad','deg');   %pitch angle corresponding to max Aero_Torque;
disp(theta_T_max);
disp(phi_T_max);

% M_Aero_Torque_Magnitude_ORF_x_y_z=Aero_Torque_ORF_y0(:,max_row_idx,max_col_idx);
% M_Aero_Torque_Magnitude_BRF_x_y_z=Aero_Torque_BRF_y0(:,max_row_idx,max_col_idx);
% disp(M_Aero_Torque_Magnitude_BRF_x_y_z);

M_Aero_Torque_Magnitude_BRF_x=max(max(Aero_Torque_BRF_y0(1,:,:)));
M_Aero_Torque_Magnitude_BRF_y=max(max(Aero_Torque_BRF_y0(2,:,:)));
M_Aero_Torque_Magnitude_BRF_z=max(max(Aero_Torque_BRF_y0(3,:,:)));
[max_row_idx_x,max_col_idx_x,u]=ind2sub(size(Aero_Torque_BRF_y0(1,:,:)),find(Aero_Torque_BRF_y0(1,:,:)==M_Aero_Torque_Magnitude_BRF_x));
[max_row_idx_y,max_col_idx_y,u_]=ind2sub(size(Aero_Torque_BRF_y0(2,:,:)),find(Aero_Torque_BRF_y0(2,:,:)==M_Aero_Torque_Magnitude_BRF_y));
[max_row_idx_z,max_col_idx_z,k_]=ind2sub(size(Aero_Torque_BRF_y0(3,:,:)),find(Aero_Torque_BRF_y0(3,:,:)==M_Aero_Torque_Magnitude_BRF_z));
theta_T_max_x=convang(theta(max_col_idx_x),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max_x=convang(phi(u),'rad','deg');   %pitch angle corresponding to max Aero_Torque;
theta_T_max_y=convang(theta(max_col_idx_y),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max_y=convang(phi(u_),'rad','deg');   %pitch angle corresponding to max Aero_Torque;
theta_T_max_z=convang(theta(max_col_idx_z),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max_z=convang(phi(k_),'rad','deg'); 




%BODY

%x components BRF
% figure(3)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_y0(1,:,i));
% hold on
% end
% title('Aerodynamic Torque x_B_R_F components as a function of attitude; \gamma=0°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque x_B_R_F [Nm]');
% saveas(figure(3),'x_BRF.png')
% 
% 
% %y components BRF
% figure(4)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_y0(2,:,i));
% hold on
% end
% title('Aerodynamic Torque y_B_R_F components as a function of attitude; \gamma=0°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque y_B_R_F [Nm]');
% saveas(figure(4),'y_BRF.png')
% 
% 
% %z components BRF
% figure(5)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_BRF_y0(3,:,i));
% hold on
% end
% title('Aerodynamic Torque z_B_R_F components as a function of attitude; \gamma=0°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque z_B_R_F [Nm]');
% saveas(figure(5),'z_BRF.png')
% 
% 
% %Aerodynamic Torque Magnitude 
% figure(6)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_Magnitude(:,i));
% hold on
% end
% title('Aerodynamic Torque as function of attitude; \gamma=0°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque Magnitude [Nm]');
% saveas(figure(6),'magnitude.png')


figure(7)
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,21),'k--','LineWidth',1.2); hold on
legend('\beta = -10°', '\beta = -5°', '\beta = 0°', '\beta = 5°', '\beta = 10°')
title('Aerodynamic Torque Magnitude, \gamma = 0°')
ylabel('Aerodynamic Torque Magnitude [Nm]')
xlabel('Roll \alpha [deg]')
saveas(figure(7),'aero_torque_3beta.png')

figure(8)
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(1,:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(1,:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(1,:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(1,:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(1,:,21),'k--','LineWidth',1.2);
legend('\beta = -10°','\beta = -5°','\beta = 0°','\beta = 5°','\beta = 10°')
title('Aerodynamic Torque x component in BRF, \gamma = 0°')
xlabel('Roll \alpha [deg]')
ylabel('x_B_R_F [Nm]')
saveas(figure(8),'aero_torque_x_BRF_3beta.png')

figure(9)
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(2,:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(2,:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(2,:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(2,:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(2,:,21),'k--','LineWidth',1.2);
legend('\beta = -10°','\beta = -5°','\beta = 0°','\beta = 5°','\beta = 10°')
title('Aerodynamic Torque y component in BRF, \gamma = 0°')
xlabel('Roll \alpha [deg]')
ylabel('y_B_R_F [Nm]')
saveas(figure(9),'aero_torque_y_BRF_3beta.png')

figure(10)
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(3,:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(3,:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(3,:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(3,:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y0(3,:,21),'k--','LineWidth',1.2);
legend('\beta = -10°','\beta = -5°','\beta = 0°','\beta = 5°','\beta = 10°')
title('Aerodynamic Torque z component in BRF, \gamma = 0°')
xlabel('Roll \alpha [deg]')
ylabel('z_B_R_F [Nm]')
saveas(figure(10),'aero_torque_z_BRF_3beta.png')





%% COMPONENTS YAW=1°

% %ORF
% 
% %y components ORF
% figure(1)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_1(2,:,i));
% hold on
% end
% title('Torque factor y_O_R_F components as a function of attitude; \psi=1°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor y_O_R_F [m^3]');
% 
% %z components ORF
% figure(2)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_1(3,:,i));
% hold on
% end
% title('Torque factor z_O_R_B components as a function of attitude; \psi=1°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor z_O_R_F [m^3]');

%BODY

for i=1:length(phi)
for j=1:length(theta)
MM5(:,:,j,i)=angle2dcm(convang(1,'deg','rad'),phi(i),theta(j),'ZYX');
yaw_1_body(:,j,i)=MM5(:,:,j,i)*yaw_1(:,j,i);
end
end

% %x components body
% figure(3)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_1_body(1,:,i));
% hold on
% end
% title('Torque factor x_b_o_d_y components as a function of attitude; \psi=1°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor x_b_o_d_y [m^3]');
% 
% %y components body
% figure(4)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_1_body(2,:,i));
% hold on
% end
% title('Torque factor y_b_o_d_y components as a function of attitude; \psi=1°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor y_b_o_d_y [m^3]');
% 
% %z components body
% figure(5)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_1_body(3,:,i));
% hold on
% end
% title('Torque factor z_b_o_d_y components as a function of attitude; \psi=1°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor z_b_o_d_y [m^3]');


% Aerodynamic Torque Components YAW=1°

TorqueParam = SNAP_aeromodel.T(:,:,12);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;
Aero_Torque_ORF_y1=yaw_1*rho*VelMax^2;        %Aerodynamic Torque ORF;
Aero_Torque_BRF_y1=yaw_1_body*rho*VelMax^2;   %Aerodynamic Torque BRF;

%ORF

%y components ORF
figure(1)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_ORF_y1(2,:,i));
hold on
end
title('Aerodynamic Torque y_O_R_F components as a function of attitude; \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_O_R_F [Nm]');

%z components ORF
figure(2)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_ORF_y1(3,:,i));
hold on
end
title('Aerodynamic Torque z_O_R_F components as a function of attitude; \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_O_R_F [Nm]');

%BODY

%x components BRF
figure(3)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_y1(1,:,i));
hold on
end
title('Aerodynamic Torque x_B_R_F components as a function of attitude; \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque x_B_R_F [Nm]');

%y components BRF
figure(4)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_y1(2,:,i));
hold on
end
title('Aerodynamic Torque y_B_R_F components as a function of attitude; \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_B_R_F [Nm]');

%z components BRF
figure(5)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_BRF_y1(3,:,i));
hold on
end
title('Aerodynamic Torque z_B_R_F components as a function of attitude; \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_B_R_F [Nm]');

%Aerodynamic Torque Magnitude 
figure(6)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_Magnitude(:,i));
hold on
end
title('Aerodynamic Torque as function of attitude; \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque Magnitude [Nm]');



%% COMPONENTS YAW=5°

% %ORF
% 
% %y components ORF
% figure(1)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_5(2,:,i));
% hold on
% end
% title('Torque factor y_O_R_F components as a function of attitude; \psi=5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor y_O_R_F [m^3]');
% 
% %z components ORF
% figure(2)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_5(3,:,i));
% hold on
% end
% title('Torque factor z_O_R_F components as a function of attitude; \psi=5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor z_O_R_F [m^3]');

%BODY

for i=1:length(phi)
for j=1:length(theta)
MM6(:,:,j,i)=angle2dcm(convang(-5,'deg','rad'),phi(i),theta(j),'ZYX');
yaw_5_body(:,j,i)=MM6(:,:,j,i)*yaw_5(:,j,i);
end
end

% %x components body
% figure(3)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_5_body(1,:,i));
% hold on
% end
% title('Torque factor x_b_o_d_y components as a function of attitude; \psi=5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor x_b_o_d_y [m^3]');
% 
% %y components body
% figure(4)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_5_body(2,:,i));
% hold on
% end
% title('Torque factor y_b_o_d_y components as a function of attitude; \psi=5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor y_b_o_d_y [m^3]');
% 
% %z components body
% figure(5)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  yaw_5_body(3,:,i));
% hold on
% end
% title('Torque factor z_b_o_d_y components as a function of attitude; \psi=5°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Torque Factor z_b_o_d_y [m^3]');



% Aerodynamic Torque Components YAW=-5°

TorqueParam = SNAP_aeromodel.T(:,:,16);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;
Aero_Torque_ORF_y5=yaw_5*rho*VelMax^2;        %Aerodynamic Torque ORF;
Aero_Torque_BRF_y5=yaw_5_body*rho*VelMax^2;   %Aerodynamic Torque BRF;

%ORF

%y components ORF
figure(1)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_ORF_y5(2,:,i));
hold on
end
title('Aerodynamic Torque y_O_R_F components as a function of attitude; \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_O_R_F [Nm]');

%z components ORF
figure(2)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_ORF_y5(3,:,i));
hold on
end
title('Aerodynamic Torque z_O_R_F components as a function of attitude; \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_O_R_F [Nm]');

%BODY

%x components BRF
figure(3)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_y5(1,:,i));
hold on
end
title('Aerodynamic Torque x_B_R_F components as a function of attitude; \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque x_B_R_F [Nm]');

%y components BRF
figure(4)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_y5(2,:,i));
hold on
end
title('Aerodynamic Torque y_B_R_F components as a function of attitude; \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_B_R_F [Nm]');

%z components BRF
figure(5)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_BRF_y5(3,:,i));
hold on
end
title('Aerodynamic Torque z_B_R_F components as a function of attitude; \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_B_R_F [Nm]');

%Aerodynamic Torque Magnitude 
figure(6)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_Magnitude(:,i));
hold on
end
title('Aerodynamic Torque as function of attitude; \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque Magnitude [Nm]');




%% COMPONENTS YAW=10°

%BODY

for i=1:length(phi)
for j=1:length(theta)
MM7(:,:,j,i)=angle2dcm(convang(10,'deg','rad'),phi(i),theta(j),'ZYX');
yaw_10_body(:,j,i)=MM7(:,:,j,i)*yaw_10(:,j,i);
end
end

% Aerodynamic Torque Components YAW=10°

TorqueParam = SNAP_aeromodel.T(:,:,21);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;
Aero_Torque_ORF_y10=yaw_10*rho*VelMax^2;        %Aerodynamic Torque ORF;
Aero_Torque_BRF_y10=yaw_10_body*rho*VelMax^2;   %Aerodynamic Torque BRF;

M_Aero_Torque_Magnitude=max(max(Aero_Torque_Magnitude));
disp(M_Aero_Torque_Magnitude)

[max_row_idx,max_col_idx]=ind2sub(size(Aero_Torque_Magnitude),find(Aero_Torque_Magnitude==M_Aero_Torque_Magnitude));
theta_T_max=convang(theta(max_row_idx),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max=convang(phi(max_col_idx),'rad','deg');   %pitch angle corresponding to max Aero_Torque;
disp(theta_T_max);
disp(phi_T_max);

M_Aero_Torque_Magnitude_BRF_x=max(max(Aero_Torque_BRF_y10(1,:,:)));
M_Aero_Torque_Magnitude_BRF_y=max(max(Aero_Torque_BRF_y10(2,:,:)));
M_Aero_Torque_Magnitude_BRF_z=max(max(Aero_Torque_BRF_y10(3,:,:)));
[max_row_idx_x,max_col_idx_x,u]=ind2sub(size(Aero_Torque_BRF_y10(1,:,:)),find(Aero_Torque_BRF_y10(1,:,:)==M_Aero_Torque_Magnitude_BRF_x));
[max_row_idx_y,max_col_idx_y,u_]=ind2sub(size(Aero_Torque_BRF_y10(2,:,:)),find(Aero_Torque_BRF_y10(2,:,:)==M_Aero_Torque_Magnitude_BRF_y));
[max_row_idx_z,max_col_idx_z,k_]=ind2sub(size(Aero_Torque_BRF_y10(3,:,:)),find(Aero_Torque_BRF_y10(3,:,:)==M_Aero_Torque_Magnitude_BRF_z));
theta_T_max_x=convang(theta(max_col_idx_x),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max_x=convang(phi(u),'rad','deg');   %pitch angle corresponding to max Aero_Torque;
theta_T_max_y=convang(theta(max_col_idx_y),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max_y=convang(phi(u_),'rad','deg');   %pitch angle corresponding to max Aero_Torque;
theta_T_max_z=convang(theta(max_col_idx_z),'rad','deg'); %roll angle corresponding to max Aero_Torque;
phi_T_max_z=convang(phi(k_),'rad','deg'); 

%ORF

%y components ORF
% figure(1)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_ORF_y10(2,:,i));
% hold on
% end
% title('Aerodynamic Torque y_O_R_F components as a function of attitude; \psi=10°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque y_O_R_F [Nm]');
% 
% %z components ORF
% figure(2)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_ORF_y10(3,:,i));
% hold on
% end
% title('Aerodynamic Torque z_O_R_F components as a function of attitude; \psi=10°');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque z_O_R_F [Nm]');
% 
% %BODY
% 
% %x components BRF
% figure(3)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_y10(1,:,i));
% hold on
% end
% title('Aerodynamic Torque x_B_R_F components as a function of attitude; \gamma=10°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque x_B_R_F [Nm]');
% 
% %y components BRF
% figure(4)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  Aero_Torque_BRF_y10(2,:,i));
% hold on
% end
% title('Aerodynamic Torque y_B_R_F components as a function of attitude; \gamma=10°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque y_B_R_F [Nm]');
% 
% %z components BRF
% figure(5)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_BRF_y10(3,:,i));
% hold on
% end
% title('Aerodynamic Torque z_B_R_F components as a function of attitude; \gamma=10°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque z_B_R_F [Nm]');
% 
% %Aerodynamic Torque Magnitude 
% figure(6)
% for i = 1:length(phi)
% phi_const = phi(1,i)*ones(length(theta),1);
% plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'), Aero_Torque_Magnitude(:,i));
% hold on
% end
% title('Aerodynamic Torque as function of attitude; \gamma=10°');
% xlabel('Roll \alpha [deg]');
% ylabel('Pitch \beta [deg]' );
% zlabel('Aerodynamic Torque Magnitude [Nm]');
% saveas(figure(6),'magnitude.png')

figure(7)
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,11),'k-o','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_Magnitude(:,21),'k--','LineWidth',1.2); 
legend('\beta = -10°',  '\beta = -5°', '\beta = 0°', '\beta = 5°', '\beta = 10°')
title('Aerodynamic Torque Magnitude, \gamma = 10°')
xlabel('Roll \alpha [deg]')
ylabel('Aerodynamic Torque Magnitude [Nm]')
saveas(figure(7),'aero_torque_3beta.png')

figure(8)
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(1,:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(1,:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(1,:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(1,:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(1,:,21),'k--','LineWidth',1.2);
legend('\beta = -10°','\beta = -5°','\beta = 0°', '\beta = 5°', '\beta = 10°')
title('Aerodynamic Torque x component in BRF, \gamma = 10°')
xlabel('Roll \alpha [deg]')
ylabel('x_B_R_F [Nm]')
saveas(figure(8),'aero_torque_x_BRF_3beta.png')

figure(9)
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(2,:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(2,:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(2,:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(2,:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(2,:,21),'k--','LineWidth',1.2);
legend('\beta = -10°','\beta = -5°','\beta = 0°', '\beta = 5°', '\beta = 10°')
title('Aerodynamic Torque y component in BRF, \gamma = 10°')
xlabel('Roll \alpha [deg]')
ylabel('y_B_R_F [Nm]')
saveas(figure(9),'aero_torque_y_BRF_3beta.png')

figure(10)
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(3,:,1),'k-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(3,:,6),'k-.','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(3,:,11),'ko-','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(3,:,16),'k-*','LineWidth',1.2); hold on
plot(convang(theta,'rad','deg'),Aero_Torque_BRF_y10(3,:,21),'k--','LineWidth',1.2);
legend('\beta = -10°','\beta = -5°','\beta = 0°', '\beta = 5°', '\beta = 10°')
title('Aerodynamic Torque z component in BRF, \gamma = 10°')
xlabel('Roll \alpha [deg]')
ylabel('z_B_R_F [Nm]')
saveas(figure(10),'aero_torque_z_BRF_3beta.png')
























%%

% %% Importa i momenti per la seconda configurazione
% load("SIDECONFBACK.mat");
% phi = -15:1:15;         %pitch angle 
% theta = -10:1:10;       %roll angle
% altitude = 302;         %quota in km
% rho = atmosphereNew(altitude);
% TorqueParam2 = SNAP_aeromodel.T;    %righe = phi; colonne = 2*theta
% Aero_Torque_Magnitude2 = rho*VelMax^2*TorqueParam2*10^-2;
% 
% %% Plotting della magnitude dei torque seconda conf
% figure (3);
% plot(phi,Aero_Torque_Magnitude2(:,11),'o-r');
% xlabel('Pitch \Phi [deg]');
% ylabel('Aerodynamic Torque [Nm]');
% title('Roll \Theta = 0 deg / CONFIGURAZIONE SIDE')
% grid on
% 
% figure(4)
% for i = 1:length(theta)
% theta_const = theta(1,i)*ones(length(phi),1);
% plot3(theta_const(:),phi(1,:),Aero_Torque_Magnitude2(:,i));
% hold on
% end
% title('Aerodynamic Torque as function of attitude/CONFIGURAZIONE SIDE');
% xlabel('Roll \Theta [deg]');
% ylabel('Pitch \Phi [deg]' );
% zlabel('Aerodynamic Torque');

%% Searching for the component

psi = 0.1745;
% for j = 1:length(theta) 
%     theta_for = theta(j);
%     for k = 1:length(phi)
%         phi_for = phi(k);
%         R1 = [1 0 0
%               0 cos(theta_for) sin(theta_for)
%               0 -sin(theta_for) cos(theta_for)];
%         R2 = [cos(phi_for) 0 -sin(phi_for)
%               0 1 0 
%               sin(phi_for) 0 cos(phi_for)];
%         R3 = [cos(psi) sin(psi) 0 
%              -sin(psi) cos(psi) 0
%               0 0 1];
%         Cib = R1*R2*R3;
%         normalized_vel = [-1 0 0];
%         Maero = cross(normalized_vel',Cib*xbody');
%         M_ECI(k,j,1:3) = Aero_Torque_Magnitude(k,j)*Maero;
%         M_body(k,j,1:3) = Cib'*Aero_Torque_Magnitude(k,j)*Maero';
%     end
% end


%% YAW -10°
x_body = [1 0 0]';
TorqueParam = SNAP_aeromodel.T(:,:,1);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;

for i=1:length(phi)
for j=1:length(theta)
MM(:,:,j,i)=angle2dcm(convang(-10,'deg','rad'),phi(i),theta(j),'ZYX'); %Cbi (from ECI to BRF);

        normalized_vel = [-1 0 0]';
        Maero = cross(normalized_vel,MM(:,:,j,i)'*x_body);
        Maero=Maero/norm(Maero);
        M_ECI(1:3,j,i) = Aero_Torque_Magnitude(j,i)*Maero; %components in ECI/ORF
        M_body(1:3,j,i) = MM(:,:,j,i)*Aero_Torque_Magnitude(j,i)*Maero; %components in BRF
end
end


%y components ORF
figure(1)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(2,:,i));
hold on
end
title('Aerodynamic Torque y_O_R_F components as a function of attitude (MUV); \psi=-10°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_O_R_F [Nm]');
saveas(figure(1),'y_ORF_MUV.png')

%z components ORF
figure(2)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(3,:,i));
hold on
end
title('Aerodynamic Torque z_O_R_F components as a function of attitude (MUV); \psi=-10°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_O_R_F [Nm]');
saveas(figure(2),'z_ORF_MUV.png')

%x components BRF
figure(3)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(1,:,i));
hold on
end
title('Aerodynamic Torque x_B_R_F components as a function of attitude (MUV); \psi=-10°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque x_B_R_F [Nm]');
saveas(figure(3),'x_BRF_MUV.png')

%y components BRF
figure(4)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(2,:,i));
hold on
end
title('Aerodynamic Torque y_B_R_F components as a function of attitude (MUV); \psi=-10°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_B_R_F [Nm]');
saveas(figure(4),'y_BRF_MUV.png')

%z components BRF
figure(5)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(3,:,i));
hold on
end
title('Aerodynamic Torque z_B_R_F components as a function of attitude (MUV); \psi=-10°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_B_R_F [Nm]');
saveas(figure(5),'z_BRF_MUV.png')

%% YAW -5°
x_body = [1 0 0]';
TorqueParam = SNAP_aeromodel.T(:,:,6);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;

for i=1:length(phi)
for j=1:length(theta)
MM(:,:,j,i)=angle2dcm(convang(-5,'deg','rad'),phi(i),theta(j),'ZYX'); %Cbi (from ECI to BRF);

        normalized_vel = [-1 0 0]';
        Maero = cross(normalized_vel,MM(:,:,j,i)'*x_body);
        Maero=Maero/norm(Maero);
        M_ECI(1:3,j,i) = Aero_Torque_Magnitude(j,i)*Maero; %components in ECI/ORF
        M_body(1:3,j,i) = MM(:,:,j,i)*Aero_Torque_Magnitude(j,i)*Maero; %components in BRF
end
end

%y components ORF
figure(1)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(2,:,i));
hold on
end
title('Aerodynamic Torque y_O_R_F components as a function of attitude (MUV); \psi=-5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_O_R_F [Nm]');
saveas(figure(1),'y_ORF_MUV.png')

%z components ORF
figure(2)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(3,:,i));
hold on
end
title('Aerodynamic Torque z_O_R_F components as a function of attitude (MUV); \psi=-5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_O_R_F [Nm]');
saveas(figure(2),'z_ORF_MUV.png')

%x components BRF
figure(3)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(1,:,i));
hold on
end
title('Aerodynamic Torque x_B_R_F components as a function of attitude (MUV); \psi=-5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque x_B_R_F [Nm]');
saveas(figure(3),'x_BRF_MUV.png')

%y components BRF
figure(4)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(2,:,i));
hold on
end
title('Aerodynamic Torque y_B_R_F components as a function of attitude (MUV); \psi=-5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_B_R_F [Nm]');
saveas(figure(4),'y_BRF_MUV.png')

%z components BRF
figure(5)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(3,:,i));
hold on
end
title('Aerodynamic Torque z_B_R_F components as a function of attitude (MUV); \psi=-5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_B_R_F [Nm]');
saveas(figure(5),'z_BRF_MUV.png')


%% YAW -1°
x_body = [1 0 0]';
TorqueParam = SNAP_aeromodel.T(:,:,10);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;

for i=1:length(phi)
for j=1:length(theta)
MM(:,:,j,i)=angle2dcm(convang(-1,'deg','rad'),phi(i),theta(j),'ZYX'); %Cbi (from ECI to BRF);

        normalized_vel = [-1 0 0]';
        Maero = cross(normalized_vel,MM(:,:,j,i)'*x_body);
        Maero=Maero/norm(Maero);
        M_ECI(1:3,j,i) = Aero_Torque_Magnitude(j,i)*Maero; %components in ECI/ORF
        M_body(1:3,j,i) = MM(:,:,j,i)*Aero_Torque_Magnitude(j,i)*Maero; %components in BRF
end
end

%y components ORF
figure(1)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(2,:,i));
hold on
end
title('Aerodynamic Torque y_O_R_F components as a function of attitude (MUV); \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_O_R_F [Nm]');
saveas(figure(1),'y_ORF_MUV.png')

%z components ORF
figure(2)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(3,:,i));
hold on
end
title('Aerodynamic Torque z_O_R_F components as a function of attitude (MUV); \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_O_R_F [Nm]');
saveas(figure(2),'z_ORF_MUV.png')

%x components BRF
figure(3)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(1,:,i));
hold on
end
title('Aerodynamic Torque x_B_R_F components as a function of attitude (MUV); \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque x_B_R_F [Nm]');
saveas(figure(3),'x_BRF_MUV.png')

%y components BRF
figure(4)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(2,:,i));
hold on
end
title('Aerodynamic Torque y_B_R_F components as a function of attitude (MUV); \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_B_R_F [Nm]');
saveas(figure(4),'y_BRF_MUV.png')

%z components BRF
figure(5)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(3,:,i));
hold on
end
title('Aerodynamic Torque z_B_R_F components as a function of attitude (MUV); \psi=-1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_B_R_F [Nm]');
saveas(figure(5),'z_BRF_MUV.png')


%% YAW 0°
x_body = [1 0 0]';
TorqueParam = SNAP_aeromodel.T(:,:,11);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;

for i=1:length(phi)
for j=1:length(theta)
MM(:,:,j,i)=angle2dcm(convang(0,'deg','rad'),phi(i),theta(j),'ZYX'); %Cbi (from ECI to BRF);

        normalized_vel = [-1 0 0]';
        Maero = cross(normalized_vel,MM(:,:,j,i)'*x_body);
        Maero=Maero/norm(Maero);
        M_ECI(1:3,j,i) = Aero_Torque_Magnitude(j,i)*Maero; %components in ECI/ORF
        M_body(1:3,j,i) = MM(:,:,j,i)*Aero_Torque_Magnitude(j,i)*Maero; %components in BRF
end
end

%y components ORF
figure(1)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(2,:,i));
hold on
end
title('Aerodynamic Torque y_O_R_F components as a function of attitude (MUV); \psi=0°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_O_R_F [Nm]');
saveas(figure(1),'y_ORF_MUV.png')


%z components ORF
figure(2)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(3,:,i));
hold on
end
title('Aerodynamic Torque z_O_R_F components as a function of attitude (MUV); \psi=0°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_O_R_F [Nm]');
saveas(figure(2),'z_ORF_MUV.png')


%x components BRF
figure(3)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(1,:,i));
hold on
end
title('Aerodynamic Torque x_B_R_F components as a function of attitude (MUV); \psi=0°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque x_B_R_F [Nm]');
saveas(figure(3),'x_BRF_MUV.png')


%y components BRF
figure(4)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(2,:,i));
hold on
end
title('Aerodynamic Torque y_B_R_F components as a function of attitude (MUV); \psi=0°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_B_R_F [Nm]');
saveas(figure(4),'y_BRF_MUV.png')


%z components BRF
figure(5)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(3,:,i));
hold on
end
title('Aerodynamic Torque z_B_R_F components as a function of attitude (MUV); \psi=0°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_B_R_F [Nm]');
saveas(figure(5),'z_BRF_MUV.png')



%% YAW 1°
x_body = [1 0 0]';
TorqueParam = SNAP_aeromodel.T(:,:,12);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;

for i=1:length(phi)
for j=1:length(theta)
MM(:,:,j,i)=angle2dcm(convang(1,'deg','rad'),phi(i),theta(j),'ZYX'); %Cbi (from ECI to BRF);

        normalized_vel = [-1 0 0]';
        Maero = cross(normalized_vel,MM(:,:,j,i)'*x_body);
        Maero=Maero/norm(Maero);
        M_ECI(1:3,j,i) = Aero_Torque_Magnitude(j,i)*Maero; %components in ECI/ORF
        M_body(1:3,j,i) = MM(:,:,j,i)*Aero_Torque_Magnitude(j,i)*Maero; %components in BRF
end
end

%y components ORF
figure(1)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(2,:,i));
hold on
end
title('Aerodynamic Torque y_O_R_F components as a function of attitude (MUV); \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_O_R_F [Nm]');

%z components ORF
figure(2)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(3,:,i));
hold on
end
title('Aerodynamic Torque z_O_R_F components as a function of attitude (MUV); \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_O_R_F [Nm]');

%x components BRF
figure(3)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(1,:,i));
hold on
end
title('Aerodynamic Torque x_B_R_F components as a function of attitude (MUV); \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque x_B_R_F [Nm]');

%y components BRF
figure(4)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(2,:,i));
hold on
end
title('Aerodynamic Torque y_B_R_F components as a function of attitude (MUV); \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_B_R_F [Nm]');

%z components BRF
figure(5)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(3,:,i));
hold on
end
title('Aerodynamic Torque z_B_R_F components as a function of attitude (MUV); \psi=1°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_B_R_F [Nm]');


%% YAW 5°
x_body = [1 0 0]';
TorqueParam = SNAP_aeromodel.T(:,:,16);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;

for i=1:length(phi)
for j=1:length(theta)
MM(:,:,j,i)=angle2dcm(convang(5,'deg','rad'),phi(i),theta(j),'ZYX'); %Cbi (from ECI to BRF);

        normalized_vel = [-1 0 0]';
        Maero = cross(normalized_vel,MM(:,:,j,i)'*x_body);
        Maero=Maero/norm(Maero);
        M_ECI(1:3,j,i) = Aero_Torque_Magnitude(j,i)*Maero; %components in ECI/ORF
        M_body(1:3,j,i) = MM(:,:,j,i)*Aero_Torque_Magnitude(j,i)*Maero; %components in BRF
end
end

%y components ORF
figure(1)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(2,:,i));
hold on
end
title('Aerodynamic Torque y_O_R_F components as a function of attitude (MUV); \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_O_R_F [Nm]');

%z components ORF
figure(2)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(3,:,i));
hold on
end
title('Aerodynamic Torque z_O_R_F components as a function of attitude (MUV); \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_O_R_F [Nm]');

%x components BRF
figure(3)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(1,:,i));
hold on
end
title('Aerodynamic Torque x_B_R_F components as a function of attitude (MUV); \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque x_B_R_F [Nm]');

%y components BRF
figure(4)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(2,:,i));
hold on
end
title('Aerodynamic Torque y_B_R_F components as a function of attitude (MUV); \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_B_R_F [Nm]');

%z components BRF
figure(5)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(3,:,i));
hold on
end
title('Aerodynamic Torque z_B_R_F components as a function of attitude (MUV); \psi=5°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_B_R_F [Nm]');



%% YAW 10°
x_body = [1 0 0]';
TorqueParam = SNAP_aeromodel.T(:,:,21);    %righe = phi; colonne = 2*theta
Aero_Torque_Magnitude = rho*VelMax^2*TorqueParam;

for i=1:length(phi)
for j=1:length(theta)
        MM(:,:,j,i)=angle2dcm(convang(10,'deg','rad'),phi(i),theta(j),'ZYX'); %Cbi (from ECI to BRF);
        normalized_vel = [-1 0 0]';
        Maero = cross(normalized_vel,MM(:,:,j,i)'*x_body);
        Maero=Maero/norm(Maero);
        M_ECI(1:3,j,i) = Aero_Torque_Magnitude(j,i)*Maero; %components in ECI/ORF
        M_body(1:3,j,i) = MM(:,:,j,i)*Aero_Torque_Magnitude(j,i)*Maero; %components in BRF
end
end

%y components ORF
figure(1)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(2,:,i));
hold on
end
title('Aerodynamic Torque y_O_R_F components as a function of attitude (MUV); \psi=10°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_O_R_F [Nm]');

%z components ORF
figure(2)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_ECI(3,:,i));
hold on
end
title('Aerodynamic Torque z_O_R_F components as a function of attitude (MUV); \psi=10°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_O_R_F [Nm]');

%x components BRF
figure(3)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(1,:,i));
hold on
end
title('Aerodynamic Torque x_B_R_F components as a function of attitude (MUV); \psi=10°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque x_B_R_F [Nm]');

%y components BRF
figure(4)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(2,:,i));
hold on
end
title('Aerodynamic Torque y_B_R_F components as a function of attitude (MUV); \psi=10°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque y_B_R_F [Nm]');

%z components BRF
figure(5)
for i = 1:length(phi)
phi_const = phi(1,i)*ones(length(theta),1);
plot3(convang(theta(1,:),'rad','deg'), convang(phi_const(:),'rad','deg'),  M_body(3,:,i));
hold on
end
title('Aerodynamic Torque z_B_R_F components as a function of attitude (MUV); \psi=10°');
xlabel('Roll \Theta [deg]');
ylabel('Pitch \Phi [deg]' );
zlabel('Aerodynamic Torque z_B_R_F [Nm]');

