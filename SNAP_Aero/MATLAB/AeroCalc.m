clear all; clc; close all;

addpath('libaero')
tic
%% Define "home" volume

Resolution = 0.5; % cm cube per dot
res = 300;  %home volume dimentions
home_volume = zeros(res,res,res) * NaN;
body_length=30; %cm

%% Define satellite shape: 16U Satellite with solar panels

[origin, home_volume] = draw_cubesat(body_length, 0, home_volume, Resolution);
gc=[res res res]/2;
delta_cg=[0 0 0]*0.1;       % delta_cg [cm];
origin=gc+delta_cg;                     % centre of mass [cm];

%% Volume's plot

Sat_Aeromodel.PointCloudModel = home_volume;

figure;
plot_volume(home_volume)
drawnow
axis square

% plot of centre of mass;
hold on
plot3(origin(1),origin(2),origin(3),'.','Markersize',25); hold on
plot3(gc(1),gc(2),gc(3),'.','Markersize',20);hold on

% Body axes
quiver3(origin(1),origin(2),origin(3),80,0,0,'b','LineWidth',2);
quiver3(origin(1),origin(2),origin(3),0,80,0,'Color',[0 0.5 0],'LineWidth',2);
quiver3(origin(1),origin(2),origin(3),0,0,80,'Color',[0.75 0 0],'LineWidth',2);
quiver3(0,0,0,300,0,0,'b-','LineWidth',2); hold on
quiver3(0,0,0,0,300,0,'Color',[0 0.5 0],'LineWidth',2); hold on
quiver3(0,0,0,0,0,300,'Color',[0.75 0 0],'LineWidth',2);
set(gca,'YDir','reverse')
set(gca,'ZDir','reverse')


%% Torque factor calculation
% 
% roll = [0:4:20]* pi/180;   % roll  angles range;
% pitch = [-10:2:10] * pi/180;  % pitch angles range;
% yaw = [0:5:20]* pi/180;    % yaw   angles range;

roll = [0:25:360,360]* pi/180;   % roll  angles range; it was [0:25:360]
pitch = [-180:25:180,180] * pi/180;  % pitch angles range;
yaw = [0:25:360,360]* pi/180;    % yaw   angles range;

% Matrix initialization;
T = zeros(length(roll), length(pitch), length(yaw));
MatC=zeros(3,length(roll),length(pitch),length(yaw));
Area=zeros(length(roll),length(pitch),length(yaw));

for iroll = 1:length(roll)
    for ipitch = 1:length(pitch)
        for iyaw=1:length(yaw)

        DCM_roll = angle2dcm( roll(iroll), 0 , 0, 'XYZ');
        DCM_pitch = angle2dcm( 0, pitch(ipitch) , 0, 'XYZ');
        DCM_yaw=angle2dcm(0, 0, yaw(iyaw), 'XYZ');
        DCM = DCM_roll*DCM_pitch*DCM_yaw;                     % DCM matrix;
        rot_volume = rotate_volume(home_volume, DCM, origin); % Volume rotation;


        %Torque factor in ORF
        temp = calc_torque(rot_volume, origin);                            % Torque factor components in ORF;
        MatC(:,iroll,ipitch,iyaw)=temp;                                    % Matrix of torque factor components in ORF;
        T(iroll, ipitch, iyaw) =  sqrt(temp(1)^2+temp(2)^2+temp(3)^2);     % Matrix of Torque factor magnitude;
       
        %Drag area
        Area(iroll, ipitch, iyaw)=calc_area(rot_volume);


        %disp(['roll: ' num2str(roll(iroll)*180/pi) ' deg ', ', pitch: '  num2str(pitch(ipitch)*180/pi) ' deg ', ', yaw: ' num2str(yaw(iyaw)*180/pi) ' deg ', ' , T_c: ' num2str(temp)])

    end


    end

end
%% Definition of the aerodynamic model SNAP_aeromodel
% 
Sat_Aeromodel_det.roll = roll;   % roll angle;
Sat_Aeromodel_det.pitch = pitch; % pitch angle;
Sat_Aeromodel_det.yaw=yaw;       % yaw angle;

Sat_Aeromodel_det_det.T=T;         % Torque factor magnitude;
Sat_Aeromodel_det.Area=Area;   % Drag area;
Sat_Aeromodel_det.T_comp=MatC; % 4D-Matrix of torque factor components;

Sat_Aeromodel_det.xComp= reshape(Sat_Aeromodel_det.T_comp(1,:), length(Sat_Aeromodel_det.roll), length(Sat_Aeromodel_det.pitch), length(Sat_Aeromodel_det.yaw));
Sat_Aeromodel_det.yComp= reshape(Sat_Aeromodel_det.T_comp(2,:), length(Sat_Aeromodel_det.roll), length(Sat_Aeromodel_det.pitch), length(Sat_Aeromodel_det.yaw));
Sat_Aeromodel_det.zComp= reshape(Sat_Aeromodel_det.T_comp(3,:), length(Sat_Aeromodel_det.roll), length(Sat_Aeromodel_det.pitch), length(Sat_Aeromodel_det.yaw));
% 

% % Save structure
%save('Sat_Aeromodel','Sat_Aeromodel_det')
save('Sat_Aeromodel_coarse25step','Sat_Aeromodel_det')

toc