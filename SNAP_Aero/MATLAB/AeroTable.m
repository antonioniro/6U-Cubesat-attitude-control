clear all; close all; clc;

load Sat_Aer.mat;

xAero.roll=Sat_Aeromodel.roll;
xAero.pitch=Sat_Aeromodel.pitch;
xAero.yaw=Sat_Aeromodel.yaw;
xAero.compX= reshape(Sat_Aeromodel.T_comp(1,:), length(xAero.roll), length(xAero.pitch), length(xAero.yaw));

yAero.roll=Sat_Aeromodel.roll;
yAero.pitch=Sat_Aeromodel.pitch;
yAero.yaw=Sat_Aeromodel.yaw;
yAero.compY= reshape(Sat_Aeromodel.T_comp(2,:), length(yAero.roll), length(yAero.pitch), length(yAero.yaw));

zAero.roll=Sat_Aeromodel.roll;
zAero.pitch=Sat_Aeromodel.pitch;
zAero.yaw=Sat_Aeromodel.yaw;
zAero.compZ= reshape(Sat_Aeromodel.T_comp(3,:), length(zAero.roll), length(zAero.pitch), length(zAero.yaw));

save('x_Aero', 'xAero');
save('y_Aero', 'yAero');
save('z_Aero', 'zAero');



