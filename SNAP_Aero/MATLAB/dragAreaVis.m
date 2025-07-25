clear all; clc; close all;

addpath('libaero')
load Sat_Aer_541;
% Define "home" volume

Resolution = 1; % cm cube per dot
res = 300;  %home volume dimentions
home_volume = zeros(res,res,res) * NaN;
body_length=30; %cm

% Define satellite shape: 16U Satellite with solar panels

[origin, home_volume] = draw_cubesat(body_length, 0, home_volume, Resolution);
gc=[res res res]/2;
delta_cg=[0 0 0]*0.1; % delta_cg [cm];
origin=gc+delta_cg;                    % centre of mass [cm];


MM=angle2dcm(convang(25,'deg','rad'),convang(0,'deg','rad'),convang(0,'deg','rad'),'ZYX');
rot=rotate_volume(home_volume,MM,origin);
plot_volume(rot);
hold on
vv=[1 0 0]*MM;
vv=vv';
quiver3(origin(1),origin(2),origin(3),50*vv(1),50*vv(2),50*vv(3),'b','LineWidth',2);
vv2=[0 1 0]*MM;
vv2=vv2';
quiver3(origin(1),origin(2),origin(3),50*vv2(1),50*vv2(2),50*vv2(3),'Color',[0 0.5 0],'LineWidth',2);
vv3=[0 0 1]*MM;
vv3=vv3';
quiver3(origin(1),origin(2),origin(3),50*vv3(1),50*vv3(2),50*vv3(3),'Color',[0.75 0 0],'LineWidth',2);
set(gca,'yDir','reverse')
set(gca,'ZDir','reverse')
quiver3(0,0,0,300,0,0,'b-','LineWidth',2.5); hold on
quiver3(0,0,0,0,300,0,'Color',[0 0.5 0],'LineWidth',2); hold on
quiver3(0,0,0,0,0,300,'Color',[0.75 0 0],'LineWidth',2);
 %%
hold on;
for y = 1:size(rot,2)
for z = 1:size(rot,3)
w = find(rot(:,y,z) == 1);
if ~isempty(w)
T=[w(end),y,z];
% T = T + 0.5 * Cd * rho * V^2 * A * cross(Uv, ([w(1),y,z]-origin));
plot3(T(1),T(2),T(3),'y.'); hold on
            %temp(y,z) = norm(0.5 * Cd * A * cross(Uv, [w(1),y,z]-origin));
end
end
end
axis equal