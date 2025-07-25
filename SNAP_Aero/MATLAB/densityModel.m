clear all; close all; clc;

load('density_model.txt');

alt_range=density_model(:,1)'; % altitude [km] range;
density=density_model(:,2)';   % density  [g/cm^3] range (MSISE90, from https://ccmc.gsfc.nasa.gov/modelweb/models/msis_vitmo.php);

densModel.altitude=alt_range; % altitude [km]
densModel.rho=density*1000;   % density [kg/m^3];

save('densityModel','densModel')

