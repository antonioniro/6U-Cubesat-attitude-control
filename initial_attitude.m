function [q, omega] = initial_attitude(R0, V0, init)

mu = 398600; % (km^3/s^2) Earth gravit. constant 

rm = sqrt(R0(1)^2 + R0(2)^2 + R0(3)^2);
w = [-R0(1)/rm -R0(2)/rm -R0(3)/rm];
v = -cross(R0, V0);
vm = sqrt(v(1)^2 + v(2)^2 + v(3)^2);
v = v./vm;
u = cross(v,w);

Min_orb = [u;v;w];

yaw = init(1);
pitch = init(2);
rol = init(3);
yawp = init(4);
pitchp = init(5);
rolp = init(6);

cy = cos(yaw);
sy = sin(yaw);
cp = cos(pitch);
sp = sin(pitch);
cr = cos(rol);
sr = sin(rol);

Morb_body=[cp*cy cp*sy  -sp;-cr*sy+sr*sp*cy cr*cy+sr*sp*sy  sr*cp;sr*sy+cr*sp*cy  -sr*cy+cr*sp*sy  cr*cp];
M313=Morb_body*Min_orb;                         

omega0 = sqrt(mu/rm^3);

omega1=-omega0*cos(pitch)*sin(yaw)-yawp*sin(pitch)+rolp;
omega2=-omega0*cos(rol)*cos(yaw)-omega0*sin(rol)*sin(pitch)*sin(yaw)+yawp*sin(rol)*cos(pitch)+pitchp*cos(rol);
omega3=omega0*sin(rol)*cos(yaw)-omega0*cos(rol)*sin(pitch)*sin(yaw)+yawp*cos(rol)*cos(pitch)-pitchp*sin(rol);                                    

q4=(1/2)*sqrt(1+M313(1,1)+M313(2,2)+M313(3,3));  %parametri di Eulero iniziali, Wertz pag. 415
q1=(1/(4*q4))*(M313(2,3)-M313(3,2));
q2=(1/(4*q4))*(M313(3,1)-M313(1,3));
q3=(1/(4*q4))*(M313(1,2)-M313(2,1));

q = [q1 q2 q3 q4];
omega = [omega1 omega2 omega3];

end

