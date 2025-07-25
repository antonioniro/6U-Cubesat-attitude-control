%% Simplified Plotting
simtime = 0:1:tsim;
roll = out.roll.signals.values;
pitch = out.pitch.signals.values;
yaw = out.yaw.signals.values;

%% Figure 1
figure(1)
plot(simtime,roll,'b')
xlabel('Time [s]');
ylabel('Roll [deg]')
grid on

figure(2)
plot(simtime,pitch,'b')
xlabel('Time [s]');
ylabel('Pitch [deg]')
grid on

figure(3)
plot(simtime,yaw,'b')
xlabel('Time [s]');
ylabel('yaw [deg]')
grid on

%% Angular momentum Management
hw_1=out.h_wheels.signals.values(:,1);
hw_2=out.h_wheels.signals.values(:,2);
hw_3=out.h_wheels.signals.values(:,3);
hw_4=out.h_wheels.signals.values(:,4);

figure(4);
plot(simtime/(3600),hw_1,'LineWidth',1.2); hold on
plot(simtime/(3600),hw_2,'LineWidth',1.2); hold on 
plot(simtime/(3600),hw_3,'LineWidth',1.2); hold on 
plot(simtime/(3600),hw_4,'LineWidth',1.2); hold on
hold on
plot([simtime(1),simtime(end)/3600],[0.085 0.085],'-.k')
hold on
plot([simtime(1)/3600,simtime(end)/3600],[-0.085 -0.085],'-.k')
hold on
plot([simtime(1)/3600,simtime(end)/3600],[0.1 0.1],'-.r')
hold on
plot([simtime(1)/3600,simtime(end)/3600],[-0.1 -0.1],'-.r')
legend('RW 1', 'RW 2', 'RW 3', 'RW 4','Max. Sat. Limit -15%','Min. Sat. Limit + 15%','Max Sat Limit','Min Sat Limit','Location','northoutside','NumColumns',4);
xlabel('Time [hr]');
ylabel('h_w [Nms]');
%title('RWs angular momentum - m = 0.1 Am^2 - pre/post-rate,'NumColumns','4')
grid on
axis([simtime(1)/(3600) simtime(end)/(3600) -0.15 0.15])


%% Power Simplified
P1 = out.T_wheel_power1.data(:);
P2 = out.T_wheel_power2.data(:);
P3 = out.T_wheel_power3.data(:);
P4 = out.T_wheel_power4.data(:);

figure
plot(simtime/3600,P1,simtime/3600,P2,simtime/3600,P3,simtime/3600,P4)
legend('RW 1', 'RW 2', 'RW 3', 'RW 4');
xlabel('Time [hr]');
ylabel('Power [W]');
grid on