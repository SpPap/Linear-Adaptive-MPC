% Function for plotting responses
%
% Input: Struct quad
% Output: Plots
%
% Author: Spiros Papadopoulos
%

function [] = plotResponses(quad)

% Plot settings
set(0, 'DefaultFigureColor', 'white');

% Get data from quad struct
tout = quad.Simulation.tout;

x_ref = quad.Output.linearPosition.reference.x_ref;
y_ref = quad.Output.linearPosition.reference.y_ref; 
z_ref = quad.Output.linearPosition.reference.z_ref; 

x = quad.Output.linearPosition.actual.x;
y = quad.Output.linearPosition.actual.y;
z = quad.Output.linearPosition.actual.z;


phi_ref = quad.Output.angularPosition.reference.phi_ref;     % [rad]
theta_ref = quad.Output.angularPosition.reference.theta_ref; % [rad] 
psi_ref = quad.Output.angularPosition.reference.psi_ref;     % [rad]

phi = quad.Output.angularPosition.actual.phi;     % [rad]
theta = quad.Output.angularPosition.actual.theta; % [rad]
psi = quad.Output.angularPosition.actual.psi;     % [rad]

u = quad.Output.LinearVelocities.actual.u; % [m/s]
v = quad.Output.LinearVelocities.actual.v; % [m/s]
w = quad.Output.LinearVelocities.actual.w; % [m/s]

p = quad.Output.AngularVelocities.actual.p; % [rad/s]
q = quad.Output.AngularVelocities.actual.q; % [rad/s]
r = quad.Output.AngularVelocities.actual.r; % [rad/s]

ft = quad.Output.controlSignals.ft;
tx = quad.Output.controlSignals.tx;
ty = quad.Output.controlSignals.ty;
tz = quad.Output.controlSignals.tz;

speed1 = quad.Output.Motor.speeds.speed1; % [rpm]
speed2 = quad.Output.Motor.speeds.speed2; % [rpm]
speed3 = quad.Output.Motor.speeds.speed3; % [rpm]
speed4 = quad.Output.Motor.speeds.speed4; % [rpm]

error_phi = quad.Output.angularPosition.error.phi;     % [rad]
error_theta = quad.Output.angularPosition.error.theta; % [rad]
error_psi = quad.Output.angularPosition.error.psi;     % [rad]

%-----------------------------------------------------------------------------------------%

%% Plot quad states

%% x 
figure('Name', 'x response') 

plot(tout,x_ref, 'm');
hold on 
plot(tout,x, 'b');
hold off

title("\textbf{x position}",'interpreter','latex');
xlabel('time [s]','interpreter','latex');
ylabel('x [m]','interpreter','latex');
legend('reference','output','interpreter','latex')

%% y 
figure('Name', 'y response') 

plot(tout,y_ref, 'm');
hold on 
plot(tout,y, 'b');
hold off

title("\textbf{y position}",'interpreter','latex');
xlabel('time [s]','interpreter','latex');
ylabel('y [m]','interpreter','latex');
legend('reference','output','interpreter','latex')

%% z 
figure('Name', 'z response') 

plot(tout,z_ref, 'm');
hold on 
plot(tout,z, 'b');
hold off

title("\textbf{z position}",'interpreter','latex');
xlabel('time [s]','interpreter','latex');
ylabel('z [m]','interpreter','latex');
legend('reference','output','interpreter','latex')

%% phi 
figure('Name', 'phi response') 

plot(tout,rad2deg(phi_ref), 'm');
hold on 
plot(tout,rad2deg(phi), 'b');
hold off

title("\textbf{Roll angle}",'interpreter','latex');
xlabel('time [s]','interpreter','latex');
ylabel('$\phi\ [^\circ]$','interpreter','latex');
legend('reference','output','interpreter','latex')

%% theta
figure('Name', 'theta response') 

plot(tout,rad2deg(theta_ref), 'm');
hold on 
plot(tout,rad2deg(theta), 'b');
hold off

title("\textbf{Pitch angle}",'interpreter','latex');
xlabel('time [s]','interpreter','latex');
ylabel('$\theta\ [^\circ]$','interpreter','latex');
legend('reference','output','interpreter','latex')

%% psi 
figure('Name', 'psi response') 

plot(tout,rad2deg(psi_ref), 'm');
hold on 
plot(tout,rad2deg(psi), 'b');
hold off

title("\textbf{Yaw angle}",'interpreter','latex');
xlabel('time [s]','interpreter','latex');
ylabel('$\psi\ [^\circ]$','interpreter','latex');
legend('reference','output','interpreter','latex')

%% u 
figure('Name', 'u response') 

plot(tout,u, 'b')

title("\textbf{Linear velocity (x-axis)}", 'interpreter','latex');
xlabel('time [s]','interpreter','latex')
ylabel('u [m/s]','interpreter','latex')

%% v 
figure('Name', 'v response') 

plot(tout,v, 'b')

title("\textbf{Linear velocity (y-axis)}", 'interpreter','latex');
xlabel('time [s]','interpreter','latex')
ylabel('v [m/s]','interpreter','latex')

%% w 
figure('Name', 'w response') 

plot(tout,w, 'b')

title("\textbf{Linear velocity (z-axis)}", 'interpreter','latex');
xlabel('time [s]','interpreter','latex')
ylabel('w [m/s]','interpreter','latex')

%% p 
figure('Name', 'p response') 
plot(tout,p*180/pi, 'b') % [deg/sec]
title("\textbf{Roll rate}", 'interpreter','latex');
xlabel('time [s]','interpreter','latex')
ylabel('$p\ [^\circ/s]$','interpreter','latex')
hold off

%% q 
figure('Name', 'q response') 
plot(tout,q*180/pi, 'b') % [deg/sec]
title("\textbf{Pitch rate}", 'interpreter','latex');
xlabel('time [s]','interpreter','latex')
ylabel('$q\ [^\circ/s]$','interpreter','latex')
hold off

%% r 
figure('Name', 'r response') 
plot(tout,r*180/pi, 'b') % [deg/sec]
title("\textbf{Yaw rate}", 'interpreter','latex');
xlabel('time [s]','interpreter','latex')
ylabel('$r\ [^\circ/s]$','interpreter','latex')
hold off

%% Plot 3D trajectory
figure('Name', '3D trajectory') 

plot3(x_ref,y_ref,-z_ref, 'm', 'LineWidth',0.25)
hold on 
plot3(x,y,-z, 'b' ,'LineWidth',1.5)
hold off

grid on
title('\textbf{3D trajectory}','interpreter','latex')
xlabel('x [m]','interpreter','latex')
ylabel('y [m]','interpreter','latex')
zlabel('z [m]','interpreter','latex')
legend('reference','output','interpreter','latex')

%-----------------------------------------------------------------------------------------%

%% Plot control signals

%% Torque in x-axis tx
figure('Name', 'tau x') 
plot(tout,tx, 'b')
title('\textbf{Roll PID output}','interpreter','latex');
xlabel('time [s]','interpreter','latex')
ylabel('$\tau_x$\ [N.m]','interpreter','latex')

%% Torque in y-axis ty
figure('Name', 'tau y')
plot(tout,ty, 'b')
title('\textbf{Pitch PID output}','interpreter','latex');
xlabel('time [s]','interpreter','latex')
ylabel('$\tau_y$\ [N.m]','interpreter','latex')

%% Torque in z-axis tz
figure('Name', 'tau z')
plot(tout,tz, 'b')
title('\textbf{Yaw PID output}','interpreter','latex');
xlabel('time [s]','interpreter','latex')
ylabel('$\tau_z$\ [N.m]','interpreter','latex')

%% Total thrust ft
figure('Name', 'Total thrust') 
plot(tout,ft, 'b')
title('\textbf{Total Thrust}','interpreter','latex');
xlabel('time [s]','interpreter','latex')
ylabel('$f_t$\ [N]','interpreter','latex')

%-----------------------------------------------------------------------------------------%

%% Motors speeds
customColors = [
                0.0000 0.4470 0.7410; % Color 1 
                0.8500 0.3250 0.0980; % Color 2 
                0.4660 0.6740 0.1880; % Color 3 
                0.9290 0.6940 0.1250; % Color 4 
               ];

figure('Name', 'motor speeds') 

subplot(4,1,1)
plot(tout,speed1,'Color',customColors(1,:));
title('Motor 1 speed [rpm]','interpreter','latex');

subplot(4,1,2)
plot(tout,-speed2, 'Color',customColors(2,:));
title('Motor 2 speed [rpm]','interpreter','latex');

subplot(4,1,3)
plot(tout,speed3,'Color',customColors(3,:));
title('Motor 3 speed [rpm]','interpreter','latex');

subplot(4,1,4)
plot(tout,-speed4,'Color',customColors(4,:));
title('Motor 4 speed [rpm]','interpreter','latex');

sgtitle('\textbf{Motor Speeds}','interpreter','latex');
xlabel('time [s]');

%-----------------------------------------------------------------------------------------%

%% Error signals

%% phi error
figure('Name', 'error phi') 
plot(tout,rad2deg(error_phi), 'b'); % [deg]
title('\textbf{Roll angle error}','interpreter','latex');
xlabel('time [s]','interpreter','latex');
ylabel('$e_{\phi}\ [^\circ]$','interpreter','latex');


%% theta error
figure('Name', 'error theta') 
plot(tout,rad2deg(error_theta), 'b');  % [deg]
title('\textbf{Pitch angle error}','interpreter','latex');
xlabel('time [s]','interpreter','latex');
ylabel('$e_{\theta}\ [^\circ]$','interpreter','latex');

%% psi error
figure('Name', 'error psi') 
plot(tout,rad2deg(error_psi), 'b');  % [deg]
title('\textbf{Yaw angle error}','interpreter','latex');
xlabel('time [s]','interpreter','latex');
ylabel('$e_{\psi}\ [^\circ]$', 'interpreter', 'latex')

end