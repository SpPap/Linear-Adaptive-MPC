%% Simulation Settings

% Total Mass of quadcopter [kg]
m = 1;
% Gravitational acceleration [m/s^2]
g = 9.81;
% Thrust factor [N*s^2]
b = 53.4*10^(-6);
% Distance between any motor and the COM [m]
l = 235*10^(-3); 
% Drag factor  [N*m*s^2]
d = 1.1*10^(-6);

% Simulation duration [s]
duration = 35;

% Sampling time [s] 
Ts = 20e-3; % (or 50 Hz)