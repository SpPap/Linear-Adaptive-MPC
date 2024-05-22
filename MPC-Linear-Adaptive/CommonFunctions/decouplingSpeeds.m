%
% Decoupling function
% input force,torques to speeds of motors
% [ft,tx,ty,tz]-->[W1,W2,W3,W3]
%
function W = decouplingSpeeds(ft,tx,ty,tz,quad)

%% Define constants
% thrust factor [N*s^2]
b = quad.Characteristics.thrustFactor;
% distance between any rotor and the COM [m]
l = quad.Characteristics.length; 
% drag factor  [N*m*s^2]
d = quad.Characteristics.dragFactor;

%% Input Matrix
T = [ft tx ty tz]';

A = [ b b b b;
     -b*l 0 b*l 0; 
      0 -b*l 0 b*l; 
     -d d -d d ];

% Calculate speeds.^2
W_squared = A\T;

% Return speeds 
W = sqrt(abs(W_squared))'; % rad/s

% Return speeds^2
% W = W_squared';