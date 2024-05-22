% Function which returns the prediction for altitude
% based on Linear Model and solved using forward Euler's method
%
  %-----------------------------------------------------% 
  %            z'' = g-ft/m*[c(phi)*c(theta)]           %
  %-----------------------------------------------------%
%
% Author: Spiros Papadopoulos
%


function [z1Vector, z2Vector] = createPredictionVectorAlt(zPrev, ft, Np, quad, simOut)

        % Extract phi, theta, z from Sim
        phi = simOut.states(end,1);
        theta = simOut.states(end,2); 

        % quad.Characteristics
        m = quad.Characteristics.mass; 
        g = quad.Characteristics.g;
        Ts = quad.Simulation.samplingTime;

       % Create Prediction Vector for Altutude
       for iPred = 1:Np
        
        % Define Az, Bz matrices
        Az = [1 Ts;0 1];
        Bz = [0 -Ts/m*cos(phi(end,1))*cos(theta(end,1))]';

        % Gravity compensation
        W = [0 Ts*g]';

        zPred = Az*zPrev + Bz*ft(iPred) + W;
        zPrev = zPred;

        altitudePred(iPred) = zPred(1); % Get predicted altitude   ( z )
        zDot(iPred) = zPred(2);         % Get predicted velocity   ( z')
       
       end
      
       z1Vector = altitudePred;
       z2Vector = zDot;

end


