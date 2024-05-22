% Function which returns the prediction for x
% based on Linear Model and solved using forward Euler's method
%
% :------------------------------------------------------: 
% : x'' = -ft/m*[s(phi)*s(psi) + c(phi)*c(psi)*s(theta)] :                                           
% :------------------------------------------------------: 
%
% Author: Spiros Papadopoulos
%


function [x1Vector, x2Vector] = createPredictionVectorX(xPrev, ft, ux, Np, quad)

        % quad.Characteristics
        m = quad.Characteristics.mass; 
        g = quad.Characteristics.g;
        Ts = quad.Simulation.samplingTime;

        % Define Ax, Bx matrices
        Ax = [1 Ts;0 1];
        Bx = [0 -Ts/m]';

       % Create Prediction Vector for x
       for iPred = 1:Np

        xPred = Ax*xPrev + Bx*ft*ux(iPred);
        xPrev = xPred;

        xPos(iPred) = xPred(1);         % Get predicted x position   ( x )
        xDot(iPred) = xPred(2);         % Get predicted velocity     ( x')
       
       end
      
       x1Vector = xPos;
       x2Vector = xDot;

end


