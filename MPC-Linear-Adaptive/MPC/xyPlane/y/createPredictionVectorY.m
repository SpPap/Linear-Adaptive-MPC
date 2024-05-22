% Function which returns the prediction for y
% based on Linear Model and solved using forward Euler's method
%
  %-----------------------------------------------------------% 
  %   y'' = -ft/m*[c(phi)*s(psi)*s(theta) - c(psi)*s(phi)]    %                                           
  %-----------------------------------------------------------% 
%
% Author: Spiros Papadopoulos
%


function [y1Vector, y2Vector] = createPredictionVectorY(yPrev, ft, uy, Np, quad)

        % quad.Characteristics
        m = quad.Characteristics.mass; 
        g = quad.Characteristics.g;
        Ts = quad.Simulation.samplingTime;

        % Define Ay, By matrices
        Ay = [1 Ts;0 1];
        By = [0 -Ts/m]';

       % Create Prediction Vector for y
       for iPred = 1:Np
        
        yPred = Ay*yPrev + By*ft*uy(iPred);
        yPrev = yPred;

        yPos(iPred) = yPred(1);         % Get predicted y position   ( y )
        yDot(iPred) = yPred(2);         % Get predicted velocity     ( y')
       
       end
      
       y1Vector = yPos;
       y2Vector = yDot;

end


