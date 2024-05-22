% Function which returns non-linear constraints for altitude
% Terminal equality constraint derived from Lyapunov
%
%
% Author: Spiros Papadopoulos
%

function [c,ceq] = terminalConstraintAlt(Dft,ftPrev,simOut,refVector,quad,mpcParams)
 
    %% Non-Linear Inequality Constraint
    c = [];     % Inequality non-linear constraints

    % Get MPC parameters
    Np = mpcParams.Np;
    Nc = mpcParams.Nc;

    % Extract phi, theta, z from Sim
    phi = simOut.states(end,1);
    theta = simOut.states(end,2); 
    zLast = simOut.states(end,end);
    zDotLast = simOut.states(end,9); % w

    %% Calculate new ft (Total Thrust) -- Dimensions: (Npx1) 
    for i = 1:Nc
        ft(i) = Dft(i) + ftPrev;
        ftPrev = ft(i);
    end

    %% Implement equality constraint -- Dft(i)=0,  i=Nc+1,...,Np
    % If Control Horizon is smaller than Prediction Horizon, keep last calculated thrust for the rest 
    ft(Nc+1:Np) = ft(Nc);

    % [z z']
    zPrev = [zLast zDotLast]';

    % Use Linear Prediction Model for z, z' predictions & Create Prediction Vector
    [z1Vector, z2Vector] = createPredictionVectorAlt(zPrev, ft, Np, quad, simOut);
        
    zPrev = [z1Vector(end) z2Vector(end)];

    % Create Error Vector   
    zPredError = refVector - z1Vector;
    
    %% Non-Linear Equality Constraint
    ceq = refVector(end) -  z1Vector(end); 
end


