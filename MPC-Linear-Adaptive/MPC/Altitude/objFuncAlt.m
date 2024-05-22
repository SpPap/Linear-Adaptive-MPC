% Function which returns the Objective Function for z-axis to be minimized
%
%
%
% Author: Spiros Papadopoulos
%

function Jz = objFuncAlt(Dft,ftPrev,simOut,refVector,quad,mpcParams)
       
    % Extract z, z' from Sim
    zLast = simOut.states(end,end);
    zDotLast = simOut.states(end,9); % w

    % Get MPC parameters
    Nc = mpcParams.Nc;
    Np = mpcParams.Np;

    Q = mpcParams.Q;
    R = mpcParams.R;

    %% Calculate new ft (Total Thrust) -- Dimensions: (Npx1) 
    for i = 1:Nc
        ft(i) = Dft(i) + ftPrev;
        ftPrev = ft(i);
    end

    %% Implement equality constraint -- Dft(i)=0,  i=Nc+1,...,Np
    % If Control Horizon is smaller than Prediction Horizon, keep last calculated thrust for the rest 
    ft(Nc+1:Np) = ft(Nc);

    %% Use Prediction Model (Linear for this case) to predict [z{k+1} z{k+1}']
    
    % [z z']
    zPrev = [zLast zDotLast]';
    
    % Use Linear Prediction Model for z, z' predictions & Create Prediction Vector
    [z1Vector, z2Vector] = createPredictionVectorAlt(zPrev, ft, Np, quad, simOut);
  
    zPrev = [z1Vector(end) z2Vector(end)];

    % Create Error Vector   
    zPredError = refVector - z1Vector;

    %% Return Objective Function Evaluation
    Jz = zPredError*Q*zPredError' + Dft*R*Dft'; % Dimensions: (1xNp)*(NpxNp)*(Npx1) + (1xNc)*(NcxNc)*(Ncx1)

end
