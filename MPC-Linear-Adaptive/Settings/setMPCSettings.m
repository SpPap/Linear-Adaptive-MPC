%% MPC Settings

%-----%
%  z  %
%-----%

% Horizons
Nc = 3;     % Control Horizon
Np = 50;    % Prediction Horizon (Np <= Nc) % Use Np = Nc for now

% Penalty matrices [Diagonal]
Q = eye(Np);                % Np x Np
R = 0.01*eye(Nc);             % Nc x Nc

% Create struct for MPC Altitude Parameters
mpcParamsAlt.Nc = Nc;
mpcParamsAlt.Np = Np;
mpcParamsAlt.Q = Q;
mpcParamsAlt.R = R;

% Define Min/Max Total Thrust
ftMin = -35;
ftMax = -ftMin;

% Define Min/Max Dft
DftMin = -6*ones(Nc,1);
DftMax = -DftMin;

%------%
%  xy  %
%------%

% Horizons
Nc = 5;    % Control Horizon
Np = 20;   % Prediction Horizon (Np <= Nc) % Use Np = Nc for now

% Penalty matrices [Diagonal]
Q = 1*eye(Np);                      % Np x Np
R = 1*eye(Nc);                      % Nc x Nc

% Create struct for MPC xy-plane Parameters
mpcParamsXY.Nc = Nc;
mpcParamsXY.Np = Np;
mpcParamsXY.Q = Q;
mpcParamsXY.R = R;

% Define Min/Max ux/uy
uxMin = -0.1736; %-sin(deg2rad(10))
uxMax = -uxMin;

uyMin = uxMin; % use same as x
uyMax = uxMax; % use same as x

% Define Min/Max Duxy
DuxyMin = -0.08*ones(Nc,2);
DuxyMax = -DuxyMin;