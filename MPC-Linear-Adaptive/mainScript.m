% Model Predicted Controller using Linear Adaptive Prediction model
% MATLAB main script
%
%
% Author: Spiros Papadopoulos
%

%% Clear Workspace & Command Window
close all % Optional
clc
clear 
format long

%% Fix MATLAB Path
path = pwd();
addpath(genpath(path));

%% Load constant data for the model
setSimulationSettings()

%% Create quadcopter struct (quad) & Load basic fields
quadStructBasics()

%% Define MPC Parameters for Altitude & x-y Controllers
setMPCSettings()

%% Define options for Optimizer

% Use 'sqp' solver (Sequential Quadratic Programming)
options = optimoptions('fmincon','Display','None','Algorithm','sqp');

%% Define initial conditions for ftPrev,xPrev,yPrev,DftPrev,DuxPrev,DuyPrev

ftPrev = 0;                   % Previous Total Thrust 
DftPrev = zeros(1,mpcParamsAlt.Nc);

uxPrev = 0;                   % Previous ux
uyPrev = 0;                   % Previous uy

DuxyPrev = zeros(mpcParamsXY.Nc,2);

initConditions = zeros(1,12); % Initial conditions for system states [phi theta psi p q r u v w x y z] 

% Save initial conditions in struct
simOut.states = initConditions;

%% Initialize system states

% --- Earth Frame --- %
%---------------------%

% Linear Positions 
x = [];
y = [];
z = [];

% Angular Positions 
phi   =  [];
theta =  [];
psi   =  [];


% ---- Body Frame --- %
%---------------------%

% Linear Velocities 
u = [];
v = [];
w = [];

% Angular Velocities 
p = [];
q = [];
r = [];

%-------------------------------------------%

% Initialize Objective Functions Evaluations
Jz = [];
Jxy = [];

% Initialize reference angles
phi_ref   = [];
theta_ref = [];
psi_ref = [];

%% PID Settings
setPIDSettings()

%% Main Loop

fprintf("\nSimulation started\n")

% Yaw Reference Vector
psi_ref = zeros(length(1:(duration/Ts)+1),1);

for k = 1:(duration/Ts)+1

    %% Generate reference sub-vector for current time step

    % Altitude z
    refVectorAlt = getReferenceSignal(Ts,(k+1:k+mpcParamsAlt.Np),'Signal','ramp_z');

    % x
    refVectorX = getReferenceSignal(Ts,(k+1:k+mpcParamsXY.Np),'Signal','cos_x');
    % y
    refVectorY = getReferenceSignal(Ts,(k+1:k+mpcParamsXY.Np),'Signal','sin_y');

    % x'
    refVectorXDot= getReferenceSignal(Ts,(k+1:k+mpcParamsXY.Np),'Signal','cos_xDot');
    % y'
    refVectorYDot = getReferenceSignal(Ts,(k+1:k+mpcParamsXY.Np),'Signal','sin_yDot');

    %% Print current step k 
    progress = k/((duration/Ts)+1)*100;
    fprintf("\nSimulation Time Step %d - Progress: %.1f%%", k, progress)

    z = [z; simOut.states(end,end)];
    
    % Update x,y
    x = [x; simOut.states(end,end-2)];
    y = [y; simOut.states(end,end-1)];
    
    % Update phi,theta,psi
    phi = [phi; simOut.states(end,1)]; 
    theta = [theta; simOut.states(end,2)];
    psi = [psi; simOut.states(end,3)];
    
    %% Define Optimization problem for Altitude-z & Solve

    %-----%
    %  z  %
    %-----%

    % Define optimization problem 
    x0 = DftPrev;
    [A, b] = generateAbMatrices(ftMax,ftMin,ftPrev,mpcParamsAlt);
    
    % Equality constraints
    Aeq = []; % Not used
    beq = []; % Not used

    % Boundary constraints
    lb = DftMin;
    ub = DftMax;

    % Create handle for Non-Linear constraints // Terminal Constraints
    nonlcon = @(Dft) terminalConstraintAlt(Dft,ftPrev,simOut,refVectorAlt,quad,mpcParamsAlt); 

    % Derive objective function and create a function handle
    objectiveFunctionAltitude = @(Dft) objFuncAlt(Dft,ftPrev,simOut,refVectorAlt,quad,mpcParamsAlt);

    % Solve & Store time needed for optimization 
    % problem to be solved
    tStart = tic;

    [Dft, JzCurrent] = fmincon(objectiveFunctionAltitude,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
    tEnd = toc(tStart);

    Jz = [Jz; JzCurrent];

    optTime(k) = tEnd;
    %fprintf(" - z Optimization Duration: %.2fms\n", optTime(k)*1000)

    % Calculate Total Thrust ft
    ft(k) = Dft(1) + ftPrev;

    %% Define Optimization problem for x-y Plane & Solve

    %-----%
    %  xy %
    %-----%

    % Define optimization problem 

    x0 = DuxyPrev;
    
    % for x
    [Ax, bx] = generateAbMatrices(uxMax,uxMin,uxPrev,mpcParamsXY);
    % for y
    [Ay, by] = generateAbMatrices(uyMax,uyMin,uyPrev,mpcParamsXY);

    % Construct  A, b
    A = blkdiag(Ax,Ay);
    b = [bx; by];
    
    % Equality constraints
    Aeq = []; % Not used
    beq = []; % Not used

    % Boundary constraints
    lb = DuxyMin;
    ub = DuxyMax;

    % Create handle for Non-Linear constraints // Terminal Constraints
    nonlcon = @(Duxy) terminalConstraintXY(Duxy,uxPrev,uyPrev,ft(k),simOut,refVectorX,refVectorY,refVectorXDot,refVectorYDot,quad,mpcParamsXY);

    % Derive objective function for x-y and create a function handle
    objectiveFunctionXY = @(Duxy) objFuncXY(Duxy,uxPrev,uyPrev,ft(k),simOut,refVectorX,refVectorY,refVectorXDot,refVectorYDot,quad,mpcParamsXY);

    % Solve & Store time needed for optimization 
    % problem to be solved
    tStart = tic;

    [Duxy, JxyCurrent] = fmincon(objectiveFunctionXY,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
    tEnd = toc(tStart);

    Jxy = [Jxy; JxyCurrent];

    optTime(k) = tEnd;
    %fprintf(" - Optimization Duration: %.2fms\n", optTime(k)*1000)

    % Calculate ux, uy
    ux(k) = Duxy(1,1) + uxPrev;
    uy(k) = Duxy(1,2) + uyPrev;

    %% Transform Virtual Control signals ux,uy to reference signals phi_ref,theta_ref

%     x_ref ---> %--------% --ux--> %-------------% ---> phi_ref 
                 % MPC xy %         % Transformer %  
%     y_ref ---> %--------% --uy--> %-------------% ---> theta_ref

    [phi_ref(k), theta_ref(k)] = transformVirtualControlToRef(ux,uy);

    %% Create error for x,y and pass it to PID Controllers
    
    % PID Pitch
    error_theta(k) = theta_ref(k) - theta(k);
    [ty(k), integralSumPrevPitch] = discretePID(error_theta, integralSumPrevPitch, pitchPID.Kp, pitchPID.Ki, pitchPID.Kd, Ts, k);

    % PID Roll
    error_phi(k) = phi_ref(k) - phi(k);
    [tx(k), integralSumPrevRoll] = discretePID(error_phi, integralSumPrevRoll, rollPID.Kp, rollPID.Ki, rollPID.Kd, Ts, k);
  
    %% Yaw Controller

    %-------%
    %  yaw  %
    %-------%
    error_psi(k) = 0 - psi(k);
    [tz(k), integralSumPrevYaw] = discretePID(error_psi, integralSumPrevYaw, yawPID.Kp, yawPID.Ki, yawPID.Kd, Ts, k);

    %% Run Model Simulation

    % Use Simulink with ode45 solver
    %simOut = sim(simFile,"Solver","ode45","StopTime",string(Ts));

    % Use ode15s solver 
    ode_options = odeset('RelTol',1e-6,'AbsTol',1e-9);
    [~,xx] = ode15s(@(t,xx)ode_quad(t,xx,ft(k),tx(k),ty(k),tz(k)),[0 Ts],initConditions,ode_options);
    simOut.states = xx(end,:);


    % Update initial conditions
    initConditions = simOut.states;

    %-----%
    %  z  %
    %-----%

    % Update ftPrev - "Previous" Total Thrust
    ftPrev = ft(k);

    % Update Dft intitial conditions
    DftPrev = [Dft(2:end) Dft(end)];
    
    %------%
    %  xy  %
    %------%

    % Update uxPrev 
    uxPrev = ux(k);
    % Update uyPrev 
    uyPrev = uy(k);

    % Update Duxy intitial conditions
    DuxyPrev = [Duxy(2:end,:); Duxy(end,:)];


    %% Save new system states

    u = [u; simOut.states(end,7)];
    v = [v; simOut.states(end,8)];
    w = [w; simOut.states(end,9)];

    p = [p; simOut.states(end,4)];
    q = [q; simOut.states(end,5)];
    r = [r; simOut.states(end,6)];

end
fprintf("\n\nSimulation completed succesfully")

%% Create complete reference vectors for plotting

% z
z_ref = getReferenceSignal(Ts,(1:duration/Ts+1),'Signal','ramp_z')';
% x
x_ref = getReferenceSignal(Ts,(1:duration/Ts+1),'Signal','cos_x')';
% y
y_ref = getReferenceSignal(Ts,(1:duration/Ts+1),'Signal','sin_y')';

%% Update quad struct
fprintf("\nCreating quadcopter struct...\n")
quadStructUpdate()

%% Plot responses
plotResponses(quad)

