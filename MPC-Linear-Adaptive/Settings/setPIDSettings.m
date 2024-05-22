%% PID Settings

% Pitch
integralSumPrevPitch = [];
error_theta = [];

pitchPID.Kp = 0.035;
pitchPID.Ki = 0;
pitchPID.Kd = 0.25;
% Roll
integralSumPrevRoll = [];
error_phi = [];

rollPID.Kp = 0.035;
rollPID.Ki = 0;
rollPID.Kd = 0.25;

% Yaw
integralSumPrevYaw = [];
error_yaw = [];

yawPID.Kp = 0.1;
yawPID.Ki = 0.001;
yawPID.Kd = 0.1;