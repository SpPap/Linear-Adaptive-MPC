% Function which generates desired input reference signal 
% for the quadcopter
%
% 
%
% Author: Spiros Papadopoulos
%

function y = getReferenceSignal(Ts,k,varargin)

% Define the available signal options (you can add more if needed)
validSignals = {'step', 'ramp_z', 'sin_y', 'cos_x', 'sin_yDot', 'cos_xDot'};

% Create parser for variable input arguments
p = inputParser;
p.addParameter('Signal', 'step')

p.parse(varargin{:});

switch p.Results.Signal
    case 'step'
        y = 1;
    case 'ramp_z'
        y = -1 - 0.1*Ts*k;
    case 'sin_y'
        y = 0.5*sin(0.5*Ts*k);
    case 'cos_x'
        y = 0.5*cos(0.5*Ts*k);
    case 'sin_yDot'
        y = 0.25*cos(0.5*Ts*k);
    case 'cos_xDot'
        y = -0.25*sin(0.5*Ts*k);
    otherwise
        error('Error in signal option')
end


end

