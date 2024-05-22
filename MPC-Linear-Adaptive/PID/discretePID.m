% Function for a discrete PID controller
%
% For the integral term, 2 methods are being used for numerical
% integration (Trapezoidal rule and Composire Simpson's 1/3 rule). We have 
% an advanced integral term handling based on whether k is even or odd.
%
% Author: Spiros Papadopoulos
%

function [output, integralSum] = discretePID(error, integralSumPrev, Kp, Ki, Kd, Ts, k)

    % Initialize the integral sum if not provided (on the first iteration)
    if isempty(integralSumPrev)
        integralSum = 0;
    else
        integralSum = integralSumPrev;
    end
   
    % Set P,I,D terms to zero
    proportionalTerm = 0; % P
    integralTerm = 0;     % I
    derivativeTerm = 0;   % D

    
    if k > 1 
        proportionalTerm = Kp * error(k);
        if mod(k,2) == 0
            integralTerm = Ki * Ts * (integralSum + (error(k) + error(k-1)) / 2);     % Trapezoidal rule
        else
            % Update the integral sum for the next iteration
            integralSum = integralSum + (error(k) + 4 * error(k-1) + error(k-2)) / 3; % Composite Simpson's rule
            integralTerm = Ki * Ts * integralSum;
        end
        derivativeTerm = Kd * (error(k) - error(k-1)) / Ts;
    end    
    
    % Calculate the controller output
    output = proportionalTerm + integralTerm + derivativeTerm;
end

% function [output, integralSum] = discretePID(error, integralSumPrev, Kp, Ki, Kd, Ts, k)
% 
%     % Initialize the integral sum if not provided (on the first iteration)
%     if isempty(integralSumPrev)
%         integralSum = 0;
%     else
%         integralSum = integralSumPrev;
%     end
%    
%     % Set P,I,D terms to zero
%     proportionalTerm = 0; % P
%     integralTerm = 0;     % I
%     derivativeTerm = 0;   % D
% 
%     if k > 0
%         proportionalTerm = Kp * error(k);
%     end
%     
%     if k > 1
%         % Update the integral sum for the next iteration
%         integralSum = integralSum + (error(k) + error(k-1)) / 2; % Trapezoidal rule for integration
%         integralTerm = Ki * Ts * integralSum;
%         derivativeTerm = Kd * (error(k) - error(k-1)) / Ts;
%     end    
% 
%     % Calculate controller's output
%     output = proportionalTerm + integralTerm + derivativeTerm;
% end

% function [output, integralSum] = discretePID(error, integralSumPrev, Kp, Ki, Kd, Ts, k, N)
% 
%     %% Initialize the integral sum if not provided (on the first iteration)
%     if isempty(integralSumPrev)
%         integralSum = 0;
%     else
%         integralSum = integralSumPrev;
%     end
%    
%     %% Set P,I,D terms to zero
%     proportionalTerm = 0; % P
%     integralTerm = 0;     % I
%     derivativeTerm = 0;   % D
% 
%      %% Calculate Derivative term using moving average filter
%      if k > N
%         % Calculate the moving average of the error signal over the last N samples
%         movingAverageError = movmean(error(k-N+1:k), N);
%         
%         % Calculate the derivative term using the moving average error
%         derivativeTerm = Kd * (error(k) - movingAverageError(end)) / Ts;
%      end
% 
%     %% Calculate Proportional and Integral terms
%     if k > 1 
%         proportionalTerm = Kp * error(k);
%         if mod(k,2) == 0
%             integralTerm = Ki * Ts * (integralSum + (error(k) + error(k-1)) / 2);     % Trapezoidal rule
%         else
%             % Update the integral sum for the next iteration
%             integralSum = integralSum + (error(k) + 4 * error(k-1) + error(k-2)) / 3; % Composite Simpson's rule
%             integralTerm = Ki * Ts * integralSum;
%         end
%     end    
% 
%     %% Calculate the controller's output
%     output = proportionalTerm + integralTerm + derivativeTerm;
% 
% end

