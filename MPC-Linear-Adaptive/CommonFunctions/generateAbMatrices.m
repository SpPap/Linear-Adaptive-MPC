% Function which returns the b matrix for the A*x<=b constraint inside the optimization problem
%
%
%
% Author: Spiros Papadopoulos
%

% A matrix (2*Nc x Nc)
% Constists of 2 lower triangular matrices each of Nc x Nc
%
%  [ 
%     1  0   0   0   0   0 . . .             
%     1  1   0   0   0   0 . . .   (Nc x Nc)   
%     1  1   1   0   0   0 . . .             
%
%    -1  0   0   0   0   0 . . . 
%    -1 -1   0   0   0   0 . . .   (Nc x Nc)
%    -1 -1  -1   0   0   0 . . .  
%                                ]
%
%
%

% b Matrix (2*Nc x Nc)
% Consists of two submatrices each of Nc x Nc 
%
% [  ftMax - ftPrev
%    ftMax - ftPrev
%    ftMax - ftPrev
%          .
%          .
%   -ftMin + ftPrev
%   -ftMin + ftPrev
%   -ftMin + ftPrev ]

% IMPORTANT: In case of ft, it could also be : ux, uy

function [A b] = generateAbMatrices(ftMax,ftMin,ftPrev,mpcParams)
    
% Get Control Horizon    
Nc = mpcParams.Nc;

%% Generate A matrix

plusOnes  =  ones(Nc);
minusOnes = -ones(Nc);

% Create upper part of A
Aupper = tril(plusOnes);

% Create lower part of A
Alower = tril(minusOnes);

% Concatenate the two matrices and create A
A = [Aupper;Alower];


%% Generate b matrix

b(1:Nc) = ftMax - ftPrev;
b(Nc+1:2*Nc) = -ftMin + ftPrev;

b = b(:);
    
end