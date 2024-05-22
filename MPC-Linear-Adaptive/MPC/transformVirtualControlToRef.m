% This function transforms virtual control signals ux, uy to theta_ref,
% phi_ref
%
% INPUTS:  ux, uy
% OUTPUTS: theta_ref, phi_ref
%
% Author: Spiros Papadopoulos
%

% IMPORTANT :
% We assume that psi = 0
%

function [phi_ref, theta_ref] = transformVirtualControlToRef(ux,uy)

phi_ref = asin(-uy(end));
theta_ref = asin(ux(end)/cos(phi_ref));

end