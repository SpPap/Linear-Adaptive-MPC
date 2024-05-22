% This function is used to describe the mathematical model of the
% quadcopter using the set of coupled non-linear differential equations
% describing the kinematics 
%
% Author: Spiros Papadopoulos
%

function dx = ode_quad(~,x,ft,tau_x,tau_y,tau_z)

    % --- Quadcopter Coefficients --- %
    m = 1;
    g = 9.80665;
    Ix = 8.0*10^(-3);
    Iy = 8.0*10^(-3);
    Iz = 14.2*10^(-3);
    Jr = 1.08*10^(-6);
    % ------------------------------- %

    % --- Drag Coefficients --- %
    Kx = 0.048;
    Ky = 0.11;
    Kz = 0.046;
    Kp = 0.03;
    Kq = 0.03;
    Kr = 0.01;
    % ------------------------- %

    % --- Quadcopter States --- %
    phi   = x(1);
    theta = x(2);
    psi   = x(3);
    p     = x(4);
    q     = x(5);
    r     = x(6);
    u     = x(7);
    v     = x(8);
    w     = x(9);
    x_pos = x(10);
    y     = x(11);
    z     = x(12);
    % ------------------------- %

    % --- Quadcopter Differential Equations --- %
    dx = [ p + r*(cos(phi)*tan(theta)) + q*(sin(phi)*tan(theta)) ;
           q*cos(phi) - r*sin(phi) ;
           r*(cos(phi)/cos(theta)) + q*(sin(phi)/cos(theta)) ;
           ((Iy-Iz)/Ix)*r*q + (tau_x/Ix) - (Kp*p);
           ((Iz-Ix)/Iy)*p*r + (tau_y/Iy) - (Kq*q) ;
           ((Ix-Iy)/Iz)*p*q + (tau_z/Iz) - (Kr*r) ;
           r*v - q*w - g*sin(theta) - Kx*u;
           p*w - r*u + g*(sin(phi)*cos(theta)) - Ky*v;
           q*u - p*v + g*(cos(theta)*cos(phi)) - (ft/m) - Kz*w ;
           w*((sin(phi)*sin(psi)) + (cos(phi)*cos(psi)*sin(theta))) - v*((cos(phi)*sin(psi)) - (cos(psi)*sin(phi)*sin(theta))) + u*(cos(psi)*cos(theta)) ;
           v*((cos(phi)*cos(psi)) + (sin(phi)*sin(psi)*sin(theta))) - w*((cos(psi)*sin(phi)) - (cos(phi)*sin(psi)*sin(theta))) + u*(cos(theta)*sin(psi)) ;
           w*(cos(phi)*cos(theta)) - u*sin(theta) + v*(cos(theta)*sin(phi)) ];
    % ----------------------------------------- %

end