% quad.Output
quad.Output.linearPosition.reference.x_ref = x_ref;
quad.Output.linearPosition.reference.y_ref = y_ref;
quad.Output.linearPosition.reference.z_ref = z_ref;

quad.Output.linearPosition.actual.x = x;
quad.Output.linearPosition.actual.y = y;
quad.Output.linearPosition.actual.z = z;

quad.Output.LinearVelocities.actual.u  = u;
quad.Output.LinearVelocities.actual.v  = v;
quad.Output.LinearVelocities.actual.w  = w;

quad.Output.angularPosition.reference.phi_ref    =  phi_ref;
quad.Output.angularPosition.reference.theta_ref  =  theta_ref;
quad.Output.angularPosition.reference.psi_ref    =  psi_ref; 

quad.Output.angularPosition.actual.phi   = phi;
quad.Output.angularPosition.actual.theta = theta;
quad.Output.angularPosition.actual.psi   = psi;

quad.Output.AngularVelocities.actual.p  = p;
quad.Output.AngularVelocities.actual.q  = q;
quad.Output.AngularVelocities.actual.r  = r;

quad.Output.controlSignals.ft = ft;
quad.Output.controlSignals.tx = tx;
quad.Output.controlSignals.ty = ty;
quad.Output.controlSignals.tz = tz;
 
% Calculate Speeds, Convert to rpm and add them to struct
speeds = decouplingSpeeds(ft',tx',ty',tz',quad); % [rad/s]
speeds = speeds*30/pi; % [rpm]

quad.Output.Motor.speeds.speed1 = speeds(:,1);
quad.Output.Motor.speeds.speed2 = speeds(:,2);
quad.Output.Motor.speeds.speed3 = speeds(:,3);
quad.Output.Motor.speeds.speed4 = speeds(:,4);


% Calculate Mean Square Error & add it to struct
MSEx     = mse(x_ref,x);
MSEy     = mse(y_ref,y);
MSEz     = mse(z_ref,z);
MSEphi   = mse(rad2deg(phi_ref'),rad2deg(phi));
MSEtheta = mse(rad2deg(theta_ref'),rad2deg(theta));
MSEpsi   = mse(rad2deg(psi_ref'),rad2deg(psi));

% Calculate Euclidean distance between Reference and Actual Trajectories
% Tracking error
errorXYZ = sqrt((x_ref-x).^2 + (y_ref-y).^2 + (z_ref-z).^2);

quad.Output.Metrics.MSE.x     = MSEx;
quad.Output.Metrics.MSE.y     = MSEy;
quad.Output.Metrics.MSE.z     = MSEz;
quad.Output.Metrics.MSE.phi   = MSEphi;
quad.Output.Metrics.MSE.theta = MSEtheta;
quad.Output.Metrics.MSE.psi   = MSEpsi;

quad.Output.Metrics.Trajectory3D = mean(errorXYZ);

% Add roll, pitch, yaw angles errors in struct
quad.Output.angularPosition.error.phi   = error_phi;
quad.Output.angularPosition.error.theta = error_theta;
quad.Output.angularPosition.error.psi   = error_psi;