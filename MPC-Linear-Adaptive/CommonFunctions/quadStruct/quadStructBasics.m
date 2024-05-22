%% Create quadcopter's basic structure fields

% quad.Characteristics
quad.Characteristics.mass = m; 
quad.Characteristics.thrustFactor = b; 
quad.Characteristics.dragFactor = d; 
quad.Characteristics.length = l;
quad.Characteristics.g = g;

% quad.Simulation
quad.Simulation.duration = duration;
quad.Simulation.samplingTime = Ts;
quad.Simulation.tout = (0:Ts:duration)';