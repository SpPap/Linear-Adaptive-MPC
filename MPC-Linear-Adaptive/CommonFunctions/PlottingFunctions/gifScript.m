% This script creates an animated plot for quad reference tracking
%
%
% Author: Spiros Papadopoulos
%

close all

% Create a figure for the animation
fig = figure('Name', '3D trajectory');

% Initialize the reference trajectory plot
ref_plot = plot3(x_ref, y_ref, -z_ref, 'm', 'LineWidth', 0.25);
hold on;
grid on;
xlabel('x [m]', 'interpreter', 'latex');
ylabel('y [m]', 'interpreter', 'latex');
zlabel('z [m]', 'interpreter', 'latex');
title('3D trajectory', " Mean Tracking Error = " + errorXYZ + "",'interpreter','latex')
legend('reference', 'output', 'quad', 'interpreter', 'latex');

%
curve = animatedline('Color', 'b','LineWidth', 1.5);

for i = 1:length(z)
    addpoints(curve,x(i),y(i),-z(i));
    head = scatter3(x(i),y(i),-z(i),'filled','MarkerFaceColor','k');
    legend('reference', 'output', 'quad', 'interpreter', 'latex');
    drawnow
    pause(1e-100);
    delete(head);
end
