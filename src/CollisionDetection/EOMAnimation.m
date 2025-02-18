%% Parameters and Time Setup
l = -1;                     % Length of the vector
t = linspace(0, 10, 200);    % Time vector (adjust span/frames as desired)

% Define Euler angle functions (customize as needed)
theta_fun = @(t) 1 %(pi/4) * t*0+ pi / 6 * t;   % θ oscillates between -pi/4 and pi/4
psi_fun   = @(t) t %(pi/12) * cos(t)*0 ;                 % ψ increases linearly
phi_fun   = @(t) 1 %t*0;     % φ oscillates between -pi/3 and pi/3

% Length for the moving coordinate frame axes (a fraction of l)
frameLength = 0.3 * l;

%% Figure Setup
figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Animated Vector with Moving Coordinate Frame');
view(3);  % 3D view

%% Initialize Plot Elements

% Plot the animated vector (line from origin to moving tip)
h_line = plot3([0 0], [0 0], [0 0], 'k', 'LineWidth', 2);
% Marker at the tip of the vector
h_point = plot3(0, 0, 0, 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
% Optional: Trace the tip’s path
h_trail = animatedline('LineStyle', '-', 'Color', 'm', 'Marker', 'none');

% (Pre)compute initial angles at t(1)
theta = theta_fun(t(1));
psi   = psi_fun(t(1));
phi   = phi_fun(t(1));

% Compute the vector’s tip (using your given equations)
x = l * cos(psi) * sin(phi);
y = -l * ( cos(theta)*sin(psi) + cos(phi)*cos(psi)*sin(theta) );
z = l * ( cos(phi)*cos(psi)*cos(theta) - sin(psi)*sin(theta) );

% Compute the rotation matrix for the moving frame.
% Here we use a Z-Y-X rotation sequence:
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
R = Rz * Ry * Rx;   % Overall rotation matrix

% Create the moving frame axes at the tip.
% The columns of R are the directions of the moving x', y', and z' axes.
h_moving_x = quiver3(x, y, z, frameLength*R(1,1), frameLength*R(2,1), frameLength*R(3,1), ...
    'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
h_moving_y = quiver3(x, y, z, frameLength*R(1,2), frameLength*R(2,2), frameLength*R(3,2), ...
    'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
h_moving_z = quiver3(x, y, z, frameLength*R(1,3), frameLength*R(2,3), frameLength*R(3,3), ...
    'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');

%% Animation Loop
for k = 1:length(t)
    % Compute the current Euler angles
    theta = theta_fun(t(k));
    psi   = psi_fun(t(k));
    phi   = phi_fun(t(k));
    
    % Compute the vector’s tip position using your equations
    x = l * cos(psi) * sin(phi);
    y = -l * ( cos(theta)*sin(psi) + cos(phi)*cos(psi)*sin(theta) );
    z = l * ( cos(phi)*cos(psi)*cos(theta) - sin(psi)*sin(theta) );
    
    % Update the animated vector (line and tip marker)
    set(h_line, 'XData', [0, x], 'YData', [0, y], 'ZData', [0, z]);
    set(h_point, 'XData', x, 'YData', y, 'ZData', z);
    addpoints(h_trail, x, y, z);
    
    % Compute the rotation matrix for the moving frame at this time step.
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R = Rz * Ry * Rx;
    
    % Update the moving coordinate frame axes (attached at the tip)
    % Update x'-axis
    set(h_moving_x, 'XData', x, 'YData', y, 'ZData', z, ...
        'UData', frameLength * R(1,1), 'VData', frameLength * R(2,1), 'WData', frameLength * R(3,1));
    % Update y'-axis
    set(h_moving_y, 'XData', x, 'YData', y, 'ZData', z, ...
        'UData', frameLength * R(1,2), 'VData', frameLength * R(2,2), 'WData', frameLength * R(3,2));
    % Update z'-axis
    set(h_moving_z, 'XData', x, 'YData', y, 'ZData', z, ...
        'UData', frameLength * R(1,3), 'VData', frameLength * R(2,3), 'WData', frameLength * R(3,3));
    
    % Refresh the plot
    drawnow;
    pause(0.05);  % Adjust pause for desired animation speed
end
