% Spherical Pendulum Parameters
L = 8;                    % Length of the pendulum
theta_max = pi/6;         % Maximum swing angle for theta (30 degrees)
phi_max = pi/6;           % Maximum swing angle for phi (30 degrees)

% Angular frequencies for the two motions
omega_theta = 2;          % Frequency for theta oscillation
omega_phi = 1;            % Frequency for phi oscillation

% Animation parameters
t_final = 1000;             % Total duration of the animation (in seconds)
nFrames = t_final * 30;            % Number of frames in the animation
t_values = linspace(0, t_final, nFrames);

% Create a new figure and set up the axes for 3D plotting
figure;
hold on;
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Animated 3D Spherical Pendulum with Line to Origin');
axis equal;
view(3);

% Fix the axis limits to a 10x10x10 cube
axis([-10 10 -10 10 -10 0]);
axis manual;  % Prevent MATLAB from auto-scaling the axes

% (Optional) Initialize an empty plot for the trail of the pendulum bob
hTrail = plot3(nan, nan, nan, 'b-', 'LineWidth', 1.5);

% Compute the initial angles and bob position at t = t_values(1)
initial_t = t_values(1);
theta0 = theta_max * cos(omega_theta * initial_t);
phi0   = phi_max * sin(omega_phi * initial_t);
initial_x = L * sin(theta0) * cos(phi0);
initial_y = L * sin(theta0) * sin(phi0);
initial_z = -L * cos(theta0);

% Plot the pendulum bob (marker)
hMarker = plot3(initial_x, initial_y, initial_z, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Plot the line connecting the bob to the origin
hLine = plot3([0 initial_x], [0 initial_y], [0 initial_z], 'k-', 'LineWidth', 2);

% Arrays to store the trail coordinates (optional)
x_trail = [];
y_trail = [];
z_trail = [];

% Animate the pendulum motion with two degrees of freedom
for k = 1:length(t_values)
    current_t = t_values(k);
    
    % Compute the time-varying angles for the spherical pendulum
    theta = theta_max * cos(omega_theta * current_t); % Polar angle (from vertical)
    phi   = phi_max * sin(omega_phi * current_t);       % Azimuthal angle
    
    % Compute the bob's position in Cartesian coordinates
    current_x = L * sin(theta) * cos(phi);
    current_y = L * sin(theta) * sin(phi);
    current_z = -L * cos(theta);
    
    % Before updating, check if the objects are still valid
    if isvalid(hMarker)
        set(hMarker, 'XData', current_x, 'YData', current_y, 'ZData', current_z);
    end
    
    if isvalid(hLine)
        set(hLine, 'XData', [0 current_x], 'YData', [0 current_y], 'ZData', [0 current_z]);
    end
    
    % Update the trail (optional)
    x_trail(end+1) = current_x;
    y_trail(end+1) = current_y;
    z_trail(end+1) = current_z;
    if isvalid(hTrail)
        set(hTrail, 'XData', x_trail, 'YData', y_trail, 'ZData', z_trail);
    end
    
    drawnow;         % Update the figure window
    pause(0.05);     % Pause to control the animation speed
end

hold off;
