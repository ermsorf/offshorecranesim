%% Animated 3D Rotation Plot Script (Without plotting L)
% Clear workspace, close figures, and clear command window
clear; close all; clc;

%% Define the vector and symbolic variables
L = [0; 0; -10];  % Fixed vector (used for the rotation, but not plotted)

% Define symbolic variables for the rotation angles
syms theta psi

%% Define the rotation matrices
% Rotation about Y-axis by theta
R2 = [ cos(theta)   0   sin(theta);
       0            1   0;
      -sin(theta)   0   cos(theta)];

% Rotation about Z-axis by psi
R3 = [ cos(psi)  -sin(psi)   0;
       sin(psi)   cos(psi)   0;
       0          0          1];

%% Compute the rotated vector r symbolically
r_sym = simplify(R3 * R2 * L);

% Display the symbolic expression for r
disp('The symbolic expression for r is:');
disp(r_sym);

%% Convert the symbolic expression into a numeric function handle
% r_numeric is a function of theta and psi that returns a 3x1 vector.
r_numeric = matlabFunction(r_sym, 'Vars', [theta, psi]);

%% Set up the animation parameters
t_end = 10;           % Total time for simulation in seconds
fps = 30;             % Frames per second for animation
t = linspace(0, t_end, t_end * fps);

% Define theta to oscillate (e.g., sinusoidal oscillation)
theta_vals = 0.5 * sin(2 * pi * t);  % Oscillation amplitude of 0.5 rad

% Define psi to increase linearly over time
psi_vals = t;  % psi increases linearly with time

%% Set up the figure for animation
figure;
axis equal;                % Keep aspect ratio equal for x, y, and z
axis([-10 10 -10 10 -10 0]);  % Set axis limits
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
hold on;
title('Animation of Rotated Vector r');
view(3);                   % Force a 3D view

% Create plot handles:
% h_rot: red marker at the tip of the rotated vector.
% h_line: line from the origin to the rotated vector.
h_rot = plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
h_line = plot3([0 0], [0 0], [0 0], 'r-', 'LineWidth', 2);

%% Animate the rotation
for i = 1:length(t)
    % Evaluate the rotated vector r at the current theta and psi values
    r_current = r_numeric(theta_vals(i), psi_vals(i));
    
    % Update the marker for the rotated vector
    set(h_rot, 'XData', r_current(1), ...
               'YData', r_current(2), ...
               'ZData', r_current(3));
    
    % Update the line connecting the origin to the rotated vector
    set(h_line, 'XData', [0, r_current(1)], ...
                'YData', [0, r_current(2)], ...
                'ZData', [0, r_current(3)]);
    
    drawnow;
    pause(1/fps);  % Pause to control the animation frame rate
end