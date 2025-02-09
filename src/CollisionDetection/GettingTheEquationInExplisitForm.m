%% completePendulumExplicitEquation.m
% This script:
%   1. Solves the spherical pendulum ODE for t = 0 to 5 seconds,
%      using a fixed time step.
%   2. Computes the Cartesian coordinates:
%         x(t) = L*sin(theta(t))*cos(psi(t))
%         y(t) = L*sin(theta(t))*sin(psi(t))
%         z(t) = -L*cos(theta(t))
%   3. Fits a Fourier series (using a 'fourier2' model) to each coordinate,
%      yielding an explicit equation for x(t), y(t), and z(t).
%   4. Prints the explicit equations so they can be used in another program.
%   5. Plots the data and the fitted curves (manual evaluation) in 2D.
%   6. Adds a 3D plot of the Fourier series approximated trajectory.
%
% Note: This script uses the 'fit' function from MATLAB's Curve Fitting Toolbox.
%
% Author: [Your Name]
% Date: [Today's Date]

clear; clc; close all;

%% 1. Parameters and ODE Solution Setup
g = 9.81;              % gravitational acceleration [m/s^2]
L = 1.0;               % pendulum length [m]
% Initial conditions: [theta; theta_dot; psi; psi_dot]
% Example: initial polar angle = 30° and an initial azimuthal speed.
Y0 = [pi/6; 0; 0; 2];

% Define the time span with a fixed time step.
dt = 0.02;             % output every 0.02 seconds
tspan = 0:dt:5;        % from 0 to 5 seconds

% Solve the ODE using ode45.
[t, Y] = ode45(@(t,Y) pendulumODE(t, Y, g, L), tspan, Y0);

% Extract the generalized coordinates.
theta = Y(:,1);  % polar angle (column vector)
psi   = Y(:,3);  % azimuthal angle (column vector)

%% 2. Compute Cartesian Coordinates
x = L .* sin(theta) .* cos(psi);
y = L .* sin(theta) .* sin(psi);
z = -L .* cos(theta);

%% 3. Fit Fourier Series to Each Coordinate
% We use the 'fourier2' model, which fits a model of the form:
%   f(t) = a0 + a1*sin(b*t) + b1*cos(b*t) + a2*sin(2*b*t) + b2*cos(2*b*t)
% Make sure the input vectors are column vectors.
ftype = 'fourier2';
fitX = fit(t(:), x(:), ftype);
fitY = fit(t(:), y(:), ftype);
fitZ = fit(t(:), z(:), ftype);

%% 4. Display the Explicit Equations
% Use the 'formula' function to extract the model formulas as strings.
x_eq = formula(fitX);
y_eq = formula(fitY);
z_eq = formula(fitZ);

fprintf('\nExplicit Fourier Series Equations for the Pendulum Coordinates (t in seconds):\n\n');
fprintf('x(t) = %s\n\n', x_eq);
fprintf('y(t) = %s\n\n', y_eq);
fprintf('z(t) = %s\n\n', z_eq);

%% 5. Plot Data and Fitted Curves (Manual Evaluation)
% Create a time vector for plotting the fit.
t_fit_plot = linspace(min(t), max(t), 200);
x_fit_plot = fitX(t_fit_plot);
y_fit_plot = fitY(t_fit_plot);
z_fit_plot = fitZ(t_fit_plot);

figure;
subplot(3,1,1);
plot(t, x, 'bo'); hold on;
plot(t_fit_plot, x_fit_plot, 'r-', 'LineWidth', 1.5);
title('x-coordinate and Fourier Fit');
xlabel('Time (s)'); ylabel('x (m)');

subplot(3,1,2);
plot(t, y, 'bo'); hold on;
plot(t_fit_plot, y_fit_plot, 'r-', 'LineWidth', 1.5);
title('y-coordinate and Fourier Fit');
xlabel('Time (s)'); ylabel('y (m)');

subplot(3,1,3);
plot(t, z, 'bo'); hold on;
plot(t_fit_plot, z_fit_plot, 'r-', 'LineWidth', 1.5);
title('z-coordinate and Fourier Fit');
xlabel('Time (s)'); ylabel('z (m)');
sgtitle('Pendulum Coordinates and Their Fourier Approximations');

%% 6. 3D Plot of the Fourier Series Approximated Trajectory
figure;
plot3(x_fit_plot, y_fit_plot, z_fit_plot, 'r-', 'LineWidth', 2); 
hold on;
plot3(x, y, z, 'bo'); % Original data points for reference
grid on;
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title('3D Plot of Fourier Series Approximations for Pendulum Trajectory');
view(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Nested Function: pendulumODE
function dYdt = pendulumODE(~, Y, g, L)
    % pendulumODE Defines the ODEs for a spherical pendulum.
    %   Y(1) = theta, Y(2) = theta_dot, Y(3) = psi, Y(4) = psi_dot.
    
    % Unpack the state vector.
    theta     = Y(1);
    theta_dot = Y(2);
    psi       = Y(3);
    psi_dot   = Y(4);
    
    % Equations of motion for the spherical pendulum.
    dtheta_dt     = theta_dot;
    dtheta_dot_dt = sin(theta)*cos(theta)*psi_dot^2 - (g/L)*sin(theta);
    dpsi_dt       = psi_dot;
    dpsi_dot_dt   = -2*(cos(theta)/sin(theta))*theta_dot*psi_dot;
    
    dYdt = [dtheta_dt; dtheta_dot_dt; dpsi_dt; dpsi_dot_dt];
end
