%% completePendulumExplicitEquation.m
% This script:
%   1. Solves the spherical pendulum ODE for t = 0 to 5 seconds with fixed time steps.
%   2. Computes the Cartesian coordinates:
%         x(t) = L*sin(theta(t))*cos(psi(t))
%         y(t) = L*sin(theta(t))*sin(psi(t))
%         z(t) = -L*cos(theta(t))
%   3. Fits a Fourier series (using the 'fourier2' model) to each coordinate,
%      yielding an explicit equation for x(t), y(t), and z(t).
%   4. Extracts the numerical coefficients from the fits and prints the explicit
%      numerical equations.
%   5. Plots the data and the fitted curves in 2D and also a 3D plot of the trajectory.
%
% Note: The 'fourier2' model fits a function of the form:
%       f(t) = a0 + a1*sin(w*t) + b1*cos(w*t) + a2*sin(2*w*t) + b2*cos(2*w*t)
%
% Author: [Your Name]
% Date: [Today's Date]

clear; clc; close all;

%% 1. Parameters and ODE Solution Setup
g = 9.81;              % gravitational acceleration [m/s^2]
L = 30.0;               % pendulum length [m]
% Initial conditions: [theta; theta_dot; psi; psi_dot]
% For example, an initial polar angle of 30° (pi/6 radians) and an initial azimuthal speed.
Y0 = [1; 0; 0; 0.5];

% Define the time span with a fixed time step.
dt = 0.1;             % output every 0.02 seconds
tspan = 0:dt:10;        % from 0 to 5 seconds

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
% We use the 'fourier2' model, which fits a function of the form:
%   f(t) = a0 + a1*sin(w*t) + b1*cos(w*t) + a2*sin(2*w*t) + b2*cos(2*w*t)
% Ensure that the input vectors are column vectors.
ftype = 'fourier2';
fitX = fit(t(:), x(:), ftype);
fitY = fit(t(:), y(:), ftype);
fitZ = fit(t(:), z(:), ftype);

%% 4. Display the Explicit Equations (Generic Form)
% The formula() function returns the equation as a string with symbolic coefficient names.
x_eq = formula(fitX);
y_eq = formula(fitY);
z_eq = formula(fitZ);

fprintf('\nExplicit Fourier Series Equations for the Pendulum Coordinates (Generic Form):\n\n');
fprintf('x(t) = %s\n\n', x_eq);
fprintf('y(t) = %s\n\n', y_eq);
fprintf('z(t) = %s\n\n', z_eq);

%% 4b. Extract and Display Explicit Numerical Coefficients
% For the Fourier model "fourier2", the coefficients are ordered as:
% [a0, a1, b1, a2, b2, w]
coeffsX = coeffvalues(fitX);
coeffsY = coeffvalues(fitY);
coeffsZ = coeffvalues(fitZ);

% Extract coefficients for x(t)
a0_x = coeffsX(1);
a1_x = coeffsX(2);
b1_x = coeffsX(3);
a2_x = coeffsX(4);
b2_x = coeffsX(5);
w_x  = coeffsX(6);

% Extract coefficients for y(t)
a0_y = coeffsY(1);
a1_y = coeffsY(2);
b1_y = coeffsY(3);
a2_y = coeffsY(4);
b2_y = coeffsY(5);
w_y  = coeffsY(6);

% Extract coefficients for z(t)
a0_z = coeffsZ(1);
a1_z = coeffsZ(2);
b1_z = coeffsZ(3);
a2_z = coeffsZ(4);
b2_z = coeffsZ(5);
w_z  = coeffsZ(6);

fprintf('\nExplicit Numerical Fourier Series Equations:\n\n');

fprintf('x(t) = %.6f + %.6f*sin(%.6f*t) + %.6f*cos(%.6f*t) + %.6f*sin(2*%.6f*t) + %.6f*cos(2*%.6f*t)\n\n', ...
    a0_x, a1_x, w_x, b1_x, w_x, a2_x, w_x, b2_x, w_x);

fprintf('y(t) = %.6f + %.6f*sin(%.6f*t) + %.6f*cos(%.6f*t) + %.6f*sin(2*%.6f*t) + %.6f*cos(2*%.6f*t)\n\n', ...
    a0_y, a1_y, w_y, b1_y, w_y, a2_y, w_y, b2_y, w_y);

fprintf('z(t) = %.6f + %.6f*sin(%.6f*t) + %.6f*cos(%.6f*t) + %.6f*sin(2*%.6f*t) + %.6f*cos(2*%.6f*t)\n\n', ...
    a0_z, a1_z, w_z, b1_z, w_z, a2_z, w_z, b2_z, w_z);

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
plot3(x, y, z, 'bo'); % Plot original data points for reference
grid on;
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title('3D Plot of Fourier Series Approximated Pendulum Trajectory');
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
