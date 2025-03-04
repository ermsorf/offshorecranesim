%% fullPendulumTrajectory.m
% This script:
%   1. Solves the spherical pendulum equations of motion.
%   2. Returns the pendulum's trajectory as a function handle.
%   3. Plots the 3D trajectory.
%   4. Animates the pendulum motion in 3D.
%   5. Prints the trajectory in the terminal in the form:
%         f(theta, psi) = [x; y; z]
%
% The spherical pendulum is described by:
%   dtheta/dt         = theta_dot
%   dtheta_dot/dt     = sin(theta)*cos(theta)*psi_dot^2 - (g/L)*sin(theta)
%   dpsi/dt           = psi_dot
%   dpsi_dot/dt       = -2*(cos(theta)/sin(theta))*theta_dot*psi_dot
%
% Cartesian coordinates (with the pivot at the origin):
%   x = L*sin(theta)*cos(psi)
%   y = L*sin(theta)*sin(psi)
%   z = -L*cos(theta)
%
% Adjust parameters and initial conditions as needed.

clear; clc; close all;

%% Parameters
g = 9.81;        % gravitational acceleration [m/s^2]
L = 1.0;         % pendulum length [m]
% Initial conditions: [theta; theta_dot; psi; psi_dot]
Y0 = [pi/4; 0; 0; 2]; 
tspan = [0 20];  % simulation time in seconds

%% 1. Solve the ODE and get a trajectory function handle along with the ODE solution
% Here we solve the ODEs to get the full state history.
[t, Y] = ode45(@(t, Y) pendulumODE(t, Y, g, L), tspan, Y0);
thetaSol = Y(:,1);  % polar angle from the solution
psiSol   = Y(:,3);  % azimuthal angle from the solution

% Convert to Cartesian coordinates from the ODE solution
xSol = L .* sin(thetaSol) .* cos(psiSol);
ySol = L .* sin(thetaSol) .* sin(psiSol);
zSol = -L .* cos(thetaSol);

% Also create an interpolation function (if you want to query later by time)
traj = getPendulumTrajectory(g, L, Y0, tspan);

%% 2. Plot the 3D trajectory
figure('Name','3D Pendulum Trajectory');
plot3(xSol, ySol, zSol, 'LineWidth', 2);
grid on;
axis equal;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('3D Trajectory of the Spherical Pendulum');
view(3);

%% 3. Animate the pendulum motion in 3D
figure('Name','Spherical Pendulum Animation');
hold on; grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Spherical Pendulum Animation');
view(3);
% Set axis limits for clarity
axis([-L L -L L -L L]);

% Plot the pivot point (origin)
plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% Create graphic objects for the rod and bob
rodHandle = plot3([0, xSol(1)], [0, ySol(1)], [0, zSol(1)], 'b-', 'LineWidth', 2);
bobHandle = plot3(xSol(1), ySol(1), zSol(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Animation loop: update the rod and bob positions over time.
for k = 1:length(t)
    % Update rod (line from pivot to bob)
    set(rodHandle, 'XData', [0, xSol(k)], 'YData', [0, ySol(k)], 'ZData', [0, zSol(k)]);
    % Update bob (marker at the current position)
    set(bobHandle, 'XData', xSol(k), 'YData', ySol(k), 'ZData', zSol(k));
    drawnow;
    pause(0.01);  % adjust pause for desired animation speed
end

%% 4. Print the trajectory in the terminal as f(theta, psi)
% We will sample the solution (e.g., every 10th point) for a concise output.
fprintf('\nTrajectory in the form f(theta, psi) = [ x; y; z ]:\n');
fprintf('   theta      psi         x         y         z\n');
fprintf('---------------------------------------------------\n');
step = 10;  % change this value to control the sampling frequency
for i = 1:step:length(thetaSol)
    fprintf(' %8.3f  %8.3f   %8.3f   %8.3f   %8.3f\n', thetaSol(i), psiSol(i), xSol(i), ySol(i), zSol(i));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Function: getPendulumTrajectory
function trajectory = getPendulumTrajectory(g, L, Y0, tspan)
    % getPendulumTrajectory Solves the spherical pendulum ODEs and returns
    % a function handle that interpolates the Cartesian trajectory.
    %
    %   Inputs:
    %     g     - gravitational acceleration
    %     L     - pendulum length
    %     Y0    - initial conditions [theta; theta_dot; psi; psi_dot]
    %     tspan - time span [t_start t_end]
    %
    %   Output:
    %     trajectory - a function handle such that:
    %                  pos = trajectory(tq) returns [x; y; z] at time tq.
    
    % Solve the ODE system using ode45
    [t, Y] = ode45(@(t, Y) pendulumODE(t, Y, g, L), tspan, Y0);
    
    % Extract generalized coordinates from the solution
    theta = Y(:,1);  % polar angle
    psi   = Y(:,3);  % azimuthal angle
    
    % Convert to Cartesian coordinates
    x = L * sin(theta) .* cos(psi);
    y = L * sin(theta) .* sin(psi);
    z = -L * cos(theta);
    
    % Create interpolation functions for x, y, and z using a spline
    xInterp = @(tq) interp1(t, x, tq, 'spline');
    yInterp = @(tq) interp1(t, y, tq, 'spline');
    zInterp = @(tq) interp1(t, z, tq, 'spline');
    
    % Return a function handle that evaluates [x; y; z] at given time(s) tq.
    trajectory = @(tq) [xInterp(tq); yInterp(tq); zInterp(tq)];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Function: pendulumODE
function dYdt = pendulumODE(~, Y, g, L)
    % pendulumODE Defines the ODEs for a spherical pendulum.
    %
    %   Y(1) = theta,   Y(2) = theta_dot,
    %   Y(3) = psi,     Y(4) = psi_dot.
    
    % Unpack the state vector
    theta     = Y(1);
    theta_dot = Y(2);
    psi       = Y(3);
    psi_dot   = Y(4);
    
    % Equations of motion
    dtheta_dt     = theta_dot;
    dtheta_dot_dt = sin(theta)*cos(theta)*psi_dot^2 - (g/L)*sin(theta);
    dpsi_dt       = psi_dot;
    dpsi_dot_dt   = -2*(cos(theta)/sin(theta))*theta_dot*psi_dot;
    
    dYdt = [dtheta_dt; dtheta_dot_dt; dpsi_dt; dpsi_dot_dt];
end
