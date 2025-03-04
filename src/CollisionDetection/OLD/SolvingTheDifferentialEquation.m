%% 3D_Motion.m
% This script solves the equations of motion for a particle in 3D space.
% The state vector is:
%    Y = [x; y; z; vx; vy; vz]
%
% The system of ODEs is:
%    dx/dt = vx
%    dy/dt = vy
%    dz/dt = vz
%    dvx/dt = a_x(x,y,z,vx,vy,vz,t)
%    dvy/dt = a_y(x,y,z,vx,vy,vz,t)
%    dvz/dt = a_z(x,y,z,vx,vy,vz,t)
%
% Example: Simple motion with gravity (only acceleration in z-direction)
%    a_x = 0, a_y = 0, a_z = -9.81

% Clear workspace, close figures, clear command window
clear; close all; clc;

%% Define the time span and initial conditions
tspan = [0 10];        % Time from 0 to 10 seconds

% Initial conditions:
% For example, starting at the origin with an initial velocity.
% [x0, y0, z0, vx0, vy0, vz0]
Y0 = [1; 2; 5; 5; 3; 10];

%% Solve the system using ode45
[t, sol] = ode45(@odefun, tspan, Y0);

% Extract the individual components
x  = sol(:,1);
y  = sol(:,2);
z  = sol(:,3);
vx = sol(:,4);
vy = sol(:,5);
vz = sol(:,6);

%% Plot the results

% 1. Plot the trajectory in 3D space
figure;
plot3(x, y, z, 'LineWidth', 2);
xlabel('x');
ylabel('y');
zlabel('z');
title('Trajectory of the Particle in 3D Space');
grid on;
axis equal;

% 2. Plot positions vs. time
figure;
subplot(3,1,1);
plot(t, x, 'r', 'LineWidth', 1.5);
ylabel('x (m)');
title('Position vs. Time');
grid on;

subplot(3,1,2);
plot(t, y, 'g', 'LineWidth', 1.5);
ylabel('y (m)');
grid on;

subplot(3,1,3);
plot(t, z, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('z (m)');
grid on;

% 3. Plot velocities vs. time
figure;
subplot(3,1,1);
plot(t, vx, 'r--', 'LineWidth', 1.5);
ylabel('v_x (m/s)');
title('Velocity vs. Time');
grid on;

subplot(3,1,2);
plot(t, vy, 'g--', 'LineWidth', 1.5);
ylabel('v_y (m/s)');
grid on;

subplot(3,1,3);
plot(t, vz, 'b--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('v_z (m/s)');
grid on;

%% ODE function definition
function dYdt = odefun(t, Y)
    % Unpack the state vector
    x  = Y(1);
    y  = Y(2);
    z  = Y(3);
    vx = Y(4);
    vy = Y(5);
    vz = Y(6);
    
    % Define the accelerations.
    % For example, assume only gravity acts on the particle:
    % (you can modify these to incorporate other forces or dependencies)
    ax = 0;
    ay = 0;
    az = -9.81;
    
    % Construct the derivative vector
    dYdt = zeros(6,1);
    dYdt(1) = vx;   % dx/dt = vx
    dYdt(2) = vy;   % dy/dt = vy
    dYdt(3) = vz;   % dz/dt = vz
    dYdt(4) = ax;   % dvx/dt = a_x
    dYdt(5) = ay;   % dvy/dt = a_y
    dYdt(6) = az;   % dvz/dt = a_z
end
