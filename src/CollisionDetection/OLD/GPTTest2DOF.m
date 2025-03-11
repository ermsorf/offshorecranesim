%% 3D Pendulum Simulation with Energy, State Plots, and Multi-View Animation
% This script simulates a 3D (spherical) pendulum with two degrees of freedom,
% including damping. The numerical solution is obtained using a constant-step
% fourth-order Runge–Kutta (RK4) integrator.
%
% It then:
%   - Computes and plots the kinetic energy, potential energy, and total energy.
%   - Computes and plots the height (h = L*(1-cosθ)), the angles (θ and φ), and the bob’s speed.
%   - Animates the 3D motion in one window with four subplots: Front, Right, Top, and Isometric views.

clear; clc; close all;

%% Parameters
m = 4000;         % Mass (kg)
L = 30;           % Rod length (m)
g = 9.81;         % Gravitational acceleration (m/s^2)
c_damp = 0;      % Damping coefficient
k = c_damp/(m*L^2);  % Damping constant

t0 = 0; Tf = 25; dt = 0.01;         % Simulation time and constant time step
tspan = t0:dt:Tf;                  % Time vector

% Initial conditions: [θ; φ; θ_dot; φ_dot]
% Here, θ and φ are in radians.
X0 = [0.5; 0.3; -0.1; 1];

%% Define the ODE Function
% For a spherical pendulum:
%   dθ/dt = θ_dot
%   dφ/dt = φ_dot
%   dθ_dot/dt = sinθ·cosθ·(φ_dot)² - (g/L)*sinθ - k·θ_dot
%   dφ_dot/dt = -2·cotθ·θ_dot·φ_dot - k·φ_dot
%
% To avoid singularities when sinθ is very small, we enforce a minimum value.
minSin = 1e-3;  % Minimum value for sinθ

ODEfun = @(t, X) [...
    X(3);...
    X(4);...
    sin(X(1))*cos(X(1))*(X(4))^2 - (g/L)*sin(X(1)) - k*X(3);...
    -2*(cos(X(1))/max(sin(X(1)), minSin))*X(3)*X(4) - k*X(4)...
    ];

%% Constant-Step RK4 Integrator
N = length(tspan);
X_sol = zeros(N, 4);
X_sol(1,:) = X0';
for i = 1:N-1
    t = tspan(i);
    X = X_sol(i,:)';
    k1 = ODEfun(t, X);
    k2 = ODEfun(t + dt/2, X + dt/2 * k1);
    k3 = ODEfun(t + dt/2, X + dt/2 * k2);
    k4 = ODEfun(t + dt,   X + dt * k3);
    X_next = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    X_sol(i+1,:) = X_next';
end
t_sol = tspan';

%% Extract the Numerical Solution Variables
theta_sol = X_sol(:,1);        % θ (radians)
phi_sol   = X_sol(:,2);          % φ (radians)
theta_dot_sol = X_sol(:,3);      % θ̇
phi_dot_sol   = X_sol(:,4);      % φ̇

%% Compute Cartesian Coordinates of the Bob
% Using the spherical-to-Cartesian transformation:
%   x = L*sinθ*cosφ, y = L*sinθ*sinφ, z = -L*cosθ
x = L * sin(theta_sol) .* cos(phi_sol);
y = L * sin(theta_sol) .* sin(phi_sol);
z = -L * cos(theta_sol);

%% Compute Energy from the Numerical Solution
% Kinetic energy:
K = 0.5 * m * L^2 * (theta_dot_sol.^2 + (sin(theta_sol).^2).*phi_dot_sol.^2);
% Potential energy (with U = m*g*L*(1-cosθ); U=0 when θ=0, i.e. at the bottom):
U = m * g * L * (1 - cos(theta_sol));
% Total energy:
E_total = K + U;

%% Compute Height and Velocity
% Height above the equilibrium (lowest point) is defined as:
%   h = L - L*cosθ = L*(1-cosθ)
height = L * (1 - cos(theta_sol));
% The bob's speed is given by:
velocity = L * sqrt(theta_dot_sol.^2 + (sin(theta_sol).^2).*phi_dot_sol.^2);

%% Figure 1: 3D Position Animation with Four Views
% The animation shows the bob's trajectory in four subplots:
% Front view: view([0,0])
% Right view: view([90,0]) (side view from the right)
% Top view: view([0,90])
% Isometric view: view([45,35])
figure('Name','3D Pendulum Animation');
for i = 1:10:N  % update every 10th frame to speed up the animation
    clf;
    % Front View
    subplot(2,2,1);
    hold on;
    plot3([0, x(i)], [0, y(i)], [0, z(i)], 'r-', 'LineWidth',2);
    plot3(x(i), y(i), z(i), 'bo','MarkerSize',8,'MarkerFaceColor','b');
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim([-L L]); ylim([-L L]); zlim([-L L]);
    view([0,0]);
    title('Front View');
    
    % Right View
    subplot(2,2,2);
    hold on;
    plot3([0, x(i)], [0, y(i)], [0, z(i)], 'r-', 'LineWidth',2);
    plot3(x(i), y(i), z(i), 'bo','MarkerSize',8,'MarkerFaceColor','b');
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim([-L L]); ylim([-L L]); zlim([-L L]);
    view([90,0]);
    title('Right View');
    
    % Top View
    subplot(2,2,3);
    hold on;
    plot3([0, x(i)], [0, y(i)], [0, z(i)], 'r-', 'LineWidth',2);
    plot3(x(i), y(i), z(i), 'bo','MarkerSize',8,'MarkerFaceColor','b');
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim([-L L]); ylim([-L L]); zlim([-L L]);
    view([0,90]);
    title('Top View');
    
    % Isometric View
    subplot(2,2,4);
    hold on;
    plot3([0, x(i)], [0, y(i)], [0, z(i)], 'r-', 'LineWidth',2);
    plot3(x(i), y(i), z(i), 'bo','MarkerSize',8,'MarkerFaceColor','b');
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim([-L L]); ylim([-L L]); zlim([-L L]);
    view([45,35]);
    title('Isometric View');
    
    drawnow;
end

%% Figure 2: Energy Plot
figure('Name','Energy Plot');
plot(t_sol, K, 'r-', 'LineWidth', 1.5); hold on;
plot(t_sol, U, 'b-', 'LineWidth', 1.5);
plot(t_sol, E_total, 'k--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Energy (J)');
legend('Kinetic Energy', 'Potential Energy', 'Total Energy');
title('Energy vs. Time');
grid on;

%% Figure 3: Height, Angles, and Velocity
figure('Name','State Plots');
subplot(2,2,1);
plot(t_sol, height, 'm-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Height (m)');
title('Height vs. Time');
grid on;

subplot(2,2,2);
plot(t_sol, theta_sol, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\theta (rad)');
title('\theta vs. Time');
grid on;

subplot(2,2,3);
plot(t_sol, phi_sol, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\phi (rad)');
title('\phi vs. Time');
grid on;

subplot(2,2,4);
plot(t_sol, velocity, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity vs. Time');
grid on;
