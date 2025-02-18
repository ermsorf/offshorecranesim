%% One-DOF Pendulum Simulation using Euler-Lagrange Equations
% This script:
%   1. Solves the nonlinear 1-DOF pendulum ODE using the Euler-Lagrange approach.
%   2. Computes Cartesian coordinates of the pendulum bob.
%   3. Plots the pendulum angle and the kinetic, potential, and total energy over time.
%   4. Animates the pendulum motion.

clear; clc; close all;

%% PARAMETERS
m = 4000;      % Mass [kg]
L = 30;        % Rod length [m]
g = 9.81;      % Gravitational acceleration [m/s^2]

% Time span & initial conditions
dt = 0.1; T = 25;
tspan = 0:dt:T;
% Initial state: [theta; theta_dot]
X0 = [0.1; 0.0];   % small initial angular displacement, zero initial angular velocity

%% SYMBOLIC DERIVATION OF THE EQUATION OF MOTION
syms theta(t)
thetadot = diff(theta, t);

% Kinetic Energy: T = 1/2 m L^2 (theta_dot)^2
T_sym = 1/2 * m * L^2 * thetadot^2;

% Potential Energy: U = m g L (1 - cos(theta))
U_sym = m * g * L * (1 - cos(theta));

% Lagrangian: L = T - U
Lagrangian = T_sym - U_sym;

% Euler-Lagrange Equation: d/dt (∂L/∂thetadot) - ∂L/∂theta = 0
EL_eq = simplify(diff(diff(Lagrangian, thetadot), t) - diff(Lagrangian, theta));
% Expected: m*L^2*theta_ddot + m*g*L*sin(theta) = 0

% Replace derivatives with symbolic variables for numerical integration
syms theta_val thetadot_sym theta_ddot real
EL_eq = subs(EL_eq, [diff(theta, t, 2), diff(theta, t)], [theta_ddot, thetadot_sym]);
EL_eq = subs(EL_eq, theta, theta_val);

% Solve for the second derivative: theta_ddot = - (g/L)*sin(theta)
theta_ddot_expr = solve(EL_eq, theta_ddot);
theta_ddot_expr = simplify(theta_ddot_expr);

% Convert to function handle for numerical evaluation:
f_accel = matlabFunction(theta_ddot_expr, 'Vars', {theta_val, thetadot_sym});

%% DEFINE THE ODE SYSTEM
% State vector X = [theta; theta_dot]
ODEfun = @(t, X) [ X(2); f_accel(X(1), X(2)) ];
options = odeset('RelTol',1e-8, 'AbsTol',1e-10);
[t_sol, X_sol] = ode15s(ODEfun, tspan, X0, options);

% Extract solution components:
theta_sol   = X_sol(:,1);
thetadot_sol = X_sol(:,2);

%% COMPUTE CARTESIAN COORDINATES
% (Assuming theta=0 is the vertical downward direction)
x = L * sin(theta_sol);
y = -L * cos(theta_sol);

%% ENERGY COMPUTATION
% Kinetic Energy: T = 1/2 m L^2 (theta_dot)^2
T_num = 0.5 * m * L^2 * thetadot_sol.^2;
% Potential Energy: U = m g L (1 - cos(theta))
U_num = m * g * L * (1 - cos(theta_sol));
% Total Energy:
E_total = T_num + U_num;

%% PLOTTING THE PENDULUM ANGLE AND ENERGY
figure;
subplot(2,1,1);
plot(t_sol, theta_sol, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\theta (rad)');
title('Pendulum Angle Over Time');

subplot(2,1,2);
plot(t_sol, T_num, 'r-', 'LineWidth', 1.5); hold on;
plot(t_sol, U_num, 'b-', 'LineWidth', 1.5);
plot(t_sol, E_total, 'k--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Energy (J)');
legend('Kinetic Energy','Potential Energy','Total Energy');
title('Energy of the System Over Time');
grid on;

%% ANIMATION: PENDULUM MOTION
figure;
for k = 1:length(t_sol)
    clf; hold on;
    % Plot the rod from the pivot (0,0) to the bob (x,y)
    plot([0, x(k)], [0, y(k)], 'r-', 'LineWidth', 2);
    % Plot the pendulum bob
    plot(x(k), y(k), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    axis equal;
    xlim([-L L]);
    ylim([-L L]);
    xlabel('x'); ylabel('y');
    title(sprintf('Time = %.2f s', t_sol(k)));
    grid on;
    drawnow;
end
