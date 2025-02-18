%% Two-DOF Spherical Pendulum Simulation using Euler-Lagrange Equations (3D Plot)
% This script:
%   1. Solves the nonlinear 2-DOF spherical pendulum ODE using Euler-Lagrange.
%   2. Computes Cartesian coordinates of the pendulum bob.
%   3. Plots the generalized coordinates in a 3D plot, as well as the kinetic,
%      potential, and total energy.
%   4. Animates the 3D trajectory of the pendulum bob.

clear; clc; close all;

%% PARAMETERS
m = 4000;       % Mass [kg]
L = 30;         % Rod length [m]
g = -9.81;       % Gravitational acceleration [m/s^2]

% Time span & initial conditions
dt = 0.1; T = 25;
tspan = 0:dt:T;
% Initial state: [theta; phi; theta_dot; phi_dot]
% (Use a small initial theta and nonzero phi_dot to see 3D behavior.)
X0 = [1; 1; 0.5; 0.5];

%% SYMBOLIC DERIVATION OF EQUATIONS OF MOTION
syms theta(t) phi(t)
theta_dot_t = diff(theta, t);
phi_dot_t   = diff(phi, t);

% Kinetic Energy: T = 1/2 m L^2 (theta_dot^2 + sin(theta)^2*phi_dot^2)
T_sym = 1/2 * m * L^2 * (theta_dot_t^2 + sin(theta)^2 * phi_dot_t^2);
% Potential Energy: U = m*g*L*(1 - cos(theta))
U_sym = m*g*L*(1 - cos(theta));
% Lagrangian
Lagrangian = T_sym - U_sym;

% Euler-Lagrange Equation for theta:
EL_theta = simplify(diff(diff(Lagrangian, theta_dot_t), t) - diff(Lagrangian, theta));
% Euler-Lagrange Equation for phi:
EL_phi   = simplify(diff(diff(Lagrangian, phi_dot_t), t) - diff(Lagrangian, phi));

% Replace time derivatives with symbolic placeholders:
syms theta_val phi_val theta_dot_sym phi_dot_sym theta_ddot phi_ddot real

EL_theta = subs(EL_theta, [diff(theta, t, 2), diff(theta, t), diff(phi, t)], [theta_ddot, theta_dot_sym, phi_dot_sym]);
EL_theta = subs(EL_theta, [theta, phi], [theta_val, phi_val]);

EL_phi = subs(EL_phi, [diff(phi, t, 2), diff(theta, t), diff(phi, t)], [phi_ddot, theta_dot_sym, phi_dot_sym]);
EL_phi = subs(EL_phi, [theta, phi], [theta_val, phi_val]);

% Arrange the two second-order equations into matrix form:
vars = [theta_ddot, phi_ddot];
[Mat, F_sym] = equationsToMatrix([EL_theta, EL_phi], vars);
accel_sym = -simplify(Mat\F_sym);

% Convert to a function handle for numerical evaluation:
% f_accel returns [theta_ddot; phi_ddot] given [theta, phi, theta_dot, phi_dot].
f_accel = matlabFunction(accel_sym, 'Vars', {theta_val, phi_val, theta_dot_sym, phi_dot_sym});

%% SOLVE THE ODE SYSTEM
% Our state X = [theta; phi; theta_dot; phi_dot]
ODEfun = @(t, X) [ X(3);
                   X(4);
                   f_accel(X(1), X(2), X(3), X(4)) ];
options = odeset('RelTol',1e-4, 'AbsTol',1e-4);
[t_sol, X_sol] = ode15s(ODEfun, tspan, X0, options);

% Extract solution components:
theta_sol    = X_sol(:,1);
phi_sol      = X_sol(:,2);
theta_dot_sol = X_sol(:,3);
phi_dot_sol   = X_sol(:,4);

%% COMPUTE CARTESIAN COORDINATES
% Spherical-to-Cartesian transformation:
% x = L*sin(theta)*cos(phi), y = L*sin(theta)*sin(phi), z = -L*cos(theta)
x = L * sin(theta_sol) .* cos(phi_sol);
y = L * sin(theta_sol) .* sin(phi_sol);
z = -L * cos(theta_sol);

%% ENERGY COMPUTATION
% Kinetic Energy: T = 1/2 m L^2 (theta_dot^2 + sin(theta)^2*phi_dot^2)
T_num = abs(0.5 * m * L^2 * (theta_dot_sol.^2 + (sin(theta_sol).^2) .* phi_dot_sol.^2));
% Potential Energy: U = m*g*L*(1 - cos(theta))
U_num = abs(m * g * L * (1 - cos(theta_sol)));
% Total Energy:
E_total = T_num + U_num;

%% PLOTTING GENERALIZED COORDINATES (3D Plot)
% Plot theta and phi versus time in a single 3D plot.
figure;
plot3(t_sol, theta_sol, phi_sol, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('\theta (rad)');
zlabel('\phi (rad)');
title('Evolution of Generalized Coordinates (3D Plot)');
grid on;

%% PLOTTING ENERGY OF THE SYSTEM
figure;
plot(t_sol, T_num, 'r-', 'LineWidth',1.5); hold on;
plot(t_sol, U_num, 'b-', 'LineWidth',1.5);
plot(t_sol, E_total, 'k--', 'LineWidth',2);
xlabel('Time (s)'); ylabel('Energy (J)');
legend('Kinetic Energy', 'Potential Energy', 'Total Energy');
title('Energy of the Spherical Pendulum');
grid on;

%% ANIMATION: 3D TRAJECTORY OF THE PENDULUM BOB
figure;
for k = 1:length(t_sol)
    clf; hold on;
    
    % Plot the rod from the pivot (0,0,0) to the bob (x,y,z)
    plot3([0, x(k)], [0, y(k)], [0, z(k)], 'r-', 'LineWidth', 2);
    % Plot the pendulum bob
    plot3(x(k), y(k), z(k), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    % Set axis limits and labels

    axis equal;
    xlim([-L L]);
    ylim([-L L]);
    zlim([-L L]);
    view([45,35]);             % Set the view to an isometric-like angle
    camproj('orthographic');    % Use orthographic projection for isometry
    xlabel('x'); ylabel('y'); zlabel('z');
    title(sprintf('Time = %.2f s', t_sol(k)));
    grid on;
    drawnow;
end
