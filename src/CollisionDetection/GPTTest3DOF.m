%% Three-DOF Spherical Pendulum Simulation using Euler-Lagrange Equations (3D Plot)
% This script:
%   1. Solves the nonlinear 3-DOF pendulum ODE using Euler-Lagrange.
%   2. Computes Cartesian coordinates of the pendulum bob.
%   3. Plots the generalized coordinates and the kinetic, potential, and total energy.
%   4. Animates the 3D trajectory of the pendulum bob.

clear; clc; close all;

%% PARAMETERS
m = 4000;       % Mass [kg]
L = 30;         % Rod length [m]
g = -9.81;       % Gravitational acceleration [m/s^2]
I1 = m*L^2;     % Moment of inertia about axes perpendicular to rod (assumed)
I3 = m*L^2;     % Moment of inertia about rod axis (assumed)

% Time span & initial conditions
dt = 0.1; T = 5;
tspan = 0:dt:T;
% Initial state: [phi; theta; psi; phidot; thetadot; psidot]
X0 = [0; 0; 0; 1; 0; 3.14*2;]

%% SYMBOLIC DERIVATION OF EQUATIONS OF MOTION
syms phi(t) theta(t) psi(t)
phidot_t   = diff(phi, t);
thetadot_t = diff(theta, t);
psidot_t   = diff(psi, t);

% Kinetic Energy:
% T = 1/2*I1*(thetadot^2 + sin(theta)^2*phidot^2) + 1/2*I3*(psidot + phidot*cos(theta))^2
T_sym = (psidot_t + thetadot_t*cos(phi))*((2500*psidot_t)/3 + (2500*thetadot_t*cos(phi))/3) + (phidot_t*cos(psi) + thetadot_t*sin(phi)*sin(psi))*((6227129916893867*phidot_t*cos(psi))/34359738368 + (6227129916893867*thetadot_t*sin(phi)*sin(psi))/34359738368) + (phidot_t*sin(psi) - thetadot_t*cos(psi)*sin(phi))*((6256335694506667*phidot_t*sin(psi))/34359738368 - (6256335694506667*thetadot_t*cos(psi)*sin(phi))/34359738368)
;

% Potential Energy:
U_sym = 58860 - 58860*cos(phi)


% Lagrangian:
Lagrangian = T_sym - U_sym;

% Euler-Lagrange Equation for φ:
EL_phi   = simplify(diff(diff(Lagrangian, phidot_t), t) - diff(Lagrangian, phi));
% Euler-Lagrange Equation for θ:
EL_theta = simplify(diff(diff(Lagrangian, thetadot_t), t) - diff(Lagrangian, theta));
% Euler-Lagrange Equation for ψ:
EL_psi   = simplify(diff(diff(Lagrangian, psidot_t), t) - diff(Lagrangian, psi));

% Replace derivatives with symbolic placeholders for numerical integration:
syms phi_val theta_val psi_val phidot_sym thetadot_sym psidot_sym phi_ddot theta_ddot psidot_ddot real

EL_phi   = subs(EL_phi, [diff(phi,t,2), diff(phi,t), diff(theta,t), diff(psi,t)], [phi_ddot, phidot_sym, thetadot_sym, psidot_sym]);
EL_phi   = subs(EL_phi, [phi, theta, psi], [phi_val, theta_val, psi_val]);

EL_theta = subs(EL_theta, [diff(theta,t,2), diff(phi,t), diff(theta,t), diff(psi,t)], [theta_ddot, phidot_sym, thetadot_sym, psidot_sym]);
EL_theta = subs(EL_theta, [phi, theta, psi], [phi_val, theta_val, psi_val]);

EL_psi   = subs(EL_psi, [diff(psi,t,2), diff(phi,t), diff(theta,t), diff(psi,t)], [psidot_ddot, phidot_sym, thetadot_sym, psidot_sym]);
EL_psi   = subs(EL_psi, [phi, theta, psi], [phi_val, theta_val, psi_val]);

% Arrange the three second-order equations into matrix form:
vars = [phi_ddot, theta_ddot, psidot_ddot];
[Mat, F_sym] = equationsToMatrix([EL_phi, EL_theta, EL_psi], vars);
accel_sym = -simplify(Mat\F_sym);

% Convert to a function handle for numerical evaluation:
% f_accel returns [phi_ddot; theta_ddot; psidot_ddot] given
% [phi, theta, psi, phidot, thetadot, psidot].
f_accel = matlabFunction(accel_sym, 'Vars', {phi_val, theta_val, psi_val, phidot_sym, thetadot_sym, psidot_sym});

%% SOLVE THE ODE SYSTEM
% State vector X = [phi; theta; psi; phidot; thetadot; psidot]
ODEfun = @(t, X) [ X(4);
                   X(5);
                   X(6);
                   f_accel(X(1), X(2), X(3), X(4), X(5), X(6)) ];
options = odeset('RelTol',1e-8, 'AbsTol',1e-10);
[t_sol, X_sol] = ode15s(ODEfun, tspan, X0, options);

% Extract solution components:
phi_sol    = X_sol(:,1);
theta_sol  = X_sol(:,2);
psi_sol    = X_sol(:,3);
phidot_sol = X_sol(:,4);
thetadot_sol = X_sol(:,5);
psidot_sol   = X_sol(:,6);

%% COMPUTE CARTESIAN COORDINATES
% Spherical-to-Cartesian transformation (using φ and θ):
x = L * sin(theta_sol) .* cos(phi_sol);
y = L * sin(theta_sol) .* sin(phi_sol);
z = -L * cos(theta_sol);

%% ENERGY COMPUTATION
% Kinetic Energy:
% T = 1/2*I1*(thetadot^2 + sin(theta)^2*phidot^2) + 1/2*I3*(psidot + phidot*cos(theta))^2
T_num = 0.5 * I1 * (thetadot_sol.^2 + (sin(theta_sol).^2) .* phidot_sol.^2) + ...
        0.5 * I3 * (psidot_sol + phidot_sol.*cos(theta_sol)).^2;
% Potential Energy:
U_num = m * -g * L * (1 - cos(theta_sol));
% Total Energy:
E_total = T_num + U_num;

%% PLOTTING GENERALIZED COORDINATES (3D Plot)
figure;
plot3(t_sol, phi_sol, theta_sol, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('\phi (rad)');
zlabel('\theta (rad)');
title('Evolution of Generalized Coordinates (3D Plot)');
grid on;

%% PLOTTING ENERGY OF THE SYSTEM
figure;
plot(t_sol, T_num, 'r-', 'LineWidth',1.5); hold on;
plot(t_sol, U_num, 'b-', 'LineWidth',1.5);
plot(t_sol, E_total, 'k--', 'LineWidth',2);
xlabel('Time (s)'); ylabel('Energy (J)');
legend('Kinetic Energy', 'Potential Energy', 'Total Energy');
title('Energy of the 3DOF Pendulum');
grid on;

%% ANIMATION: 3D TRAJECTORY OF THE PENDULUM BOB FROM MULTIPLE PERSPECTIVES
figure;
for k = 1:length(t_sol)
    clf; % Clear the figure for the new frame
    
    % Use tiledlayout to organize 4 views in a 2x2 grid.
    tiledlayout(2,2, 'TileSpacing', 'Compact');
    
    % --- Isometric View (Top-Left) ---
    nexttile;
    hold on;
    plot3([0, x(k)], [0, y(k)], [0, z(k)], 'r-', 'LineWidth', 2);
    plot3(x(k), y(k), z(k), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    axis equal;
    xlim([-L L]); ylim([-L L]); zlim([-L L]);
    view([45,35]);             % Isometric-like view
    camproj('orthographic');    % Use orthographic projection
    grid on;
    title('Isometric View');
    
    % --- Top View (Top-Right) ---
    nexttile;
    hold on;
    plot3([0, x(k)], [0, y(k)], [0, z(k)], 'r-', 'LineWidth', 2);
    plot3(x(k), y(k), z(k), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    axis equal;
    xlim([-L L]); ylim([-L L]); zlim([-L L]);
    view([0,90]);              % Top view: looking from above (elevation = 90°)
    camproj('orthographic');
    grid on;
    title('Top View');
    
    % --- Left View (Bottom-Left) ---
    nexttile;
    hold on;
    plot3([0, x(k)], [0, y(k)], [0, z(k)], 'r-', 'LineWidth', 2);
    plot3(x(k), y(k), z(k), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    axis equal;
    xlim([-L L]); ylim([-L L]); zlim([-L L]);
    view([-90,0]);             % Left view: camera from left (azimuth = -90°)
    camproj('orthographic');
    grid on;
    title('Left View');
    
    % --- Front View (Bottom-Right) ---
    nexttile;
    hold on;
    plot3([0, x(k)], [0, y(k)], [0, z(k)], 'r-', 'LineWidth', 2);
    plot3(x(k), y(k), z(k), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    axis equal;
    xlim([-L L]); ylim([-L L]); zlim([-L L]);
    view([0,0]);               % Front view: camera looking straight on (azimuth = 0°)
    camproj('orthographic');
    grid on;
    title('Front View');
    
    % Overall title with time information
    sgtitle(sprintf('Time = %.2f s', t_sol(k)));
    
    drawnow;
end