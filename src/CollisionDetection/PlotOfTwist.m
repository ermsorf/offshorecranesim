%% Three-Axis Pendulum Simulation using Euler-Lagrange Equations
% This script:
%   1. Solves the nonlinear 3-DOF pendulum ODE using Euler-Lagrange.
%   2. Computes Cartesian coordinates of the mass at the end of a rod.
%   3. Plots the results for both Cartesian and generalized coordinates.
%   4. Animates the motion with a proper moving coordinate system.
%   5. Plots the kinetic, potential, and total energy over time.

clear; clc; close all;

%% PARAMETERS
m = 4000;   % Mass [kg]
L = 30;     % Rod length [m]
g = 9.81;   % Gravity (corrected direction)

% Moments of Inertia (for mass-on-rod system)
I1 = m * L^2;  
I2 = m * L^2;  
I3 = m * L^2;  

% Time span & initial conditions
dt = 0.1; T = 25;
tspan = 0:dt:T;
X0 = [0; 0; 0; 0.1; 0.1; 0.1];  % Initial angles and angular velocities

%% SYMBOLIC DERIVATION OF EQUATIONS OF MOTION
syms phi(t) theta(t) psi(t)
phidot_t   = diff(phi, t);
thetadot_t = diff(theta, t);
psidot_t   = diff(psi, t);

% Potential Energy
U_sym = 6000*g*(cos(phi) - 1)



% Kinetic Energy
T_sym = (psidot_t + thetadot_t*cos(phi))*((2500*psidot_t)/3 + (2500*thetadot_t*cos(phi))/3) + (phidot_t*cos(psi) + thetadot_t*sin(phi)*sin(psi))*((6227129916893867*phidot_t*cos(psi))/34359738368 + (6227129916893867*thetadot_t*sin(phi)*sin(psi))/34359738368) + (phidot_t*sin(psi) - thetadot_t*cos(psi)*sin(phi))*((6256335694506667*phidot_t*sin(psi))/34359738368 - (6256335694506667*thetadot_t*cos(psi)*sin(phi))/34359738368)




% Lagrangian
Lagrangian = T_sym - U_sym;

% Euler-Lagrange Equations
EL_phi   = simplify(diff(diff(Lagrangian, phidot_t), t) - diff(Lagrangian, phi));
EL_theta = simplify(diff(diff(Lagrangian, thetadot_t), t) - diff(Lagrangian, theta));
EL_psi   = simplify(diff(diff(Lagrangian, psidot_t), t) - diff(Lagrangian, psi));

% Convert to numeric functions
syms phival thetaval psival phidot_sym thetadot_sym psidot_sym phiddot thetaddot psiddot real
EL_phi   = subs(EL_phi, [diff(phi,t,2), diff(phi,t), diff(theta,t), diff(psi,t)], [phiddot, phidot_sym, thetadot_sym, psidot_sym]);
EL_theta = subs(EL_theta, [diff(theta,t,2), diff(phi,t), diff(theta,t), diff(psi,t)], [thetaddot, phidot_sym, thetadot_sym, psidot_sym]);
EL_psi   = subs(EL_psi, [diff(psi,t,2), diff(phi,t), diff(theta,t), diff(psi,t)], [psiddot, phidot_sym, thetadot_sym, psidot_sym]);

EL_phi   = subs(EL_phi, [phi, theta, psi], [phival, thetaval, psival]);
EL_theta = subs(EL_theta, [phi, theta, psi], [phival, thetaval, psival]);
EL_psi   = subs(EL_psi, [phi, theta, psi], [phival, thetaval, psival]);

% Solve for accelerations
vars = [phiddot, thetaddot, psiddot];
[Mat, F_sym] = equationsToMatrix([EL_phi, EL_theta, EL_psi], vars);
accel_sym = -Mat\F_sym;

% Convert to function handle
f_accel = matlabFunction(accel_sym, 'Vars', {phival, thetaval, psival, phidot_sym, thetadot_sym, psidot_sym});

%% SOLVE THE ODE SYSTEM
ODEfun = @(t, X) fullODEfun(t, X, f_accel);
options = odeset('RelTol',1e-8, 'AbsTol',1e-10);
[t_sol, X_sol] = ode15s(ODEfun, tspan, X0, options);

% Extract Euler angles and angular velocities
phi_sol = X_sol(:,1);
theta_sol = X_sol(:,2);
psi_sol = X_sol(:,3);
phidot_sol = X_sol(:,4);
thetadot_sol = X_sol(:,5);
psidot_sol = X_sol(:,6);

% Compute Cartesian coordinates
x = L * sin(theta_sol) .* cos(phi_sol);
y = L * sin(theta_sol) .* sin(phi_sol);
z = -L * cos(theta_sol);

%% ENERGY COMPUTATION
% Kinetic Energy
T_num = 0.5 * I1 * (phidot_sol .* sin(psi_sol) - thetadot_sol .* cos(psi_sol) .* sin(phi_sol)).^2 + ...
        0.5 * I2 * (phidot_sol .* cos(psi_sol) + thetadot_sol .* sin(psi_sol) .* sin(phi_sol)).^2 + ...
        0.5 * I3 * (psidot_sol + thetadot_sol .* cos(phi_sol)).^2;
% Potential Energy (as defined in the Lagrangian)
U_num = m * g * L * (1 - cos(phi_sol));
% Total Energy
E_total = T_num + U_num;

%% PLOTTING GENERALIZED COORDINATES
figure;
subplot(3,1,1);
plot(t_sol, phi_sol, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\phi (rad)'); title('Precession Angle (\phi)');

subplot(3,1,2);
plot(t_sol, theta_sol, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\theta (rad)'); title('Nutation Angle (\theta)');

subplot(3,1,3);
plot(t_sol, psi_sol, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\psi (rad)'); title('Spin Angle (\psi)');

sgtitle('Generalized Coordinates Over Time');

%% PLOTTING ENERGY OF THE SYSTEM
figure;
plot(t_sol, T_num, 'r-', 'LineWidth', 1.5); hold on;
plot(t_sol, U_num, 'b-', 'LineWidth', 1.5);
plot(t_sol, E_total, 'k--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Energy (J)');
legend('Kinetic Energy', 'Potential Energy', 'Total Energy');
title('Energy of the System Over Time');
grid on;

%% ANIMATION: PENDULUM WITH BODY-FIXED FRAME
figure;
for k = 1:length(t_sol)
    clf; hold on;
    
    % Plot the pendulum rod
    plot3([0, x(k)], [0, y(k)], [0, z(k)], 'r-', 'LineWidth', 2);
    plot3(x(k), y(k), z(k), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    % Compute the rotation matrix
    phi_val = phi_sol(k);
    theta_val = theta_sol(k);
    psi_val = psi_sol(k);
    
    Rz = [cos(phi_val), -sin(phi_val), 0; sin(phi_val), cos(phi_val), 0; 0, 0, 1];
    Ry = [cos(theta_val), 0, sin(theta_val); 0, 1, 0; -sin(theta_val), 0, cos(theta_val)];
    Rx = [1, 0, 0; 0, cos(psi_val), -sin(psi_val); 0, sin(psi_val), cos(psi_val)];
    R = Rz * Ry * Rx;  

    % Define and transform the moving frame
    scale = 0.2 * L;
    e_x = R(:,1) * scale;
    e_y = R(:,2) * scale;
    e_z = R(:,3) * scale;
    
    % Plot the moving frame
    quiver3(x(k), y(k), z(k), e_x(1), e_x(2), e_x(3), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(x(k), y(k), z(k), e_y(1), e_y(2), e_y(3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(x(k), y(k), z(k), e_z(1), e_z(2), e_z(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % Axis settings
    axis equal;
    xlim([-L L]);
    ylim([-L L]);
    zlim([-L L]);
    view(3);
    camproj('perspective');
    xlabel('x'); ylabel('y'); zlabel('z');
    title(sprintf('Time = %.2f s', t_sol(k)));
    grid on;
    drawnow;
end

%% ODE FUNCTION
function dXdt = fullODEfun(~, X, f_accel)
    phi_val = X(1); theta_val = X(2); psi_val = X(3);
    phidot_val = X(4); thetadot_val = X(5); psidot_val = X(6);
    accel = f_accel(phi_val, theta_val, psi_val, phidot_val, thetadot_val, psidot_val);
    dXdt = [phidot_val; thetadot_val; psidot_val; accel(1); accel(2); accel(3)];
end
