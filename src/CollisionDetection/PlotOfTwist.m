%% completePendulumExplicitEquation.m
% This script:
%   1. Derives and solves the full nonlinear 3-DOF pendulum ODE (using
%      Hamilton’s principle applied to a Lagrangian with coordinates phi, theta, psi)
%      for t = 0 to T seconds with fixed time steps.
%   2. Computes the Cartesian coordinates of the mass assumed to lie at the end
%      of a rod of length L:
%         x(t) = L*sin(theta(t))*cos(phi(t))
%         y(t) = L*sin(theta(t))*sin(phi(t))
%         z(t) = -L*cos(theta(t))
%   3. Fits a Fourier series (using the 'fourier5' model) to each coordinate,
%      yielding an explicit equation for x(t), y(t), and z(t).
%   4. Extracts the numerical coefficients from the fits and prints the explicit
%      numerical equations.
%   5. Plots the data and the fitted curves in 2D and also a 3D plot of the trajectory.
%   6. (NEW) Animates the pendulum motion while overlaying a body-fixed dynamics frame
%         at the mass to visualize the twist (via the psi angle).
%
% Note: The 'fourier5' model fits a function of the form:
%       f(t) = a0 + a1*sin(w*t) + b1*cos(w*t) + a2*sin(2*w*t) + b2*cos(2*w*t) +
%              a3*sin(3*w*t) + b3*cos(3*w*t)
%
% Author: [Your Name]
% Date: [Today's Date]

clear; clc; close all;

%% 1. PARAMETERS and ODE SOLUTION SETUP
% Physical parameters
m = 4000;         % mass [kg]
L = -30;          % rod length [m] (Note: L is negative so that z = -L*cos(theta) gives z in [-30,0])
a = 5;            % body dimension [m]
b = 5;            % body dimension [m]
c = 5;            % body dimension [m]

% Inertia–like constants (from the Lagrangian)
I1 = m*L^2 + m*(a^2 + b^2)/12;
I2 = m*L^2 + m*(a^2 + c^2)/12;
I3 = a^2*m/12;

% Time span for integration (fixed time steps)
dt = 0.1;     % time step [s]
T  = 10;       % total simulation time [s]
tspan = 0:dt:T;

% Initial conditions for the state vector:
% X = [phi; theta; psi; phidot; thetadot; psidot]
X0 = [3; 3; 3; 0.1; -0.5; -0.4];

%% 2. SYMBOLIC DERIVATION OF THE EQUATIONS OF MOTION
% We use the generalized coordinates: phi(t), theta(t), psi(t).
% The Lagrangian is defined as:
%
%   Lagrangian = T - U,
%
% with kinetic energy:
%   T = 1/2 I1*(phi_dot*sin(psi) - theta_dot*cos(psi)*sin(phi))^2
%     + 1/2 I2*(phi_dot*cos(psi) + theta_dot*sin(psi)*sin(phi))^2
%     + I3*(psi_dot + theta_dot*cos(phi))^2,
%
% and potential energy:
%   U = (-981*L*m/200)*(1 - cos(theta)).
%
% (Note: 981/200 ≈ 4.905 is half of 9.81. Here L is negative so U is defined appropriately.)
%
% Define the time–dependent symbolic functions:
syms phi(t) theta(t) psi(t)
phidot_t   = diff(phi, t);
thetadot_t = diff(theta, t);
psidot_t   = diff(psi, t);

% Potential energy:
U_sym = (-981*L*m/200)*(1 - cos(theta));

% Lagrangian:
Lagrangian = 1/2*I1*( phidot_t*sin(psi) - thetadot_t*cos(psi)*sin(phi) )^2 + ...
             1/2*I2*( phidot_t*cos(psi) + thetadot_t*sin(psi)*sin(phi) )^2 + ...
                  I3*( psidot_t + thetadot_t*cos(phi) )^2 - U_sym;

% Euler–Lagrange equations for each coordinate q = phi, theta, psi:
EL_phi   = simplify( diff( diff(Lagrangian, diff(phi,t) ), t ) - diff(Lagrangian, phi) );
EL_theta = simplify( diff( diff(Lagrangian, diff(theta,t) ), t ) - diff(Lagrangian, theta) );
EL_psi   = simplify( diff( diff(Lagrangian, diff(psi,t) ), t ) - diff(Lagrangian, psi) );

% To use these equations numerically we “replace” the time derivatives
% by independent symbols.
syms phidot_sym thetadot_sym psidot_sym real
syms phival thetaval psival real
syms phiddot thetaddot psiddot real

% Replace second derivatives:
EL_phi   = subs(EL_phi, diff(phi,t,2), phiddot);
EL_theta = subs(EL_theta, diff(theta,t,2), thetaddot);
EL_psi   = subs(EL_psi, diff(psi,t,2), psiddot);

% Replace first derivatives:
EL_phi   = subs(EL_phi, [diff(phi,t), diff(theta,t), diff(psi,t)], [phidot_sym, thetadot_sym, psidot_sym]);
EL_theta = subs(EL_theta, [diff(phi,t), diff(theta,t), diff(psi,t)], [phidot_sym, thetadot_sym, psidot_sym]);
EL_psi   = subs(EL_psi, [diff(phi,t), diff(theta,t), diff(psi,t)], [phidot_sym, thetadot_sym, psidot_sym]);

% Replace the functions with independent variables:
EL_phi   = subs(EL_phi, [phi, theta, psi], [phival, thetaval, psival]);
EL_theta = subs(EL_theta, [phi, theta, psi], [phival, thetaval, psival]);
EL_psi   = subs(EL_psi, [phi, theta, psi], [phival, thetaval, psival]);

% Write the three second–order equations in matrix form:
%      M(q)*[phiddot; thetaddot; psiddot] + F(q,q_dot) = 0.
vars = [phiddot, thetaddot, psiddot];
[Mat, F_sym] = equationsToMatrix([EL_phi, EL_theta, EL_psi], vars);

% Solve for the accelerations:
accel_sym = -Mat\F_sym;  % symbolic column vector: [phiddot; thetaddot; psiddot]

% Define symbolic parameters for m, L, a, b, c so that they can be used in matlabFunction:
syms m_sym L_sym a_sym b_sym c_sym real

% Convert the symbolic accelerations to a MATLAB function handle.
% The inputs are: (phival, thetaval, psival, phidot_sym, thetadot_sym, psidot_sym, m_sym, L_sym, a_sym, b_sym, c_sym)
f_accel = matlabFunction(accel_sym, 'Vars', {phival, thetaval, psival, ...
                    phidot_sym, thetadot_sym, psidot_sym, m_sym, L_sym, a_sym, b_sym, c_sym});

%% 3. SOLVE THE ODE SYSTEM
% We recast the three second–order ODEs as a 6-D first–order system.
% The state vector is: X = [phi; theta; psi; phidot; thetadot; psidot].
ODEfun = @(t, X) fullODEfun(t, X, m, L, a, b, c, f_accel);

% Set integration options and solve with ode45.
options = odeset('RelTol',1e-8, 'AbsTol',1e-10);
[t_sol, X_sol] = ode45(ODEfun, tspan, X0, options);

% Extract the generalized coordinates from the solution:
phi_sol   = X_sol(:,1);
theta_sol = X_sol(:,2);
psi_sol   = X_sol(:,3);
% (The angular velocities are in columns 4–6, but are not used for the plots.)

%% 4. COMPUTE CARTESIAN COORDINATES
% We assume the pendulum’s mass is located at the end of a rod of length L.
% The mapping is:
%   x(t) = L*sin(theta(t))*cos(phi(t))
%   y(t) = L*sin(theta(t))*sin(phi(t))
%   z(t) = -L*cos(theta(t))
x = L * sin(theta_sol) .* cos(phi_sol);
y = L * sin(theta_sol) .* sin(phi_sol);
z = -L * cos(theta_sol);

%% 5. FIT FOURIER SERIES TO EACH COORDINATE
% We use the 'fourier5' model which fits functions of the form:
%   f(t) = a0 + a1*sin(w*t) + b1*cos(w*t) + a2*sin(2*w*t) + b2*cos(2*w*t) +
%          a3*sin(3*w*t) + b3*cos(3*w*t)
ftype = 'fourier5';
fitX = fit(t_sol, x, ftype);
fitY = fit(t_sol, y, ftype);
fitZ = fit(t_sol, z, ftype);

%% 6. DISPLAY THE EXPLICIT EQUATIONS
% Get the generic-form equations as strings.
x_eq = formula(fitX);
y_eq = formula(fitY);
z_eq = formula(fitZ);

fprintf('\nExplicit Fourier Series Equations for the Pendulum Coordinates (Generic Form):\n\n');
fprintf('x(t) = %s\n\n', x_eq);
fprintf('y(t) = %s\n\n', y_eq);
fprintf('z(t) = %s\n\n', z_eq);

% Extract numerical coefficients.
coeffsX = coeffvalues(fitX);
coeffsY = coeffvalues(fitY);
coeffsZ = coeffvalues(fitZ);

fprintf('\nExplicit Numerical Fourier Series Equations:\n\n');
fprintf('x(t) = %.6f + %.6f*sin(%.6f*t) + %.6f*cos(%.6f*t) + %.6f*sin(2*%.6f*t) + %.6f*cos(2*%.6f*t)\n\n',...
    coeffsX(1), coeffsX(2), coeffsX(6), coeffsX(3), coeffsX(6), coeffsX(4), coeffsX(6), coeffsX(5), coeffsX(6));
fprintf('y(t) = %.6f + %.6f*sin(%.6f*t) + %.6f*cos(%.6f*t) + %.6f*sin(2*%.6f*t) + %.6f*cos(2*%.6f*t)\n\n',...
    coeffsY(1), coeffsY(2), coeffsY(6), coeffsY(3), coeffsY(6), coeffsY(4), coeffsY(6), coeffsY(5), coeffsY(6));
fprintf('z(t) = %.6f + %.6f*sin(%.6f*t) + %.6f*cos(%.6f*t) + %.6f*sin(2*%.6f*t) + %.6f*cos(2*%.6f*t)\n\n',...
    coeffsZ(1), coeffsZ(2), coeffsZ(6), coeffsZ(3), coeffsZ(6), coeffsZ(4), coeffsZ(6), coeffsZ(5), coeffsZ(6));

%% Create a time vector for plotting the Fourier fits:
t_fit_plot = linspace(min(t_sol), max(t_sol), 200);
x_fit_plot = fitX(t_fit_plot);
y_fit_plot = fitY(t_fit_plot);
z_fit_plot = fitZ(t_fit_plot);

%% 7. 3D PLOT OF THE FOURIER SERIES APPROXIMATED TRAJECTORY
figure;
plot3(x_fit_plot, y_fit_plot, z_fit_plot, 'r-', 'LineWidth', 2);
axis equal;
xlim([-25 25]);
ylim([-25 25]);
zlim([-35 0]);     
hold on;
plot3(x, y, z, 'bo'); % Original data points for reference
grid on;
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title('3D Plot of Fourier Series Approximated Pendulum Trajectory');
view(3);

%% 8. (Optional) STATIC PLOTS OF THE FITTED CURVES
figure;
subplot(3,1,1);
plot(t_sol, x, 'bo'); hold on;
plot(t_sol, fitX(t_sol), 'r-', 'LineWidth', 1.5);
title('x-coordinate and Fourier Fit');
xlabel('Time (s)'); ylabel('x (m)');

subplot(3,1,2);
plot(t_sol, y, 'bo'); hold on;
plot(t_sol, fitY(t_sol), 'r-', 'LineWidth', 1.5);
title('y-coordinate and Fourier Fit');
xlabel('Time (s)'); ylabel('y (m)');

subplot(3,1,3);
plot(t_sol, z, 'bo'); hold on;
plot(t_sol, fitZ(t_sol), 'r-', 'LineWidth', 1.5);
title('z-coordinate and Fourier Fit');
xlabel('Time (s)'); ylabel('z (m)');
sgtitle('Pendulum Coordinates and Their Fourier Approximations');

%% 9. ANIMATION: OVERLAY DYNAMICS FRAME TO VISUALIZE TWISTING MOTION
figure;
for k = 1:length(t_sol)
    clf; hold on;
    
    % Plot the pendulum rod (from the origin to the mass):
    plot3([0, x(k)], [0, y(k)], [0, z(k)], 'r-', 'LineWidth', 2);
    % Plot the pendulum mass:
    plot3(x(k), y(k), z(k), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    % Extract the current generalized coordinates:
    phi_val   = phi_sol(k);
    theta_val = theta_sol(k);
    psi_val   = psi_sol(k);
    
    % 1. Define the rod direction (body z-axis):
    e3 = [ sin(theta_val)*cos(phi_val);
           sin(theta_val)*sin(phi_val);
          -cos(theta_val) ];
      
    % 2. Define a horizontal axis in the global plane:
    e2 = [ -sin(phi_val);
            cos(phi_val);
            0 ];
    
    % 3. Compute e1 as the cross product (ensuring a right-handed system):
    e1 = cross(e2, e3);  % e1 is automatically normalized
    
    % 4. Apply the twist rotation about e3 by the angle psi:
    e_x = cos(psi_val)*e1 - sin(psi_val)*e2;  % body-fixed x-axis
    e_y = sin(psi_val)*e1 + cos(psi_val)*e2;  % body-fixed y-axis
    
    % Set a scale for drawing the coordinate frame arrows:
    scale = 0.2 * abs(L);
    
    % Overlay the body-fixed frame at the mass location:
    % Draw the body x-axis in green, y-axis in blue, and z-axis in red.
    quiver3(x(k), y(k), z(k), scale*e_x(1), scale*e_x(2), scale*e_x(3), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(x(k), y(k), z(k), scale*e_y(1), scale*e_y(2), scale*e_y(3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(x(k), y(k), z(k), scale*e3(1), scale*e3(2), scale*e3(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % Set the axis properties and 3D view:
    axis equal;
    xlim([-25 25]);
    ylim([-25 25]);
    zlim([-35 0]);
    view(3);
    camproj('perspective');
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    title(sprintf('Time = %.2f s', t_sol(k)));
    grid on;
    drawnow;
    pause(0.05);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Nested Function: fullODEfun
function dXdt = fullODEfun(~, X, m, L, a, b, c, f_accel)
    % fullODEfun defines the 6-D state derivative for the pendulum.
    %
    % Input:
    %   X      = [phi; theta; psi; phidot; thetadot; psidot]
    %   m, L, a, b, c  = physical parameters
    %   f_accel = function handle that returns [phiddot; thetaddot; psiddot]
    %
    % Output:
    %   dXdt   = time derivative of the state vector.
    
    % Unpack the state vector:
    phi_val      = X(1);
    theta_val    = X(2);
    psi_val      = X(3);
    phidot_val   = X(4);
    thetadot_val = X(5);
    psidot_val   = X(6);
    
    % Compute the accelerations using the derived symbolic expressions:
    accel = f_accel(phi_val, theta_val, psi_val, phidot_val, thetadot_val, psidot_val, m, L, a, b, c);
    
    % Assemble the derivative of the state:
    dXdt = [phidot_val;
            thetadot_val;
            psidot_val;
            accel(1);
            accel(2);
            accel(3)];
end
