%% 3D Pendulum Simulation with Container Perpendicular to Wire
% This script simulates a 3D pendulum composed of a massless wire (length L_wire)
% with a container (40 ft ≈ 12.192 m long) hanging from its end. The container is
% modeled as a rigid box with mass m and twist inertia. Its orientation is defined 
% by three Euler angles:
%   θ  (nutation: angle from vertical),
%   φ  (precession: azimuth), and
%   ψ  (spin/twist about the wire).
%
% The effective pendulum length is L_tot = L_wire + (L_cont/2). Damping is applied 
% to the swinging and twist motions. The container is forced to remain perpendicular 
% to the wire by constructing an orthonormal frame from the wire's unit vector. 
%
% After integrating the ODE system (using a constant-step RK4 integrator), the 
% precession angle is unwrapped (and its derivative recomputed), and the container’s 
% Cartesian position, energy (kinetic, potential, total), height, and speed are computed.
%
% Three figures are produced:
%   Figure 1: A 3D animation with four subplots (Front, Right, Top, Isometric) with 
%             fixed x and y limits [0,30] and z limits [–30,30], and the current time displayed.
%   Figure 2: An energy plot (kinetic, potential, and total energy vs. time).
%   Figure 3: A combined state plot (left subplot: height and speed on dual y-axes; right subplot: Euler angles vs. time).
%
% The generalized coordinates (Euler angles) are stored in theta_sol, phi_sol, and psi_sol.
%
% All units are SI.

clear; clc; close all;

%% NUMERIC PARAMETERS
% Geometry
L_wire = 30;              % Wire length (m)
L_cont = 12.192;          % Container length in meters (40 ft converted)
d = L_cont/2;             % Distance from container top to its center-of-mass
L_tot = L_wire + d;       % Effective pendulum length

% For drawing the container as a box:
W_cont = 2.44;            % Container width (m)
H_cont = 2.59;            % Container height (m)

% Physical parameters
m = 4000;                 % Container mass (kg)
g = 9.81;                 % Gravitational acceleration (m/s²)
% Damping coefficients
d_theta = 0;             % Damping for nutation (θ)
d_phi   = 0;             % Damping for precession (φ)
d_psi   = 0;             % Damping for twist (ψ)

% Simulation settings
t0 = 0; Tf = 8; dt = 0.01;
tspan = t0:dt:Tf;

%% ODE SYSTEM DEFINITION (Simplified Model)
% State vector X = [θ; φ; ψ; θ_dot; φ_dot; ψ_dot]
% Equations:
%   θ_dot = X(4)
%   φ_dot = X(5)
%   ψ_dot = X(6)
%
%   θ_ddot = sin(θ)*cos(θ)*(X(5))^2 - (g/L_tot)*sin(θ) - d_theta*X(4)
%   φ_ddot = -2*(X(4)/max(sin(X(1)),1e-3))*X(5) - d_phi*X(5)
%   ψ_ddot = - d_psi*X(6)
ODEfun = @(t, X) [ X(4);
    X(5);
    X(6);
    sin(X(1))*cos(X(1))*(X(5))^2 - (g/L_tot)*sin(X(1)) - d_theta*X(4);
    -2*(X(4)/max(sin(X(1)),1e-3))*X(5) - d_phi*X(5);
    - d_psi*X(6) ];

%% CONSTANT-STEP RK4 INTEGRATION
N = length(tspan);
X_sol = zeros(N, 6);
% Initial conditions: [θ; φ; ψ; θ_dot; φ_dot; ψ_dot]
X_sol(1,:) = [0.5; 0; 0; 0; 0; 0]';
for i = 1:N-1
    t_i = tspan(i);
    X_i = X_sol(i,:)';
    k1 = ODEfun(t_i, X_i);
    k2 = ODEfun(t_i + dt/2, X_i + dt/2*k1);
    k3 = ODEfun(t_i + dt/2, X_i + dt/2*k2);
    k4 = ODEfun(t_i + dt, X_i + dt*k3);
    X_sol(i+1,:) = (X_i + dt/6*(k1 + 2*k2 + 2*k3 + k4))';
end
t_sol = tspan';

%% Extract and Unwrap State Variables
% Generalized coordinates (Euler angles) are stored as:
%   theta_sol  = nutation (θ) [column 1],
%   phi_sol    = precession (φ) [column 2],
%   psi_sol    = twist (ψ) [column 3].
theta_sol = X_sol(:,1);
phi_sol = unwrap(X_sol(:,2));  % Unwrap φ to remove jumps
psi_sol = X_sol(:,3);
theta_dot_sol = X_sol(:,4);
phi_dot_sol = X_sol(:,5);
psi_dot_sol = X_sol(:,6);
% Recompute φ_dot numerically for smoothness:
phi_dot_sol = gradient(phi_sol, dt);

%% Compute Cartesian Coordinates of the Container's Center
% Using spherical coordinates (container's center-of-mass is along the wire):
%   x = L_tot*sin(θ)*cos(φ), y = L_tot*sin(θ)*sin(φ), z = -L_tot*cos(θ)
x = L_tot * sin(theta_sol).*cos(phi_sol);
y = L_tot * sin(theta_sol).*sin(phi_sol);
z = -L_tot * cos(theta_sol);

%% Compute Energy
% Kinetic energy from swinging:
T = 0.5 * m * L_tot^2 * (theta_dot_sol.^2 + (sin(theta_sol).^2).*phi_dot_sol.^2);
% Potential energy: U = m*g*L_tot*(1 - cos(θ)), so that U = 0 at θ = 0.
U = m * g * L_tot * (1 - cos(theta_sol));
E_total = T + U;

%% Compute Height and Container Speed
% Height: measured from the lowest point (θ = 0 gives z = -L_tot)
% Define height = L_tot + z (thus, height = 0 when z = -L_tot).
height = L_tot + z;
% Container speed (from swinging motion):
v_cont = L_tot * sqrt(theta_dot_sol.^2 + (sin(theta_sol).^2).*phi_dot_sol.^2);

%% Figure 1: 3D Animation with Four Fixed Views and Time Display
figure('Name','3D Animation');
fixedLimitsXY = [-40 40];   % For x and y axes
fixedLimitsZ = [-40 0];  % For z axis (fixed as requested)
for i = 1:10:N
    pos = [x(i), y(i), z(i)];  % Container's center-of-mass position
    % Wire: line from [0,0,0] to pos.
    
    % Compute container orientation so that it is perpendicular to the wire.
    % Let n be the wire's unit vector.
    n = [sin(theta_sol(i))*cos(phi_sol(i)); sin(theta_sol(i))*sin(phi_sol(i)); -cos(theta_sol(i))];
    n = n / norm(n);
    % Use global vertical as reference (if nearly parallel, switch to [1;0;0]).
    r_ref = [0; 0; 1];
    if abs(dot(n, r_ref)) > 0.99
        r_ref = [1; 0; 0];
    end
    % Construct an orthonormal basis:
    e1 = r_ref - dot(r_ref, n)*n;
    e1 = e1 / norm(e1);
    e2 = cross(n, e1);
    % The container's orientation is given by the matrix R_cont; then apply twist ψ.
    R_cont = [e1, e2, n];
    twist = psi_sol(i);
    R_twist = [cos(twist), -sin(twist), 0; sin(twist), cos(twist), 0; 0, 0, 1];
    R_final = R_cont * R_twist;
    
    % Define container as a box (centered at pos) with dimensions L_cont x W_cont x H_cont.
    vertices = [ -L_cont/2, -W_cont/2, -H_cont/2;
                  L_cont/2, -W_cont/2, -H_cont/2;
                  L_cont/2,  W_cont/2, -H_cont/2;
                 -L_cont/2,  W_cont/2, -H_cont/2;
                 -L_cont/2, -W_cont/2,  H_cont/2;
                  L_cont/2, -W_cont/2,  H_cont/2;
                  L_cont/2,  W_cont/2,  H_cont/2;
                 -L_cont/2,  W_cont/2,  H_cont/2 ];
    vertices_global = (R_final * vertices')' + pos;
    
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    
    clf;
    % Front View
    subplot(2,2,1);
    hold on;
    plot3([0 pos(1)], [0 pos(2)], [0 pos(3)], 'k-', 'LineWidth',2);
    plot3(pos(1), pos(2), pos(3), 'ro','MarkerSize',8,'MarkerFaceColor','r');
    patch('Vertices', vertices_global, 'Faces', faces, 'FaceColor','cyan','FaceAlpha',0.7);
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim(fixedLimitsXY); ylim(fixedLimitsXY); zlim(fixedLimitsZ);
    view([0,0]);
    title(sprintf('Front View, t = %.2f s', t_sol(i)));
    
    % Right View
    subplot(2,2,2);
    hold on;
    plot3([0 pos(1)], [0 pos(2)], [0 pos(3)], 'k-', 'LineWidth',2);
    plot3(pos(1), pos(2), pos(3), 'ro','MarkerSize',8,'MarkerFaceColor','r');
    patch('Vertices', vertices_global, 'Faces', faces, 'FaceColor','cyan','FaceAlpha',0.7);
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim(fixedLimitsXY); ylim(fixedLimitsXY); zlim(fixedLimitsZ);
    view([90,0]);
    title(sprintf('Right View, t = %.2f s', t_sol(i)));
    
    % Top View
    subplot(2,2,3);
    hold on;
    plot3([0 pos(1)], [0 pos(2)], [0 pos(3)], 'k-', 'LineWidth',2);
    plot3(pos(1), pos(2), pos(3), 'ro','MarkerSize',8,'MarkerFaceColor','r');
    patch('Vertices', vertices_global, 'Faces', faces, 'FaceColor','cyan','FaceAlpha',0.7);
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim(fixedLimitsXY); ylim(fixedLimitsXY); zlim(fixedLimitsZ);
    view([0,90]);
    title(sprintf('Top View, t = %.2f s', t_sol(i)));
    
    % Isometric View
    subplot(2,2,4);
    hold on;
    plot3([0 pos(1)], [0 pos(2)], [0 pos(3)], 'k-', 'LineWidth',2);
    plot3(pos(1), pos(2), pos(3), 'ro','MarkerSize',8,'MarkerFaceColor','r');
    patch('Vertices', vertices_global, 'Faces', faces, 'FaceColor','cyan','FaceAlpha',0.7);
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim(fixedLimitsXY); ylim(fixedLimitsXY); zlim(fixedLimitsZ);
    view([45,35]);
    title(sprintf('Isometric View, t = %.2f s', t_sol(i)));
    
    drawnow;
end

%% Figure 2: Energy Plot
figure('Name','Energy Plot');
plot(t_sol, T, 'r-', 'LineWidth',1.5); hold on;
plot(t_sol, U, 'b-', 'LineWidth',1.5);
plot(t_sol, E_total, 'k--', 'LineWidth',2);
xlabel('Time (s)'); ylabel('Energy (J)');
legend('Kinetic','Potential','Total');
title('Energy vs. Time'); grid on;

%% Figure 3: Combined State Plot (Height & Speed, and Euler Angles)
figure('Name','State Variables');
subplot(1,2,1); % Height and Speed
yyaxis left;
plot(t_sol, height, 'm-', 'LineWidth',1.5);
ylabel('Height (m)');
yyaxis right;
plot(t_sol, v_cont, 'k-', 'LineWidth',1.5);
ylabel('Speed (m/s)');
xlabel('Time (s)');
title('Height & Container Speed vs. Time');
grid on;
subplot(1,2,2); % Euler Angles
plot(t_sol, phi_sol, 'b-', 'LineWidth',1.5); hold on;
plot(t_sol, theta_sol, 'g-', 'LineWidth',1.5);
plot(t_sol, psi_sol, 'c-', 'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('\phi','\theta','\psi');
title('Euler Angles vs. Time');
grid on;
