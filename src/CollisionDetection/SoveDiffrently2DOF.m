%% Get the EOM for the single rigid body pendulum with a frame at fixed point O
%% No added energy from external torques or forces.
%% Point O is located at the trolley, and it is assumed that the trolley and boom stop instantaneously.
%% Theta is precession, phi is nutation, and psi is spin.

clear; clc;

%% Define symbolic functions for the angles and their first derivatives
syms theta(t)  phi(t) g m h w d l

%% Set parameter values
g = -9.81;   % gravitational acceleration (m/s^2)
m = 4000;    % mass (kg)
h = 3;       % height (m)
w = 3;       % width (m)
d = 10;      % depth (m)
l = -30;     % length offset (m)

%% Define the angular velocity vector (using the derivative functions)
omega = [diff(phi(t), t); sin(phi(t))*diff(theta(t), t); cos(phi(t))*diff(theta(t), t)]

%% Define the moments of inertia about the center of mass and then shift to point O
J1 = 50
J2 = 50
J3 = 50
J_C = diag([J1, J2, J3]);
J_O = J_C + diag([l^2*m, l^2*m, 0]);

%% Angular momentum about O
H_O = J_O * omega;

%% Kinetic Energy (K)
K = (1/2) * (omega.') * J_O * omega;  % scalar expression

%% Potential Energy (UE)
UE = m * g * l * (1 - cos(phi(t)));

%% Define the Euler–Lagrange (dynamic) equations.
%     d/dt ( dK/d(q_dot) ) - dK/dq + d(UE)/dq == 0.
% For theta:
eq1 = simplify(diff(diff(K, diff(theta(t), t)), t)  + diff(UE, theta(t)) == 0)
% For phi:
eq2 = simplify(diff(diff(K, diff(phi(t), t)), t)    + diff(UE, phi(t))   == 0)

vars = [theta(t); diff(theta(t), t); phi(t); diff(phi(t), t)];
[V, S] = odeToVectorField([eq1, eq2]);

% Convert symbolic system to numerical function
f = matlabFunction(V, 'Vars', {'t', 'Y'});

% Time span
tspan = [0 20];

% Initial conditions: [theta0, dtheta0/dt, psi0, dpsi0/dt, phi0, dphi0/dt]
Y0 = [1; 1; -1; 0];

tic

% Solve using ode45
options = odeset('RelTol',1e-8, 'AbsTol',1e-10);
[t, Y] = ode15s(f, tspan, Y0, options);

% Plot solutions
figure;
hold on;
plot(t, Y(:,1), 'r', 'DisplayName', '\theta Precession');
plot(t, Y(:,2), 'g', 'DisplayName', '\phi Nutation');
legend;
xlabel('Time');
ylabel('Values');
title('Solution of Coupled Differential Equations');
grid on;

toc





