%% Get the EOM for the single rigid body pendulum with a frame at fixed point O
%% No added energy from external torques or forces.
%% Point O is located at the trolley, and it is assumed that the trolley and boom stop instantaneously.
%% Theta is precession, phi is nutation, and psi is spin.

clear; clc;

%% Define symbolic functions for the angles and their first derivatives
syms theta(t) psi(t) phi(t) theta_dot(t) psi_dot(t) phi_dot(t) t l w d h m g

%% Set parameter values
g = -9.81;   % gravitational acceleration (m/s^2)
m = 4000;    % mass (kg)
h = 3;       % height (m)
w = 3;       % width (m)
d = 10;      % depth (m)
l = -30;     % length offset (m)

%% Define the angular velocity vector (using the derivative functions)
omega = [ phi_dot(t)*cos(psi(t)) + theta_dot(t)*sin(phi(t))*sin(psi(t)); 
    -phi_dot(t)*sin(psi(t)) + theta_dot(t)*sin(phi(t))*cos(psi(t)); 
    theta_dot(t)*cos(phi(t)) + psi_dot(t) ];

%% Define the moments of inertia about the center of mass and then shift to point O
J1 = (1/12)*m*(w^2 + d^2);
J2 = (1/12)*m*(d^2 + h^2);
J3 = (1/12)*m*(w^2 + h^2);
J_C = diag([J1, J2, J3]);
J_O = J_C + diag([l^2*m, l^2*m, 0]);

%% Angular momentum about O
H_O = J_O * omega;

%% Kinetic Energy (K)
K = (1/2) * (omega.') * H_O;  % scalar expression

%% Potential Energy (UE)
UE = m * g * l * (1 - cos(phi(t)));

%% Define the Euler–Lagrange (dynamic) equations.
%     d/dt ( dK/d(q_dot) ) - dK/dq + d(UE)/dq == 0.
% For theta:
eq1 = diff(diff(K, theta_dot(t)), t)  + diff(UE, theta(t)) == 0;
% For psi:
eq2 = diff(diff(K, phi_dot(t)), t)    + diff(UE, phi(t))   == 0;
% For phi:
eq3 = diff(diff(K, psi_dot(t)), t)    + diff(UE, psi(t))   == 0;

%% Define the kinematic equations (relating angles to their defined first derivatives)
eq_a = diff(theta(t), t)  == theta_dot(t);
eq_b = diff(psi(t), t)    == psi_dot(t);
eq_c = diff(phi(t), t)    == phi_dot(t);

%% Combine into a full system of 6 ODEs:
FullSystem = [eq_a; eq_b; eq_c; eq1; eq2; eq3];

%% Specify the vector of dependent variables in the desired order:
Vars = [theta(t); phi(t); psi(t);  theta_dot(t); phi_dot(t); psi_dot(t) ];

%% Define initial conditions (as a 6x1 column vector)
y0 = [1; 2; 1; 0; 0; 0];

%% Convert the full system to a first-order vector field.
V = odeToVectorField(FullSystem, Vars);

%% Convert the symbolic vector field to a MATLAB function handle.
% M(t, Y) will return a 6x1 vector (dY/dt).
M = matlabFunction(V, 'vars', {'t','Y'});

%% Define simulation time interval
t0 = 0;    % initial time (seconds)
dt  = 0.001;
tf = 10;   % final time (seconds)
interval = [t0 tf];



%% Solve the ODE system using ode45
ySol = ode15s(M, interval, y0);

%% Evaluate the solution at dt interval for plotting
tValues = linspace(t0, tf, tf/dt);
yValues = deval(ySol, tValues);

%% Plot the solutions for the angles (first three components)
figure;
subplot(3,1,1)
plot(tValues, yValues(1,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('\theta(t)')
title('Solution for Precession / \theta(t)')

subplot(3,1,2)
plot(tValues, yValues(2,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('\phi(t)')
title('Solution for Nutation / \phi(t)')

subplot(3,1,3)
plot(tValues, yValues(3,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('\psi(t)')
title('Solution for Spin / \psi(t)')




%% Part 2. Compute and Plot the Kinetic and Potential Energy

nPoints = length(tValues);
K_values = zeros(1, nPoints);
UE_values = zeros(1, nPoints);

for i = 1:nPoints
    % Unpack the solution at time step i
    theta_val     = yValues(1,i);
    psi_val       = yValues(2,i);
    phi_val       = yValues(3,i);
    theta_dot_val = yValues(4,i);
    psi_dot_val   = yValues(5,i);
    phi_dot_val   = yValues(6,i);
    
    % Compute the angular velocity vector at this time:
    omega_val = [ phi_dot_val * cos(psi_val) + theta_dot_val * sin(phi_val)* sin(psi_val);
                 -phi_dot_val * sin(psi_val) + theta_dot_val * sin(phi_val)* cos(psi_val);
                  theta_dot_val * cos(phi_val) + psi_dot_val ];



    % Compute the angular momentum H_O = J_O * omega
    H_O_val = J_O * omega_val;
    
    % Kinetic energy K = 0.5 * omega' * H_O
    K_values(i) = 0.5 * (omega_val.' * H_O_val);
    
    % Potential energy UE = m * g * l * (1 - cos(phi))
    UE_values(i) = m * g * l * (1 - cos(phi_val));
end

% Total energy = Kinetic + Potential
TotalEnergy = K_values + UE_values;

% Plot the energies
figure;
plot(tValues, K_values, 'r', 'LineWidth', 1.5); hold on;
plot(tValues, UE_values, 'b', 'LineWidth', 1.5);
plot(tValues, TotalEnergy, 'k:', 'LineWidth', 1.5);  % black dotted line for total energy
xlabel('Time (s)');
ylabel('Energy (J)');
legend('Kinetic Energy', 'Potential Energy', 'Total Energy');
title('Energy vs Time');
grid on;













