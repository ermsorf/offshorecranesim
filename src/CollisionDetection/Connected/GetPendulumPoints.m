% This code is to get the location of the crate using the MFM method.
% 
% The load is modeled as a rigid dendulum to approximate the 
% 
% The crate has two initial angels, and initial velocity. There is no forces acting on the load. 





% Initial conditions in the form of [Theta, Phi, Theta_dot and Phi_dot]
function points = GetPendulumPoints(params)
    % Extract input parameters from the list
    theta_0 = params(1);    % Initial theta
    thetadot_0 = params(2); % Initial thetadot
    phi_0 = params(3);      % Initial phi
    phidot_0 = params(4);   % Initial phidot
    l = params(5);          % Pendulum length (constant)

    g = 9.81;  % Gravity

    % Initial conditions: [theta, thetadot, phi, phidot]
    y0 = [theta_0, thetadot_0, phi_0, phidot_0];

    % Time span for simulation
    tspan = [0 5]; % Simulate from t = 0 to t = 10 seconds

    % Solve the system using ode45
    [t, y] = ode45(@(t, y) pendulumODE(t, y, l, g), tspan, y0);

    % Return time + angles
    points = [t, y(:, 1), y(:, 3)];
end