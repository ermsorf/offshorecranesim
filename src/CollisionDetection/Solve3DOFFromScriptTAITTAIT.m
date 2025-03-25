clear
clc

%% Define symbolic functions and parameters
syms theta1(t) theta2(t) theta3(t) l g

% Define derivatives
thetadot1  = diff(theta1, t);
thetadot2  = diff(theta2, t);
thetadot3  = diff(theta3, t);
thetaddot1 = diff(theta1, t, 2);
thetaddot2 = diff(theta2, t, 2);
thetaddot3 = diff(theta3, t, 2);

%% Write the equations in the form (LHS - RHS == 0)
% Equation 1:
eq1 = 6250603*thetaddot1 + 750000*l^2*thetaddot1 + 6250000*thetaddot3*cos(theta2) + 3000*l*thetaddot1*cos(2*theta2) + 6*thetadot1*thetadot2*sin(2*theta2) + 1500000*l^2*thetadot1*thetadot2*sin(2*theta2) == 3*thetaddot1*cos(2*theta2) + 3000*l*thetaddot1 + 6250000*thetadot2*thetadot3*sin(theta2) + 750000*l^2*thetaddot1*cos(2*theta2) + 6000*l*thetadot1*thetadot2*sin(2*theta2);

% Equation 2:
eq2 = 1500000*thetaddot2*l^2 + 3000*sin(2*theta2)*l*thetadot1^2 + 1500000*g*sin(theta2)*l + 6250000*thetadot3*sin(theta2)*thetadot1 + 6250306*thetaddot2 == 750000*sin(2*theta2)*l^2*thetadot1^2 + 6000*thetaddot2*l + 3*sin(2*theta2)*thetadot1^2 + 4500*g*sin(theta2);

% Equation 3:
eq3 = thetaddot3 + thetaddot1*cos(theta2) == thetadot1*thetadot2*sin(theta2);

%% Convert the system to first-order form
% The odeToVectorField command will introduce state variables:
%   x1 = theta1, x2 = theta2, x3 = theta3, 
%   x4 = theta1dot, x5 = theta2dot, x6 = theta3dot.
[V,S] = odeToVectorField(eq1, eq2, eq3);

% Optional: display the vector field for inspection
% pretty(S)  % Uncomment to see the state variable definitions

%% Create a MATLAB function handle from the vector field
% The resulting function will have the form dY/dt = M(t,Y,l,g)
M = matlabFunction(V, 'vars', {'t', 'Y', 'l', 'g'});

%% Set parameters and initial conditions
% (Replace these example values with the ones appropriate for your problem)
l_val = 30;      % example value for l
g_val = 9.81;   % gravitational acceleration
% Y0: [theta1(0), theta2(0), theta3(0), theta1dot(0), theta2dot(0), theta3dot(0)]
Y0 = [0, 0.1, 0, 0, 0, 0];  % example initial conditions

%% Solve the system using ode45
tspan = [0 10];  % time interval for the simulation
[t, Y] = ode45(@(t,Y) M(t, Y, l_val, g_val), tspan, Y0);

%% Plot the results
figure;
subplot(3,1,1)
plot(t, Y(:,1))
xlabel('Time (s)')
ylabel('\theta_1')
title('Theta1 vs Time')

subplot(3,1,2)
plot(t, Y(:,2))
xlabel('Time (s)')
ylabel('\theta_2')
title('Theta2 vs Time')

subplot(3,1,3)
plot(t, Y(:,3))
xlabel('Time (s)')
ylabel('\theta_3')
title('Theta3 vs Time')


%% Compute Cartesian coordinates for the pendulum bob
% Using the spherical-to-Cartesian conversion for a pendulum:
%   x = L*sin(theta2)*cos(theta1)
%   y = L*sin(theta2)*sin(theta1)
%   z = -L*cos(theta2)
L = abs(l_val);  % use the absolute value of l_val as the pendulum length
x = L * sin(Y(:,2)) .* cos(Y(:,1));
y = L * sin(Y(:,2)) .* sin(Y(:,1));
z = -L * cos(Y(:,2));

%% Create a figure with four subplots: top view, side view, front view, and isometric view.
figure;

% --- Top View (looking down from above) ---
subplot(2,2,1);
h_line1 = line([0 x(1)], [0 y(1)], [0 z(1)], 'LineWidth', 2);
hold on;
h_bob1  = plot3(x(1), y(1), z(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on; axis equal;
xlim([-L L]); ylim([-L L]); zlim([-L L]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Top View');
view(0,90);   % Top view: looking down the z-axis

% --- Side View (e.g., viewing along the x-axis) ---
subplot(2,2,2);
h_line2 = line([0 x(1)], [0 y(1)], [0 z(1)], 'LineWidth', 2);
hold on;
h_bob2  = plot3(x(1), y(1), z(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on; axis equal;
xlim([-L L]); ylim([-L L]); zlim([-L L]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Side View');
view(90,0);  % Side view: camera rotated to view from the side

% --- Front View (e.g., looking along the y-axis) ---
subplot(2,2,3);
h_line3 = line([0 x(1)], [0 y(1)], [0 z(1)], 'LineWidth', 2);
hold on;
h_bob3  = plot3(x(1), y(1), z(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on; axis equal;
xlim([-L L]); ylim([-L L]); zlim([-L L]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Front View');
view(0,0);   % Front view: looking head-on

% --- Isometric View ---
subplot(2,2,4);
h_line4 = line([0 x(1)], [0 y(1)], [0 z(1)], 'LineWidth', 2);
hold on;
h_bob4  = plot3(x(1), y(1), z(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on; axis equal;
xlim([-L L]); ylim([-L L]); zlim([-L L]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Isometric View');
view(45,45);  % Isometric view

%% Animate the pendulum motion in all subplots
for k = 1:length(t)
    % Update Top View
    subplot(2,2,1);
    set(h_line1, 'XData', [0, x(k)], 'YData', [0, y(k)], 'ZData', [0, z(k)]);
    set(h_bob1, 'XData', x(k), 'YData', y(k), 'ZData', z(k));
    
    % Update Side View
    subplot(2,2,2);
    set(h_line2, 'XData', [0, x(k)], 'YData', [0, y(k)], 'ZData', [0, z(k)]);
    set(h_bob2, 'XData', x(k), 'YData', y(k), 'ZData', z(k));
    
    % Update Front View
    subplot(2,2,3);
    set(h_line3, 'XData', [0, x(k)], 'YData', [0, y(k)], 'ZData', [0, z(k)]);
    set(h_bob3, 'XData', x(k), 'YData', y(k), 'ZData', z(k));
    
    % Update Isometric View
    subplot(2,2,4);
    set(h_line4, 'XData', [0, x(k)], 'YData', [0, y(k)], 'ZData', [0, z(k)]);
    set(h_bob4, 'XData', x(k), 'YData', y(k), 'ZData', z(k));
    
    drawnow;         % Refresh the plots
    pause(0.10);     % Adjust pause duration for desired animation speed
end
