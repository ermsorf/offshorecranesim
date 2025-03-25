clear
clc
close all

% Initial conditions in the form of [Theta, Phi, Theta_dot and Phi_dot, l]
initialConditions = [0,0.2,0.3,0,-30];
pivot = [20,0,35];

% Read in list of objects from other file
stationaryObjects = ListOfObjects();

%Display objects
disp(stationaryObjects);

%Display object in 3D plot
VisiualizeObjectList(stationaryObjects);

tic
points = GetPendulumPoints(initialConditions);
toc

 % Plot results
    figure;
    plot(points(:, 1), points(:, 2), 'r-', 'LineWidth', 2); % Theta vs Time
    hold on;
    plot(points(:, 1), points(:, 3), 'b-', 'LineWidth', 2); % Phi vs Time
    xlabel('Time (s)');
    ylabel('Angle (radians)');
    legend('\theta', '\phi');
    title('Pendulum Motion');
    grid on;

% Extract time and angles
    t = points(:, 1);
    theta = points(:, 3); % Polar angle
    phi = points(:, 2);   % Azimuthal angle
    l = initialConditions(5);        % Pendulum length

    % Convert polar angles to Cartesian coordinates
    X = l * sin(theta) .* cos(phi);
    Y = l * sin(theta) .* sin(phi);
    Z = l * cos(theta);  % Negative to place pivot at origin

    % Set up figure
    figure;
    % 3D View
    subplot(2,2,1);
    view([1, 1, 1]); 
    axis equal;
    xlim([-40,40]);
    ylim([-40,40]);
    zlim([-40,40]);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('3D View');
    grid on; hold on;
    plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    pendulum3D = plot3([0, X(1)], [0, Y(1)], [0, Z(1)], 'b-', 'LineWidth', 3);
    bob3D = plot3(X(1), Y(1), Z(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    % Top View (XY-plane)
    subplot(2,2,2);
    axis equal;
    xlim([-40,40]);
    ylim([-40,40]);
    xlabel('X'); ylabel('Y');
    title('Top View (XY-plane)');
    grid on; hold on;
    plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    pendulumTop = plot([0, X(1)], [0, Y(1)], 'b-', 'LineWidth', 3);
    bobTop = plot(X(1), Y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    % Front View (XZ-plane)
    subplot(2,2,3);
    axis equal;
    xlim([-40,40]);
    ylim([-40,40]); % Z-axis up
    xlabel('X'); ylabel('Z');
    title('Front View (XZ-plane)');
    grid on; hold on;
    plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    pendulumFront = plot([0, X(1)], [0, Z(1)], 'b-', 'LineWidth', 3);
    bobFront = plot(X(1), Z(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    % Side View (YZ-plane)
    subplot(2,2,4);
    axis equal;
    xlim([-40,40]);
    ylim([-40,40]); % Z-axis up
    xlabel('Y'); ylabel('Z');
    title('Side View (YZ-plane)');
    grid on; hold on;
    plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    pendulumSide = plot([0, Y(1)], [0, Z(1)], 'b-', 'LineWidth', 3);
    bobSide = plot(Y(1), Z(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');


    figure;
    axis equal;
    xlim([-40,40]);
    ylim([-40,40]);
    zlim([-40,40]);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('3D Trajectory of the Pendulum');
    grid on; hold on;
    view(3); % Set to isometric view

    % Plot fixed pivot point
    plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

    % Initialize trajectory plot
    trajectory = plot3(X(1), Y(1), Z(1), 'g-', 'LineWidth', 1.5);
    pendulumArm = plot3([0, X(1)], [0, Y(1)], [0, Z(1)], 'b-', 'LineWidth', 3);
    bob = plot3(X(1), Y(1), Z(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');


    % Animation loop
for i = 1:length(t)
        % Update 3D animation
        set(pendulum3D, 'XData', [0, X(i)], 'YData', [0, Y(i)], 'ZData', [0, Z(i)]);
        set(bob3D, 'XData', X(i), 'YData', Y(i), 'ZData', Z(i));

        % Update Top View (XY-plane)
        set(pendulumTop, 'XData', [0, X(i)], 'YData', [0, Y(i)]);
        set(bobTop, 'XData', X(i), 'YData', Y(i));

        % Update Front View (XZ-plane)
        set(pendulumFront, 'XData', [0, X(i)], 'YData', [0, Z(i)]);
        set(bobFront, 'XData', X(i), 'YData', Z(i));

        % Update Side View (YZ-plane)
        set(pendulumSide, 'XData', [0, Y(i)], 'YData', [0, Z(i)]);
        set(bobSide, 'XData', Y(i), 'YData', Z(i));

        % Update Trajectory Plot
        set(pendulumArm, 'XData', [0, X(i)], 'YData', [0, Y(i)], 'ZData', [0, Z(i)]);
        set(bob, 'XData', X(i), 'YData', Y(i), 'ZData', Z(i));

        % Update full trajectory path
        set(trajectory, 'XData', X(1:i), 'YData', Y(1:i), 'ZData', Z(1:i));

        % Pause to control animation speed
        pause(0.02);
end

R1 = zeros(length(X),1);
R2 = zeros(length(X),1);
R3 = zeros(length(X),1);

CartesianCoordinatesAndRotation = [t,X, Y, Z, R1, R2, R3];

loadObj = DefineLoad();

tic
minDistancesStruct = GJKAlgorithm(stationaryObjects, loadObj, pivot, CartesianCoordinatesAndRotation);    
toc

animateTrajectoryWithContacts(stationaryObjects, loadObj, pivot, CartesianCoordinatesAndRotation, minDistancesStruct);