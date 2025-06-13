%% This script plots the evaluated reaction forces for a 3 link system without translation

% Load the MAT file containing the reaction forces matrix.
load('ReactionForces_3_link_fixedL..mat');

% Determine the number of time steps (rows)
numSteps = size(reactionForces, 1);

%% Column 16 ( Re34)
% Define the columns you want to plot
colsToPlot = 16;

% Create a time vector (assuming each row corresponds to one time step)
timeVec = reactionForces(:, 1);

% Plot the selected columns versus time
figure;
plot(timeVec, reactionForces(:, colsToPlot), 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Reaction Force Value [N]');
legend( 'Re34');
title('Reaction Forces vs Time');
grid on;
% Adjust the y-axis limits. For example, set the limits from -500 to 500:
ylim([98000 102000]);

% Find the maximum and minimum values along with their indices
[max_val, idx_max] = max(reactionForces(:, colsToPlot));
[min_val, idx_min] = min(reactionForces(:, colsToPlot));

% Hold on to the current plot to overlay additional markers
hold on;

% Plot the maximum and minimum values on the plot
plot(timeVec(idx_max), max_val, 'ro', 'MarkerSize', 8, 'LineWidth', 2 , 'DisplayName', 'Maximum Value');  % Red circle for max
plot(timeVec(idx_min), min_val, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Minimum Value');  % Green circle for min

% Optionally, add text labels for the max and min points
text(timeVec(idx_max), max_val, sprintf(' Max: %.2f', max_val), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
text(timeVec(idx_min), min_val, sprintf(' Min: %.2f', min_val), ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');

hold off;

%% Column 13 ( Re33)
% Define the columns you want to plot
colsToPlot = 13;

% Create a time vector (assuming each row corresponds to one time step)
timeVec = reactionForces(:, 1);

% Plot the selected columns versus time
figure;
plot(timeVec, reactionForces(:, colsToPlot), 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Reaction Force Value [N]');
legend( 'Re33');
title('Reaction Forces vs Time');
grid on;
ylim([100500 103500]);

% Find the maximum and minimum values along with their indices
[max_val, idx_max] = max(reactionForces(:, colsToPlot));
[min_val, idx_min] = min(reactionForces(:, colsToPlot));

% Hold on to the current plot to overlay additional markers
hold on;

% Plot the maximum and minimum values on the plot
plot(timeVec(idx_max), max_val, 'ro', 'MarkerSize', 8, 'LineWidth', 2 , 'DisplayName', 'Maximum Value');  % Red circle for max
plot(timeVec(idx_min), min_val, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Minimum Value');  % Green circle for min

% Optionally, add text labels for the max and min points
text(timeVec(idx_max), max_val, sprintf(' Max: %.2f', max_val), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
text(timeVec(idx_min), min_val, sprintf(' Min: %.2f', min_val), ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');

hold off;

%% Column 9 ( Re32)
% Define the columns you want to plot
colsToPlot = 10;

% Create a time vector (assuming each row corresponds to one time step)
timeVec = reactionForces(:, 1);

% Plot the selected columns versus time
figure;
plot(timeVec, reactionForces(:, colsToPlot), 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Reaction Force Value [N]');
legend( 'Re32');
title('Reaction Forces vs Time');
grid on;

% Find the maximum and minimum values along with their indices
[max_val, idx_max] = max(reactionForces(:, colsToPlot));
[min_val, idx_min] = min(reactionForces(:, colsToPlot));

% Hold on to the current plot to overlay additional markers
hold on;

% Plot the maximum and minimum values on the plot
plot(timeVec(idx_max), max_val, 'ro', 'MarkerSize', 8, 'LineWidth', 2 , 'DisplayName', 'Maximum Value');  % Red circle for max
plot(timeVec(idx_min), min_val, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Minimum Value');  % Green circle for min

% Optionally, add text labels for the max and min points
text(timeVec(idx_max), max_val, sprintf(' Max: %.2f', max_val), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
text(timeVec(idx_min), min_val, sprintf(' Min: %.2f', min_val), ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');

hold off;

%% Column 6 ( Re31)
% Define the columns you want to plot
colsToPlot = 7;

% Create a time vector (assuming each row corresponds to one time step)
timeVec = reactionForces(:, 1);

% Plot the selected columns versus time
figure;
plot(timeVec, reactionForces(:, colsToPlot), 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Reaction Force Value [N]');
legend( 'Re31');
title('Reaction Forces vs Time');
grid on;

% Find the maximum and minimum values along with their indices
[max_val, idx_max] = max(reactionForces(:, colsToPlot));
[min_val, idx_min] = min(reactionForces(:, colsToPlot));

% Hold on to the current plot to overlay additional markers
hold on;

% Plot the maximum and minimum values on the plot
plot(timeVec(idx_max), max_val, 'ro', 'MarkerSize', 8, 'LineWidth', 2 , 'DisplayName', 'Maximum Value');  % Red circle for max
plot(timeVec(idx_min), min_val, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Minimum Value');  % Green circle for min
% Optionally, add text labels for the max and min points
text(timeVec(idx_max), max_val, sprintf(' Max: %.2f', max_val), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
text(timeVec(idx_min), min_val, sprintf(' Min: %.2f', min_val), ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');

hold off;

%% Column 3 ( Re3I)
% Define the columns you want to plot
colsToPlot = 4;

% Create a time vector (assuming each row corresponds to one time step)
timeVec = reactionForces(:, 1);

% Plot the selected columns versus time
figure;
plot(timeVec, reactionForces(:, colsToPlot), 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Reaction Force Value [N]');
legend( 'Re3I');
title('Reaction Forces vs Time');
grid on;

% Find the maximum and minimum values along with their indices
[max_val, idx_max] = max(reactionForces(:, colsToPlot));
[min_val, idx_min] = min(reactionForces(:, colsToPlot));

% Hold on to the current plot to overlay additional markers
hold on;

% Plot the maximum and minimum values on the plot
plot(timeVec(idx_max), max_val, 'ro', 'MarkerSize', 8, 'LineWidth', 2 , 'DisplayName', 'Maximum Value');  % Red circle for max
plot(timeVec(idx_min), min_val, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Minimum Value');  % Green circle for min
% Optionally, add text labels for the max and min points
text(timeVec(idx_max), max_val, sprintf(' Max: %.2f', max_val), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
text(timeVec(idx_min), min_val, sprintf(' Min: %.2f', min_val), ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');

hold off;



