%% This script plots the load cell output in terms of the mass 

load('LoadCellOutput_3_link_fixed_L.mat');

% Determine the number of time steps (rows)
numSteps = size(LoadCellOutput, 1);


%% Plot Load cell output

% Define the columns you want to plot
colsToPlot = 7;

% Create a time vector (assuming each row corresponds to one time step)
timeVec = LoadCellOutput(:, 1);

% Plot the selected columns versus time
figure;
plot(timeVec, LoadCellOutput(:, colsToPlot), 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Lifting load [kg]');
legend( 'Re31');
title('Load cell measurement vs time');
grid on;

% Extract the data vector (assuming one column)
y = LoadCellOutput(:, colsToPlot);

% Find the minimum and maximum values and their indices
[minVal, minIdx] = min(y);
[maxVal, maxIdx] = max(y);

% Get the corresponding time values
minTime = timeVec(minIdx);
maxTime = timeVec(maxIdx);

% Hold the plot to add markers
hold on;

% Plot markers for the minimum and maximum values
% Plot markers for the minimum and maximum values with assigned names
plot(minTime, minVal, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Minimum Value');
plot(maxTime, maxVal, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Maximum Value');


% Optionally, add text annotations
text(minTime, minVal, sprintf('Min: %.2f', minVal), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
text(maxTime, maxVal, sprintf('Max: %.2f', maxVal), 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');

% Release the hold if no further plotting is needed
hold off;