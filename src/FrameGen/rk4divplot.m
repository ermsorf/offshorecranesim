data = readmatrix('rk4_results.csv'); % Read CSV file
time = data(:, 1); % Time values
q1 = data(:, 2);   % Theta1
q2 = data(:, 3);   % Theta2
qd1 = data(:, 4);  % ThetaDot1
qd2 = data(:, 5);  % ThetaDot2

% Plot Theta1 and Theta2
figure;
plot(time, q1, 'r', time, q2, 'b');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('\theta_1', '\theta_2');
title('Double Pendulum Angles Over Time');

% Plot Velocities
figure;
plot(time, qd1, 'r', time, qd2, 'b');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('\thetȧ_1', '\thetȧ_2');
title('Double Pendulum Angular Velocities Over Time');