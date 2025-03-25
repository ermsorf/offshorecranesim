    % Parameters
    l = 30; % Length of the pendulum
    g = 9.81; % Gravity

    % Initial conditions
    y0 = [0.1; 0.2; 0.1; 0.05; 0; 0]; % [theta1; theta2; theta3; omega1; omega2; omega3]

    % Time span
    tspan = [0 10];

    % Solve the system using ode45
    [t, y] = ode45(@(t, y) odefun(t, y, l, g), tspan, y0);

    % Convert angles to Cartesian coordinates
    x = l * sin(y(:, 2)) .* cos(y(:, 1));
    y_cart = l * sin(y(:, 2)) .* sin(y(:, 1));
    z = -l * cos(y(:, 2));

    % 3D Plot of the pendulum's motion
        figure(1);

    figure;
    plot3(x, y_cart, z, 'b', 'LineWidth', 1.5);
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Pendulum Motion');
    axis([-40 40 -40 40 -40 0]); % Set axis limits

        figure(2);

    % Plot the results
    figure;
    plot(t, y(:, 1), 'r', t, y(:, 2), 'g', t, y(:, 3), 'b');
    legend('\theta_1', '\theta_2', '\theta_3');
    xlabel('Time');
    ylabel('Angles');
    title('Solution of the System');

function dydt = odefun(t, y, l, g)
    % y = [theta1; theta2; theta3; omega1; omega2; omega3]
    theta1 = y(1);
    theta2 = y(2);
    theta3 = y(3);
    omega1 = y(4);
    omega2 = y(5);
    omega3 = y(6);
    
    % Coefficients for the system of equations
    A = [1201 + 1000000*l^2 + 2000*l*cos(2*theta2) - cos(2*theta2) - 2000*l - 1000000*l^2*cos(2*theta2), 0, 400*cos(theta2);
         0, 401/200 - 10*l + 5000*l^2, 0;
         cos(theta2), 0, 1];
    
    % Right-hand side of the equations
    B = [2000*l*omega1 + 400*omega2*omega3*sin(theta2) + 1000000*l^2*omega1*cos(2*theta2) + 4000*l*omega1*omega2*sin(2*theta2) - 2*omega1*omega2*sin(2*theta2) - 2000000*l^2*omega1*omega2*sin(2*theta2);
         2500*sin(2*theta2)*l^2*omega1^2 - 5*sin(2*theta2)*l*omega1^2 + (sin(2*theta2)*omega1^2)/400 - omega3*sin(theta2)*omega1 + 5*g*sin(theta2) - 5000*g*l*sin(theta2);
         omega1*omega2*sin(theta2)];
    
    % Solve for angular accelerations
    alpha = A \ B;
    alpha1 = alpha(1);
    alpha2 = alpha(2);
    alpha3 = alpha(3);
    
    % Derivatives
    dydt = [omega1; omega2; omega3; alpha1; alpha2; alpha3];
end