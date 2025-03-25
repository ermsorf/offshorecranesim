    % Close any existing figures
    close all;
    clear
    clc

    % Define parameters
    l = 30;          % pendulum length (adjust as needed)
    g = 9.81;       % gravitational acceleration
    
    % Set initial conditions: [theta1, theta1_dot, theta2, theta2_dot]
    x0 = [0.5, 1, 0.2, 0.1];
    
    % Define the simulation time span
    tspan = [0 100];
    tic
    % Solve the ODE system using ode45
    [t, sol] = ode45(@(t,x) odefun(t, x, l, g), tspan, x0);
    toc
    %-------------------------------
    % Plot the time series of angles
    %-------------------------------
    figure(1);
    subplot(2,1,1);
    plot(t, sol(:,1), 'b-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\theta_1 (rad)');
    title('Time Series of \theta_1');
    grid on;
    
    subplot(2,1,2);
    plot(t, sol(:,3), 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\theta_2 (rad)');
    title('Time Series of \theta_2');
    grid on;
    
    %-------------------------------
    % Compute the 3D trajectory
    %-------------------------------
    theta1 = sol(:,1);
    theta2 = sol(:,3);
    % Spherical-to-Cartesian conversion
    x = l * sin(theta2) .* cos(theta1);
    y = l * sin(theta2) .* sin(theta1);
    z = -l * cos(theta2);  % negative z positions the bob below the pivot
    
    %-------------------------------
    % Plot the 3D trajectory
    %-------------------------------
    figure(2);
    plot3(x, y, z, 'LineWidth', 2);
    grid on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    title('3D Trajectory of the Pendulum Bob');
    axis equal;


function dx = odefun(t, x, l, g)
    % Unpack the state variables
    theta1     = x(1);
    theta1_dot = x(2);
    theta2     = x(3);
    theta2_dot = x(4);
    
    %--- Equation 1: for theta1̈ ---
    % Coefficient multiplying theta1̈
    A1 = cos(theta2)^2 + sin(theta2)^2 + 5000*l^2*cos(theta1)^2*sin(theta2)^2 + ...
         5000*l^2*sin(theta1)^2*sin(theta2)^2 + 1;
    
    % Terms from velocity products (first set)
    term1 = l*cos(theta1)*sin(theta2) * (5000*l*theta1_dot*sin(theta1)*sin(theta2) - 5000*l*theta2_dot*cos(theta1)*cos(theta2)) - ...
            l*sin(theta1)*sin(theta2) * (5000*l*theta1_dot*cos(theta1)*sin(theta2) + 5000*l*theta2_dot*cos(theta2)*sin(theta1));
    % Terms from velocity products (second set)
    term2 = l*cos(theta1)*sin(theta2) * (5000*l*theta2_dot*sin(theta1)*sin(theta2) - 5000*l*theta1_dot*cos(theta1)*cos(theta2)) - ...
            l*sin(theta1)*sin(theta2) * (5000*l*theta1_dot*cos(theta2)*sin(theta1) + 5000*l*theta2_dot*cos(theta1)*sin(theta2));
    
    % Solve for theta1̈
    theta1_ddot = (theta1_dot*term1 + theta2_dot*term2) / A1;
    
    %--- Equation 2: for theta2̈ ---
    A2 = 5000*l^2*sin(theta2)^2 + 5000*l^2*cos(theta1)^2*cos(theta2)^2 + ...
         5000*l^2*cos(theta2)^2*sin(theta1)^2 + 1;
    
    term3 = theta2_dot * ( l*cos(theta1)*cos(theta2) * (5000*l*theta1_dot*cos(theta2)*sin(theta1) + 5000*l*theta2_dot*cos(theta1)*sin(theta2)) - ...
             5000*l^2*theta2_dot*cos(theta2)*sin(theta2) + ...
             l*cos(theta2)*sin(theta1) * (5000*l*theta2_dot*sin(theta1)*sin(theta2) - 5000*l*theta1_dot*cos(theta1)*cos(theta2)));
         
    term4 = theta1_dot * ( l*cos(theta1)*cos(theta2) * (5000*l*theta1_dot*cos(theta1)*sin(theta2) + 5000*l*theta2_dot*cos(theta2)*sin(theta1)) + ...
             l*cos(theta2)*sin(theta1) * (5000*l*theta1_dot*sin(theta1)*sin(theta2) - 5000*l*theta2_dot*cos(theta1)*cos(theta2)));
    
    % Solve for theta2̈, including gravitational effects
    theta2_ddot = (term3 + term4 - 5000*g*l*sin(theta2)) / A2;
    
    % Return the state derivative
    dx = [theta1_dot; theta1_ddot; theta2_dot; theta2_ddot];
end
