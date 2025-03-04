function rk4_pendulum()
    % Time step and number of steps
    dt = 0.02; % Ensure it matches JS
    steps = 1000;
    
    % Initial conditions [theta1, theta2, thetad1, thetad2]
    state = [0; 0; 0; 0];
    
    % Store results: [time, theta1, theta2, thetad1, thetad2]
    results = zeros(steps+1, 5);
    results(1, :) = [0, state'];
    
    % RK4 Loop
    for i = 1:steps
        t = (i-1) * dt;
        k1 = dt * dynamics(state);
        k2 = dt * dynamics(state + 0.5 * k1);
        k3 = dt * dynamics(state + 0.5 * k2);
        k4 = dt * dynamics(state + k3);
        
        state = state + (k1 + 2*k2 + 2*k3 + k4) / 6;
        results(i+1, :) = [t + dt, state'];
    end
    
    % Load JavaScript RK4 results from CSV
    js_data = readmatrix('rk4_results.csv'); % Columns: [time, q1, q2, qd1, qd2]

    % Extract MATLAB and JS results
    time = results(:, 1);
    theta1_matlab = results(:, 2);
    theta2_matlab = results(:, 3);
    thetad1_matlab = results(:, 4);
    thetad2_matlab = results(:, 5);

    theta1_js = js_data(:, 2);
    theta2_js = js_data(:, 3);
    thetad1_js = js_data(:, 4);
    thetad2_js = js_data(:, 5);

    % Compute absolute errors
    theta1_error = abs(theta1_matlab - theta1_js);
    theta2_error = abs(theta2_matlab - theta2_js);
    thetad1_error = abs(thetad1_matlab - thetad1_js);
    thetad2_error = abs(thetad2_matlab - thetad2_js);

    % Plot angles
    figure;
    subplot(2,1,1);
    plot(time, theta1_matlab, 'r', time, theta1_js, 'b--');
    xlabel('Time (s)'); ylabel('\theta_1 (rad)');
    legend('MATLAB', 'JavaScript');
    title('Comparison of \theta_1');

    subplot(2,1,2);
    plot(time, theta2_matlab, 'r', time, theta2_js, 'b--');
    xlabel('Time (s)'); ylabel('\theta_2 (rad)');
    legend('MATLAB', 'JavaScript');
    title('Comparison of \theta_2');

    % Plot velocities
    figure;
    subplot(2,1,1);
    plot(time, thetad1_matlab, 'r', time, thetad1_js, 'b--');
    xlabel('Time (s)'); ylabel('\thetȧ_1 (rad/s)');
    legend('MATLAB', 'JavaScript');
    title('Comparison of \thetȧ_1');

    subplot(2,1,2);
    plot(time, thetad2_matlab, 'r', time, thetad2_js, 'b--');
    xlabel('Time (s)'); ylabel('\thetȧ_2 (rad/s)');
    legend('MATLAB', 'JavaScript');
    title('Comparison of \thetȧ_2');

    % Plot errors
    figure;
    subplot(2,1,1);
    plot(time, theta1_error, 'r', time, theta2_error, 'b');
    xlabel('Time (s)'); ylabel('Absolute Error');
    legend('\theta_1 error', '\theta_2 error');
    title('Angle Errors (MATLAB vs JS)');

    subplot(2,1,2);
    plot(time, thetad1_error, 'r', time, thetad2_error, 'b');
    xlabel('Time (s)'); ylabel('Absolute Error');
    legend('\thetȧ_1 error', '\thetȧ_2 error');
    title('Velocity Errors (MATLAB vs JS)');

    disp('MATLAB vs JavaScript RK4 comparison complete.');
end

function dq = dynamics(q)
    theta1 = q(1); theta2 = q(2);
    thetad1 = q(3); thetad2 = q(4);
    
    % Define system equations
    M = [...  
        1000*cos(theta2) + 1500, 500*cos(theta2) + 250;
        500*cos(theta2) + 250, 250
    ];
    
    N = [...  
        -500*thetad2*sin(theta2), -500*sin(theta2)*(thetad1 + thetad2);
        500*thetad1*sin(theta2), 0
    ];
    
    F = [...  
        (981*cos(theta1 + theta2))/2 + (2943*cos(theta1))/2;
        (981*cos(theta1 + theta2))/2
    ];
    
    qdd = inv(M) * (F - N * [thetad1; thetad2]);
    
    dq = [thetad1; thetad2; qdd];
end
