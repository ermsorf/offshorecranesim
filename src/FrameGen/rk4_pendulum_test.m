function rk4_pendulum()
    % Time step and number of steps
    dt = 0.1;
    steps = 10;
    
    % Initial conditions [theta1, theta2, thetad1, thetad2]
    state = [1; 1; 0; 0];
    
    % Store results
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
    
    % Display results
    disp('Time     Theta1    Theta2    ThetaDot1  ThetaDot2');
    disp(results);
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
