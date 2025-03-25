function pendulumPointsAndRotations = Solve313(initialConditions)

tic
%% Define symbolic functions and parameters
syms theta1(t) theta2(t) theta3(t)

%% Set parameters and initial conditions
% Y0: [theta3, theta3dot, theta1, theta1dot, theta2, theta2dot], wierd
% order as matlab is funcy
Y0 = [initialConditions.psi, initialConditions.psidot, initialConditions.theta, initialConditions.thetadot, initialConditions.phi, initialConditions.phidot];  % example initial conditions

% Define derivatives
thetadot1  = diff(theta1, t);
thetadot2  = diff(theta2, t);
thetadot3  = diff(theta3, t);
thetaddot1 = diff(theta1, t, 2);
thetaddot2 = diff(theta2, t, 2);
thetaddot3 = diff(theta3, t, 2);

g = 9.81;
l = initialConditions.length;

%% Write the equations in the form (LHS - RHS == 0)
% Equation 1:
eq1 = 6250603*thetaddot1 + 750000*l^2*thetaddot1 + 6250000*thetaddot3*cos(theta2) + 3000*l*thetaddot1*cos(2*theta2) + 6*thetadot1*thetadot2*sin(2*theta2) + 1500000*l^2*thetadot1*thetadot2*sin(2*theta2) == 3*thetaddot1*cos(2*theta2) + 3000*l*thetaddot1 + 6250000*thetadot2*thetadot3*sin(theta2) + 750000*l^2*thetaddot1*cos(2*theta2) + 6000*l*thetadot1*thetadot2*sin(2*theta2);

% Equation 2:
eq2 = 1500000*thetaddot2*l^2 + 3000*sin(2*theta2)*l*thetadot1^2 + 1500000*g*sin(theta2)*l + 6250000*thetadot3*sin(theta2)*thetadot1 + 6250306*thetaddot2 == 750000*sin(2*theta2)*l^2*thetadot1^2 + 6000*thetaddot2*l + 3*sin(2*theta2)*thetadot1^2 + 4500*g*sin(theta2);

% Equation 3:
eq3 = thetaddot3 + thetaddot1*cos(theta2) == thetadot1*thetadot2*sin(theta2);

toc
tic
tic

%% Convert the system to first-order form
% Note: The order is now Y = [theta3, theta3dot, theta1, theta1dot, theta2, theta2dot]
[V,S] = odeToVectorField(eq1, eq2, eq3);
% disp(S)  % Uncomment to inspect the variable mapping
toc
tic

%% Create a MATLAB function handle from the vector field
M = matlabFunction(V, 'vars', {'t', 'Y', 'l', 'g'});

toc
tic

%% Solve the system using ode45
tspan = linspace(0, 10, 100);;  % simulation time interval
[t, YGeneralized] = ode45(@(t,Y) M(t, Y, l, g), tspan, Y0);
toc


tic
%% Convert to cartesian cordinates and create the rotation matrix

x = l * sin(YGeneralized(:,5)) .* cos(YGeneralized(:,3));
y = l * sin(YGeneralized(:,5)) .* sin(YGeneralized(:,3));
z = -l * cos(YGeneralized(:,5));


steps = length(t);
R_all = zeros(3,3,steps);

for k = 1:steps;



    R1 = YGeneralized(k,3);
    R2   = -YGeneralized(k,5);
    R3       = YGeneralized(k,1);
    
    % Build rotation matrices for 3–1–3 Euler angles:
    Rz1 = [cos(R1) -sin(R1) 0; 
           sin(R1) cos(R1) 0; 
           0 0 1];
    Ry = [cos(R2) 0 sin(R2);
          0 1 0;
         -sin(R2) 0 cos(R2)];
    Rz2 = [cos(R3) -sin(R3) 0;
           sin(R3) cos(R3) 0;
           0 0 1];
    R_all(:,:,k) = Rz1 * Ry * Rz2;

end 


pendulumPointsAndRotations = [];
pendulumPointsAndRotations.xPendulumFromPivot = x;
pendulumPointsAndRotations.yPendulumFromPivot = y;
pendulumPointsAndRotations.zPendulumFromPivot = z;
pendulumPointsAndRotations.R = R_all;
pendulumPointsAndRotations.t = t;
pendulumPointsAndRotations.pivot = initialConditions.pivot;


%% Plot the time responses
figure;
subplot(3,1,1)
plot(t, YGeneralized(:,3))
xlabel('Time (s)')
ylabel('\theta_1 (R1)')
title('R1 vs Time')

subplot(3,1,2)
plot(t, YGeneralized(:,5))
xlabel('Time (s)')
ylabel('\theta_2 (R2)')
title('R2 vs Time')

subplot(3,1,3)
plot(t, YGeneralized(:,1))
xlabel('Time (s)')
ylabel('\theta_3 (R3)')
title('R3 vs Time')

end