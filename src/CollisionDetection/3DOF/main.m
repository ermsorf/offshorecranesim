clear
clc
close all;

%% Initial condtitions in the form of [theta, thetadot, phi, phidot, psi, psidot, l, pivot point ([x,y,z])]
% Theta is inital 
initialConditions = []

initialConditions.pivot = [1,2,35];
initialConditions.length = [30];
initialConditions.theta = [0.3];
initialConditions.thetadot = [0.2];
initialConditions.phi = [0.2];
initialConditions.phidot = [0.2];
initialConditions.psi = [0.2];
initialConditions.psidot = [0.2];

pendulumPointsAndRotations = Solve313(initialConditions);


disp(pendulumPointsAndRotations);

% figure;
% plot3(pendulumPointsAndRotations.x, pendulumPointsAndRotations.y, pendulumPointsAndRotations.z, '-o');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Pendulum Cartesian Coordinates');
% grid on;

% collisions = collisionAlgorithmx(pendulumPointsAndRotations);

collisionResults = ComputeCollisionsForPendulumLoad(pendulumPointsAndRotations);