%% Define Vertices for Each Object
% Cube: vertices for a cube centered at the origin (side length = 1)
cubeVertices = [ -0.5, -0.5, -0.5;
                  0.5, -0.5, -0.5;
                  0.5,  0.5, -0.5;
                 -0.5,  0.5, -0.5;
                 -0.5, -0.5,  0.5;
                  0.5, -0.5,  0.5;
                  0.5,  0.5,  0.5;
                 -0.5,  0.5,  0.5];

% Cylinder: approximated by two circles (top and bottom)
numPoints = 16;
theta = linspace(0, 2*pi, numPoints+1);
theta(end) = [];  % Remove duplicate endpoint
radius = 0.5;
z_bottom = -0.5;
z_top = 0.5;
circleBottom = [radius*cos(theta)', radius*sin(theta)', z_bottom*ones(numPoints,1)];
circleTop    = [radius*cos(theta)', radius*sin(theta)', z_top*ones(numPoints,1)];
cylVertices = [circleBottom; circleTop];

% Triangle: simple 3-vertex 3D shape lying in the XY plane (z=0)
triangleVertices = [0, 0, 0; 1, 0, 0; 0.5, 1, 0];

%% Create a Nested List (Cell Array) of Objects
% Each object is defined as: { type, position, rotation, scale, vertices }
objectList = {
    {'cube',        [0, 0, 0],     [0, 0, 0],       1, cubeVertices}, ...
    {'cylinder',    [-2, -2, 0],   [0, pi/4, 0],    1, cylVertices}, ...
};

%% Plot the 3D Objects from the Nested List
figure;
hold on;
axis equal;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Objects from a Nested List with Scaling');

for i = 1:length(objectList)
    obj = objectList{i};
    type = obj{1};         % e.g., 'cube'
    pos = obj{2};          % 3D position: [x, y, z]
    eul = obj{3};          % Euler angles: [alpha, beta, gamma]
    scale = obj{4};        % Scale factor (scalar)
    vertices = obj{5};     % Local vertices (each row is [x, y, z])
    
    % Compute the rotation matrix from Euler angles (ZYX order)
    R = eul2rotm(eul);
    
    % Apply scaling, then rotation, then translation
    transformedVertices = (R * (scale * vertices)')' + pos;
    
    % Compute the convex hull to generate faces for plotting
    K = convhulln(transformedVertices);
    
    % Plot the object as a patch with transparency
    patch('Vertices', transformedVertices, 'Faces', K, ...
          'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'EdgeColor', 'blue');
      
    % Optionally, plot the vertices as markers
    plot3(transformedVertices(:,1), transformedVertices(:,2), transformedVertices(:,3), ...
          'ko', 'MarkerFaceColor', 'red');
      
    % Label the object near its center
    text(pos(1), pos(2), pos(3), type, 'FontSize', 12, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center');
end

hold off;

%% Define Helper Function for Euler to Rotation Matrix Conversion (ZYX Order)
function R = eul2rotm(eul)
    % eul: [alpha, beta, gamma] in radians.
    % Rotations are applied in ZYX order: R = Rz * Ry * Rx
    alpha = eul(1);
    beta  = eul(2);
    gamma = eul(3);
    
    R_x = [1, 0, 0;
           0, cos(alpha), -sin(alpha);
           0, sin(alpha), cos(alpha)];
       
    R_y = [cos(beta), 0, sin(beta);
           0, 1, 0;
           -sin(beta), 0, cos(beta)];
       
    R_z = [cos(gamma), -sin(gamma), 0;
           sin(gamma), cos(gamma), 0;
           0, 0, 1];
    
    R = R_z * R_y * R_x;
end