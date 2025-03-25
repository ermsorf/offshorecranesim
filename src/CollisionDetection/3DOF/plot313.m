%% Define the cube geometry (side = 5, centered at the bob)
cube_side = 5;
half_side = cube_side/2;
% Define cube vertices (8 vertices)
V_cube = [ half_side,  half_side,  half_side;
          -half_side,  half_side,  half_side;
          -half_side, -half_side,  half_side;
           half_side, -half_side,  half_side;
           half_side,  half_side, -half_side;
          -half_side,  half_side, -half_side;
          -half_side, -half_side, -half_side;
           half_side, -half_side, -half_side];
% Define cube faces (each row indexes vertices forming a face)
faces = [1 2 3 4;  % front face
         5 6 7 8;  % back face
         1 5 8 4;  % right face
         2 6 7 3;  % left face
         1 2 6 5;  % top face
         4 3 7 8]; % bottom face

%% Create a figure with four subplots: Top, Side, Front, and Isometric views.
figure;

% --- Top View ---
subplot(2,2,1);
h_line1 = line([0 x(1)], [0 y(1)], [0 z(1)], 'LineWidth', 2);
hold on;
h_bob1  = plot3(x(1), y(1), z(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% Compute initial cube transform using 3–1–3 Euler angles:
% R1 = Y(1,3), R2 = Y(1,5), R3 = Y(1,1)
Rz1 = [cos(Y(1,3)) -sin(Y(1,3)) 0; sin(Y(1,3)) cos(Y(1,3)) 0; 0 0 1];
% Use rotation about Y for R2:
Ry = [cos(Y(1,5)) 0 sin(Y(1,5)); 0 1 0; -sin(Y(1,5)) 0 cos(Y(1,5))];
Rz2 = [cos(Y(1,1)) -sin(Y(1,1)) 0; sin(Y(1,1)) cos(Y(1,1)) 0; 0 0 1];
R = Rz1 * Ry * Rz2;
V_rot = (R * V_cube')';
V_init = V_rot + [x(1), y(1), z(1)];
h_cube1 = patch('Vertices', V_init, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3);
grid on; axis equal;
xlim([-40 40]); ylim([-40 40]); zlim([-40 0]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Top View');
view(0,90);

% --- Side View ---
subplot(2,2,2);
h_line2 = line([0 x(1)], [0 y(1)], [0 z(1)], 'LineWidth', 2);
hold on;
h_bob2  = plot3(x(1), y(1), z(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
h_cube2 = patch('Vertices', V_init, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3);
grid on; axis equal;
xlim([-40 40]); ylim([-40 40]); zlim([-40 0]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Side View');
view(90,0);

% --- Front View ---
subplot(2,2,3);
h_line3 = line([0 x(1)], [0 y(1)], [0 z(1)], 'LineWidth', 2);
hold on;
h_bob3  = plot3(x(1), y(1), z(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
h_cube3 = patch('Vertices', V_init, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3);
grid on; axis equal;
xlim([-40 40]); ylim([-40 40]); zlim([-40 0]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Front View');
view(0,0);

% --- Isometric View ---
subplot(2,2,4);
h_line4 = line([0 x(1)], [0 y(1)], [0 z(1)], 'LineWidth', 2);
hold on;
h_bob4  = plot3(x(1), y(1), z(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
h_cube4 = patch('Vertices', V_init, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.3);
grid on; axis equal;
xlim([-40 40]); ylim([-40 40]); zlim([-40 0]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Isometric View');
view(45,45);

%% Animate the pendulum and cube in all subplots
for k = 1:length(t)
    % Update rod and bob in each subplot
    currPos = [x(k), y(k), z(k)];
    
    % Top View
    subplot(2,2,1);
    set(h_line1, 'XData', [0, x(k)], 'YData', [0, y(k)], 'ZData', [0, z(k)]);
    set(h_bob1, 'XData', x(k), 'YData', y(k), 'ZData', z(k));
    
    % Side View
    subplot(2,2,2);
    set(h_line2, 'XData', [0, x(k)], 'YData', [0, y(k)], 'ZData', [0, z(k)]);
    set(h_bob2, 'XData', x(k), 'YData', y(k), 'ZData', z(k));
    
    % Front View
    subplot(2,2,3);
    set(h_line3, 'XData', [0, x(k)], 'YData', [0, y(k)], 'ZData', [0, z(k)]);
    set(h_bob3, 'XData', x(k), 'YData', y(k), 'ZData', z(k));
    
    % Isometric View
    subplot(2,2,4);
    set(h_line4, 'XData', [0, x(k)], 'YData', [0, y(k)], 'ZData', [0, z(k)]);
    set(h_bob4, 'XData', x(k), 'YData', y(k), 'ZData', z(k));
    
    % --- Update the Cube ---
    % Extract Euler angles according to the 3–1–3 convention:
    % R1 = Y(k,3), R2 = Y(k,5), R3 = Y(k,1)
    R1 = Y(k,3);
    R2   = -Y(k,5);
    R3       = Y(k,1);
    
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
    R = Rz1 * Ry * Rz2;
    
    % Rotate cube vertices and translate to the bob position
    V_rot = (R * V_cube')';
    V_trans = V_rot + repmat(currPos, size(V_cube,1), 1);
    
    % Update cube patch in all subplots
    subplot(2,2,1); set(h_cube1, 'Vertices', V_trans);
    subplot(2,2,2); set(h_cube2, 'Vertices', V_trans);
    subplot(2,2,3); set(h_cube3, 'Vertices', V_trans);
    subplot(2,2,4); set(h_cube4, 'Vertices', V_trans);
    
    drawnow;
    pause(0.10);  % adjust pause for desired animation speed
end
