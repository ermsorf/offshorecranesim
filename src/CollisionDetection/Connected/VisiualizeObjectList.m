% Code to visualize the list of objects for humans.
% 
% Not intended to run each time the program is, just to make it easier to validate that the obects are the correct size and location. 

function plotObjects(objects)
    figure;
    hold on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Object Visualization');
    grid on;
    
    % Loop through each object
    for i = 1:length(objects)
        obj = objects(i);
        
        if strcmp(obj.type, 'cube')
            position = obj.position;
            extents = obj.extents;
            rotations = obj.rotations; % Rotation angles [Rx, Ry, Rz]
            
            % Get the 8 vertices of the cube (local coordinates)
            vertices = [
                -1 -1 -1;  1 -1 -1;  1  1 -1; -1  1 -1;  % Bottom 4 vertices
                -1 -1  1;  1 -1  1;  1  1  1; -1  1  1   % Top 4 vertices
            ] .* (extents / 2); % Scale vertices
            
            % Convert rotation angles from degrees to radians
            rx = deg2rad(rotations(1));
            ry = deg2rad(rotations(2));
            rz = deg2rad(rotations(3));

            % Create rotation matrices
            Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
            Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
            Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];

            % Combine rotations (Z * Y * X)
            R = Rz * Ry * Rx;

            % Apply rotation
            rotated_vertices = (R * vertices')';

            % Translate vertices to final position
            translated_vertices = rotated_vertices + position;

            % Define the 6 faces of the cube
            faces = [
                1 2 3 4;  % Bottom
                5 6 7 8;  % Top
                1 2 6 5;  % Front
                3 4 8 7;  % Back
                1 4 8 5;  % Left
                2 3 7 6;  % Right
            ];

            % Plot the rotated cube
            patch('Vertices', translated_vertices, 'Faces', faces, ...
                'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'EdgeColor', 'black');

            % Add label at the center of the cube
            text(position(1), position(2), position(3), obj.name, ...
                'Color', 'red', 'FontSize', 10, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center');
        end
    end

    % Set the view to isometric
    view([1, 1, 1]); 

    hold off;
end
