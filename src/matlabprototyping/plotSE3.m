function plotSE3(T, scale, color)
    % Plots an SE(3) frame in 3D space
    % Inputs:
    %   T     - 4x4 SE(3) transformation matrix
    %   scale - Length of the axes for visualization
    %   color - (Optional) Color for the frame ('r', 'g', 'b', etc.)
    
    if nargin < 2
        scale = 1; % Default scale
    end
    if nargin < 3
        color = ['r', 'g', 'b']; % Default RGB colors
    end
    
    % Extract position (translation)
    origin = T(1:3, 4);
    
    % Extract rotation axes
    x_axis = T(1:3, 1) * scale;
    y_axis = T(1:3, 2) * scale;
    z_axis = T(1:3, 3) * scale;
    
    % Plot frame axes using quiver3
    hold on;
    quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), color(1), 'LineWidth', 2);
    quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), color(2), 'LineWidth', 2);
    quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), color(3), 'LineWidth', 2);
    
    % Plot origin
    plot3(origin(1), origin(2), origin(3), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
    
    % Labels
    text(origin(1) + x_axis(1), origin(2) + x_axis(2), origin(3) + x_axis(3), 'X', 'Color', 'r');
    text(origin(1) + y_axis(1), origin(2) + y_axis(2), origin(3) + y_axis(3), 'Y', 'Color', 'g');
    text(origin(1) + z_axis(1), origin(2) + z_axis(2), origin(3) + z_axis(3), 'Z', 'Color', 'b');
    
    axis equal;
    grid on;
end
