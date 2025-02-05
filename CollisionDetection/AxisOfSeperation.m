
% Run the main function
main();

function main()
    % Define Cylinder (position, axis, height, radius)
    C_cyl = [1, 6, 0]'; % Cylinder center
    R_cyl = [0; 0; 1];  % Cylinder axis (aligned with Z-axis)
    H_cyl = 3;          % Cylinder height
    R_cyl_radius = 1;   % Cylinder radius

    % Define OBB1 (position, size, and orientation)
    C1 = [0, 0, 0]';   % Center of OBB1
    L1 = [1, 2, 1]';   % Half-lengths (X, Y, Z)
    R1 = eye(3);       % No rotation (Identity matrix)

    % Define OBB2 (position, size, and orientation)
    C2 = [3.1, 0, 0]'; % Center of OBB2
    L2 = [1, 2, 1]';   % Half-lengths
    theta = pi/4;      % Rotation angle around Z-axis
    R2 = [cos(theta), -sin(theta), 0; 
          sin(theta), cos(theta), 0; 
          0, 0, 1];    % Rotation matrix

    % Compute collision point
    [collision, collision_point] = SAT_OBB_With_Contact(C1, R1, L1, C2, R2, L2);
    
    % Display result
    if collision
        fprintf('Collision detected squares at: [%.3f, %.3f, %.3f]\n', collision_point(1), collision_point(2), collision_point(3));
    else
        disp('No collision.');
    end

    % Visualize the OBBs and collision point
    visualize_OBBs(C1, R1, L1, C2, R2, L2, collision_point);

    % Compute collision point
    %[collision, collision_point] = SAT_Cylinder_OBB(C_cyl, R_cyl, H_cyl, R_cyl_radius, C1, R1, L1);
    
    % Display result
    %if collision
    %    fprintf('Collision detected cylinder at: [%.3f, %.3f, %.3f]\n', collision_point(1), collision_point(2), collision_point(3));
    %else
    %    disp('No collision.');
    %end

    % Visualize the OBB and Cylinder with collision point
    %visualize_Cylinder_OBB(C1, R1, L1, C_cyl, R_cyl, H_cyl, R_cyl_radius, collision_point);
end

function visualize_OBBs(C1, R1, L1, C2, R2, L2, collision_point)
    % Visualize two OBBs in 3D with axes and collision point
    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    
    % Draw OBB1
    draw_OBB(C1, R1, L1, 'r');
    draw_axes(C1, R1, L1);
    
    % Draw OBB2
    draw_OBB(C2, R2, L2, 'b');
    draw_axes(C2, R2, L2);
    
    % Plot collision point if collision occurred
    if ~isnan(collision_point(1))
        plot3(collision_point(1), collision_point(2), collision_point(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    end
    
    % Set plot limits
    xlim([-5 5]); ylim([-5 5]); zlim([-5 5]);
    
    title('3D Visualization of Oriented Bounding Boxes (OBBs) with Axes and Collision Point');
end

function draw_OBB(center, R, half_lengths, color)
    % Define the 8 corner points of a unit cube
    corners = [
        -1 -1 -1;
         1 -1 -1;
         1  1 -1;
        -1  1 -1;
        -1 -1  1;
         1 -1  1;
         1  1  1;
        -1  1  1
    ];
    
    % Scale the corners by the half-lengths
    corners = corners .* half_lengths';
    
    % Rotate and translate the corners
    for i = 1:8
        corners(i, :) = (R * corners(i, :)')' + center';
    end
    
    % Define faces of the OBB
    faces = [
        1 2 3 4;
        5 6 7 8;
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8
    ];
    
    % Draw the OBB using patch
    for i = 1:6
        patch('Vertices', corners, 'Faces', faces(i,:), 'FaceColor', color, 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    end
end

function draw_axes(center, R, half_lengths)
    % Draw the local coordinate axes of the OBB
    axes_colors = ['r', 'g', 'b'];
    for i = 1:3
        axis_end = center + R(:,i) * half_lengths(i) * 1.5;
        plot3([center(1), axis_end(1)], [center(2), axis_end(2)], [center(3), axis_end(3)], axes_colors(i), 'LineWidth', 2);
    end
end

function [collision, collision_point] = SAT_OBB_With_Contact(C1, R1, L1, C2, R2, L2)
    % Detects a collision between two OBBs using SAT.
    % Computes the collision point from the surface, not the center.
    
    collision = false;
    collision_point = NaN(3,1);
    
    % Compute translation vector from OBB1 to OBB2
    T = C2 - C1;
    
    % Compute rotation matrix from OBB2's local space to OBB1
    R = R1' * R2;
    
    % Compute absolute rotation matrix to avoid floating point errors
    absR = abs(R) + 1e-6;
    
    % Half-lengths of both OBBs
    A = L1;
    B = L2;
    min_overlap = Inf;
    best_axis = NaN(3,1);

    % Test the 3 axes of OBB1
    for i = 1:3
        ra = A(i);
        rb = B(1) * absR(i,1) + B(2) * absR(i,2) + B(3) * absR(i,3);
        overlap = (ra + rb) - abs(T(i));
        if overlap < 0
            return;
        elseif overlap < min_overlap
            min_overlap = overlap;
            best_axis = R1(:,i);
        end
    end
    
    % Test the 3 axes of OBB2
    for i = 1:3
        ra = A(1) * absR(1,i) + A(2) * absR(2,i) + A(3) * absR(3,i);
        rb = B(i);
        overlap = (ra + rb) - abs(dot(T, R(:,i)));
        if overlap < 0
            return;
        elseif overlap < min_overlap
            min_overlap = overlap;
            best_axis = R2(:,i);
        end
    end
    
    % If no separating axis found, collision exists
    collision = true;
    
    % Compute collision point from the surface, not the CM
    closest_point_on_OBB1 = C1 + best_axis * min_overlap * 0.5;
    closest_point_on_OBB2 = C2 - best_axis * min_overlap * 0.5;
    collision_point = (closest_point_on_OBB1 + closest_point_on_OBB2) / 2;
end
function [collision, collision_point] = SAT_Cylinder_OBB(C_cyl, R_cyl, H, R, C_obb, R_obb, L_obb)
    % Detects a collision between a cylinder and an OBB using SAT.
    % Computes the collision point from the surface, not the center.
    
    collision = false;
    collision_point = NaN(3,1);
    
    % Compute translation vector from OBB to Cylinder
    T = C_obb - C_cyl;
    
    % Compute the rotation matrix from the OBB to cylinder frame
    R_rel = R_cyl' * R_obb;
    
    % Compute absolute rotation matrix
    absR = abs(R_rel) + 1e-6;
    
    % Cylinder axis (primary axis to check)
    cylinder_axes = [R_cyl]; 
    
    % OBB axes (the 3 primary axes of the box)
    obb_axes = [R_obb(:,1), R_obb(:,2), R_obb(:,3)];
    
    % Check for separation along cylinder axis
    ra = H / 2; % Half-height of the cylinder
    rb = sum(absR * L_obb);
    if abs(dot(T, R_cyl)) > (ra + rb)
        return; % No collision
    end
    
    % Check for separation along OBB axes
    for i = 1:3
        ra = L_obb(i);
        rb = abs(dot(R_cyl, obb_axes(:,i))) * H/2 + R;
        if abs(dot(T, obb_axes(:,i))) > (ra + rb)
            return; % No collision
        end
    end
    
    % Check for separation along cross-product axes (Edge-Edge)
    for i = 1:3
        axis = cross(R_cyl, obb_axes(:,i));
        axis = axis / norm(axis + 1e-6); % Normalize
        
        ra = R;
        rb = L_obb(i);
        
        if abs(dot(T, axis)) > (ra + rb)
            return; % No collision
        end
    end
    
    % If no separating axis found, collision exists
    collision = true;
    
    % Compute collision point from the surface, not the CM
    closest_point_on_cylinder = C_cyl + R_cyl * min(H/2, max(-H/2, dot(T, R_cyl)));
    normal_to_surface = (C_obb - closest_point_on_cylinder);
    normal_to_surface = normal_to_surface / norm(normal_to_surface + 1e-6);
    collision_point = closest_point_on_cylinder + normal_to_surface * R;
end

function visualize_Cylinder_OBB(C1, R1, L1, C_cyl, R_cyl, H, R, collision_point)
    % Visualize the OBB and Cylinder with collision point
    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    
    % Draw OBB
    draw_OBB(C1, R1, L1, 'r');
    draw_axes(C1, R1, L1);
    
    % Draw Cylinder
    draw_Cylinder(C_cyl, R_cyl, H, R, 'b');
    
    % Plot collision point if collision occurred
    if ~isnan(collision_point(1))
        plot3(collision_point(1), collision_point(2), collision_point(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    end
    
    title('3D Visualization of OBB and Cylinder with Collision Point');
end

function draw_Cylinder(center, axis, height, radius, color)
    % Draw a cylinder using surf
    [X, Y, Z] = cylinder(radius, 20);
    Z = Z * height - height / 2;
    
    % Rotate and translate cylinder
    [theta, phi, ~] = cart2sph(axis(1), axis(2), axis(3));
    R = makehgtform('zrotate', theta, 'yrotate', -phi);
    new_coords = R(1:3,1:3) * [X(:)'; Y(:)'; Z(:)'];
    X = reshape(new_coords(1, :), size(X)) + center(1);
    Y = reshape(new_coords(2, :), size(Y)) + center(2);
    Z = reshape(new_coords(3, :), size(Z)) + center(3);
    
    surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end