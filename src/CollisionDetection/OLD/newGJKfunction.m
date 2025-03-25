% Run the main function
clear
clc
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Define the dynamic (moving) box using a struct.
movingObj = DefineMovingObject();
movingBox = objectsToBoxes(movingObj);  % Convert to box format (fields: center, L, R)

%% Load the stationary boxes from a struct.
objects = ListOfObjects();
stationaryBoxes = objectsToBoxes(objects);
nBoxes = length(stationaryBoxes);

%% Test collision between the moving box and each stationary box.
collisions = struct([]);
for i = 1:nBoxes
    [collision, contact_point, min_distance] = GJK_OBB(...
        movingBox.center, movingBox.R, movingBox.L, ...
        stationaryBoxes(i).center, stationaryBoxes(i).R, stationaryBoxes(i).L);
    collisions(i).collision = collision;
    collisions(i).contact_point = contact_point;
    collisions(i).min_distance = min_distance;
    
    if collision
        fprintf('Collision with Stationary Box %d at point: [%.3f, %.3f, %.3f]\n',...
            i, contact_point(1), contact_point(2), contact_point(3));
    else
        fprintf('No collision with Stationary Box %d. Min distance: %.3f at closest point: [%.3f, %.3f, %.3f]\n',...
            i, min_distance, contact_point(1), contact_point(2), contact_point(3));
    end
end

%% Visualize the moving box, stationary boxes, and contact/closest points.
visualize_boxes(movingBox, stationaryBoxes, collisions);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define Moving Object Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function movingObj = DefineMovingObject()
    % Define the moving object as a cube with the same struct format as the static objects.
    % Fields: 'name', 'type', 'position', 'extents', 'rotations'
    % Rotations are given as [yaw, pitch, roll] in degrees.
    movingObj = struct( ...
        'name', 'MovingBox', ...
        'type', 'cube', ...
        'position', [-2.8, 1, 1], ...        % Center of the moving box.
        'extents', [1.2, 1.2, 1.2], ...     % Full extents of the cube.
        'rotations', [10, 0, 0] ...         % Example: yaw=30°, pitch=0, roll=0.
    );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ListOfObjects Function (Static Boxes)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function objects = ListOfObjects()
    % Define static boxes as objects with fields:
    % 'name', 'type', 'position', 'extents', and 'rotations'
    % Rotations are given as [yaw, pitch, roll] in degrees.
    
    % Box 1: No rotation.
    objects(1) = struct( ...
        'name', 'Box1', ...
        'type', 'cube', ...
        'position', [-4, 0, 1], ...
        'extents', [1, 2, 1], ...
        'rotations', [0, 0, 0] );
    
    % Box 2: Rotated 45° about the Y-axis.
    objects(2) = struct( ...
        'name', 'Box2', ...
        'type', 'cube', ...
        'position', [-2, 3, 2], ...
        'extents', [1.5, 1, 1], ...
        'rotations', [0, 0, 0] );
    
    % Box 3: A combination rotation.
    objects(3) = struct( ...
        'name', 'Box3', ...
        'type', 'cube', ...
        'position', [0, -4, 0.5], ...
        'extents', [0.8, 1.2, 1.5], ...
        'rotations', [0, 0, 0] );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Conversion: Objects to Boxes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function boxes = objectsToBoxes(objects)
    n = length(objects);
    boxes = repmat(struct('center', [], 'L', [], 'R', []), n, 1);
    for i = 1:n
        % Use the object's position as the box center (as a column vector).
        boxes(i).center = objects(i).position(:);
        % Divide full extents by 2 to obtain half-lengths.
        boxes(i).L = objects(i).extents(:) / 2;
        % Convert rotations (in degrees) to radians.
        yaw   = deg2rad(objects(i).rotations(1));
        pitch = deg2rad(objects(i).rotations(2));
        roll  = deg2rad(objects(i).rotations(3));
        % Compute the rotation matrix: R = rotz(yaw) * roty(pitch) * rotx(roll).
        boxes(i).R = rotz(yaw) * roty(pitch) * rotx(roll);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualization Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function visualize_boxes(movingBox, stationaryBoxes, collisions)
    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    
    % Draw the moving box (blue).
    draw_OBB(movingBox.center, movingBox.R, movingBox.L, [0 0 1]);
    draw_axes(movingBox.center, movingBox.R, movingBox.L, [0 0 1]);
    
    % Draw stationary boxes (each with a distinct color).
    n = length(stationaryBoxes);
    colors = lines(n);
    for i = 1:n
        draw_OBB(stationaryBoxes(i).center, stationaryBoxes(i).R, stationaryBoxes(i).L, colors(i,:));
        draw_axes(stationaryBoxes(i).center, stationaryBoxes(i).R, stationaryBoxes(i).L, colors(i,:));
    end
    
    % Plot computed contact (or closest-approach) points.
    for i = 1:n
        cp = collisions(i).contact_point;
        if collisions(i).collision
            marker = 'ko'; % collision point in black
            markerFace = 'k';
        else
            marker = 'mo'; % non-collision point in magenta
            markerFace = 'm';
        end
        plot3(cp(1), cp(2), cp(3), marker, 'MarkerSize', 10, 'MarkerFaceColor', markerFace);
        text(cp(1), cp(2), cp(3), sprintf(' %d', i), 'FontSize',8, 'Color', markerFace);
    end
    
    xlim([-8 8]); ylim([-8 8]); zlim([-8 8]);
    title('Collision Detection: Moving Box vs. Stationary Boxes');
end

function draw_OBB(center, R, half_lengths, color)
    % Define the 8 corners of a unit cube.
    corners = [ -1 -1 -1;
                 1 -1 -1;
                 1  1 -1;
                -1  1 -1;
                -1 -1  1;
                 1 -1  1;
                 1  1  1;
                -1  1  1];
    % Scale, rotate, and translate.
    corners = corners .* half_lengths';
    for i = 1:8
        corners(i,:) = (R * corners(i,:)')' + center';
    end
    faces = [1 2 3 4;
             5 6 7 8;
             1 2 6 5;
             2 3 7 6;
             3 4 8 7;
             4 1 5 8];
    for i = 1:size(faces,1)
        patch('Vertices', corners, 'Faces', faces(i,:), ...
              'FaceColor', color, 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    end
end

function draw_axes(center, R, half_lengths, color)
    axis_scale = 1.5;
    for i = 1:3
        endpoint = center + R(:,i) * half_lengths(i) * axis_scale;
        plot3([center(1) endpoint(1)], [center(2) endpoint(2)], [center(3) endpoint(3)],...
            'Color', color, 'LineWidth', 2);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% GJK Collision Detection Function (No EPA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [collision, contact_point, min_distance] = GJK_OBB(C1, R1, L1, C2, R2, L2)
    % This function uses only GJK to determine collision.
    % It returns collision=true if the origin is inside the Minkowski difference.
    % For non-collision, it returns the closest distance and point.
    
    collision = false;
    contact_point = NaN(3,1);
    min_distance = NaN;
    
    tol = 1e-4;  % termination tolerance
    
    % Nested support function: computes support point of Minkowski difference.
    function [v, p1, p2] = support(A, RA, LA, B, RB, LB, dir)
        p1 = get_farthest_point(A, RA, LA, dir);
        p2 = get_farthest_point(B, RB, LB, -dir);
        v = p1 - p2;
    end

    dir = C1 - C2;
    if norm(dir) < tol, dir = [1; 0; 0]; end
    [v0, p1_0, p2_0] = support(C1, R1, L1, C2, R2, L2, dir);
    simplex(1) = struct('v', v0, 'p1', p1_0, 'p2', p2_0);
    dir = -v0;
    
    maxGJKIter = 50;
    for iter = 1:maxGJKIter
        [v_new, p1_new, p2_new] = support(C1, R1, L1, C2, R2, L2, dir);
        if dot(v_new, dir) < tol
            % No collision: use the current simplex to compute the closest point.
            [closest, w1, w2] = closestPointOnSimplex(simplex);
            min_distance = norm(closest);
            contact_point = (w1 + w2) / 2;
            collision = false;
            return;
        end
        simplex = [struct('v', v_new, 'p1', p1_new, 'p2', p2_new), simplex];
        [collision, simplex, dir] = handle_simplex(simplex);
        if collision
            % Collision detected: approximate contact point from simplex.
            [closest, w1, w2] = closestPointOnSimplex(simplex);
            contact_point = (w1 + w2) / 2;
            min_distance = 0;
            return;
        end
    end
    
    [closest, w1, w2] = closestPointOnSimplex(simplex);
    min_distance = norm(closest);
    contact_point = (w1 + w2) / 2;
end

function p = get_farthest_point(center, R, L, dir)
    local_dir = R' * dir;
    p_local = sign(local_dir) .* L;
    p = R * p_local + center;
end

function [collision, simplex, dir] = handle_simplex(simplex)
    num_points = length(simplex);
    collision = false;
    dir = zeros(3,1);
    
    if num_points == 2
        A = simplex(1).v;
        B = simplex(2).v;
        AB = B - A;
        AO = -A;
        if dot(AB, AO) > 0
            dir = cross(cross(AB, AO), AB);
        else
            simplex = simplex(1);
            dir = AO;
        end
    elseif num_points == 3
        A = simplex(1).v;
        B = simplex(2).v;
        C = simplex(3).v;
        AB = B - A;
        AC = C - A;
        AO = -A;
        ABC = cross(AB, AC);
        if dot(cross(ABC, AC), AO) > 0
            simplex = [simplex(1), simplex(3)];  % drop B
            dir = cross(cross(AC, AO), AC);
        elseif dot(cross(AB, ABC), AO) > 0
            simplex = [simplex(1), simplex(2)];  % drop C
            dir = cross(cross(AB, AO), AB);
        else
            collision = true;
        end
    end
end

function [closest, w1, w2] = closestPointOnSimplex(simplex)
    n = length(simplex);
    if n == 1
        closest = simplex(1).v;
        w1 = simplex(1).p1;
        w2 = simplex(1).p2;
    elseif n == 2
        A = simplex(1).v;
        B = simplex(2).v;
        AB = B - A;
        t = -dot(A, AB) / dot(AB, AB);
        t = max(0, min(1, t));
        closest = A + t * AB;
        w1 = simplex(1).p1 + t * (simplex(2).p1 - simplex(1).p1);
        w2 = simplex(1).p2 + t * (simplex(2).p2 - simplex(1).p2);
    elseif n == 3
        A = simplex(1).v;
        B = simplex(2).v;
        C = simplex(3).v;
        [closest, bary] = closestPointOnTriangle(A, B, C);
        w1 = bary(1)*simplex(1).p1 + bary(2)*simplex(2).p1 + bary(3)*simplex(3).p1;
        w2 = bary(1)*simplex(1).p2 + bary(2)*simplex(2).p2 + bary(3)*simplex(3).p2;
    else
        closest = simplex(1).v;
        w1 = simplex(1).p1;
        w2 = simplex(1).p2;
    end
end

function [closest, bary] = closestPointOnTriangle(A, B, C)
    AB = B - A;
    AC = C - A;
    AO = -A;
    
    d1 = dot(AB, AO);
    d2 = dot(AC, AO);
    if d1 <= 0 && d2 <= 0
        closest = A;
        bary = [1, 0, 0];
        return;
    end
    
    BO = -B;
    d3 = dot(AB, BO);
    d4 = dot(AC, BO);
    if d3 >= 0 && d4 <= d3
        closest = B;
        bary = [0, 1, 0];
        return;
    end
    
    CO = -C;
    d5 = dot(AB, CO);
    d6 = dot(AC, CO);
    if d6 >= 0 && d5 <= d6
        closest = C;
        bary = [0, 0, 1];
        return;
    end
    
    vc = d1*d4 - d3*d2;
    if vc <= 0 && d1 >= 0 && d3 <= 0
        v = d1 / (d1 - d3);
        closest = A + v * AB;
        bary = [1 - v, v, 0];
        return;
    end
    
    vb = d2*d5 - d1*d6;
    if vb <= 0 && d2 >= 0 && d6 <= 0
        w = d2 / (d2 - d6);
        closest = A + w * AC;
        bary = [1 - w, 0, w];
        return;
    end
    
    va = d3*d6 - d5*d4;
    if va <= 0 && (d4-d3) >= 0 && (d5-d6) >= 0
        w = (d4-d3) / ((d4-d3) + (d5-d6));
        closest = B + w * (C - B);
        bary = [0, 1-w, w];
        return;
    end
    
    denom = 1 / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    u = 1 - v - w;
    closest = u*A + v*B + w*C;
    bary = [u, v, w];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rotation Helper Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R = rotx(angle)
    R = [1,         0,          0;
         0, cos(angle), -sin(angle);
         0, sin(angle),  cos(angle)];
end

function R = roty(angle)
    R = [ cos(angle), 0, sin(angle);
                0,    1,     0;
         -sin(angle), 0, cos(angle)];
end

function R = rotz(angle)
    R = [cos(angle), -sin(angle), 0;
         sin(angle),  cos(angle), 0;
         0,               0,      1];
end
