% Run the main function
main();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function main()
    %% Define the dynamic (moving) box.
    movingBox.center = [1; 1; 3];
    movingBox.L = [1; 1; 1];
    % For example, rotate the moving box by 30° about the Z-axis.
    theta = deg2rad(30);
    movingBox.R = rotz(theta);
    movingBox.R = roty(theta);

    %% Define the stationary boxes (part of a larger structure).
    % Stationary Box 1
    stationaryBoxes(1).center = [3; 0; 1];
    stationaryBoxes(1).L = [1; 2; 1];
    stationaryBoxes(1).R = eye(3);
    
    % Stationary Box 2 (rotated about the Y-axis)
    stationaryBoxes(2).center = [-2; 3; 2];
    stationaryBoxes(2).L = [1.5; 1; 1];
    psi = deg2rad(45);
    stationaryBoxes(2).R = roty(psi);
    
    % Stationary Box 3 (rotated arbitrarily)
    stationaryBoxes(3).center = [0; -4; 0.5];
    stationaryBoxes(3).L = [0.8; 1.2; 1.5];
    phi = deg2rad(20);    % rotation about X-axis
    gamma = deg2rad(35);  % rotation about Z-axis
    stationaryBoxes(3).R = rotx(phi) * rotz(gamma);
    
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
    
    % Draw the stationary boxes (each with a distinct color).
    n = length(stationaryBoxes);
    colors = lines(n);
    for i = 1:n
        draw_OBB(stationaryBoxes(i).center, stationaryBoxes(i).R, stationaryBoxes(i).L, colors(i,:));
        draw_axes(stationaryBoxes(i).center, stationaryBoxes(i).R, stationaryBoxes(i).L, colors(i,:));
    end
    
    % Plot the computed contact (or closest-approach) points.
    for i = 1:n
        cp = collisions(i).contact_point;
        if collisions(i).collision
            marker = 'ko'; % collision point shown in black
            markerFace = 'k';
        else
            marker = 'mo'; % closest-approach point shown in magenta
            markerFace = 'm';
        end
        plot3(cp(1), cp(2), cp(3), marker, 'MarkerSize', 10, 'MarkerFaceColor', markerFace);
        text(cp(1), cp(2), cp(3), sprintf(' %d', i), 'FontSize',8, 'Color', markerFace);
    end
    
    % Adjust plot limits as needed.
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
    % Define the 6 faces.
    faces = [1 2 3 4;
             5 6 7 8;
             1 2 6 5;
             2 3 7 6;
             3 4 8 7;
             4 1 5 8];
    % Draw each face.
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
%% GJK + EPA Collision Detection Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [collision, contact_point, min_distance] = GJK_OBB(C1, R1, L1, C2, R2, L2)
    % Implements a GJK algorithm that, when a collision is detected, calls EPA
    % to obtain a more accurate collision (penetration) contact point.
    collision = false;
    contact_point = NaN(3,1);
    min_distance = NaN;
    
    % Nested support function: returns a point from the Minkowski difference
    % along with the corresponding witness points.
    function [v, p1, p2] = support(A, RA, LA, B, RB, LB, dir)
        p1 = get_farthest_point(A, RA, LA, dir);
        p2 = get_farthest_point(B, RB, LB, -dir);
        v = p1 - p2;
    end

    % Initialize the simplex (as an array of structures with fields v, p1, p2).
    dir = C1 - C2;
    if norm(dir) < 1e-6, dir = [1; 0; 0]; end
    [v0, p1_0, p2_0] = support(C1, R1, L1, C2, R2, L2, dir);
    simplex(1) = struct('v', v0, 'p1', p1_0, 'p2', p2_0);
    dir = -v0;
    
    maxGJKIter = 50;
    for iter = 1:maxGJKIter
        [v_new, p1_new, p2_new] = support(C1, R1, L1, C2, R2, L2, dir);
        if dot(v_new, dir) < 0
            % No collision: compute closest point on the current simplex.
            [closest, w1, w2] = closestPointOnSimplex(simplex);
            min_distance = norm(closest);
            contact_point = (w1 + w2) / 2;
            collision = false;
            return;
        end
        % Add new vertex to simplex.
        simplex = [struct('v', v_new, 'p1', p1_new, 'p2', p2_new), simplex];
        [collision, simplex, dir] = handle_simplex(simplex);
        if collision
            % Run EPA to refine the contact information.
            [~, ~, contact_point] = EPA(simplex, C1, R1, L1, C2, R2, L2);
            min_distance = 0;
            collision = true;
            return;
        end
    end
    
    % If max iterations reached without a collision decision:
    [closest, w1, w2] = closestPointOnSimplex(simplex);
    min_distance = norm(closest);
    contact_point = (w1 + w2) / 2;
end

function p = get_farthest_point(center, R, L, dir)
    % Compute the farthest point on an OBB (with given center, rotation R,
    % and half-lengths L) in the given direction.
    local_dir = R' * dir;
    p_local = sign(local_dir) .* L;
    p = R * p_local + center;
end

function [collision, simplex, dir] = handle_simplex(simplex)
    % Process the current simplex (handling the line and triangle cases)
    % and update the search direction.
    num_points = length(simplex);
    collision = false;
    dir = zeros(3,1);
    
    if num_points == 2
        % Line segment.
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
        % Triangle.
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
    % Given a simplex (with 1–3 points), compute the point on it closest to the
    % origin and return the corresponding witness points.
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
    % Compute the point on triangle ABC closest to the origin and its barycentrics.
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
%% EPA (Expanding Polytope Algorithm) Functions (Updated)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [penetration_depth, contact_normal, contact_point] = EPA(simplex, C1, R1, L1, C2, R2, L2)
    % EPA refines the collision (penetration) information after GJK
    tolerance = 1e-6;
    maxEPAIter = 50;
    
    % Ensure the simplex is a tetrahedron. If not, add one extra point.
    if length(simplex) < 4
        A = simplex(1).v;
        B = simplex(2).v;
        C = simplex(3).v;
        normal = cross(B-A, C-A);
        if norm(normal) < tolerance
            normal = [1;0;0];
        else
            normal = normal / norm(normal);
        end
        [v_new, p1_new, p2_new] = EPA_support(C1, R1, L1, C2, R2, L2, normal);
        simplex(end+1) = struct('v', v_new, 'p1', p1_new, 'p2', p2_new);
    end
    
    % Initialize the polytope from the (now 4-point) simplex.
    polytope = initializePolytope(simplex);
    
    % Main EPA loop.
    for iter = 1:maxEPAIter
        [minFaceIndex, minFace] = findClosestFace(polytope);
        [v_new, p1_new, p2_new] = EPA_support(C1, R1, L1, C2, R2, L2, minFace.normal);
        d_new = dot(v_new, minFace.normal);
        if (d_new - minFace.distance) < tolerance
            penetration_depth = d_new;
            contact_normal = minFace.normal;
            % Compute contact point using barycentrics on the face.
            A = simplex(minFace.idx(1)).v;
            B = simplex(minFace.idx(2)).v;
            C = simplex(minFace.idx(3)).v;
            [~, bary] = closestPointOnTriangle(A, B, C);
            p1A = simplex(minFace.idx(1)).p1;
            p1B = simplex(minFace.idx(2)).p1;
            p1C = simplex(minFace.idx(3)).p1;
            p2A = simplex(minFace.idx(1)).p2;
            p2B = simplex(minFace.idx(2)).p2;
            p2C = simplex(minFace.idx(3)).p2;
            w1 = bary(1)*p1A + bary(2)*p1B + bary(3)*p1C;
            w2 = bary(1)*p2A + bary(2)*p2B + bary(3)*p2C;
            contact_point = (w1 + w2)/2;
            return;
        end
        newIndex = length(simplex) + 1;
        simplex(newIndex) = struct('v', v_new, 'p1', p1_new, 'p2', p2_new);
        polytope = updatePolytope(polytope, simplex, newIndex, tolerance);
    end
    
    % If EPA did not converge, return the best available face.
    [~, minFace] = findClosestFace(polytope);
    penetration_depth = minFace.distance;
    contact_normal = minFace.normal;
    A = simplex(minFace.idx(1)).v;
    B = simplex(minFace.idx(2)).v;
    C = simplex(minFace.idx(3)).v;
    [~, bary] = closestPointOnTriangle(A, B, C);
    p1A = simplex(minFace.idx(1)).p1;
    p1B = simplex(minFace.idx(2)).p1;
    p1C = simplex(minFace.idx(3)).p1;
    p2A = simplex(minFace.idx(1)).p2;
    p2B = simplex(minFace.idx(2)).p2;
    p2C = simplex(minFace.idx(3)).p2;
    w1 = bary(1)*p1A + bary(2)*p1B + bary(3)*p1C;
    w2 = bary(1)*p2A + bary(2)*p2B + bary(3)*p2C;
    contact_point = (w1 + w2)/2;
end

function [v, p1, p2] = EPA_support(C1, R1, L1, C2, R2, L2, dir)
    % EPA support function (same as in GJK)
    p1 = get_farthest_point(C1, R1, L1, dir);
    p2 = get_farthest_point(C2, R2, L2, -dir);
    v = p1 - p2;
end

function polytope = initializePolytope(simplex)
    % Initialize the polytope (a set of faces) from a tetrahedron.
    polytope = struct('idx', {}, 'normal', {}, 'distance', {});
    polytope(end+1) = makeFace([1,2,3], simplex);
    polytope(end+1) = makeFace([1,4,2], simplex);
    polytope(end+1) = makeFace([1,3,4], simplex);
    polytope(end+1) = makeFace([2,4,3], simplex);
end

function face = makeFace(idxs, simplex)
    % Create a face from 3 vertex indices.
    A = simplex(idxs(1)).v;
    B = simplex(idxs(2)).v;
    C = simplex(idxs(3)).v;
    normal = cross(B-A, C-A);
    normal = normal / norm(normal);
    % Ensure the normal points outward.
    if dot(normal, A) < 0
        normal = -normal;
        idxs([2,3]) = idxs([3,2]);
    end
    face.idx = idxs;
    face.normal = normal;
    face.distance = dot(normal, A);
end

function [minIndex, minFace] = findClosestFace(polytope)
    % Find the face with the smallest distance from the origin.
    minDistance = inf;
    minIndex = -1;
    for i = 1:length(polytope)
        if polytope(i).distance < minDistance
            minDistance = polytope(i).distance;
            minIndex = i;
        end
    end
    minFace = polytope(minIndex);
end

function polytope = updatePolytope(polytope, simplex, newIndex, tolerance)
    % Update the polytope by removing faces visible from the new vertex
    % and re-triangulating the horizon.
    newVertex = simplex(newIndex).v;
    
    % Initialize newPolytope as an empty struct array with the correct fields.
    newPolytope = struct('idx', {}, 'normal', {}, 'distance', {});
    for i = 1:length(polytope)
        A = simplex(polytope(i).idx(1)).v;
        % Keep face if it is NOT visible from the new vertex.
        if dot(polytope(i).normal, newVertex - A) <= tolerance
            newPolytope(end+1) = polytope(i);  %#ok<AGROW>
        end
    end
    polytope = newPolytope;
    
    % Find horizon edges.
    horizon = [];
    for i = 1:length(polytope)
        face = polytope(i);
        edges = [face.idx([1,2]); face.idx([2,3]); face.idx([3,1])];
        for j = 1:size(edges,1)
            edge = edges(j,:);
            shared = false;
            for k = 1:length(polytope)
                if k == i, continue; end
                otherEdges = [polytope(k).idx([1,2]); polytope(k).idx([2,3]); polytope(k).idx([3,1])];
                for m = 1:size(otherEdges,1)
                    if isequal(edge, otherEdges(m,:)) || isequal(edge, fliplr(otherEdges(m,:)))
                        shared = true;
                        break;
                    end
                end
                if shared, break; end
            end
            if ~shared
                horizon = [horizon; edge];  %#ok<AGROW>
            end
        end
    end
    
    % Create new faces from each horizon edge.
    for i = 1:size(horizon,1)
        newFace = makeFace([horizon(i,1), horizon(i,2), newIndex], simplex);
        polytope(end+1) = newFace;
    end
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