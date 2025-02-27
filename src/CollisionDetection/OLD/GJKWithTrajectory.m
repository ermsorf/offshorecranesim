%% Run the trajectory-based main function
main_trajectory();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main Function (Trajectory-based Collision Checking with First Collision Reporting)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function main_trajectory()
    %% Define the dynamic (moving) box.
    movingBox.startCenter = [-1; -1; -3];   % Starting position
    movingBox.endCenter   = [-4; -4; -3];    % Ending position (change as needed)
    movingBox.center = movingBox.startCenter; % initial center
    movingBox.L = [1; 1; 1];
    % For example, rotate the moving box by 30° about the Z-axis.
    theta = deg2rad(30);
    movingBox.R = rotz(theta);
    % (If you wish to apply more rotations, adjust as needed.)
    
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
    
    %% Define the trajectory parameters.
    T = 3;               % total time in seconds for the trajectory
    dt = 1/30;           % time step (30 frames per second)
    timeSteps = 0:dt:T;  % array of time samples
    nSteps = length(timeSteps);
    
    % Preallocate for visualization and logging:
    trajectoryPositions = zeros(3, nSteps);
    collisionsOverTime = cell(nSteps,1);
    
    % Variables to record the first collision:
    firstCollisionFound = false;
    firstCollisionTime = NaN;
    firstCollisionBody = NaN;
    firstCollisionContactPoint = [];
    
    %% Loop over each time step to update the moving box and check collisions.
    for step = 1:nSteps
        t = timeSteps(step);
        % Linearly interpolate the center position along the trajectory.
        movingBox.center = movingBox.startCenter + (movingBox.endCenter - movingBox.startCenter) * (t/T);
        trajectoryPositions(:, step) = movingBox.center;
        
        % Initialize this step's collision array to have nBoxes entries.
        collisionsOverTime{step} = repmat(struct('collision', false, 'contact_point', NaN(3,1), 'min_distance', NaN), nBoxes, 1);
        
        foundCollisionInStep = false;
        for i = 1:nBoxes
            [collision, contact_point, min_distance] = GJK_OBB(...
                movingBox.center, movingBox.R, movingBox.L, ...
                stationaryBoxes(i).center, stationaryBoxes(i).R, stationaryBoxes(i).L);
            
            collisionsOverTime{step}(i).collision = collision;
            collisionsOverTime{step}(i).contact_point = contact_point;
            collisionsOverTime{step}(i).min_distance = min_distance;
            
            if collision
                fprintf('Time %.3f sec: Collision with Stationary Box %d at point: [%.3f, %.3f, %.3f]\n',...
                    t, i, contact_point(1), contact_point(2), contact_point(3));
                % Record the first collision if not already recorded.
                if ~firstCollisionFound
                    firstCollisionFound = true;
                    firstCollisionTime = t;
                    firstCollisionBody = i;
                    firstCollisionContactPoint = contact_point;
                end
                foundCollisionInStep = true;
                % Fill the remaining indices with default (non-collision) values.
                for j = i+1:nBoxes
                    collisionsOverTime{step}(j).collision = false;
                    collisionsOverTime{step}(j).contact_point = NaN(3,1);
                    collisionsOverTime{step}(j).min_distance = NaN;
                end
                break;  % exit the inner loop
            else
                fprintf('Time %.3f sec: No collision with Stationary Box %d. Min distance: %.3f at closest point: [%.3f, %.3f, %.3f]\n',...
                    t, i, min_distance, contact_point(1), contact_point(2), contact_point(3));
            end
        end
        
        % If a collision was found at this time step, stop checking further.
        if foundCollisionInStep
            break;
        end
    end
    
    % Use only the time steps that were actually processed.
    nStepsUsed = step;
    trajectoryPositions = trajectoryPositions(:, 1:nStepsUsed);
    collisionsOverTime = collisionsOverTime(1:nStepsUsed);
    
    % Report the first collision details to the user.
    if firstCollisionFound
        fprintf('\nFIRST COLLISION:\n');
        fprintf('Time: %.3f sec\n', firstCollisionTime);
        fprintf('Collided with Stationary Box %d\n', firstCollisionBody);
        fprintf('Collision point: [%.3f, %.3f, %.3f]\n\n',...
            firstCollisionContactPoint(1), firstCollisionContactPoint(2), firstCollisionContactPoint(3));
    else
        fprintf('\nNo collision occurred along the trajectory.\n');
    end
    
    %% Visualize the moving box trajectory, stationary boxes, and collision/closest points.
    visualize_trajectory(movingBox, stationaryBoxes, trajectoryPositions, collisionsOverTime);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualization Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function visualize_trajectory(movingBox, stationaryBoxes, trajectoryPositions, collisionsOverTime)
    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    
    % Draw the stationary boxes (each with a distinct color).
    n = length(stationaryBoxes);
    colors = lines(n);
    for i = 1:n
        draw_OBB(stationaryBoxes(i).center, stationaryBoxes(i).R, stationaryBoxes(i).L, colors(i,:));
        draw_axes(stationaryBoxes(i).center, stationaryBoxes(i).R, stationaryBoxes(i).L, colors(i,:));
    end
    
    % Plot the trajectory of the moving box.
    plot3(trajectoryPositions(1, :), trajectoryPositions(2, :), trajectoryPositions(3, :), 'k.-', 'MarkerSize', 15);
    
    % Optionally, draw the moving box at a few sample positions along the trajectory.
    sampleIndices = round(linspace(1, size(trajectoryPositions,2), 5));
    for idx = sampleIndices
        tempCenter = trajectoryPositions(:, idx);
        draw_OBB(tempCenter, movingBox.R, movingBox.L, [0 0 1]);
    end
    
    % Plot the collision contact points (red) or closest-approach points (magenta).
    for step = 1:length(collisionsOverTime)
        for i = 1:length(stationaryBoxes)
            if collisionsOverTime{step}(i).collision
                cp = collisionsOverTime{step}(i).contact_point;
                plot3(cp(1), cp(2), cp(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            else
                cp = collisionsOverTime{step}(i).contact_point;
                plot3(cp(1), cp(2), cp(3), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
            end
        end
    end
    
    % Adjust plot limits as needed.
    xlim([-8 8]); ylim([-8 8]); zlim([-8 8]);
    title('Trajectory Collision Detection: Moving Box vs. Stationary Boxes');
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
    % Scale, rotate, and translate the corners.
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
%% GJK + EPA Collision Detection Functions and Helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [collision, contact_point, min_distance] = GJK_OBB(C1, R1, L1, C2, R2, L2)
    collision = false;
    contact_point = NaN(3,1);
    min_distance = NaN;
    
    % Nested support function.
    function [v, p1, p2] = support(A, RA, LA, B, RB, LB, dir)
        p1 = get_farthest_point(A, RA, LA, dir);
        p2 = get_farthest_point(B, RB, LB, -dir);
        v = p1 - p2;
    end

    % Initialize the simplex.
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
    
    % If max iterations reached without a collision decision.
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

function [penetration_depth, contact_normal, contact_point] = EPA(simplex, C1, R1, L1, C2, R2, L2)
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
    
    polytope = initializePolytope(simplex);
    
    for iter = 1:maxEPAIter
        % Try to find the closest face.
        try
            [minFaceIndex, minFace] = findClosestFace(polytope);
        catch ME
            warning('EPA: No valid face found in polytope; returning fallback contact point.');
            [closest, w1, w2] = closestPointOnSimplex(simplex);
            penetration_depth = norm(closest);
            contact_point = (w1 + w2) / 2;
            contact_normal = [0; 0; 1]; % fallback normal
            return;
        end
        
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
        
        % Add new point to simplex and update polytope.
        newIndex = length(simplex) + 1;
        simplex(newIndex) = struct('v', v_new, 'p1', p1_new, 'p2', p2_new);
        polytope = updatePolytope(polytope, simplex, newIndex, tolerance);
    end
    
    % If maxEPAIter reached, try a final fallback.
    try
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
    catch ME
        warning('EPA did not converge; returning fallback contact point.');
        [closest, w1, w2] = closestPointOnSimplex(simplex);
        penetration_depth = norm(closest);
        contact_point = (w1 + w2) / 2;
        contact_normal = [0; 0; 1];
    end
end

function [v, p1, p2] = EPA_support(C1, R1, L1, C2, R2, L2, dir)
    p1 = get_farthest_point(C1, R1, L1, dir);
    p2 = get_farthest_point(C2, R2, L2, -dir);
    v = p1 - p2;
end

function polytope = initializePolytope(simplex)
    polytope = struct('idx', {}, 'normal', {}, 'distance', {});
    polytope(end+1) = makeFace([1,2,3], simplex);
    polytope(end+1) = makeFace([1,4,2], simplex);
    polytope(end+1) = makeFace([1,3,4], simplex);
    polytope(end+1) = makeFace([2,4,3], simplex);
end

function face = makeFace(idxs, simplex)
    A = simplex(idxs(1)).v;
    B = simplex(idxs(2)).v;
    C = simplex(idxs(3)).v;
    normal = cross(B-A, C-A);
    normal = normal / norm(normal);
    if dot(normal, A) < 0
        normal = -normal;
        idxs([2,3]) = idxs([3,2]);
    end
    face.idx = idxs;
    face.normal = normal;
    face.distance = dot(normal, A);
end

function [minIndex, minFace] = findClosestFace(polytope)
    % Initialize the minimum distance and index.
    minDistance = inf;
    minIndex = [];
    % Loop over each face in the polytope.
    for i = 1:length(polytope)
        currentDistance = polytope(i).distance;
        % Skip this face if the distance is NaN.
        if isnan(currentDistance)
            continue;
        end
        if currentDistance < minDistance
            minDistance = currentDistance;
            minIndex = i;
        end
    end
    % If no valid face was found, throw an error.
    if isempty(minIndex)
        error('No valid face found in polytope.');
    end
    minFace = polytope(minIndex);
end

function polytope = updatePolytope(polytope, simplex, newIndex, tolerance)
    newVertex = simplex(newIndex).v;
    newPolytope = struct('idx', {}, 'normal', {}, 'distance', {});
    for i = 1:length(polytope)
        A = simplex(polytope(i).idx(1)).v;
        if dot(polytope(i).normal, newVertex - A) <= tolerance
            newPolytope(end+1) = polytope(i);  %#ok<AGROW>
        end
    end
    polytope = newPolytope;
    
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
