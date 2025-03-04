%% Run the trajectory-based main function
main_trajectory();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main Function (Trajectory-based Collision Checking with First Collision Reporting)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function main_trajectory()
    %% Define the dynamic (moving) box.
    movingBox.startCenter = [-1; -1; -3];   % Starting position
    movingBox.endCenter   = [4; 4; 3];        % Ending position
    movingBox.center = movingBox.startCenter; % initial center
    movingBox.L = [1; 1; 1];                  % half-dimensions (as a column vector)
    % For example, rotate the moving box by 30° about the Z-axis.
    theta = deg2rad(30);
    movingBox.R = rotz(theta);
    
    %% Define the obstacles as a nested list.
    % Each obstacle is defined as: { type, position, rotation, scale, vertices }
    % Here we convert the three stationary boxes from your original script.
    
    % Obstacle 1 (originally Stationary Box 1)
    % center: [2.7654; 0; 0.324], L: [1; 2; 1], R: eye(3)
    obs1_type = 'Box1';
    obs1_pos = [2.7654, 0, 0.324];
    obs1_rot = [0, 0, 0];    % no rotation
    obs1_scale = 1;          % scale of 1 (the half-dimensions are built into vertices)
    obs1_vertices = boxVertices([1,2,1]);  % box vertices for half-dimensions [1,2,1]
    obstacle1 = {obs1_type, obs1_pos, obs1_rot, obs1_scale, obs1_vertices};
    
    % Obstacle 2 (originally Stationary Box 2)
    % center: [-2; 3; 2], L: [1.5; 1; 1], rotated about Y-axis by 45°
    obs2_type = 'Box2';
    obs2_pos = [-2, 3, 2];
    obs2_rot = [0, deg2rad(45), 0];  % rotation about Y-axis
    obs2_scale = 1;
    obs2_vertices = boxVertices([1.5,1,1]);
    obstacle2 = {obs2_type, obs2_pos, obs2_rot, obs2_scale, obs2_vertices};
    
    % Obstacle 3 (originally Stationary Box 3)
    % center: [0; -4; 0.5], L: [0.8; 1.2; 1.5], rotated about X (20°) and Z (35°)
    obs3_type = 'Box3';
    obs3_pos = [0, -4, 0.5];
    obs3_rot = [deg2rad(20), 0, deg2rad(35)];  % here we use [phi, 0, gamma]
    obs3_scale = 1;
    obs3_vertices = boxVertices([0.8,1.2,1.5]);
    obstacle3 = {obs3_type, obs3_pos, obs3_rot, obs3_scale, obs3_vertices};
    
    % Combine obstacles into a nested list.
    objectList = {obstacle1, obstacle2, obstacle3};
    nObstacles = length(objectList);
    
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
    firstCollisionIdx = NaN;
    firstCollisionContactPoint = [];
    
    %% Loop over each time step to update the moving box and check collisions.
    for step = 1:nSteps
        t = timeSteps(step);
        % Linearly interpolate the center position along the trajectory.
        movingBox.center = movingBox.startCenter + (movingBox.endCenter - movingBox.startCenter) * (t/T);
        trajectoryPositions(:, step) = movingBox.center;
        
        % Initialize this step's collision array (one entry per obstacle).
        collisionsOverTime{step} = repmat(struct('collision', false, 'contact_point', NaN(3,1), 'min_distance', NaN), nObstacles, 1);
        
        % Transform the moving box into world-space vertices.
        mv_transformed = transformMovingBox(movingBox);
        
        foundCollisionInStep = false;
        for i = 1:nObstacles
            obs = objectList{i};
            obs_transformed = transformObject(obs);
            
            [collision, contact_point, min_distance] = GJK_OBB(...
                mv_transformed, obs_transformed);
            
            collisionsOverTime{step}(i).collision = collision;
            collisionsOverTime{step}(i).contact_point = contact_point;
            collisionsOverTime{step}(i).min_distance = min_distance;
            
            if collision
                fprintf('Time %.3f sec: Collision with Obstacle %d at point: [%.3f, %.3f, %.3f]\n',...
                    t, i, contact_point(1), contact_point(2), contact_point(3));
                if ~firstCollisionFound
                    firstCollisionFound = true;
                    firstCollisionTime = t;
                    firstCollisionIdx = i;
                    firstCollisionContactPoint = contact_point;
                end
                foundCollisionInStep = true;
                % (Optional: fill remaining entries with defaults)
                for j = i+1:nObstacles
                    collisionsOverTime{step}(j).collision = false;
                    collisionsOverTime{step}(j).contact_point = NaN(3,1);
                    collisionsOverTime{step}(j).min_distance = NaN;
                end
                break;  % exit inner loop upon collision
            else
                fprintf('Time %.3f sec: No collision with Obstacle %d. Min distance: %.3f at closest point: [%.3f, %.3f, %.3f]\n',...
                    t, i, min_distance, contact_point(1), contact_point(2), contact_point(3));
            end
        end
        
        if foundCollisionInStep
            break; % stop processing further time steps if a collision occurred
        end
    end
    
    % Use only the time steps that were actually processed.
    nStepsUsed = step;
    trajectoryPositions = trajectoryPositions(:, 1:nStepsUsed);
    collisionsOverTime = collisionsOverTime(1:nStepsUsed);
    
    % Report the first collision details.
    if firstCollisionFound
        fprintf('\nFIRST COLLISION:\n');
        fprintf('Time: %.3f sec\n', firstCollisionTime);
        fprintf('Collided with Obstacle %d\n', firstCollisionIdx);
        fprintf('Collision point: [%.3f, %.3f, %.3f]\n\n',...
            firstCollisionContactPoint(1), firstCollisionContactPoint(2), firstCollisionContactPoint(3));
    else
        fprintf('\nNo collision occurred along the trajectory.\n');
    end
    
    %% Animate the moving box along the trajectory.
    animate_trajectory(movingBox, objectList, trajectoryPositions, collisionsOverTime);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Animation Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function animate_trajectory(movingBox, objectList, trajectoryPositions, collisionsOverTime)
    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    
    % Draw the obstacles (static).
    nObs = length(objectList);
    colors = lines(nObs);
    for i = 1:nObs
        obs = objectList{i};
        obs_transformed = transformObject(obs);
        K = convhulln(obs_transformed);
        patch('Vertices', obs_transformed, 'Faces', K, ...
              'FaceColor', colors(i,:), 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    end
    
    % Pre-plot an empty trajectory line.
    hTrajectory = plot3([], [], [], 'k.-', 'MarkerSize', 15);
    
    % Animate the moving box along its trajectory.
    nSteps = size(trajectoryPositions, 2);
    hMoving = []; % handle for the moving box patch
    for step = 1:nSteps
        currentCenter = trajectoryPositions(:, step);
        set(hTrajectory, 'XData', trajectoryPositions(1,1:step), ...
                         'YData', trajectoryPositions(2,1:step), ...
                         'ZData', trajectoryPositions(3,1:step));
        if ~isempty(hMoving)
            delete(hMoving);
        end
        mv_transformed = transformMovingBox(movingBox);
        K = convhulln(mv_transformed);
        hMoving = patch('Vertices', mv_transformed, 'Faces', K, ...
                        'FaceColor', [0 0 1], 'FaceAlpha', 0.3, 'EdgeColor', 'k');
        drawnow;
        pause(0.05);
    end
    
    % Optionally, mark collision contact points.
    for step = 1:length(collisionsOverTime)
        for i = 1:length(objectList)
            if collisionsOverTime{step}(i).collision
                cp = collisionsOverTime{step}(i).contact_point;
                plot3(cp(1), cp(2), cp(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            else
                cp = collisionsOverTime{step}(i).contact_point;
                plot3(cp(1), cp(2), cp(3), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helper Function: Convert Moving Box Structure to World Vertices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vertices_world = transformMovingBox(movingBox)
    % Convert the moving box (a structure with center, L, and R) into world-space vertices.
    vertices = boxVertices(movingBox.L');  % Note: L is a column vector, so transpose it
    vertices_world = (movingBox.R * vertices')' + movingBox.center';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helper Function: Transform Object from Nested List to World Vertices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vertices_world = transformObject(obj)
    % obj format: { type, position, rotation, scale, vertices }
    pos = obj{2};       % 1x3 position vector
    eul = obj{3};       % 1x3 Euler angles [alpha, beta, gamma]
    scale = obj{4};     % scalar scale factor
    vertices = obj{5};  % local vertices (each row is [x, y, z])
    R = eul2rotm(eul);
    % Apply scaling, then rotation, then translation.
    vertices_world = (R * (scale * vertices)')' + pos;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Euler Angles to Rotation Matrix (ZYX Order)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function R = eul2rotm(eul)
    % Converts Euler angles [alpha, beta, gamma] (radians) to a rotation matrix.
    alpha = eul(1); beta = eul(2); gamma = eul(3);
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Box Vertices Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vertices = boxVertices(L)
    % Returns the 8 vertices of a box centered at the origin given half-dimensions L.
    % L is a 1x3 vector.
    vertices = [ -L(1), -L(2), -L(3);
                  L(1), -L(2), -L(3);
                  L(1),  L(2), -L(3);
                 -L(1),  L(2), -L(3);
                 -L(1), -L(2),  L(3);
                  L(1), -L(2),  L(3);
                  L(1),  L(2),  L(3);
                 -L(1),  L(2),  L(3)];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Drawing and Rotation Helper Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function h = draw_OBB(center, R, half_lengths, color)
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
    % Draw the box and return a handle.
    h = patch('Vertices', corners, 'Faces', faces, 'FaceColor', color, 'FaceAlpha', 0.3, 'EdgeColor', 'k');
end

function draw_axes(center, R, half_lengths, color)
    axis_scale = 1.5;
    for i = 1:3
        endpoint = center + R(:,i) * half_lengths(i) * axis_scale;
        plot3([center(1) endpoint(1)], [center(2) endpoint(2)], [center(3) endpoint(3)],...
            'Color', color, 'LineWidth', 2);
    end
end

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% GJK + EPA Collision Detection Functions and Helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [collision, contact_point, min_distance] = GJK_OBB(verticesA, verticesB)
    collision = false;
    contact_point = NaN(3,1);
    min_distance = NaN;
    
    % Nested support function.
    function [v, p1, p2] = support(A, B, dir)
        p1 = get_farthest_point(A, dir);
        p2 = get_farthest_point(B, -dir);
        v = p1 - p2;
    end

    % Initialize the simplex.
    dir = mean(verticesA,1)' - mean(verticesB,1)';
    if norm(dir) < 1e-6, dir = [1; 0; 0]; end
    [v0, p1_0, p2_0] = support(verticesA, verticesB, dir);
    simplex(1) = struct('v', v0, 'p1', p1_0, 'p2', p2_0);
    dir = -v0;
    
    maxGJKIter = 50;
    for iter = 1:maxGJKIter
        [v_new, p1_new, p2_new] = support(verticesA, verticesB, dir);
        if dot(v_new, dir) < 0
            [closest, w1, w2] = closestPointOnSimplex(simplex);
            min_distance = norm(closest);
            contact_point = (w1 + w2) / 2;
            collision = false;
            return;
        end
        simplex = [struct('v', v_new, 'p1', p1_new, 'p2', p2_new), simplex];
        [collision, simplex, dir] = handle_simplex(simplex);
        if collision
            [~, ~, contact_point] = EPA(simplex, verticesA, verticesB);
            min_distance = 0;
            collision = true;
            return;
        end
    end
    
    [closest, w1, w2] = closestPointOnSimplex(simplex);
    min_distance = norm(closest);
    contact_point = (w1 + w2) / 2;
end

function p = get_farthest_point(vertices, dir)
    projections = vertices * dir;
    [~, idx] = max(projections);
    p = vertices(idx,:)';
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
            simplex = [simplex(1), simplex(3)];
            dir = cross(cross(AC, AO), AC);
        elseif dot(cross(AB, ABC), AO) > 0
            simplex = [simplex(1), simplex(2)];
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

function [penetration_depth, contact_normal, contact_point] = EPA(simplex, verticesA, verticesB)
    tolerance = 1e-6;
    maxEPAIter = 50;
    
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
        [v_new, p1_new, p2_new] = EPA_support(verticesA, verticesB, normal);
        simplex(end+1) = struct('v', v_new, 'p1', p1_new, 'p2', p2_new);
    end
    
    polytope = initializePolytope(simplex);
    
    for iter = 1:maxEPAIter
        try
            [minFaceIndex, minFace] = findClosestFace(polytope);
        catch
            warning('EPA: No valid face found in polytope; returning fallback contact point.');
            [closest, w1, w2] = closestPointOnSimplex(simplex);
            penetration_depth = norm(closest);
            contact_point = (w1 + w2) / 2;
            contact_normal = [0; 0; 1];
            return;
        end
        
        [v_new, p1_new, p2_new] = EPA_support(verticesA, verticesB, minFace.normal);
        d_new = dot(v_new, minFace.normal);
        if (d_new - minFace.distance) < tolerance
            penetration_depth = d_new;
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
            return;
        end
        
        newIndex = length(simplex) + 1;
        simplex(newIndex) = struct('v', v_new, 'p1', p1_new, 'p2', p2_new);
        polytope = updatePolytope(polytope, simplex, newIndex, tolerance);
    end
    
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
    catch
        warning('EPA did not converge; returning fallback contact point.');
        [closest, w1, w2] = closestPointOnSimplex(simplex);
        penetration_depth = norm(closest);
        contact_point = (w1 + w2) / 2;
        contact_normal = [0; 0; 1];
    end
end

function [v, p1, p2] = EPA_support(verticesA, verticesB, dir)
    p1 = get_farthest_point(verticesA, dir);
    p2 = get_farthest_point(verticesB, -dir);
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
    minDistance = inf;
    minIndex = [];
    for i = 1:length(polytope)
        currentDistance = polytope(i).distance;
        if isnan(currentDistance)
            continue;
        end
        if currentDistance < minDistance
            minDistance = currentDistance;
            minIndex = i;
        end
    end
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
            newPolytope(end+1) = polytope(i);
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
                horizon = [horizon; edge];
            end
        end
    end
    
    for i = 1:size(horizon,1)
        newFace = makeFace([horizon(i,1), horizon(i,2), newIndex], simplex);
        polytope(end+1) = newFace;
    end
end
