function minDistancesStruct = GJKAlgorithm(stationaryObjects, loadObj, pivot, CartesianCoordinatesAndRotation)
    % GJKAlgorithm computes the minimum distance between a moving load and
    % each stationary object (cubes) over a trajectory.
    %
    % Inputs:
    %   stationaryObjects - struct array with fields: name, type, position, extents, rotations.
    %                       (For now, all are cubes.)
    %   loadObj           - struct for the moving load.
    %                       For a sphere: loadObj.type = 'sphere' and loadObj.extents is its radius (scalar).
    %                       For a cylinder: loadObj.type = 'cylinder' and loadObj.extents = [radius, height].
    %   pivot             - 1x3 vector [x, y, z] representing the pivot point.
    %   CartesianCoordinatesAndRotation - N×7 matrix where each row is:
    %           [time, X, Y, Z, rot1, rot2, rot3]
    %           The translation [X Y Z] is relative to the pivot.
    %
    % Output:
    %   minDistancesStruct - struct array (one element per obstacle) with fields:
    %           .objectName   - Name of the obstacle.
    %           .minDistance  - Minimum distance encountered.
    %           .time         - Time at which that minimum occurred.
    %           .contactPoint - Computed contact (or closest) point.
    %           .collided     - Boolean flag; true if a collision occurred.
    
    nSteps = size(CartesianCoordinatesAndRotation, 1);
    nObs = length(stationaryObjects);
    
    globalMinDist = inf(nObs, 1);
    globalTime = nan(nObs, 1);
    globalContact = cell(nObs, 1);
    globalCollided = false(nObs, 1);
    
    % Open a figure for debugging the geometry.
    figure(999); 
    % For stationary objects, plot their vertices once.
    clf; hold on; grid on; axis equal;
    title('Stationary Object Vertices');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    for j = 1:nObs
        obj = stationaryObjects(j);
        objVertices = boxTransform(obj);
        scatter3(objVertices(:,1), objVertices(:,2), objVertices(:,3), 36, 'r', 'filled');
    end

    % Loop through each time step in the trajectory.
    for step = 1:nSteps
        curr = CartesianCoordinatesAndRotation(step, :);
        t = curr(1);
        offset = curr(2:4);      % translation offset relative to pivot
        rotations = curr(5:7);   % Euler angles in radians
        
        % Update load global state.
        loadObj.center = pivot + offset;
        loadObj.rotations = rotations;
        
        % Choose the correct transformation for the load based on its type.
        switch lower(loadObj.type)
            case 'sphere'
                loadVertices = sphereTransform(loadObj);
            case 'cylinder'
                loadVertices = cylinderTransform(loadObj);
            case 'cube'
                loadObj.extents = reshape(loadObj.extents, 1, []);
                loadVertices = boxTransform(loadObj);
            otherwise
                error('Unknown load type: %s', loadObj.type);
        end
        
        % (Optional debugging: plot the load vertices)
        scatter3(loadVertices(:,1), loadVertices(:,2), loadVertices(:,3), 20, 'b', 'filled');
        
        % Loop through each stationary object.
        for j = 1:nObs
            obj = stationaryObjects(j);
            % For now, stationary objects are cubes.
            objVertices = boxTransform(obj);
            
            % Run the generic GJK algorithm on the two convex sets.
            [collision, contactPoint, dist] = GJK_generic(loadVertices, objVertices);
            
            if collision
                dist = 0;
                if ~globalCollided(j)
                    globalCollided(j) = true;
                    globalTime(j) = t;
                    globalContact{j} = contactPoint;
                    globalMinDist(j) = 0;
                end
            else
                if ~globalCollided(j) && (dist < globalMinDist(j))
                    globalMinDist(j) = dist;
                    globalTime(j) = t;
                    globalContact{j} = contactPoint;
                end
            end
        end
    end
    
    % Build the output struct.
    minDistancesStruct = struct('objectName', {}, 'minDistance', {}, 'time', {}, 'contactPoint', {}, 'collided', {});
    for j = 1:nObs
        minDistancesStruct(j).objectName = stationaryObjects(j).name;
        minDistancesStruct(j).minDistance = globalMinDist(j);
        minDistancesStruct(j).time = globalTime(j);
        minDistancesStruct(j).contactPoint = globalContact{j};
        minDistancesStruct(j).collided = globalCollided(j);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function vertices_world = sphereTransform(sphereObj)
    % sphereTransform computes vertices for a sphere load in world coordinates.
    % sphereObj.extents is its radius (scalar).
    vertices = sphereVertices(sphereObj.extents);
    R = eul2rotm(sphereObj.rotations);
    vertices_world = (R * vertices')' + sphereObj.center;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function vertices = sphereVertices(r)
    [X, Y, Z] = sphere(10);  % Generates an 11x11 grid.
    vertices = [X(:), Y(:), Z(:)] * r;
    vertices = unique(vertices, 'rows');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function vertices_world = cylinderTransform(cylObj)
    % cylinderTransform computes vertices for a cylinder load in world coordinates.
    % cylObj.extents is [radius, height].
    if numel(cylObj.extents) ~= 2
        error('For a cylinder, extents must be a 1x2 vector: [radius, height].');
    end
    vertices = cylinderVertices(cylObj.extents);
    R = eul2rotm(cylObj.rotations);
    vertices_world = (R * vertices')' + cylObj.center;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function vertices = cylinderVertices(extents)
    % cylinderVertices generates a set of sample points on a cylinder.
    % extents is [radius, height].
    r = extents(1);
    h = extents(2);
    n = 20;  % number of points along the circumference
    [X, Y, Z] = cylinder(r, n);
    % Cylinder returns X,Y,Z matrices of size 2 x (n+1) with Z values 0 and 1.
    % Center vertically and scale the height.
    Z = (Z - 0.5) * h;
    vertices = [X(:), Y(:), Z(:)];
    vertices = unique(vertices, 'rows');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function vertices_world = boxTransform(obj)
    % boxTransform computes vertices for a box (cube) in world coordinates.
    % obj.extents should be a 1x3 vector (full dimensions).
    if isfield(obj, 'center')
        pos = obj.center;
    else
        pos = obj.position;
    end
    obj.extents = reshape(obj.extents, 1, []);
    if numel(obj.extents) ~= 3
        error('Extents must be a 1x3 vector.');
    end
    vertices = boxVertices(obj.extents);
    R = eul2rotm(obj.rotations);
    vertices_world = (R * vertices')' + pos;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function vertices = boxVertices(extents)
    extents = reshape(extents, 1, []);
    if numel(extents) ~= 3
        error('Extents must be a 1x3 vector.');
    end
    halfDims = extents / 2;
    vertices = [ -halfDims(1), -halfDims(2), -halfDims(3);
                  halfDims(1), -halfDims(2), -halfDims(3);
                  halfDims(1),  halfDims(2), -halfDims(3);
                 -halfDims(1),  halfDims(2), -halfDims(3);
                 -halfDims(1), -halfDims(2),  halfDims(3);
                  halfDims(1), -halfDims(2),  halfDims(3);
                  halfDims(1),  halfDims(2),  halfDims(3);
                 -halfDims(1),  halfDims(2),  halfDims(3)];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R = eul2rotm(eul)
    % eul2rotm converts Euler angles [r1, r2, r3] (radians) into a rotation matrix.
    alpha = eul(1); beta = eul(2); gamma = eul(3);
    R_x = [1, 0, 0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
    R_y = [cos(beta), 0, sin(beta); 0, 1, 0; -sin(beta), 0, cos(beta)];
    R_z = [cos(gamma), -sin(gamma), 0; sin(gamma), cos(gamma), 0; 0, 0, 1];
    R = R_z * R_y * R_x;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [collision, contact_point, min_distance] = GJK_generic(verticesA, verticesB)
    % GJK_generic performs the GJK algorithm on two convex sets given by their vertices.
    d = (mean(verticesA,1) - mean(verticesB,1))';
    if norm(d) < 1e-6, d = [1; 0; 0]; end
    pA = support_generic(verticesA, d);
    pB = support_generic(verticesB, -d);
    p = pA - pB;
    simplex = struct('v', p, 'p1', pA, 'p2', pB);
    d = -p;
    maxIter = 50;
    for iter = 1:maxIter
        pA = support_generic(verticesA, d);
        pB = support_generic(verticesB, -d);
        p = pA - pB;
        if dot(p, d) < 0
            collision = false;
            [closest, ~] = closestPointSimplex_generic(simplex);
            min_distance = norm(closest);
            contact_point = closest;
            return;
        end
        simplex = [struct('v', p, 'p1', pA, 'p2', pB), simplex];
        [containsOrigin, simplex, d] = handle_simplex_generic(simplex);
        if containsOrigin
            collision = true;
            min_distance = 0;
            contact_point = p;
            return;
        end
    end
    collision = false;
    [closest, ~] = closestPointSimplex_generic(simplex);
    min_distance = norm(closest);
    contact_point = closest;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p = support_generic(vertices, d)
    projections = vertices * d;
    [~, idx] = max(projections);
    p = vertices(idx,:)';
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [containsOrigin, simplex, d] = handle_simplex_generic(simplex)
    if numel(simplex) == 1
        containsOrigin = false;
        d = -simplex(1).v;
    elseif numel(simplex) == 2
        A = simplex(1).v; B = simplex(2).v;
        AB = B - A;
        AO = -A;
        if dot(AB, AO) > 0
            d = tripleCross(AB, AO, AB);
        else
            simplex = simplex(1);
            d = AO;
        end
        containsOrigin = false;
    else % 3 points
        A = simplex(1).v; B = simplex(2).v; C = simplex(3).v;
        AO = -A;
        AB = B - A;
        AC = C - A;
        ABC = cross(AB, AC);
        if dot(cross(ABC, AC), AO) > 0
            simplex = [simplex(1), simplex(3)];
            d = tripleCross(AC, AO, AC);
        elseif dot(cross(AB, ABC), AO) > 0
            simplex = [simplex(1), simplex(2)];
            d = tripleCross(AB, AO, AB);
        else
            containsOrigin = true;
            d = zeros(3,1);
            return;
        end
        containsOrigin = false;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [closest, bary] = closestPointSimplex_generic(simplex)
    closest = simplex(1).v;
    bary = 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function result = tripleCross(a, b, c)
    result = cross(a, cross(b, c));
end
