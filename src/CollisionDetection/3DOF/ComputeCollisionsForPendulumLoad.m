function allResults = ComputeCollisionsForPendulumLoad(pendulumPointsAndRotations)
% ComputeCollisionsForPendulumLoad
%   Computes collisions (or minimum distances) between a dynamic load (modeled
%   as a spherical pendulum load) and a set of stationary objects. It also
%   animates the scene in an isometric 3D view.
%
%   This function loads the load object from DefineLoad.m and the stationary
%   objects from ListOfObjects.m.
%
% Input:
%   pendulumPointsAndRotations - a struct with fields:
%       xPendulumFromPivot  : [n x 1] bob x-coordinates (relative to pivot)
%       yPendulumFromPivot  : [n x 1] bob y-coordinates (relative to pivot)
%       zPendulumFromPivot  : [n x 1] bob z-coordinates (relative to pivot)
%       R                   : [3 x 3 x n] rotation matrices (one per time step)
%       t                   : [n x 1] time vector
%       pivot               : [1 x 3] pivot coordinates (global)
%
% Output:
%   allResults - a struct array (nSteps×1) where each element contains:
%       time       : simulation time
%       collisions : a struct array with collision info for each stationary object

    %% Load the dynamic load (from external DefineLoad.m)
    baseLoad = DefineLoad();

    %% Number of time steps.
    nSteps = length(pendulumPointsAndRotations.t);
    
    %% Build dynamic load states.
    % For each time step, update the load's center and rotation using the 
    % provided pendulum data (adding the pivot to get the global position).
    loadStates = repmat(baseLoad, nSteps, 1);
    for i = 1:nSteps
        globalCenter = [ pendulumPointsAndRotations.xPendulumFromPivot(i) + pendulumPointsAndRotations.pivot(1), ...
                         pendulumPointsAndRotations.yPendulumFromPivot(i) + pendulumPointsAndRotations.pivot(2), ...
                         pendulumPointsAndRotations.zPendulumFromPivot(i) + pendulumPointsAndRotations.pivot(3) ];
        loadStates(i).center = globalCenter;
        loadStates(i).rotation = pendulumPointsAndRotations.R(:,:,i);
        loadStates(i).time = pendulumPointsAndRotations.t(i);
    end
    
    %% Load stationary objects (from external ListOfObjects.m)
    objectsArray = ListOfObjects();
    nObjects = numel(objectsArray);
    stationaryObjects = cell(nObjects, 1);
    for i = 1:nObjects
        obj = objectsArray(i);
        % (The objects from ListOfObjects already contain the fields:
        %  name, type, center, extents, and rotation (as a 1x3 vector of angles).
        %  We ensure the rotation is converted to a 3x3 matrix.)
        obj = ensureRotationMatrix(obj);
        stationaryObjects{i} = obj;
    end

    %% Collision Detection Loop.
    allResults = repmat(struct('time', 0, 'collisions', []), nSteps, 1);
    for tIdx = 1:nSteps
        currentLoad = loadStates(tIdx);
        timeResults = [];
        for i = 1:nObjects
            obj = stationaryObjects{i};
            [collision, contactPoint] = detectCollision(currentLoad, obj);
            if collision
                result = struct('objectName', obj.name, 'collision', true, ...
                    'collisionPoint', contactPoint, 'minDistance', 0, 'witnessPoints', []);
            else
                [minDist, cp1, cp2] = computeDistance(currentLoad, obj);
                result = struct('objectName', obj.name, 'collision', false, ...
                    'collisionPoint', [], 'minDistance', minDist, 'witnessPoints', [cp1; cp2]);
            end
            timeResults = [timeResults; result];  %#ok<AGROW>
        end
        allResults(tIdx).time = currentLoad.time;
        allResults(tIdx).collisions = timeResults;
    end

    %% Visualization in ISO 3D View.
    figure;
    for tIdx = 1:nSteps
        clf;  % clear current figure frame
        hold on;
        % Plot each stationary object in green.
        for i = 1:nObjects
            plotShape(stationaryObjects{i}, 'g');
        end
        % Plot the current load state in blue.
        plotShape(loadStates(tIdx), 'b');
        
        % Plot collision indicators.
        currentResults = allResults(tIdx).collisions;
        for j = 1:length(currentResults)
            res = currentResults(j);
            if res.collision
                % Collision: plot a small red sphere at the contact point.
                [sx, sy, sz] = sphere(20);
                sphereRadius = 0.2;
                surf(sphereRadius*sx + res.collisionPoint(1), ...
                     sphereRadius*sy + res.collisionPoint(2), ...
                     sphereRadius*sz + res.collisionPoint(3), ...
                     'FaceColor', 'r', 'EdgeColor', 'none');
            else
                % No collision: plot a red line connecting witness points.
                wp = res.witnessPoints;  % Expected as a 2x3 matrix.
                plot3(wp(:,1), wp(:,2), wp(:,3), 'r-', 'LineWidth', 2);
            end
        end
        title(sprintf('Time = %.3f', loadStates(tIdx).time));
        xlabel('X'); ylabel('Y'); zlabel('Z');
        axis equal; grid on;
        view(45,45);  % Isometric view.
        drawnow;
        pause(0.05);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local Helper Functions

function shape = ensureRotationMatrix(shape)
% ensureRotationMatrix converts a 1x3 rotation vector (Tait-Bryan angles in degrees)
% into a 3x3 rotation matrix if necessary.
    if isfield(shape, 'rotation')
        if isvector(shape.rotation) && length(shape.rotation)==3
            shape.rotation = computeRotationMatrix(shape.rotation);
        elseif ismatrix(shape.rotation) && ~all(size(shape.rotation)==[3,3])
            error('Rotation field has invalid dimensions.');
        end
    else
        shape.rotation = eye(3);
    end
end

function R = computeRotationMatrix(rotation)
% computeRotationMatrix converts a 1x3 vector (yaw, pitch, roll in degrees)
% into a 3x3 rotation matrix.
    % Assume Tait-Bryan angles in the order yaw-pitch-roll.
    yaw   = deg2rad(rotation(1));
    pitch = deg2rad(rotation(2));
    roll  = deg2rad(rotation(3));
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw), cos(yaw),  0;
          0,        0,         1];
    Ry = [cos(pitch), 0, sin(pitch);
          0,          1, 0;
          -sin(pitch),0, cos(pitch)];
    Rx = [1, 0,         0;
          0, cos(roll), -sin(roll);
          0, sin(roll), cos(roll)];
    R = Rz * Ry * Rx;
end

function plotShape(shape, color)
% plotShape visualizes a shape (box, sphere, or cylinder) based on its type.
    if strcmp(shape.type, 'box')
        vertices = getBoxVertices(shape);
        plotBox(vertices, color);
    elseif strcmp(shape.type, 'sphere')
        [sx, sy, sz] = sphere(20);
        surf(shape.extents(1)*sx + shape.center(1), ...
             shape.extents(1)*sy + shape.center(2), ...
             shape.extents(1)*sz + shape.center(3), ...
             'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    elseif strcmp(shape.type, 'cylinder')
        [X, Y, Z] = cylinder(shape.extents(1), 20);
        Z = Z * shape.extents(2) - shape.extents(2)/2;
        pts = [X(:), Y(:), Z(:)]';
        ptsT = shape.rotation * pts;
        Xp = reshape(ptsT(1,:) + shape.center(1), size(X));
        Yp = reshape(ptsT(2,:) + shape.center(2), size(Y));
        Zp = reshape(ptsT(3,:) + shape.center(3), size(Z));
        surf(Xp, Yp, Zp, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    end
end

function vertices = getBoxVertices(box)
% getBoxVertices returns the eight vertices of a box given its center, extents, and rotation.
    hx = box.extents(1) / 2; 
    hy = box.extents(2) / 2; 
    hz = box.extents(3) / 2;
    localVerts = [ -hx, -hy, -hz;
                    hx, -hy, -hz;
                    hx,  hy, -hz;
                   -hx,  hy, -hz;
                   -hx, -hy,  hz;
                    hx, -hy,  hz;
                    hx,  hy,  hz;
                   -hx,  hy,  hz];
    vertices = (box.rotation * localVerts')' + box.center;
end

function plotBox(vertices, color)
% plotBox draws a box given its vertices.
    faces = [1 2 3 4;
             5 6 7 8;
             1 2 6 5;
             2 3 7 6;
             3 4 8 7;
             4 1 5 8];
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', color, 'FaceAlpha', 0.3);
end

function [collision, mtv] = SATCollision(box1, box2)
% SATCollision uses the Separating Axis Theorem to test collision between two boxes.
    A = box1.rotation;
    B = box2.rotation;
    T = box2.center - box1.center;
    
    % Build candidate axes: face normals of each box and cross products.
    axes = zeros(3,15);
    idx = 1;
    for i = 1:3
        axes(:,idx) = A(:,i); idx = idx + 1;
    end
    for i = 1:3
        axes(:,idx) = B(:,i); idx = idx + 1;
    end
    for i = 1:3
        for j = 1:3
            cp = cross(A(:,i), B(:,j));
            if norm(cp) > 1e-6, cp = cp/norm(cp); end
            axes(:,idx) = cp; idx = idx + 1;
        end
    end
    
    minOverlap = Inf;
    collision = true;
    collisionAxis = [0; 0; 0];
    for i = 1:size(axes,2)
        axis_i = axes(:,i);
        if norm(axis_i) < 1e-6, continue; end
        rA = (box1.extents(1)/2)*abs(dot(axis_i, A(:,1))) + ...
             (box1.extents(2)/2)*abs(dot(axis_i, A(:,2))) + ...
             (box1.extents(3)/2)*abs(dot(axis_i, A(:,3)));
        rB = (box2.extents(1)/2)*abs(dot(axis_i, B(:,1))) + ...
             (box2.extents(2)/2)*abs(dot(axis_i, B(:,2))) + ...
             (box2.extents(3)/2)*abs(dot(axis_i, B(:,3)));
        overlap = rA + rB - abs(dot(T, axis_i));
        if overlap < 0
            collision = false;
            mtv = [0; 0; 0];
            return;
        elseif overlap < minOverlap
            minOverlap = overlap;
            collisionAxis = axis_i;
        end
    end
    if dot(T, collisionAxis) < 0, collisionAxis = -collisionAxis; end
    mtv = collisionAxis * minOverlap;
end

function contactPoint = computeContactPointBoxBox(box1, box2, mtv)
% Computes the contact point for two colliding boxes.
    [minDist, cp1, cp2] = analyticalDistanceBoxBox(box1, box2);
    contactPoint = (cp1 + cp2) / 2;
end

function [minDist, cp1, cp2] = analyticalDistanceBoxBox(box1, box2)
% analyticalDistanceBoxBox computes the minimum distance between two boxes.
    vertices1 = getBoxVertices(box1);
    vertices2 = getBoxVertices(box2);
    [minDist, cp1, cp2] = analyticalDistance(box1, box2, vertices1, vertices2);
end

function [minDist, cp1, cp2] = analyticalDistance(box1, box2, vertices1, vertices2)
    minDist = Inf; cp1 = []; cp2 = [];
    for i = 1:size(vertices1,1)
        cp = closestPointOnBox(box2, vertices1(i,:));
        d = norm(vertices1(i,:) - cp);
        if d < minDist, minDist = d; cp1 = vertices1(i,:); cp2 = cp; end
    end
    for i = 1:size(vertices2,1)
        cp = closestPointOnBox(box1, vertices2(i,:));
        d = norm(vertices2(i,:) - cp);
        if d < minDist, minDist = d; cp1 = cp; cp2 = vertices2(i,:); end
    end
end

function cp = closestPointOnBox(box, point)
% closestPointOnBox returns the closest point on a box to a given point.
    localPoint = box.rotation' * (point - box.center)';
    halfExt = box.extents(:) / 2;
    cpLocal = min(max(localPoint, -halfExt), halfExt);
    cp = (box.rotation * cpLocal)' + box.center;
end

function [minDist, cp1, cp2] = distanceBoxSphere(box, sph)
% distanceBoxSphere computes the minimum distance between a box and a sphere.
    cp1 = closestPointOnBox(box, sph.center);
    d = norm(sph.center - cp1);
    minDist = max(0, d - sph.extents(1));
    if d > 1e-6
        dir = (sph.center - cp1) / d;
    else
        dir = [1, 0, 0];
    end
    cp2 = sph.center - sph.extents(1) * dir;
end

function cp = closestPointOnCylinder(cyl, point)
% closestPointOnCylinder returns the closest point on a capped cylinder.
    localPoint = cyl.rotation' * (point - cyl.center)';
    % Project onto the cylinder's side.
    cp_side = localPoint;
    cp_side(3) = min(max(localPoint(3), -cyl.extents(2)/2), cyl.extents(2)/2);
    r = norm(localPoint(1:2));
    if r > 1e-6
        cp_side(1:2) = (cyl.extents(1)/r) * localPoint(1:2);
    else
        cp_side(1:2) = [cyl.extents(1); 0];
    end
    % Caps: if point is above or below the cylinder.
    if localPoint(3) > cyl.extents(2)/2
        cp_cap = localPoint; cp_cap(3) = cyl.extents(2)/2;
    elseif localPoint(3) < -cyl.extents(2)/2
        cp_cap = localPoint; cp_cap(3) = -cyl.extents(2)/2;
    else
        cp_cap = cp_side;
    end
    r_xy = norm(cp_cap(1:2));
    if r_xy > cyl.extents(1)
        cp_cap(1:2) = (cyl.extents(1)/r_xy)*cp_cap(1:2);
    end
    if norm(localPoint - cp_side) <= norm(localPoint - cp_cap)
        cpLocal = cp_side;
    else
        cpLocal = cp_cap;
    end
    cp = (cyl.rotation * cpLocal)' + cyl.center;
end

function [minDist, cpCyl, cpSph] = distanceCylinderSphere(cyl, sph)
% distanceCylinderSphere computes the minimum distance between a cylinder and a sphere.
    cpCyl = closestPointOnCylinder(cyl, sph.center);
    d = norm(sph.center - cpCyl);
    minDist = max(0, d - sph.extents(1));
    if d > 1e-6
        dir = (sph.center - cpCyl) / d;
    else
        dir = [1, 0, 0];
    end
    cpSph = sph.center - sph.extents(1) * dir;
end

function [minDist, cpBox, cpCyl] = distanceBoxCylinder(box, cyl)
% distanceBoxCylinder computes the minimum distance between a box and a cylinder.
    vertices = getBoxVertices(box);
    halfExt = box.extents(:)' / 2;
    faceCenters_local = [ halfExt(1), 0, 0;
                         -halfExt(1), 0, 0;
                          0, halfExt(2), 0;
                          0,-halfExt(2), 0;
                          0, 0, halfExt(3);
                          0, 0,-halfExt(3)];
    faceCenters = (box.rotation * faceCenters_local')' + box.center;
    boxCandidates = [vertices; faceCenters];
    
    r = cyl.extents(1);
    numAng = 12;
    angSamples = linspace(0, 2*pi, numAng+1); angSamples(end) = [];
    cylSideCandidates = zeros(numAng, 3);
    for j = 1:length(angSamples)
        localCandidate = [r*cos(angSamples(j)); r*sin(angSamples(j)); 0];
        cylSideCandidates(j,:) = (cyl.rotation * localCandidate)' + cyl.center;
    end
    
    minDist = Inf; cpBox = []; cpCyl = [];
    for i = 1:size(boxCandidates, 1)
        v = boxCandidates(i, :);
        cp = closestPointOnCylinder(cyl, v);
        d = norm(v - cp);
        if d < minDist
            minDist = d;
            cpBox = v;
            cpCyl = cp;
        end
    end
    for i = 1:size(cylSideCandidates, 1)
        v = cylSideCandidates(i, :);
        cp = closestPointOnBox(box, v);
        d = norm(v - cp);
        if d < minDist
            minDist = d;
            cpBox = cp;
            cpCyl = v;
        end
    end
end

function [minDist, cp1, cp2] = distanceCylinderCylinder(cyl1, cyl2)
% distanceCylinderCylinder computes the minimum distance between two cylinders.
    angles = linspace(0, 2*pi, 9); angles(end) = [];
    minDist = Inf; cp1 = []; cp2 = [];
    for i = 1:length(angles)
        localPoint = [cyl1.extents(1)*cos(angles(i)); cyl1.extents(1)*sin(angles(i)); 0];
        pt1 = (cyl1.rotation * localPoint)' + cyl1.center;
        pt2 = closestPointOnCylinder(cyl2, pt1);
        d = norm(pt1 - pt2);
        if d < minDist, minDist = d; cp1 = pt1; cp2 = pt2; end
    end
    for i = 1:length(angles)
        localPoint = [cyl2.extents(1)*cos(angles(i)); cyl2.extents(1)*sin(angles(i)); 0];
        pt2 = (cyl2.rotation * localPoint)' + cyl2.center;
        pt1 = closestPointOnCylinder(cyl1, pt2);
        d = norm(pt1 - pt2);
        if d < minDist, minDist = d; cp1 = pt1; cp2 = pt2; end
    end
end

function [collision, contactPoint] = detectCollisionCylinderSphere(cyl, sph)
% detectCollisionCylinderSphere tests collision between a cylinder and a sphere.
    [minDist, cpCyl, cpSph] = distanceCylinderSphere(cyl, sph);
    collision = (minDist <= 1e-6);
    if collision
        dir = (sph.center - cpCyl) / norm(sph.center - cpCyl);
        contactPoint = sph.center - sph.extents(1) * dir;
    else
        contactPoint = [];
    end
end

function [collision, contactPoint] = detectCollisionBoxCylinder(box, cyl)
% detectCollisionBoxCylinder tests collision between a box and a cylinder.
    [minDist, cpBox, cpCyl] = distanceBoxCylinder(box, cyl);
    collision = (minDist <= 1e-6);
    if collision
        contactPoint = (cpBox + cpCyl) / 2;
    else
        contactPoint = [];
    end
end

function [collision, contactPoint] = detectCollisionCylinderCylinder(cyl1, cyl2)
% detectCollisionCylinderCylinder tests collision between two cylinders.
    [minDist, cp1, cp2] = distanceCylinderCylinder(cyl1, cyl2);
    collision = (minDist <= 1e-6);
    if collision
        contactPoint = (cp1 + cp2) / 2;
    else
        contactPoint = [];
    end
end

function [collision, contactPoint] = detectCollision(shape1, shape2)
% detectCollision determines if shape1 collides with shape2 using type-specific
% methods. For now, we implement box-box and sphere-sphere collisions.
    if strcmp(shape1.type, 'box') && strcmp(shape2.type, 'box')
        [collision, mtv] = SATCollision(shape1, shape2);
        if collision
            contactPoint = computeContactPointBoxBox(shape1, shape2, mtv);
        else
            contactPoint = [];
        end
    elseif strcmp(shape1.type, 'sphere') && strcmp(shape2.type, 'sphere')
        d = norm(shape2.center - shape1.center);
        collision = (d <= (shape1.extents(1) + shape2.extents(1)));
        if collision
            dir = (shape2.center - shape1.center) / d;
            contactPoint = shape1.center + shape1.extents(1) * dir;
        else
            contactPoint = [];
        end
    elseif strcmp(shape1.type, 'box') && strcmp(shape2.type, 'sphere')
        cp_box = closestPointOnBox(shape1, shape2.center);
        d = norm(shape2.center - cp_box);
        collision = (d <= shape2.extents(1));
        if collision
            dir = (shape2.center - cp_box) / d;
            contactPoint = shape2.center - shape2.extents(1) * dir;
        else
            contactPoint = [];
        end
    elseif strcmp(shape1.type, 'sphere') && strcmp(shape2.type, 'box')
        [collision, contactPoint] = detectCollision(shape2, shape1);
    elseif strcmp(shape1.type, 'cylinder') && strcmp(shape2.type, 'sphere')
        [collision, contactPoint] = detectCollisionCylinderSphere(shape1, shape2);
    elseif strcmp(shape1.type, 'sphere') && strcmp(shape2.type, 'cylinder')
        [collision, contactPoint] = detectCollisionCylinderSphere(shape2, shape1);
    elseif strcmp(shape1.type, 'box') && strcmp(shape2.type, 'cylinder')
        [collision, contactPoint] = detectCollisionBoxCylinder(shape1, shape2);
    elseif strcmp(shape1.type, 'cylinder') && strcmp(shape2.type, 'box')
        [collision, contactPoint] = detectCollisionBoxCylinder(shape2, shape1);
    elseif strcmp(shape1.type, 'cylinder') && strcmp(shape2.type, 'cylinder')
        [collision, contactPoint] = detectCollisionCylinderCylinder(shape1, shape2);
    else
        error('Collision detection for these shape types is not implemented.');
    end
end

function [minDist, cp1, cp2] = computeDistance(shape1, shape2)
% computeDistance computes the minimum distance and witness points between shape1 and shape2.
    if strcmp(shape1.type, 'box') && strcmp(shape2.type, 'box')
        [minDist, cp1, cp2] = analyticalDistanceBoxBox(shape1, shape2);
    elseif strcmp(shape1.type, 'sphere') && strcmp(shape2.type, 'sphere')
        d = norm(shape2.center - shape1.center);
        minDist = max(0, d - shape1.extents(1) - shape2.extents(1));
        if d > 1e-6
            dir = (shape2.center - shape1.center) / d;
        else
            dir = [1, 0, 0];
        end
        cp1 = shape1.center + shape1.extents(1) * dir;
        cp2 = shape2.center - shape2.extents(1) * dir;
    elseif strcmp(shape1.type, 'box') && strcmp(shape2.type, 'sphere')
        [minDist, cpBox, cpSphere] = distanceBoxSphere(shape1, shape2);
        cp1 = cpBox; cp2 = cpSphere;
    elseif strcmp(shape1.type, 'sphere') && strcmp(shape2.type, 'box')
        [minDist, cpSphere, cpBox] = distanceBoxSphere(shape2, shape1);
        cp1 = cpBox; cp2 = cpSphere;
    elseif strcmp(shape1.type, 'cylinder') && strcmp(shape2.type, 'sphere')
        [minDist, cpCyl, cpSph] = distanceCylinderSphere(shape1, shape2);
        cp1 = cpCyl; cp2 = cpSph;
    elseif strcmp(shape1.type, 'sphere') && strcmp(shape2.type, 'cylinder')
        [minDist, cpSph, cpCyl] = distanceCylinderSphere(shape2, shape1);
        cp1 = cpCyl; cp2 = cpSph;
    elseif strcmp(shape1.type, 'box') && strcmp(shape2.type, 'cylinder')
        [minDist, cpBox, cpCyl] = distanceBoxCylinder(shape1, shape2);
        cp1 = cpBox; cp2 = cpCyl;
    elseif strcmp(shape1.type, 'cylinder') && strcmp(shape2.type, 'box')
        [minDist, cpCyl, cpBox] = distanceBoxCylinder(shape2, shape1);
        cp1 = cpBox; cp2 = cpCyl;
    elseif strcmp(shape1.type, 'cylinder') && strcmp(shape2.type, 'cylinder')
        [minDist, cp1, cp2] = distanceCylinderCylinder(shape1, shape2);
    else
        error('Distance computation for these shape types is not implemented.');
    end
end
