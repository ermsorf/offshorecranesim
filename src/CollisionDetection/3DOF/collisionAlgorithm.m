function allResults = ComputeCollisionsForPendulumLoad(pendulumPointsAndRotations)
% ComputeCollisionsForPendulumLoad
%   Computes collision detection (or minimum distance) between a dynamic
%   load (modeled here as a spherical pendulum load) and a set of stationary
%   objects.
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

    %% Get the baseline load (dynamic load) object.
    % (DefineLoad should return a struct with fields such as type, center,
    % extents, rotation, name, etc. Here we provide a dummy version.)
    baseLoad = DefineLoad();

    %% Number of time steps.
    nSteps = length(pendulumPointsAndRotations.t);
    
    %% Build the dynamic load states from the pendulum data.
    % (For each time step, update the baseline load's center and rotation.)
    loadStates = repmat(baseLoad, nSteps, 1);
    for i = 1:nSteps
        % Global bob position is computed by adding the pivot to the local position.
        globalCenter = [ pendulumPointsAndRotations.xPendulumFromPivot(i) + pendulumPointsAndRotations.pivot(1), ...
                         pendulumPointsAndRotations.yPendulumFromPivot(i) + pendulumPointsAndRotations.pivot(2), ...
                         pendulumPointsAndRotations.zPendulumFromPivot(i) + pendulumPointsAndRotations.pivot(3) ];
        loadStates(i).center = globalCenter;
        loadStates(i).rotation = pendulumPointsAndRotations.R(:,:,i);
        loadStates(i).time = pendulumPointsAndRotations.t(i);
    end
    
    %% Load Stationary Objects.
    % (ListOfObjects should return an array of objects with fields: type, 
    % center, extents, rotation, name, etc. Here we supply a dummy version.)
    objectsArray = ListOfObjects();
    nObjects = numel(objectsArray);
    stationaryObjects = cell(nObjects, 1);
    for i = 1:nObjects
        obj = objectsArray(i);
        % Check that necessary fields are present.
        switch obj.type
            case 'cylinder'
                if ~isfield(obj, 'extents')
                    error('Cylinder object "%s" must have an "extents" field ([radius, height]).', obj.name);
                end
            case 'sphere'
                if ~isfield(obj, 'extents')
                    error('Sphere object "%s" must have an "extents" field (first element is radius).', obj.name);
                end
            case 'box'
                if ~isfield(obj, 'extents')
                    error('Box object "%s" must have an "extents" field.', obj.name);
                end
            otherwise
                error('Unknown object type: %s', obj.type);
        end
        % Ensure rotation field is a 3x3 matrix.
        obj = ensureRotationMatrix(obj);
        stationaryObjects{i} = obj;
    end

    %% Collision Detection Loop.
    % For each time step, check collisions between the current load state and each stationary object.
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
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local Helper Functions

function baseLoad = DefineLoad()
% Dummy implementation of DefineLoad.
% Returns a baseline load object (e.g., modeled as a cylinder).
    baseLoad.type = 'cylinder';
    baseLoad.center = [0,0,0];  % will be updated later
    baseLoad.extents = [1, 10]; % [radius, height]
    baseLoad.rotation = eye(3);
    baseLoad.name = 'DynamicLoad';
    baseLoad.time = 0;
end

function objectsArray = ListOfObjects()
% Dummy implementation of ListOfObjects.
% Returns an array of stationary objects. Here we provide one box and one sphere.
    % Box object
    obj1.type = 'box';
    obj1.center = [15, 0, -5];
    obj1.extents = [4, 4, 4]; % [width, depth, height]
    obj1.rotation = eye(3);
    obj1.name = 'Box1';
    
    % Sphere object
    obj2.type = 'sphere';
    obj2.center = [0, 10, -10];
    obj2.extents = [3];  % radius = 3
    obj2.rotation = eye(3);
    obj2.name = 'Sphere1';
    
    % Cylinder object
    obj3.type = 'cylinder';
    obj3.center = [-10, -10, -5];
    obj3.extents = [2, 8];  % [radius, height]
    obj3.rotation = eye(3);
    obj3.name = 'Cylinder1';
    
    objectsArray = [obj1, obj2, obj3];
end

function shape = ensureRotationMatrix(shape)
% If the rotation field is a 3-element vector, convert it to a 3x3 matrix.
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
% Converts a 3-element rotation vector (in degrees) into a rotation matrix.
    thetaX = deg2rad(rotation(1));
    thetaY = deg2rad(rotation(2));
    thetaZ = deg2rad(rotation(3));
    Rx = [1, 0, 0;
          0, cos(thetaX), -sin(thetaX);
          0, sin(thetaX), cos(thetaX)];
    Ry = [cos(thetaY), 0, sin(thetaY);
          0, 1, 0;
          -sin(thetaY), 0, cos(thetaY)];
    Rz = [cos(thetaZ), -sin(thetaZ), 0;
          sin(thetaZ), cos(thetaZ), 0;
          0, 0, 1];
    R = Rz * Ry * Rx;
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

function vertices = getBoxVertices(box)
% Returns the eight vertices of a box given its center, extents, and rotation.
    hx = box.extents(1) / 2; hy = box.extents(2) / 2; hz = box.extents(3) / 2;
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

function [collision, mtv] = SATCollision(box1, box2)
% Separating Axis Theorem (SAT) based collision detection for two boxes.
    A = box1.rotation;
    B = box2.rotation;
    C1 = box1.center;
    C2 = box2.center;
    T = C2 - C1;
    
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
        projT = abs(dot(T, axis_i));
        overlap = rA + rB - projT;
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
% Compute a contact point (here, the midpoint between witness points) for two colliding boxes.
    [minDist, cp1, cp2] = analyticalDistanceBoxBox(box1, box2);
    contactPoint = (cp1 + cp2) / 2;
end

function cp = closestPointOnBox(box, point)
% Returns the closest point on a box (in world coordinates) to a given point.
    localPoint = box.rotation' * (point - box.center)';
    halfExt = box.extents(:) / 2;
    cpLocal = min(max(localPoint, -halfExt), halfExt);
    cp = (box.rotation * cpLocal)' + box.center;
end

function [minDist, cpBox, cpSphere] = distanceBoxSphere(box, sph)
% Computes the distance between a box and a sphere.
    cpBox = closestPointOnBox(box, sph.center);
    d = norm(sph.center - cpBox);
    minDist = max(0, d - sph.extents(1));
    if d > 1e-6
        dir = (sph.center - cpBox) / d;
    else
        dir = [1, 0, 0];
    end
    cpSphere = sph.center - sph.extents(1) * dir;
end

function cp = closestPointOnCylinder(cyl, point)
% Computes the closest point on a capped cylinder (in world coordinates) to a given point.
    localPoint = cyl.rotation' * (point - cyl.center)';
    
    % Project point onto cylinder's axis (assumed to be z-axis in local coords)
    cp_side = localPoint;
    cp_side(3) = min(max(localPoint(3), -cyl.extents(2)/2), cyl.extents(2)/2);
    r = norm(localPoint(1:2));
    if r > 1e-6
        cp_side(1:2) = (cyl.extents(1)/r) * localPoint(1:2);
    else
        cp_side(1:2) = [cyl.extents(1); 0];
    end
    
    % For the caps
    if localPoint(3) > cyl.extents(2)/2
        cp_cap = localPoint;
        cp_cap(3) = cyl.extents(2)/2;
    elseif localPoint(3) < -cyl.extents(2)/2
        cp_cap = localPoint;
        cp_cap(3) = -cyl.extents(2)/2;
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
% Computes the distance between a cylinder and a sphere.
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
% Computes the distance between a box and a cylinder.
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
    
    % For cylinder candidates, sample the curved side.
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
% Computes the distance between two cylinders by sampling points on their perimeters.
    angles = linspace(0, 2*pi, 9); angles(end) = [];
    minDist = Inf; cp1 = []; cp2 = [];
    for i = 1:length(angles)
        localPoint = [cyl1.extents(1)*cos(angles(i)); cyl1.extents(1)*sin(angles(i)); 0];
        pt1 = (cyl1.rotation * localPoint)' + cyl1.center;
        pt2 = closestPointOnCylinder(cyl2, pt1);
        d = norm(pt1 - pt2);
        if d < minDist
            minDist = d; cp1 = pt1; cp2 = pt2;
        end
    end
    for i = 1:length(angles)
        localPoint = [cyl2.extents(1)*cos(angles(i)); cyl2.extents(1)*sin(angles(i)); 0];
        pt2 = (cyl2.rotation * localPoint)' + cyl2.center;
        pt1 = closestPointOnCylinder(cyl1, pt2);
        d = norm(pt1 - pt2);
        if d < minDist
            minDist = d; cp1 = pt1; cp2 = pt2;
        end
    end
end

function [collision, contactPoint] = detectCollisionCylinderSphere(cyl, sph)
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
    [minDist, cpBox, cpCyl] = distanceBoxCylinder(box, cyl);
    collision = (minDist <= 1e-6);
    if collision
        contactPoint = (cpBox + cpCyl) / 2;
    else
        contactPoint = [];
    end
end

function [collision, contactPoint] = detectCollisionCylinderCylinder(cyl1, cyl2)
    [minDist, cp1, cp2] = distanceCylinderCylinder(cyl1, cyl2);
    collision = (minDist <= 1e-6);
    if collision
        contactPoint = (cp1 + cp2) / 2;
    else
        contactPoint = [];
    end
end

function [minDist, cp1, cp2] = analyticalDistanceBoxBox(box1, box2)
    vertices1 = getBoxVertices(box1);
    vertices2 = getBoxVertices(box2);
    [minDist, cp1, cp2] = analyticalDistance(box1, box2, vertices1, vertices2);
end

function [minDist, cp1, cp2] = analyticalDistance(box1, box2, vertices1, vertices2)
    minDist = Inf; cp1 = []; cp2 = [];
    for i = 1:size(vertices1,1)
        cp = closestPointOnBox(box2, vertices1(i,:));
        d = norm(vertices1(i,:) - cp);
        if d < minDist
            minDist = d; cp1 = vertices1(i,:); cp2 = cp;
        end
    end
    for i = 1:size(vertices2,1)
        cp = closestPointOnBox(box1, vertices2(i,:));
        d = norm(vertices2(i,:) - cp);
        if d < minDist
            minDist = d; cp1 = cp; cp2 = vertices2(i,:);
        end
    end
end

function R = axisAngleToMatrix(axis, angle)
% Computes the rotation matrix for a rotation about a given axis by a given angle.
    K = [    0,   -axis(3),  axis(2);
          axis(3),     0,   -axis(1);
         -axis(2),  axis(1),    0];
    R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
end
