%% Analytical Collision and Distance between Boxes, Spheres, and Cylinders
% This script demonstrates analytical collision detection and minimum distance
% computation for three types of primitives:
%   - Boxes (oriented, defined by center, halfExtents, and rotation)
%   - Spheres (defined by center and radius)
%   - Cylinders (capped, defined by center, radius, height, and rotation,
%                where the cylinder's local z-axis is its axis)
%
% The dispatcher functions select an appropriate analytical test based on the
% shape types. For sphere collisions the contact point is always biased to lie
% on the sphere's surface. For non-colliding pairs, the minimum distance and
% witness points are computed and output.
%
% The shapes are then visualized in 3D.

clear; clc; close all;

%% Define two shapes
% To test different pairs, change the type field ('box','sphere','cylinder')
% and adjust the parameters accordingly.

% Example 1: Box vs. Sphere

tic

shape1.type = 'box';
shape1.center = [4, 0, 3];
shape1.halfExtents = [1, 1, 1];
% Define rotation for box1 (angles in radians)
thetaX1 = deg2rad(45);    
thetaY1 = deg2rad(45);   
thetaZ1 = deg2rad(45);
Rx1 = [1, 0, 0;
       0, cos(thetaX1), -sin(thetaX1);
       0, sin(thetaX1), cos(thetaX1)];
Ry1 = [cos(thetaY1), 0, sin(thetaY1);
       0, 1, 0;
       -sin(thetaY1), 0, cos(thetaY1)];
Rz1 = [cos(thetaZ1), -sin(thetaZ1), 0;
       sin(thetaZ1), cos(thetaZ1), 0;
       0, 0, 1];
shape1.rotation = Rz1 * Ry1 * Rx1;

% Example: shape2 as a sphere
shape2.type = 'sphere';
shape2.center = [3, 0, 0];  
shape2.radius = 2;


% --- To test other combinations, you can change shape2 as follows:
% For a cylinder:
shape2.type = 'cylinder';
shape2.center = [4, 0, 0];
shape2.radius = 1;
shape2.height = 2;
shape2.rotation = eye(3); % or set a rotation

%% Collision and Distance Computation
[collision, contactPoint] = detectCollision(shape1, shape2);

if ~collision
    [minDist, cp1, cp2] = computeDistance(shape1, shape2);
    fprintf('Minimum distance: %f\n', minDist);
    fprintf('Witness point on shape1: (%.3f, %.3f, %.3f)\n', cp1(1), cp1(2), cp1(3));
    fprintf('Witness point on shape2: (%.3f, %.3f, %.3f)\n', cp2(1), cp2(2), cp2(3));
end


toc

%% Visualization
figure;
hold on;
plotShape(shape1, 'b');
plotShape(shape2, 'g');

if collision
    % Draw a small sphere at the collision (contact) point.
    [sx, sy, sz] = sphere(20);
    sphereRadius = 0.1;
    surf(sphereRadius*sx + contactPoint(1), sphereRadius*sy + contactPoint(2), sphereRadius*sz + contactPoint(3),...
         'FaceColor', 'r', 'EdgeColor', 'none');
    title('Collision Detected');
    disp('Shapes are colliding.');
else
    % Draw a red line connecting the closest points.
    plot3([cp1(1) cp2(1)], [cp1(2) cp2(2)], [cp1(3) cp2(3)], 'r-', 'LineWidth', 2);
    title('No Collision - Minimum Distance Computed');
end

xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; axis equal; view(3);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dispatcher Functions

function [collision, contactPoint] = detectCollision(shape1, shape2)
    % Dispatch collision detection based on shape types.
    if strcmp(shape1.type, 'box') && strcmp(shape2.type, 'box')
        [collision, mtv] = SATCollision(shape1, shape2);
        if collision
            contactPoint = computeContactPointBoxBox(shape1, shape2, mtv);
        else
            contactPoint = [];
        end
    elseif strcmp(shape1.type, 'sphere') && strcmp(shape2.type, 'sphere')
        d = norm(shape2.center - shape1.center);
        collision = (d <= (shape1.radius + shape2.radius));
        if collision
            % Bias the contact point to lie on sphere1's surface.
            dir = (shape2.center - shape1.center) / d;
            contactPoint = shape1.center + shape1.radius * dir;
        else
            contactPoint = [];
        end
    elseif strcmp(shape1.type, 'box') && strcmp(shape2.type, 'sphere')
        cp_box = closestPointOnBox(shape1, shape2.center);
        d = norm(shape2.center - cp_box);
        collision = (d <= shape2.radius);
        if collision
            % Bias the contact point to lie on shape2 (sphere)'s surface.
            dir = (shape2.center - cp_box) / d;
            contactPoint = shape2.center - shape2.radius * dir;
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
    % Dispatch distance computation based on shape types.
    if strcmp(shape1.type, 'box') && strcmp(shape2.type, 'box')
        [minDist, cp1, cp2] = analyticalDistanceBoxBox(shape1, shape2);
    elseif strcmp(shape1.type, 'sphere') && strcmp(shape2.type, 'sphere')
        d = norm(shape2.center - shape1.center);
        minDist = max(0, d - shape1.radius - shape2.radius);
        if d > 1e-6
            dir = (shape2.center - shape1.center) / d;
        else
            dir = [1, 0, 0];
        end
        cp1 = shape1.center + shape1.radius * dir;
        cp2 = shape2.center - shape2.radius * dir;
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

function plotShape(shape, color)
    % Plots the shape based on its type.
    if strcmp(shape.type, 'box')
        vertices = getBoxVertices(shape);
        plotBox(vertices, color);
    elseif strcmp(shape.type, 'sphere')
        [sx, sy, sz] = sphere(20);
        surf(shape.radius*sx + shape.center(1), shape.radius*sy + shape.center(2), shape.radius*sz + shape.center(3),...
             'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    elseif strcmp(shape.type, 'cylinder')
        [X, Y, Z] = cylinder(shape.radius, 20);
        Z = Z * shape.height - shape.height/2;
        pts = [X(:), Y(:), Z(:)]';
        ptsT = shape.rotation * pts;
        Xp = reshape(ptsT(1,:) + shape.center(1), size(X));
        Yp = reshape(ptsT(2,:) + shape.center(2), size(Y));
        Zp = reshape(ptsT(3,:) + shape.center(3), size(Z));
        surf(Xp, Yp, Zp, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Box-Box Collision/Distance Functions

function [collision, mtv] = SATCollision(box1, box2)
    % SATCollision for two boxes.
    A = box1.rotation;
    B = box2.rotation;
    C1 = box1.center;
    C2 = box2.center;
    T = C2 - C1;
    
    axes = zeros(3,15);
    idx = 1;
    for i = 1:3
        axes(:,idx) = A(:,i); idx = idx+1;
    end
    for i = 1:3
        axes(:,idx) = B(:,i); idx = idx+1;
    end
    for i = 1:3
        for j = 1:3
            cp = cross(A(:,i), B(:,j));
            if norm(cp) > 1e-6, cp = cp/norm(cp); end
            axes(:,idx) = cp; idx = idx+1;
        end
    end
    
    minOverlap = Inf;
    collision = true;
    collisionAxis = [0;0;0];
    for i = 1:size(axes,2)
        axis_i = axes(:,i);
        if norm(axis_i) < 1e-6, continue; end
        rA = box1.halfExtents(1)*abs(dot(axis_i,A(:,1))) + ...
             box1.halfExtents(2)*abs(dot(axis_i,A(:,2))) + ...
             box1.halfExtents(3)*abs(dot(axis_i,A(:,3)));
        rB = box2.halfExtents(1)*abs(dot(axis_i,B(:,1))) + ...
             box2.halfExtents(2)*abs(dot(axis_i,B(:,2))) + ...
             box2.halfExtents(3)*abs(dot(axis_i,B(:,3)));
        projT = abs(dot(T,axis_i));
        overlap = rA + rB - projT;
        if overlap < 0
            collision = false;
            mtv = [0;0;0];
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
    vertices1 = getBoxVertices(box1);
    vertices2 = getBoxVertices(box2);
    candidates = [];
    for i = 1:size(vertices1,1)
        if isPointInsideBox(box2, vertices1(i,:))
            candidates = [candidates; vertices1(i,:)]; %#ok<AGROW>
        end
    end
    for i = 1:size(vertices2,1)
        if isPointInsideBox(box1, vertices2(i,:))
            candidates = [candidates; vertices2(i,:)]; %#ok<AGROW>
        end
    end
    if ~isempty(candidates)
        contactPoint = mean(candidates,1);
    else
        contactPoint = box1.center + 0.5 * mtv;
    end
end

function inside = isPointInsideBox(box, point)
    localPoint = box.rotation' * (point - box.center)';
    inside = all(abs(localPoint) <= box.halfExtents' + 1e-6);
end

function vertices = getBoxVertices(box)
    hx = box.halfExtents(1); hy = box.halfExtents(2); hz = box.halfExtents(3);
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
    faces = [1 2 3 4;
             5 6 7 8;
             1 2 6 5;
             2 3 7 6;
             3 4 8 7;
             4 1 5 8];
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', color, 'FaceAlpha', 0.3);
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
        if d < minDist, minDist = d; cp1 = vertices1(i,:); cp2 = cp; end
    end
    for i = 1:size(vertices2,1)
        cp = closestPointOnBox(box1, vertices2(i,:));
        d = norm(vertices2(i,:) - cp);
        if d < minDist, minDist = d; cp1 = cp; cp2 = vertices2(i,:); end
    end
end

function cp = closestPointOnBox(box, point)
    localPoint = box.rotation' * (point - box.center)';
    cpLocal = min(max(localPoint, -box.halfExtents'), box.halfExtents');
    cp = (box.rotation * cpLocal)' + box.center;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Box-Sphere Functions

function [minDist, cpBox, cpSphere] = distanceBoxSphere(box, sph)
    cpBox = closestPointOnBox(box, sph.center);
    d = norm(sph.center - cpBox);
    minDist = max(0, d - sph.radius);
    if d > 1e-6
        dir = (sph.center - cpBox) / d;
    else
        dir = [1, 0, 0];
    end
    % Place contact point on sphere's surface.
    cpSphere = sph.center - sph.radius * dir;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Cylinder Functions

function cp = closestPointOnCylinder(cyl, point)
    localPoint = cyl.rotation' * (point - cyl.center)';
    cpLocal = localPoint;
    cpLocal(3) = min(max(localPoint(3), -cyl.height/2), cyl.height/2);
    r = norm(localPoint(1:2));
    if r > 1e-6
        cpLocal(1:2) = (cyl.radius / r) * localPoint(1:2);
    else
        cpLocal(1:2) = [cyl.radius; 0];
    end
    cp = (cyl.rotation * cpLocal)' + cyl.center;
end

function [minDist, cpCyl, cpSph] = distanceCylinderSphere(cyl, sph)
    localSphere = cyl.rotation' * (sph.center - cyl.center)';
    r = norm(localSphere(1:2));
    if localSphere(3) >= -cyl.height/2 && localSphere(3) <= cyl.height/2
        if r > 1e-6
            cpLocal_side = [ (cyl.radius / r)*localSphere(1:2); localSphere(3) ];
        else
            cpLocal_side = [cyl.radius; 0; localSphere(3)];
        end
        d_side = norm(localSphere - cpLocal_side);
    else
        d_side = Inf;
    end
    if localSphere(3) < -cyl.height/2
        zCap = -cyl.height/2;
    elseif localSphere(3) > cyl.height/2
        zCap = cyl.height/2;
    else
        zCap = localSphere(3);
    end
    r_xy = norm(localSphere(1:2));
    if r_xy > cyl.radius
        cpLocal_cap_xy = (cyl.radius / r_xy) * localSphere(1:2);
    else
        cpLocal_cap_xy = localSphere(1:2);
    end
    cpLocal_cap = [cpLocal_cap_xy; zCap];
    d_cap = norm(localSphere - cpLocal_cap);
    if d_side < d_cap
        cpLocal = cpLocal_side;
        dCandidate = d_side;
    else
        cpLocal = cpLocal_cap;
        dCandidate = d_cap;
    end
    minDist = max(0, dCandidate - sph.radius);
    cpCyl = (cyl.rotation * cpLocal)' + cyl.center;
    if dCandidate > 1e-6
        dir = (sph.center - cpCyl) / norm(sph.center - cpCyl);
    else
        dir = [1, 0, 0];
    end
    % Place contact point on sphere's surface.
    cpSph = sph.center - sph.radius * dir;
end

function [minDist, cpBox, cpCyl] = distanceBoxCylinder(box, cyl)
    vertices = getBoxVertices(box);
    minDist = Inf; cpBox = []; cpCyl = [];
    for i = 1:size(vertices,1)
        v = vertices(i,:);
        cp = closestPointOnCylinder(cyl, v);
        d = norm(v - cp);
        if d < minDist
            minDist = d;
            cpBox = v;
            cpCyl = cp;
        end
    end
    angles = linspace(0, 2*pi, 9); angles(end) = [];
    for i = 1:length(angles)
        localPoint = [cyl.radius*cos(angles(i)); cyl.radius*sin(angles(i)); 0];
        pt = (cyl.rotation * localPoint)' + cyl.center;
        cp = closestPointOnBox(box, pt);
        d = norm(pt - cp);
        if d < minDist
            minDist = d;
            cpBox = cp;
            cpCyl = pt;
        end
    end
end

function [minDist, cp1, cp2] = distanceCylinderCylinder(cyl1, cyl2)
    angles = linspace(0, 2*pi, 9); angles(end) = [];
    minDist = Inf; cp1 = []; cp2 = [];
    for i = 1:length(angles)
        localPoint = [cyl1.radius*cos(angles(i)); cyl1.radius*sin(angles(i)); 0];
        pt1 = (cyl1.rotation * localPoint)' + cyl1.center;
        pt2 = closestPointOnCylinder(cyl2, pt1);
        d = norm(pt1 - pt2);
        if d < minDist, minDist = d; cp1 = pt1; cp2 = pt2; end
    end
    for i = 1:length(angles)
        localPoint = [cyl2.radius*cos(angles(i)); cyl2.radius*sin(angles(i)); 0];
        pt2 = (cyl2.rotation * localPoint)' + cyl2.center;
        pt1 = closestPointOnCylinder(cyl1, pt2);
        d = norm(pt1 - pt2);
        if d < minDist, minDist = d; cp1 = pt1; cp2 = pt2; end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Collision Detection for Mixed Pairs Involving Cylinders

function [collision, contactPoint] = detectCollisionCylinderSphere(cyl, sph)
    [minDist, cpCyl, cpSph] = distanceCylinderSphere(cyl, sph);
    collision = (minDist <= 1e-6);
    if collision
        % Bias contact point to lie on the sphere's surface.
        dir = (sph.center - cpCyl) / norm(sph.center - cpCyl);
        contactPoint = sph.center - sph.radius * dir;
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
