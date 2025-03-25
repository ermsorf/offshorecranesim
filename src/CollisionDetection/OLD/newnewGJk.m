% Run the main function
clear
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define the dynamic (moving) box using a struct.
movingObj = DefineMovingObject();
movingBox = objectsToBoxes(movingObj);  % Convert to box format (center, L, R)

% Load the stationary boxes from a struct.
objects = ListOfObjects();
stationaryBoxes = objectsToBoxes(objects);
nBoxes = length(stationaryBoxes);

% Convert our box-format objects to "shape" structures (with XData, YData, ZData)
movingShape = boxToShape(movingBox);
for i = 1:nBoxes
    staticShape{i} = boxToShape(stationaryBoxes(i));
end

% Set number of iterations for GJK
iterations = 20;

% Test collision between the moving box and each stationary box using GJK.
for i = 1:nBoxes
    flag = GJK(staticShape{i}, movingShape, iterations);
    % Post-process: if GJK reports a collision, compute the closest distance
    % from the final simplex (from the triangle) and override if too large.
    if flag
         [closest, ~] = closestPointOnTriangle(a, b, c); %#ok<*ASGLU>
         if norm(closest) > 1e-3
             flag = 0;
         end
    end
    if flag
        fprintf('Collision detected with Stationary Box %d.\n', i);
    else
        fprintf('No collision detected with Stationary Box %d.\n', i);
    end
end

% Visualize the boxes as before.
visualize_boxes(movingBox, stationaryBoxes);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define Moving Object Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function movingObj = DefineMovingObject()
    % Define the moving object as a cube using the same struct format as static objects.
    % Fields: 'name', 'type', 'position', 'extents', 'rotations'
    % Rotations are given as [yaw, pitch, roll] in degrees.
    movingObj = struct( ...
        'name', 'MovingBox', ...
        'type', 'cube', ...
        'position', [-3.5, 1, 1], ...        % Center of the moving box.
        'extents', [1.2, 1.2, 1.2], ...     % Full extents of the cube.
        'rotations', [30, 0, 0] ...         % Example: yaw=30°, pitch=0, roll=0.
    );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ListOfObjects Function (Static Boxes)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function objects = ListOfObjects()
    % Define static boxes as objects with fields:
    % 'name', 'type', 'position', 'extents', and 'rotations'
    % Rotations are given as [yaw, pitch, roll] in degrees.
    
    objects(1) = struct( ...
        'name', 'Box1', ...
        'type', 'cube', ...
        'position', [-4, 0, 1], ...
        'extents', [1, 2, 1], ...
        'rotations', [0, 0, 0] );
    
    objects(2) = struct( ...
        'name', 'Box2', ...
        'type', 'cube', ...
        'position', [-2, 3, 2], ...
        'extents', [1.5, 1, 1], ...
        'rotations', [0, 45, 0] );
    
    objects(3) = struct( ...
        'name', 'Box3', ...
        'type', 'cube', ...
        'position', [0, -4, 0.5], ...
        'extents', [0.8, 1.2, 1.5], ...
        'rotations', [35, 20, 0] );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Conversion: Objects to Boxes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function boxes = objectsToBoxes(objects)
    n = length(objects);
    boxes = repmat(struct('center', [], 'L', [], 'R', []), n, 1);
    for i = 1:n
        % The object's position is the box center.
        boxes(i).center = objects(i).position(:);
        % Divide full extents by 2 to get half-lengths.
        boxes(i).L = objects(i).extents(:) / 2;
        % Convert rotations (in degrees) to radians.
        yaw   = deg2rad(objects(i).rotations(1));
        pitch = deg2rad(objects(i).rotations(2));
        roll  = deg2rad(objects(i).rotations(3));
        % Use the conventional order: yaw, then pitch, then roll.
        boxes(i).R = rotz(yaw) * roty(pitch) * rotx(roll);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Conversion: Box to Shape (for use with working GJK)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function shape = boxToShape(box)
    % Compute the 8 vertices of the oriented box.
    corners = [ -1 -1 -1;
                 1 -1 -1;
                 1  1 -1;
                -1  1 -1;
                -1 -1  1;
                 1 -1  1;
                 1  1  1;
                -1  1  1];
    % Scale by half-lengths.
    corners = corners .* (box.L)';
    % Rotate and translate.
    vertices = (box.R * corners')' + repmat(box.center', 8, 1);
    shape.XData = vertices(:,1);
    shape.YData = vertices(:,2);
    shape.ZData = vertices(:,3);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualization Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function visualize_boxes(movingBox, stationaryBoxes)
    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    
    draw_OBB(movingBox.center, movingBox.R, movingBox.L, [0 0 1]);
    draw_axes(movingBox.center, movingBox.R, movingBox.L, [0 0 1]);
    
    n = length(stationaryBoxes);
    colors = lines(n);
    for i = 1:n
        draw_OBB(stationaryBoxes(i).center, stationaryBoxes(i).R, stationaryBoxes(i).L, colors(i,:));
        draw_axes(stationaryBoxes(i).center, stationaryBoxes(i).R, stationaryBoxes(i).L, colors(i,:));
    end
    title('Collision Detection: Moving Box vs. Stationary Boxes');
end

function draw_OBB(center, R, half_lengths, color)
    corners = [ -1 -1 -1;
                 1 -1 -1;
                 1  1 -1;
                -1  1 -1;
                -1 -1  1;
                 1 -1  1;
                 1  1  1;
                -1  1  1];
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
%% Working GJK Collision Detection Functions (Based on your provided code)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function flag = GJK(shape1, shape2, iterations)
    % GJK collision detection implementation.
    % Returns true if the convex shapes intersect.
    % shape1 and shape2 are assumed to have fields XData, YData, ZData.
    
    % Start with an initial direction vector.
    v = [0.8 0.5 1];
    [a, b] = pickLine(v, shape2, shape1);
    [a, b, c, flag] = pickTriangle(a, b, shape2, shape1, iterations);
    if flag == 1  % Only proceed if a viable triangle was found.
        [a, b, c, d, flag] = pickTetrahedron(a, b, c, shape2, shape1, iterations);
    end
    % Post-process: if a tetrahedron is formed, check the closest distance.
    % (If the closest point on the final simplex is not nearly zero, override flag.)
    if flag == 1
        [closest, ~] = closestPointOnTriangle(a, b, c);
        if norm(closest) > 1e-3
            flag = 0;
        end
    end
end

function [a, b] = pickLine(v, shape1, shape2)
    % Construct the first line of the simplex.
    b = support(shape1, shape2, v);
    a = support(shape1, shape2, -v);
end

function [a, b, c, flag] = pickTriangle(a, b, shape1, shape2, IterationAllowed)
    flag = 0; % Initially, no viable triangle.
    ab = b - a;
    ao = -a;
    v = cross(cross(ab, ao), ab);
    c = b;
    b = a;
    a = support(shape1, shape2, v);
    
    for i = 1:IterationAllowed
        ab = b - a;
        ao = -a;
        ac = c - a;
        abc = cross(ab, ac);
        abp = cross(ab, abc);
        acp = cross(abc, ac);
        
        if dot(abp, ao) > 0
            c = b;
            b = a;
            v = abp;
        elseif dot(acp, ao) > 0
            b = a;
            v = acp;
        else
            flag = 1;
            break;
        end
        a = support(shape1, shape2, v);
    end
end

function [a, b, c, d, flag] = pickTetrahedron(a, b, c, shape1, shape2, IterationAllowed)
    flag = 0;
    ab = b - a;
    ac = c - a;
    abc = cross(ab, ac);
    ao = -a;
    
    if dot(abc, ao) > 0
        d = c;
        c = b;
        b = a;
        v = abc;
        a = support(shape1, shape2, v);
    else
        d = b;
        b = a;
        v = -abc;
        a = support(shape1, shape2, v);
    end
    
    for i = 1:IterationAllowed
        ab = b - a;
        ac = c - a;
        ad = d - a;
        abc = cross(ab, ac);
        if dot(abc, ao) > 0
            % Face ABC is good.
        else
            acd = cross(ac, ad);
            if dot(acd, ao) > 0
                b = c;
                c = d;
                abc = acd;
            elseif dot(acd, ao) < 0
                adb = cross(ad, ab);
                if dot(adb, ao) > 0
                    c = b;
                    b = d;
                    abc = adb;
                else
                    flag = 1;
                    break;
                end
            end
        end
        
        if dot(abc, ao) > 0
            d = c;
            c = b;
            b = a;
            v = abc;
            a = support(shape1, shape2, v);
        else
            v = -abc;
            a = support(shape1, shape2, v);
        end
    end
end

function point = getFarthestInDir(shape, v)
    % Find the furthest point in direction v for a shape.
    XData = shape.XData;
    YData = shape.YData;
    ZData = shape.ZData;
    dotted = XData * v(1) + YData * v(2) + ZData * v(3);
    [~, rowIdxSet] = max(dotted);
    rowIdx = rowIdxSet(1);
    point = [XData(rowIdx), YData(rowIdx), ZData(rowIdx)]';
end

function point = support(shape1, shape2, v)
    % Support function: returns the Minkowski difference support point.
    point1 = getFarthestInDir(shape1, v);
    point2 = getFarthestInDir(shape2, -v);
    point = point1 - point2;
end

function [closest, bary] = closestPointOnTriangle(A, B, C)
    % Compute the closest point on triangle ABC to the origin and barycentrics.
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
