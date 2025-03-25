function animateTrajectoryWithContacts(stationaryObjects, loadObj, pivot, CartesianCoordinatesAndRotation, minDistancesStruct)
    % animateTrajectoryAndContacts animates the moving load's trajectory along with
    % stationary objects and overlays red spheres at the contact points extracted
    % from minDistancesStruct.
    %
    % Inputs:
    %   stationaryObjects - a struct array with fields: name, type, position, extents, rotations.
    %   loadObj           - a struct for the moving load (a sphere) with fields: 
    %                       name, type, position, extents, rotations.
    %   pivot             - a 1x3 vector (e.g., [10,10,50]) representing the fixed pivot.
    %   CartesianCoordinatesAndRotation - an N×7 matrix where each row is:
    %                       [t, X, Y, Z, R1, R2, R3]
    %                       The translation [X, Y, Z] is relative to the pivot.
    %   minDistancesStruct- a struct array (one per obstacle) with fields:
    %                       objectName, minDistance, time, contactPoint.
    
    % Number of time steps.
    nSteps = size(CartesianCoordinatesAndRotation, 1);
    % Compute trajectory positions: for each row, global position = pivot + [X,Y,Z]
    offsets = CartesianCoordinatesAndRotation(:, 2:4);  % Nx3 matrix
    % Ensure pivot is replicated for each row.
    trajectoryPositions = offsets + repmat(pivot, nSteps, 1);
    
    % Set up the figure.
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Moving Load Trajectory with Contact Points');
    view(3);
    
    % Plot each stationary object.
    nObs = length(stationaryObjects);
    colors = lines(nObs);
    for i = 1:nObs
        obj = stationaryObjects(i);
        % Transform the obstacle geometry into world coordinates.
        objWorld = transformObject(obj);
        % Compute a convex hull for a nice patch.
        K = convhulln(objWorld);
        patch('Vertices', objWorld, 'Faces', K, 'FaceColor', colors(i,:), 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    end
    
    % Plot the complete trajectory as a line.
    plot3(trajectoryPositions(:,1), trajectoryPositions(:,2), trajectoryPositions(:,3), 'k.-', 'LineWidth', 1.5, 'MarkerSize', 10);
    
    % Animate the moving load along its trajectory.
    % We'll use a blue marker for the load.
    hLoad = plot3(trajectoryPositions(1,1), trajectoryPositions(1,2), trajectoryPositions(1,3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    for i = 1:nSteps
        % Update the moving load marker.
        set(hLoad, 'XData', trajectoryPositions(i,1), 'YData', trajectoryPositions(i,2), 'ZData', trajectoryPositions(i,3));
        drawnow;
        pause(0.02); % Adjust the pause for desired speed.
    end
    
    % Now, overlay red spheres at the contact points (if available) for each obstacle.
    for i = 1:nObs
        cp = minDistancesStruct(i).contactPoint;
        if ~isempty(cp)
            % Plot a red sphere (or marker) at the contact point.
            scatter3(cp(1), cp(2), cp(3), 100, 'r', 'filled');
            % Annotate with the obstacle name and time.
            t_str = num2str(minDistancesStruct(i).time, '%.3f');
            label = [minDistancesStruct(i).objectName, ', t=', t_str];
            text(cp(1), cp(2), cp(3), label, 'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helper Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vertices_world = transformObject(obj)
    % transformObject converts an obstacle's local geometry into world coordinates.
    % The obstacle struct has fields: name, type, position, extents, rotations.
    pos = obj.position;       % 1x3
    eul = obj.rotations;      % 1x3 Euler angles (radians)
    % If the object doesn't include a 'vertices' field, compute box vertices.
    if isfield(obj, 'vertices')
        vertices = obj.vertices;
    else
        vertices = boxVertices(obj.extents);
    end
    R = eul2rotm(eul);
    vertices_world = (R * vertices')' + pos;
end

function vertices = boxVertices(extents)
    % boxVertices returns the 8 vertices of a box centered at the origin given its half-dimensions.
    L = extents/2;
    vertices = [ -L(1), -L(2), -L(3);
                  L(1), -L(2), -L(3);
                  L(1),  L(2), -L(3);
                 -L(1),  L(2), -L(3);
                 -L(1), -L(2),  L(3);
                  L(1), -L(2),  L(3);
                  L(1),  L(2),  L(3);
                 -L(1),  L(2),  L(3)];
end

function R = eul2rotm(eul)
    % eul2rotm converts Euler angles [R1, R2, R3] (radians) to a rotation matrix.
    % We use a ZYX intrinsic rotation order.
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

function vertices_world = transformLoad(load)
    % transformLoad transforms the moving load's local geometry (a sphere) into world coordinates.
    % load.extents is the sphere's radius.
    vertices = sphereVertices(load.extents);
    R = eul2rotm(load.rotations);
    vertices_world = (R * vertices')' + load.position(:)';
end

function vertices = sphereVertices(r)
    % sphereVertices generates sample points on a sphere of radius r.
    [X, Y, Z] = sphere(10);  % 11x11 grid of points.
    vertices = [X(:), Y(:), Z(:)] * r;
    vertices = unique(vertices, 'rows');
end
