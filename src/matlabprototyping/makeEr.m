    function T = makeEr(frame)
    % Create an SE3 transformation matrix for a given rotation axis
    % Inputs:
    % axis - Integer (1, 2, or 3) specifying the axis of rotation:
    %        1 for x-axis, 2 for y-axis, 3 for z-axis
    % theta - Scalar or symbolic variable for the rotation angle (in radians)
    % Output:
    % T - 4x4 SE3 transformation matrix
    axis = frame.rotationaxis;
    theta = frame.rotationvar;

    % Validate input
    if ~ismember(axis, [1, 2, 3])
        error('Axis must be 1 (x-axis), 2 (y-axis), or 3 (z-axis).');
    end

    % Symbolic cos(theta) and sin(theta) for symbolic compatibility
    c = cos(theta);
    s = sin(theta);

    % Initialize identity SE3 matrix
    T = sym(eye(4));

    % Rotation matrix depending on the chosen axis
    switch axis
        case 1 % Rotation about the x-axis
            T(2,2) = c;  T(2,3) = -s;
            T(3,2) = s;  T(3,3) = c;
        case 2 % Rotation about the y-axis
            T(1,1) = c;  T(1,3) = s;
            T(3,1) = -s; T(3,3) = c;
        case 3 % Rotation about the z-axis
            T(1,1) = c;  T(1,2) = -s;
            T(2,1) = s;  T(2,2) = c;
    end
end