function T = makeEv(dispv)
    % Create an SE3 transformation matrix for a given translation vector
    % Inputs:
    % dispv - 1x3 vector specifying the translation along [x, y, z]
    % Output:
    % T - 4x4 SE3 transformation matrix

    % Validate input
    if ~isvector(dispv) || numel(dispv) ~= 3
        error('dispv must be a 1x3 vector [x, y, z].');
    end

    % Initialize identity SE3 matrix
    T = sym(eye(4));
    
    % Assign translation components
    T(1:3, 4) = dispv(:)'; % Ensure column vector format
end