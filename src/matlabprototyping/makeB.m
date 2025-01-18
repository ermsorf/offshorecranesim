function T_final = makeB(varargin)
    error('NotImplemented');
    % Constructs an SE(3) transformation from an unspecified number of frame relations
    % Each frame relation consists of:
    %   theta (rotation angle in radians)
    %   d_jc  (1x3 vector for joint to center of mass displacement)
    %   d_cj  (1x3 vector for center of mass to next joint displacement)
    %
    % Usage:
    %   T_final = makeSE3Frames(theta1, d_jc1, d_cj1, theta2, d_jc2, d_cj2, ...)

    % Initialize B matrix
        B = sym([])
    % Process each frame
    for i = 1:nargin
        
        
    end

end