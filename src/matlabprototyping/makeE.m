function E = makeE(framenumber, framelist)
    % Creates absolute transformation matrix E
    % Inputs:
    %   framenumber - Number of frames to process
    %   frameslist  - List of frame data
    % Output:
    %   E - Absolute transformation matrix

    E = eye(4);  % Initialize as identity matrix
    
    for i = 1:framenumber
        % Multiply current transformation matrix with frame-specific transformations
        E = E * makeEv(framelist(i).cm2joint) * makeEr(framelist(i)) * makeEv(framelist(i).joint2cm);
    end
    
    framelist(framenumber).Ematrix = E;
    E = simplify(E);  % Simplify the resulting matrix
end
