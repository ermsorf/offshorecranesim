function E = makeE(framenumber, frameslist)
    % Creates absolute transformation matrix E
    % Inputs:
    %   framenumber - Number of frames to process
    %   frameslist  - List of frame data
    % Output:
    %   E - Absolute transformation matrix

    E = eye(4);  % Initialize as identity matrix
    
    for i = 1:framenumber
        % Multiply current transformation matrix with frame-specific transformations
        E = E * makeEv(frameslist(i).cm2joint) * makeEr(frameslist(i)) * makeEv(frameslist(i).joint2cm);
    end
    
    E = simplify(E);  % Simplify the resulting matrix
end
