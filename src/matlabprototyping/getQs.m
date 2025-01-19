function Q_combined = getQs(framenumber, frameslist)
    % Combines time-dependent Q coordinates from multiple frames
    % Inputs:
    %   framenumber - Number of frames
    %   frameslist  - List of frame data
    % Output:
    %   Q_combined - Combined Q coordinates from all frames

    Q_combined = [];
    for i = 1:framenumber
        Q_combined = [Q_combined, frameslist(i).Qcoordinates];
    end
end