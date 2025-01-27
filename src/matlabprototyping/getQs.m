function Q_combined = getQs(framenumber, framelist)
    % Combines time-dependent Q coordinates from multiple frames
    % Inputs:
    %   framenumber - Number of frames
    %   frameslist  - List of frame data
    % Output:
    %   Q_combined - Combined Q coordinates from all frames

    Q_combined = sym([]);
    for i = 1:framenumber
        Q_combined = [Q_combined; framelist(i).Qcoordinates];
    end
end