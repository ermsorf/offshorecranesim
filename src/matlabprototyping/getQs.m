function Q_combined = getQs(frame,allframes)
    framenumber = frame.framenumber;
    Q_combined = [];
    for i = 1:framenumber
        Q_combined = [Q_combined, allframes(i).Qcoordinates];
    end
end