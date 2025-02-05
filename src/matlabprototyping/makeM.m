function M = makeM(framenumber, framelist)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
    
    M = sym(zeros(framenumber*6,framenumber*6));
    massI = framelist(framenumber) .* eye(3);
    for i = 1:framenumber
        M(i*6-5:i*6-3,i*6-5:i*6-3) = massI
        M(i*6-2:i*6,i*6-2:i*6); = framelist(framenumber).Jmatrix;
    end
    
end