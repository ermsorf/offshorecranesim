function Mmatrix = makeM(framenumber, framelist)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
    Mmatrix = sym(zeros(framenumber*6,framenumber*6));
    mass = framelist(framenumber).mass;
    massI = mass .* eye(3);

    for i = 1:framenumber
        Mmatrix(i*6-5:i*6-3,i*6-5:i*6-3) = massI;
        Mmatrix(i*6-2:i*6,i*6-2:i*6) = framelist(framenumber).Jmatrix;
    end
    
end