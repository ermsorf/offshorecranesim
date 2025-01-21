function O = makeO(framenumber, frameslist)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
T = makeE(framenumber, frameslist)^-1 * makeEdot(framenumber,frameslist);
O = simplify(T);
framelist(framenumber).Omatrix = O;
end