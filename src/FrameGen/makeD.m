function D = makeD(framenumber, framelist)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
D = sym(zeros(framenumber*6,framenumber*6));

for i = 1:framenumber
    O = makeO(framenumber,framelist);
    w = O(1:3,1:3);
    index1 = i*6-2  ;
    index2 = i*6 ;
    D(index1:index2,index1:index2) = w;

end