function E = makeE(frame)
% Creates E matrix from one CM to next CM
%   Detailed explanation goes here
    E = makeEv(frame.cm2joint) * makeEr(frame) * makeEv(frame.joint2cm)
end