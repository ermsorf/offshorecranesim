function E = makeE(frame)
% Creates Relative E matrix from one CM to next CM
%
    E = makeEv(frame.cm2joint) * makeEr(frame) * makeEv(frame.joint2cm);
end