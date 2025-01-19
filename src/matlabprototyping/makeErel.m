function Erel = makeErel(frame)
% Creates Relative E matrix from one CM to next CM

    Erel = makeEv(frame.cm2joint) * makeEr(frame) * makeEv(frame.joint2cm);
    
end