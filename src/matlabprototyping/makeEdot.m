function Edot = makeEdot(frame)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
    E = makeE(frame);
    Q = frame.Qcoordinates;
    Qsize = size(Q);
    
    syms t real

    prediff = E;
    for i = 1:(Qsize(2)) % Replace Q coordinates with differentiable variables
        prediff = subs(prediff,Q(1,i),t * Q(2,i));
    end
    postdiff = diff(prediff,t);

    for i = 1:(Qsize(2)) % Replace Q coordinates with differentiable variables
        Edot = subs(postdiff,t * Q(2,i),Q(1,i))
    end
end