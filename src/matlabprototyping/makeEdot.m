function Edot = makeEdot(frame,allframes)
error('Currently Broken')
    % Find the index of the input frame
    framenumber = frame.framenumber;  % Assuming this gives the indexS
    E = eye(4);  % Use numerical identity matrix unless symbolic computation is required
    for i = 1:framenumber
        E = E * makeE(allframes(i));  % Access preceding frames
    end

    Q = getQs(frame,allframes);
    Qsize = size(Q);
    
    syms t real

    prediff = E;
    for i = 1:(Qsize(2)) % Replace Q coordinates with differentiable variables
        prediff = subs(prediff,Q(1,i), t*Q(2,i));
    end

    postdiff = diff(prediff,t);
    Edot = postdiff;
    for i = 1:(Qsize(2)) 
        Edot = subs(Edot,t*Q(2,i),Q(1,i));
    end
    
    Edot = simplify(Edot);
end