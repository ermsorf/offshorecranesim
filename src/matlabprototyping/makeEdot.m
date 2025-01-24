function Edot = makeEdot(framenumber, framelist)
    % Computes the time derivative of the transformation matrix E
    % Inputs:
    %   framenumber - Frame index
    %   frameslist  - List of frame data
    % Output:
    %   Edot - Time derivative of transformation matrix
    
    % Get transformation matrix
    if isempty(framelist(framenumber).Ematrix)
        E = makeE(framenumber, framelist);
    else
        E = framelist(framenumber).Ematrix;
    end
    
    
    Q = getQs(framenumber, framelist);  % Get Q values
    Qsize = size(Q);
    
    syms t real
    
    prediff = E;
    for i = 1:Qsize(1)  % Replace Q with differentiable variables
        prediff = subs(prediff, Q(i,1), t * Q(i,2));
    end

    postdiff = diff(prediff, t);  % Differentiate with respect to t
    Edot = postdiff;
    
    for i = 1:Qsize(1)  % Replace t*Q(2,i) with Q(1,i)
        Edot = subs(Edot, t*Q(i,2), Q(i,1));
    end

    Edot = simplify(Edot);  % Simplify result
end
